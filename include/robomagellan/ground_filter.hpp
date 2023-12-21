/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOMAGELLAN__GROUND_FILTER_HPP_
#define ROBOMAGELLAN__GROUND_FILTER_HPP_

#include <memory>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robomagellan/ground_bin_model.hpp>
#include <robomagellan/ground_grid_model.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

template <typename T>
class GroundFilter : public rclcpp::Node
{
public:
  GroundFilter()
  : rclcpp::Node("ground_filter"),
    logger_(rclcpp::get_logger("ground_filter"))
  {
    // Parameters
    debug_topics_ = this->declare_parameter<bool>("debug_topics", false);

    std::string filter = this->declare_parameter<std::string>("filter_type", "grid");
    if (filter == "grid")
    {
      // Configure grid size
      double cell_size = this->declare_parameter<double>("cell_size", 0.5);
      double grid_size = this->declare_parameter<double>("grid_size", 16.0);
      std::unique_ptr<GridModel<T>> model = std::make_unique<GridModel<T>>(cell_size, grid_size);

      // Set parameters
      std::shared_ptr<GridParams> params = std::make_shared<GridParams>();
      params->min_points = this->declare_parameter<int>("grid_min_points", params->min_points);
      params->planar_tolerance =
        this->declare_parameter<double>("grid_planar_tol", params->planar_tolerance);
      params->vertical_tolerance =
        this->declare_parameter<double>("grid_vertical_tol", params->vertical_tolerance);
      double scale = this->declare_parameter<double>("grid_dist_scaling", 0.25);
      model->setParams(params, scale);

      model_ = std::move(model);
    }
    else if (filter == "bin")
    {
      // Configure bin structure
      size_t num_rings = this->declare_parameter<int>("num_rings", 7);
      std::vector<long> sectors =
        this->declare_parameter<std::vector<long>>("sectors", {4, 16, 16, 32, 32, 32, 32});
      std::vector<double> margins =
        this->declare_parameter<std::vector<double>>("margins", {0.125, 1.0, 1.6, 2.4, 3.4, 4.8, 6.4, 8.0});

      if (margins.size() != num_rings + 1 || sectors.size() != num_rings)
      {
        RCLCPP_ERROR(logger_, "Bin structure is malformed");
        exit(1);
      }

      std::unique_ptr<BinModel<T>> model =
        std::make_unique<BinModel<T>>(num_rings, margins, sectors);

      // Configure bin parameters
      std::shared_ptr<BinParams> params = std::make_shared<BinParams>();
      params->min_points = this->declare_parameter<int>("bin_min_points", 10);
      params->planar_tolerance = this->declare_parameter<double>("bin_planar_tol", 0.2);
      params->vertical_tolerance = this->declare_parameter<double>("bin_vertical_tol", 0.2);
      double scale = this->declare_parameter<double>("bin_dist_scaling", 0.15);
      model->setBinParams(params, scale);

      model_ = std::move(model);
    }

    // Optionally transform cloud into another frame (usually base_link)
    target_frame_ = this->declare_parameter<std::string>("target_frame", "");
    if (!target_frame_.empty())
    {
      tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    }

    auto qos = rclcpp::SystemDefaultsQoS();

    // Publish the points that correspond to the ground plane
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground", qos);

    // Publish the points that correspond to obstacles
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", qos);

    if (debug_topics_)
    {
      colored_bin_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_bins", qos);
    }

    // Subscribe the incoming point cloud message
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                   "cloud", qos, std::bind(&GroundFilter<T>::cloudCb, this, std::placeholders::_1));
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    pcl::PointCloud<T> cloud;
    pcl::fromROSMsg(*msg, cloud);

    if (tf2_buffer_)
    {
      try
      {
        if (!pcl_ros::transformPointCloud(target_frame_, cloud, cloud, *tf2_buffer_))
        {
          // Error message gets printed inside transformPointCloud
          return;
        }
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                     msg->header.frame_id.c_str(), target_frame_.c_str());
        return;
      }
    }

    RCLCPP_INFO(logger_, "Start process");
    model_->clear();
    model_->addPoints(cloud);
    RCLCPP_INFO(logger_, "End process");

    pcl::PointCloud<T> ground_cloud, obstacle_cloud;
    model_->getGroundCloud(ground_cloud);
    model_->getObstacleCloud(obstacle_cloud);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(ground_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = (tf2_buffer_) ? target_frame_ : msg->header.frame_id;
    ground_pub_->publish(cloud_msg);

    pcl::toROSMsg(obstacle_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = (tf2_buffer_) ? target_frame_ : msg->header.frame_id;
    obstacle_pub_->publish(cloud_msg);

    if (debug_topics_)
    {
      pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
      model_->getColorCloud(color_cloud);
      sensor_msgs::msg::PointCloud2 color_msg;
      pcl::toROSMsg(color_cloud, color_msg);
      color_msg.header.stamp = msg->header.stamp;
      color_msg.header.frame_id = (tf2_buffer_) ? target_frame_ : msg->header.frame_id;
      colored_bin_pub_->publish(color_msg);
    }
  }

  // Organization
  std::unique_ptr<GroundModel<T>> model_;

  // Parameters
  bool debug_topics_;
  std::string target_frame_;

  // ROS Interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_bin_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

#endif  // ROBOMAGELLAN__GROUND_FILTER_HPP_
