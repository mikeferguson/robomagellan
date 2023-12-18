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
    // Configure bin structure
    size_t num_rings = this->declare_parameter<int>("num_rings", 4);
    std::vector<long> sectors =
      this->declare_parameter<std::vector<long>>("sectors", {8, 8, 8, 16});
    std::vector<double> margins =
      this->declare_parameter<std::vector<double>>("margins", {0.125, 1.25, 2.5, 4.0, 8.0});

    bins_ = std::make_unique<BinModel<T>>(num_rings, margins, sectors);

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

    bins_->clear();
    bins_->addPoints(cloud);

    // Just publish the colorized mapping into bins
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    bins_->getColorCloud(color_cloud);

    sensor_msgs::msg::PointCloud2 color_msg;
    pcl::toROSMsg(color_cloud, color_msg);
    color_msg.header.stamp = msg->header.stamp;
    color_msg.header.frame_id = (tf2_buffer_) ? target_frame_ : msg->header.frame_id;
    ground_pub_->publish(color_msg);
  }

  // Organization
  std::unique_ptr<BinModel<T>> bins_;

  // Parameters
  std::string target_frame_;

  // ROS Interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

#endif  // ROBOMAGELLAN__GROUND_FILTER_HPP_
