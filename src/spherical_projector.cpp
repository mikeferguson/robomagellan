/*
 * Copyright (c) 2024 Michael Ferguson
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

#include <cmath>
#include <memory>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

template <typename T>
class SphericalProjector : public rclcpp::Node
{
public:
  SphericalProjector()
  : rclcpp::Node("spherical_projector"),
    logger_(rclcpp::get_logger("spherical_projector"))
  {
    // Parameters
    cutoff_radius_ = this->declare_parameter<double>("cutoff_radius", 5.0);
    viz_radius_ = this->declare_parameter<double>("vizualization_radius_", 5.0);
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "base_link");
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.2);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Publish the points in a spherical format that can be consumed in Python node
    auto qos = rclcpp::SystemDefaultsQoS();
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("spherical_cloud", qos);
    viz_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("spherical_cloud_viz", qos);

    // Subscribe the incoming point cloud message
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                   "cloud_in", qos, std::bind(&SphericalProjector<T>::cloudCb,
                                              this, std::placeholders::_1));
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    std::shared_ptr<pcl::PointCloud<T>> cloud = std::make_shared<pcl::PointCloud<T>>();
    pcl::fromROSMsg(*msg, *cloud);

    // Transform cloud to fixed_frame_
    try
    {
      tf2_buffer_->canTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp,
                                tf2::durationFromSec(transform_timeout_));
      if (!pcl_ros::transformPointCloud(fixed_frame_, *cloud, *cloud, *tf2_buffer_))
      {
        // Error message gets printed inside transformPointCloud
        return;
      }
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                   msg->header.frame_id.c_str(), fixed_frame_.c_str());
      return;
    }

    // Create cloud to publish
    pcl::PointCloud<T> spherical_cloud, visualization_cloud;
    spherical_cloud.points.reserve(cloud->size());
    visualization_cloud.points.reserve(cloud->size());

    // Convert into theta, altitude, radius
    for (auto & pt : cloud->points)
    {
      // Livox specific filtering
      if (pt.intensity < 0.001) continue;

      // Filter by radius
      float radius = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (radius >= cutoff_radius_) continue;

      // Now compute actual values
      float theta = atan2(pt.y, pt.x);
      float altitude = acos(pt.z / radius);

      T spherical_pt;
      spherical_pt.x = theta;
      spherical_pt.y = altitude;
      spherical_pt.z = radius;
      spherical_pt.intensity = cutoff_radius_ - radius;
      spherical_cloud.push_back(spherical_pt);

      T visualization_pt;
      visualization_pt.x = viz_radius_ * sin(altitude) * cos(theta);
      visualization_pt.y = viz_radius_ * sin(altitude) * sin(theta);
      visualization_pt.z = viz_radius_ * cos(altitude);
      visualization_pt.intensity = cutoff_radius_ - radius;
      visualization_cloud.push_back(visualization_pt);
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(spherical_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = fixed_frame_;
    cloud_pub_->publish(cloud_msg);

    pcl::toROSMsg(visualization_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = fixed_frame_;
    viz_pub_->publish(cloud_msg);
  }

  double cutoff_radius_, viz_radius_;
  double transform_timeout_;
  std::string fixed_frame_;

  // ROS Interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, viz_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

using SphericalProjectorT = SphericalProjector<pcl::PointXYZI>;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SphericalProjectorT>());
  rclcpp::shutdown();
  return 0;
}
