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

#include <iostream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robomagellan/srv/from_lla.hpp>
#include <robomagellan/srv/to_lla.hpp>
#include <robomagellan/convert_gps.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GPStoCart : public rclcpp::Node
{
public:
  GPStoCart()
  : rclcpp::Node("gps_to_cart")
  {
    gps_converter_ = new robomagellan::ConvertGPS();

    // Publish the pose of the robot within map frame
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "gps/pose", rclcpp::SystemDefaultsQoS());

    // For rviz_satellite support - publish the GPS fix that is the root of the map
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
                 "map/fix", rclcpp::SystemDefaultsQoS());

    // Subscribe the incoming fix message
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                 "fix", rclcpp::SystemDefaultsQoS(),
                 std::bind(&GPStoCart::gpsCallback, this, std::placeholders::_1));

    // Setup services
    from_service_ = this->create_service<robomagellan::srv::FromLLA>("from_lla",
                      std::bind(&GPStoCart::fromLLA, this, std::placeholders::_1, std::placeholders::_2));
    to_service_ = this->create_service<robomagellan::srv::ToLLA>("to_lla",
                    std::bind(&GPStoCart::toLLA, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg)
  {
    // Don't process this until we have an actual fix
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
      RCLCPP_WARN(this->get_logger(), "GPS does not have fix!");
      return;
    }

    // If this is the first message, this is our origin, publish it for rviz_satellite
    if (map_fix_.header.frame_id.empty())
    {
      map_fix_.header.frame_id = "map";
      map_fix_.status = msg->status;
      map_fix_.latitude = msg->latitude;
      map_fix_.longitude = msg->longitude;
      map_fix_.altitude = msg->altitude;
      map_fix_.position_covariance = msg->position_covariance;
      map_fix_.position_covariance_type = msg->position_covariance_type;
    }
    map_fix_.header.stamp = this->now();
    gps_pub_->publish(map_fix_);

    // Convert GPS fix to cartesian frame
    double e, n, u;
    gps_converter_->LLAtoCart(msg->latitude, msg->longitude, msg->altitude, &e, &n, &u);
    std::cout << e << ", " << n << ", " << u << " : "
              << std::sqrt(msg->position_covariance[0]) << ", "
              << std::sqrt(msg->position_covariance[4]) << std::endl;

    // Publish pose
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = e;
    pose.pose.pose.position.y = n;
    pose.pose.pose.position.z = u;
    pose.pose.pose.orientation.w = 1.0;
    // X/Y Covariance per GPS
    pose.pose.covariance[0] = msg->position_covariance[0];
    pose.pose.covariance[7] = msg->position_covariance[4];
    // Orientation is unknown from GPS
    pose.pose.covariance[35] = 1000.0;
    pose_pub_->publish(pose);
  }

  void fromLLA(const std::shared_ptr<robomagellan::srv::FromLLA::Request> request,
               std::shared_ptr<robomagellan::srv::FromLLA::Response> response)
  {
    if (gps_converter_->ready())
    {
      for (auto & gp : request->lla_points)
      {
        geometry_msgs::msg::Point mp;
        gps_converter_->LLAtoCart(gp.latitude, gp.longitude, gp.altitude, &(mp.x), &(mp.y), &(mp.z));
        response->map_points.push_back(mp);
      }
    }
  }

  void toLLA(const std::shared_ptr<robomagellan::srv::ToLLA::Request> request,
             std::shared_ptr<robomagellan::srv::ToLLA::Response> response)
  {
    if (gps_converter_->ready())
    {
      for (auto & mp : request->map_points)
      {
        geographic_msgs::msg::GeoPoint gp;
        gps_converter_->CartToLLA(mp.x, mp.y, mp.z, &(gp.latitude), &(gp.longitude), &(gp.altitude));
        response->lla_points.push_back(gp);
      }
    }
  }

  robomagellan::ConvertGPS * gps_converter_;
  sensor_msgs::msg::NavSatFix map_fix_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Service<robomagellan::srv::FromLLA>::SharedPtr from_service_;
  rclcpp::Service<robomagellan::srv::ToLLA>::SharedPtr to_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPStoCart>());
  rclcpp::shutdown();
  return 0;
}
