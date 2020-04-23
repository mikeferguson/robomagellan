/*
Copyright (c) 2020 Michael E. Ferguson. All right reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef ROBOMAGELLAN_TRAJECTORY_ROLLOUT_ROS_HPP
#define ROBOMAGELLAN_TRAJECTORY_ROLLOUT_ROS_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <robomagellan/TrajectoryRolloutConfig.h>
#include <robomagellan/trajectory_rollout.hpp>

class TrajectoryRolloutROS : public nav_core::BaseLocalPlanner
{
public:
  TrajectoryRolloutROS();
  virtual ~TrajectoryRolloutROS();

  /**
   * @brief  Constructs the local planner.
   * @param name The name to give this instance of the local planner.
   * @param tf A pointer to a transform listener.
   * @param costmap_ros The cost map to use for assigning costs to local plans.
   */
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   *         compute velocity commands to send to the base.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @return True if a valid velocity command was found, false otherwise.
   */
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Check if the goal pose has been achieved by the local planner.
   * @return True if achieved, false otherwise.
   */
  virtual bool isGoalReached();

  /**
   * @brief  Set the plan that the local planner is following.
   * @param plan The plan to pass to the local planner.
   * @return True if the plan was updated successfully, false otherwise.
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:
  bool getRobotPose(double& x, double& y, double& yaw);
  void configCallback(robomagellan::TrajectoryRolloutConfig& config, uint32_t level);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  // Store local copies
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_;

  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<robomagellan::TrajectoryRolloutConfig>* dserver_;
  double goal_tolerance_x_, goal_tolerance_y_;

  // To get robot velocity
  double robot_vel_x_, robot_vel_th_;
  ros::Subscriber odom_sub_;

  // Actual controller implementation
  std::shared_ptr<TrajectoryRollout> controller_;
  std::shared_ptr<Path> path_;
};

#endif  // ROBOMAGELLAN_TRAJECTORY_ROLLOUT_ROS_HPP
