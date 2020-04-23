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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robomagellan/trajectory_rollout_ros.hpp>

class TrajectoryRolloutWithCostmap : public TrajectoryRollout
{
public:
  TrajectoryRolloutWithCostmap(size_t x_samples,
                               size_t th_samples,
                               double sim_time,
                               double sim_resolution,
                               double min_vel_x,
                               double max_vel_x,
                               double min_vel_th,
                               double max_vel_th,
                               double acc_lim_x,
                               double acc_lim_th,
                               double goal_scale,
                               double path_scale,
                               double obstacle_scale,
                               double velocity_scale,
                               costmap_2d::Costmap2DROS* costmap) :
    TrajectoryRollout(x_samples, th_samples,
                      sim_time, sim_resolution,
                      min_vel_x, max_vel_x,
                      min_vel_th, max_vel_th,
                      acc_lim_x, acc_lim_th,
                      goal_scale, path_scale, obstacle_scale, velocity_scale),
    costmap_(costmap)
  {

  }

  virtual ~TrajectoryRolloutWithCostmap() {}

  virtual double getObstacleCost(double x, double y, double theta)
  {
    // TODO: implement collision checking
    return 0.0;

    if (!costmap_)
    {
      // No costmap, assume everything is in collision
      return -1.0;
    }
  }

private:
  costmap_2d::Costmap2DROS * costmap_;
};

TrajectoryRolloutROS::TrajectoryRolloutROS()
{
  // TODO?
}

TrajectoryRolloutROS::~TrajectoryRolloutROS()
{
  if (dserver_)
  {
    delete dserver_;
  }
}

void TrajectoryRolloutROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  tf_ = tf;
  costmap_ = costmap_ros;

  // Setup dynamic reconfigure
  nh_ = ros::NodeHandle("~/" + name);
  dserver_ = new dynamic_reconfigure::Server<robomagellan::TrajectoryRolloutConfig>(nh_);
  auto f = boost::bind(&TrajectoryRolloutROS::configCallback, this, _1, _2);
  dserver_->setCallback(f);

  // Subscribe to odometry feedback
  robot_vel_x_ = 0.0;
  robot_vel_th_ = 0.0;
  odom_sub_ = nh_.subscribe("odom", 1, &TrajectoryRolloutROS::odomCallback, this);

  ROS_INFO_STREAM("TrajectoryRolloutROS [" << name << "] initialized.");
}

bool TrajectoryRolloutROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!path_)
  {
    ROS_ERROR("Cannot compute commands without a plan");
    return false;
  }

  double x, y, yaw;
  if (!getRobotPose(x, y, yaw))
  {
    return false;
  }

  // Get command velocity
  return controller_->findTraj(*path_,
                               x, y, yaw,
                               robot_vel_x_, robot_vel_th_,
                               cmd_vel);
}

bool TrajectoryRolloutROS::isGoalReached()
{
  double x, y, yaw;
  if (!getRobotPose(x, y, yaw))
  {
    return false;
  }

  if (!path_)
  {
    ROS_ERROR("isGoalReached has no path!");
    return false;
  }

  if (std::abs(x - path_->waypoints.back()->x) < goal_tolerance_x_ &&
      std::abs(y - path_->waypoints.back()->y) < goal_tolerance_y_)
  {
    return true;
  }


  return false;
}

bool TrajectoryRolloutROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  std::shared_ptr<Path> path(new Path());

  // Get the transform
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform("odom", plan.front().header.frame_id, ros::Time(0), ros::Duration(0.05));
  }
  catch (tf2::TransformException& exception)
  {
    ROS_ERROR("%s", exception.what());
    return false;
  }

  // Transform plan into odometry frame path
  for (auto pose : plan)
  {
    geometry_msgs::PoseStamped pose_transformed;
    tf2::doTransform(pose, pose_transformed, transform);
    path->waypoints.push_back(waypointFromPose(pose_transformed.pose.position.x,
                                               pose_transformed.pose.position.y,
                                               0.0));
  }

  // Make sure headings are consistent
  setHeadings(*path);

  path_ = path;
  return true;
}

bool TrajectoryRolloutROS::getRobotPose(double& x, double& y, double& yaw)
{
  // Compute robot pose in odometry frame
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.05));
  }
  catch (tf2::TransformException& exception)
  {
    ROS_ERROR("%s", exception.what());
    return false;
  }

  x = transform.transform.translation.x;
  y = transform.transform.translation.y;

  // Get yaw
  double roll, pitch;
  tf2::Quaternion quat;
  tf2::fromMsg(transform.transform.rotation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw, 0);

  return true;
}

void TrajectoryRolloutROS::configCallback(robomagellan::TrajectoryRolloutConfig& config, uint32_t level)
{
  // Create controller instance
  controller_.reset(new TrajectoryRolloutWithCostmap(config.samples_x,
                                                     config.samples_th,
                                                     config.sim_time,
                                                     config.sim_resolution,
                                                     config.min_vel_x,
                                                     config.max_vel_x,
                                                     config.min_vel_th,
                                                     config.max_vel_th,
                                                     config.acc_lim_x,
                                                     config.acc_lim_th,
                                                     config.goal_scale,
                                                     config.path_scale,
                                                     config.obstacle_scale,
                                                     config.velocity_scale,
                                                     costmap_));

  goal_tolerance_x_ = config.goal_tolerance_x;
  goal_tolerance_y_ = config.goal_tolerance_y;

  ROS_INFO("TrajectoryRolloutROS updated through dynamic reconfigure.");
}

void TrajectoryRolloutROS::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  robot_vel_x_ = msg->twist.twist.linear.x;
  robot_vel_th_ = msg->twist.twist.angular.z;
}
