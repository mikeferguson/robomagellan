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

#ifndef ROBOMAGELLAN_TRAJECTORY_ROLLOUT_HPP
#define ROBOMAGELLAN_TRAJECTORY_ROLLOUT_HPP

#include <geometry_msgs/Twist.h>
#include <robomagellan/path.hpp>

/**
 * @brief Implementation of trajectory rollout, as proposed in
 *        in "Planning and Control in Unstructured Terrain",
 *        Gerkey and Konolige 2008.
 */ 
class TrajectoryRollout
{
public:
  /**
   * @brief Constructor
   * @param samples_x Number of samples in x velocity.
   * @param samples_th Number of sanples in theta velocity.
   * @param sim_time Time to forward simulate (seconds).
   * @param min_vel_x Minimum forward velocity (m/s).
   * @param max_vel_x Maximum forward velocity (m/s),
   * @param min_vel_th Minimum speed for in-place rotations (rad/s),
   * @param max_vel_th Maximum speed for forward rotations (rad/s).
   * @param acc_lim_x Maximum forward acceleration (m/s^2).
   * @param acc_lim_th Maximum angular acceleration (rad/s^2).
   * @param goal_scale Scalar for goal distance cost.
   * @param path_scale Scalar for path distance cost.
   * @param obstacle_scale Scalar for obstacle cost.
   * @param velocity_scale Scalar for forward velocity cost.
   */
  TrajectoryRollout(size_t samples_x = 25,
                    size_t samples_th = 25,
                    double sim_time = 2.0,
                    double sim_resolution = 0.1,
                    double min_vel_x = 0.25,
                    double max_vel_x = 1.0,
                    double min_vel_th = 0.5,
                    double max_vel_th = 2.0,
                    double acc_lim_x = 0.5,
                    double acc_lim_th = 1.0,
                    double goal_scale = 1.0,
                    double path_scale = 1.0,
                    double obstacle_scale = 1.0,
                    double velocity_scale = 0.1);

  virtual ~TrajectoryRollout();

  /**
   * @brief Find a trajectory.
   * @param path The path to follow, in global frame.
   * @param robot_x The robot's current x position in global frame.
   * @param robot_y The robot's current y position in global frame.
   * @param robot_th The robot's current heading in global frame.
   * @param robot_vel_x The robot's current forward velocity in robot frame.
   * @param robot_vel_th The robot's current angular velocity in robot frame.
   * @param command The generated command to follow.
   */
  bool findTraj(const Path& path,
                double robot_x, double robot_y, double robot_th,
                double robot_vel_x, double robot_vel_th,
                geometry_msgs::Twist& command);

protected:
  /**
   * @brief Get the obstacle cost for a robot pose
   *
   * This should be implemented in the derived class
   */
  virtual double getObstacleCost(double x, double y, double theta);

  /**
   * @brief Evaluate a single trajectory. 
   */
  double evaluate(const Path& path,
                  double robot_x, double robot_y, double robot_th,
                  double robot_vel_x, double robot_vel_th,
                  double trajectory_vel_x, double trajectory_vel_th);

private:
  size_t samples_x_, samples_th_;
  double sim_time_, sim_resolution_;
  double min_vel_x_, max_vel_x_;
  double min_vel_th_, max_vel_th_;
  double acc_lim_x_, acc_lim_th_;
  double goal_scale_, path_scale_, obstacle_scale_, velocity_scale_;
};

#endif  // ROBOMAGELLAN_TRAJECTORY_ROLLOUT_HPP
