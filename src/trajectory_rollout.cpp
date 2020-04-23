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

#include <cmath>
#include <limits>
#include <vector>
#include <robomagellan/trajectory_rollout.hpp>

// Helper functions to generate samples
void makeSamplesX(double x_min, double x_max, size_t num_samples,
                  std::vector<double>& samples)
{
  samples.clear();

  double step_x = (x_max - x_min) / (num_samples - 1);
  for (size_t i = 0; i < num_samples; ++i)
  {
    samples.push_back(x_max - (i * step_x));
  }
}

void makeSamplesTh(double th_min, double th_max, size_t num_samples,
                   std::vector<double>& samples)
{
  samples.clear();

  // Add zero heading first if that is valid
  if (th_min <= 0.0)
  {
    samples.push_back(0.0);
  }

  double step_th = ((th_max - th_min) * 2.0) / (num_samples - 1);
  for (size_t i = 0; i < (num_samples / 2); ++i)
  {
    double th = std::max(step_th, th_min) + i * step_th;
    samples.push_back(-th);
    samples.push_back(th);
  }
}

double getPathDistance(const Path& path, double x, double y)
{
  double d1 = std::numeric_limits<double>::max();
  double d2 = std::numeric_limits<double>::max();

  for (auto pt : path.waypoints)
  {
    double d = std::sqrt(pow(pt->x - x, 2) + pow(pt->y - y, 2));
    if (d < d1)
    {
      d2 = d1;
      d1 = d;
    }
    else if (d < d2)
    {
      d2 = d;
    }
  }

  return (d1 + d2) / 2.0;
}

TrajectoryRollout::TrajectoryRollout(size_t samples_x,
                                     size_t samples_th,
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
                                     double velocity_scale) :
  samples_x_(samples_x),
  samples_th_(samples_th),
  sim_time_(sim_time),
  sim_resolution_(sim_resolution),
  min_vel_x_(min_vel_x),
  max_vel_x_(max_vel_x),
  min_vel_th_(min_vel_th),
  max_vel_th_(max_vel_th),
  acc_lim_x_(acc_lim_x),
  acc_lim_th_(acc_lim_th),
  goal_scale_(goal_scale),
  path_scale_(path_scale),
  obstacle_scale_(obstacle_scale),
  velocity_scale_(velocity_scale)
{
  // TODO?
}

TrajectoryRollout::~TrajectoryRollout()
{
}

bool TrajectoryRollout::findTraj(const Path& path,
                                 double robot_x, double robot_y, double robot_th,
                                 double robot_vel_x, double robot_vel_th,
                                 geometry_msgs::Twist& command)
{
  double vel_x = 0.0;
  double vel_th = 0.0;
  double best_score = std::numeric_limits<double>::max();

  // Determine sample ranges
  std::vector<double> x_s, th_s;
  makeSamplesX(min_vel_x_, max_vel_x_, samples_x_, x_s);
  makeSamplesTh(0.0, max_vel_th_, samples_th_, th_s);

  // Simulate forward trajectories first
  for (auto x : x_s)
  {
    for (auto th : th_s)
    {
      double score = evaluate(path,
                              robot_x, robot_y, robot_th,
                              robot_vel_x, robot_vel_th,
                              x, th);
      if (score < 0.0)
      {
        // Collision
        continue;
      }
      else if (score < best_score)
      {
        best_score = score * 0.98;
        vel_x = x;
        vel_th = th;
      }
    }
  }

  if (best_score < 1000.0)  // TODO: remove random number
  {
    // If we have a forward velocity, return it
    command.linear.x = vel_x;
    command.angular.z = vel_th;
    return true;
  }

  // Determine sample range for in-place rotation
  makeSamplesTh(min_vel_th_, max_vel_th_, samples_th_, th_s);

  // Simulate in-place rotation
  for (auto th : th_s)
  {
    double score = evaluate(path,
                            robot_x, robot_y, robot_th,
                            robot_vel_x, robot_vel_th,
                            0.0, th);
    if (score < 0.0)
    {
      // Collision
      continue;
    }
    else if (score < best_score)
    {
      best_score = score * 0.98;
      vel_x = 0.0;
      vel_th = th;
    }
  }

  // No trajectory found
  return false;
}

double TrajectoryRollout::evaluate(const Path& path,
                                  double robot_x, double robot_y, double robot_th,
                                  double robot_vel_x, double robot_vel_th,
                                  double trajectory_vel_x, double trajectory_vel_th)
{
  double cost = 0.0;

  // Local copy of state
  double x = robot_x;
  double y = robot_y;
  double th = robot_th;
  double dx = robot_vel_x;
  double dr = robot_vel_th;

  // Convenience
  double acc_x = acc_lim_x_ * sim_resolution_;
  double acc_th = acc_lim_th_ * sim_resolution_;

  // Forward simulate trajectory
  for (double t = sim_resolution_; t < sim_time_; t += sim_resolution_)
  {
    // Update state
    dx += std::max(std::min(acc_x, trajectory_vel_x - dx), -acc_x);
    dr += std::max(std::min(acc_th, trajectory_vel_th - dr), -acc_th);

    th += dr * sim_resolution_;
    x += dx * sim_resolution_ * cos(th);
    y += dx * sim_resolution_ * sin(th);

    // Determine obstacle cost
    double obstacle_cost = getObstacleCost(x, y, th);
    if (obstacle_cost < 0)
    {
      // Collision - invalid trajectory
      return obstacle_cost;
    }

    // Determine velocity cost
    double velocity_cost = 1.0 / (pow(std::max(dx, 0.25), 2));

    // Add weighted sum
    cost += obstacle_scale_ * obstacle_cost +
            velocity_scale_ * velocity_cost;
  }

  // Evaluate path and goal distance for final point
  double goal_distance = std::sqrt(pow(x - path.waypoints.back()->x, 2.0) +
                                   pow(y - path.waypoints.back()->y, 2.0));
  double path_distance = getPathDistance(path, x, y);
  cost += goal_scale_ * goal_distance +
          path_scale_ * path_distance;

  return cost;
}

double TrajectoryRollout::getObstacleCost(double x, double y, double th)
{
  return 0.0;
}
