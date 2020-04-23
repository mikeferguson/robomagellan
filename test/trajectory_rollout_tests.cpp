
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
#include <iostream>
#include <gtest/gtest.h>
#include <robomagellan/trajectory_rollout.hpp>

// These are not in the header
void makeSamplesX(double x_min, double x_max, size_t num_samples,
                  std::vector<double>& samples);
void makeSamplesTh(double th_min, double th_max, size_t num_samples,
                   std::vector<double>& samples);

TEST(TrajetoryRolloutTests, sampler_tests)
{
  std::vector<double> samples;
  makeSamplesX(0.1, 1.0, 10, samples);

  EXPECT_EQ(samples.size(), (size_t) 10);
  EXPECT_DOUBLE_EQ(samples[0], 1.0);
  EXPECT_DOUBLE_EQ(samples[1], 0.9);
  EXPECT_DOUBLE_EQ(samples[9], 0.1);

  makeSamplesTh(0.0, 1.0, 5, samples);

  EXPECT_EQ(samples.size(), (size_t) 5);
  EXPECT_DOUBLE_EQ(samples[0], 0.0);
  EXPECT_DOUBLE_EQ(samples[1], -0.5);
  EXPECT_DOUBLE_EQ(samples[2], 0.5);
  EXPECT_DOUBLE_EQ(samples[3], -1.0);
  EXPECT_DOUBLE_EQ(samples[4], 1.0);
}

TEST(TrajectoryRolloutTests, evaluate_tests)
{
  Path path;

  WaypointPtr pt(new Waypoint);
  pt->x = 0.5;
  pt->y = 0.0;
  path.waypoints.push_back(pt);

  pt.reset(new Waypoint);
  pt->x = 1.0;
  pt->y = 0.0;
  path.waypoints.push_back(pt);

  pt.reset(new Waypoint);
  pt->x = 1.5;
  pt->y = 0.5;
  path.waypoints.push_back(pt);

  pt.reset(new Waypoint);
  pt->x = 2.0;
  pt->y = 1.0;
  path.waypoints.push_back(pt);

  pt.reset(new Waypoint);
  pt->x = 2.0;
  pt->y = 1.5;
  path.waypoints.push_back(pt);

  pt.reset(new Waypoint);
  pt->x = 2.0;
  pt->y = 2.0;
  path.waypoints.push_back(pt);

  setHeadings(path);

  // State
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double dx = 0.0;
  double dr = 0.0;

  TrajectoryRollout rollout(25 /* samples_x */,
                            25 /* samples_th */,
                            2.0 /* sim_time */,
                            0.1 /* sim_resolution */,
                            0.25 /* min_vel_x */,
                            1.0 /* max_vel_x */,
                            0.5 /* min_vel_th */,
                            2.0 /* max_vel_th */,
                            0.5 /* acc_lim_x */,
                            1.0 /* acc_lim_th */,
                            1.0 /* goal_scale */,
                            10.0 /* path_scale */,
                            1.0 /* obstacle_scale */,
                            0.1 /* velocity_scale */);
  geometry_msgs::Twist command;

  EXPECT_TRUE(rollout.findTraj(path,
                               x, y, th,
                               dx, dr,
                               command));
  EXPECT_DOUBLE_EQ(command.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);

  x = 0.261284;
  y = 0.017458;
  th = 0.163333;
  dx = 1.0;
  dr = 0.333333;

  EXPECT_TRUE(rollout.findTraj(path,
                               x, y, th,
                               dx, dr,
                               command));
  EXPECT_DOUBLE_EQ(command.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 1 / 3.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
