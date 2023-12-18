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

#ifndef ROBOMAGELLAN__GROUND_BIN_MODEL_HPP_
#define ROBOMAGELLAN__GROUND_BIN_MODEL_HPP_

#include <cmath>
#include <memory>
#include <vector>
#include <utility>
#include <angles/angles.h>

/**
 * @brief Each bin is a collection of points that are geographically close.
 */
template <typename T>
struct Bin
{
  std::vector<T> points;
};

template <typename T>
using BinPtr = std::shared_ptr<Bin<T>>;

/**
 * @brief Bins are organized in rings around the robot.
 */
template <typename T>
struct BinModel
{
  /**
   * @brief Create a new bin model
   * @param num_rings Number of rings around the robot.
   * @param margins The start/end ranges for the rings.
   * @param sectors Number of sectors in each ring.
   */
  BinModel(size_t num_rings, std::vector<double> margins, std::vector<long> sectors)
  {
    assert(margins.size() == num_rings + 1);
    assert(sectors.size() == num_rings);

    // Allocate storage
    bins.resize(num_rings);
    for (size_t i = 0; i < num_rings; ++i)
    {
      std::cout << "Ring " << i
                << " spans " << margins[i] << "-" << margins[i + 1] << "m"
                << " has " << sectors[i] << " sectors" << std::endl;
      bins[i].resize(sectors[i]);
      for (size_t j = 0; j < sectors[i]; ++j)
      {
        bins[i][j] = std::make_shared<Bin<T>>();
      }
      sector_margins.push_back(2 * M_PI / sectors[i]);
    }

    this->ring_margins = margins;
  }

  /** @brief Clear all bins. */
  void clear()
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        bin->points.clear();
      }
    }
  }

  /** @brief Add points from a cloud */
  void addPoints(pcl::PointCloud<T> & cloud)
  {
    for (size_t i = 0; i < cloud.size(); ++i)
    {
      T point = cloud.points[i];
      double radius = std::hypot(point.x, point.y);

      if (radius > ring_margins[0])
      {
        // Determine which ring
        for (size_t ring = 0; ring < bins.size(); ++ring)
        {
          if (radius < ring_margins[ring + 1])
          {
            // Point is in this ring - compute sector
            double theta = angles::normalize_angle(atan2(point.y, point.x)) + M_PI;
            size_t sector = theta / sector_margins[ring];
            bins[ring][sector]->points.push_back(point);
            break;
          }
        }
      }
    }
  }

  /** @brief Get a colorized representation of the point cloud. */
  bool getColorCloud(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
  {
    cloud.points.clear();

    // Setup colorization
    // This works well when rings have even number of sectors
    uint8_t r = 255;
    uint8_t g = 0;
    uint8_t b = 0;

    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        for (auto & point : bin->points)
        {
          cloud.points.emplace_back(point.x, point.y, point.z, r, g, b);
        }
        // Shift colors
        std::swap(b, g);
        std::swap(r, g);
      }
    }

    return true;
  }

  // Vector of rings, which are a vector of sector bins
  std::vector<std::vector<BinPtr<T>>> bins;
  std::vector<double> ring_margins;
  std::vector<double> sector_margins;
};

#endif  // ROBOMAGELLAN__GROUND_BIN_MODEL_HPP_
