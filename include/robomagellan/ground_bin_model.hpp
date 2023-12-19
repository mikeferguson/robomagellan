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
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/*
 * General algorithm:
 *  1) Use RANSAC to iteratively find largest plane in each bin. Stop when either:
 *     a) The largest plane is parallel enough ground
 *     b) The remaining cloud is smaller than min_points parameter for that bin
 *  2) For bins without enough points (less than min_points), segment them
 *     using the plane model of adjacent bins. (NOT YET IMPLEMENTED)
 *  3) Filter the obstacle cloud using clustering. (NOT YET IMPLEMENTED)
 */

struct BinParams
{
  // Minimum number of points to build internal model
  size_t min_points = 10;

  // Maximum thickness of ground surface
  double planar_tolerance = 0.2;

  // Maximum distance from vertical plane can be and still be ground
  double vertical_tolerance = 0.15;
};

/**
 * @brief Each bin is a collection of points that are geographically close.
 */
template <typename T>
struct Bin
{
  Bin()
  {
    points = std::make_shared<pcl::PointCloud<T>>();
    params = std::make_shared<BinParams>();  // TODO: pass this in with appropriate values set
  }

  void clear()
  {
    points->clear();
    obstacles.clear();
    off_axis.clear();
  }

  void process()
  {
    std::cout << "Process bin (" << ring << "," << sector << "): " << points->size() << " points" << std::endl;

    // Extract ground plane
    while (true)
    {
      if (points->size() < params->min_points)
      {
        // TODO: clearly mark that we didn't process - and then later come
        // back and filter these points by adjacent bin ground planes
        //points->clear();
        return;
      }

      pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<T> seg;
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(params->planar_tolerance);
      seg.setInputCloud(points);
      seg.segment(*inliers, *coef);

      // Get normal for the segmented plane, check against vertical
      normal(0) = coef->values[0];
      normal(1) = coef->values[1];
      normal(2) = coef->values[2];
      float angle = acos(Eigen::Vector3f::UnitZ().dot(normal));
      if (angle < params->vertical_tolerance)
      {
        std::cout << "  Extracting ground plane of size " << inliers->indices.size() << std::endl;
        // This is our ground plane - move all outliers to obstacles
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud(points);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<T> t_obstacles;
        extract.filter(t_obstacles);
        obstacles += t_obstacles;
        // Then remove the outliers from the ground points
        extract.setNegative(false);
        extract.filter(*points);
        break;
      }
      else
      {
        std::cout << "  Removing non-ground plane of size " << inliers->indices.size();
        std::cout << " (" << normal(0) << "," << normal(1) << "," << normal(2) << ")" << std::endl;
        // This is a vertical plane - add to obstacles
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud(points);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<T> t_obstacles;
        extract.filter(t_obstacles);
        obstacles += t_obstacles;
        off_axis += t_obstacles;
        // Then remove these points from the ground plane
        extract.setNegative(true);
        extract.filter(*points);
      }
    }

    pcl::computeMeanAndCovarianceMatrix(*points, covariance, mean);
  }

  // The points in the bin that may be part of ground plane
  std::shared_ptr<pcl::PointCloud<T>> points;

  // The points in this bin that are obstacles
  pcl::PointCloud<T> obstacles;

  // DEBUG ONLY
  // Points that were rejected for being off-axis
  pcl::PointCloud<T> off_axis;
  size_t ring;
  size_t sector;

  // Specification of the point cloud
  Eigen::Vector4f mean;
  Eigen::Vector3f normal;
  Eigen::Matrix3f covariance;

  // Parameters to be used for processing
  std::shared_ptr<BinParams> params;
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
        bins[i][j]->ring = i;
        bins[i][j]->sector = j;
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
        bin->clear();
      }
    }
  }

  /** @brief Add points from a cloud */
  void addPoints(pcl::PointCloud<T> & cloud)
  {
    // Insert points into bins
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
            bins[ring][sector]->points->push_back(point);
            break;
          }
        }
      }
    }

    // Do regional plane fitting in each bin
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        bin->process();
      }
    }
  }

  bool getGroundCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        cloud += *(bin->points);
      }
    }
    return true;
  }

  bool getObstacleCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        cloud += bin->obstacles;
      }
    }
    return true;
  }

  bool getOffAxisCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        cloud += bin->off_axis;
      }
    }
    return true;
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
        for (auto & point : *(bin->points))
        {
          cloud.points.emplace_back(point.x, point.y, point.z, r, g, b);
        }
        for (auto & point : bin->obstacles)
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
