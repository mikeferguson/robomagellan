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
#include <robomagellan/ground_model.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/*
 * General algorithm:
 *  1) Split point cloud into bins. Bins are organized as a set of concentric
 *     rings, each of which has a number of sectors.
 *  2) Use RANSAC to iteratively find largest plane in each bin. Stop when either:
 *     a) The largest plane is parallel enough ground
 *     b) The remaining cloud is smaller than min_points parameter for that bin
 *  3) Use adjacent bins to refine obstacle/ground segmentation:
 *     a) For outliers where the normal was not vertical enough, compare the
 *        normal to the ground plane normal in the adjacent bin. This helps
 *        when the ground is on a hill.
 *     b) For bins without enough points (less than min_points), segment them
 *        using the plane model of adjacent bins. (NOT YET IMPLEMENTED)
 *  4) Filter the obstacle cloud using clustering. (NOT YET IMPLEMENTED)
 */

struct BinParams
{
  // Minimum number of points to build internal model
  size_t min_points = 10;

  // Maximum thickness of ground surface
  double planar_tolerance = 0.05;

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
    params = std::make_shared<BinParams>();
  }

  void clear()
  {
    points->clear();
    outliers.clear();
    outlier_normals.clear();
    outlier_is_obstacle.clear();
    // Reset normal to all zeros
    normal = Eigen::Vector3f::Zero();
  }

  void process()
  {
    // Extract ground plane
    while (true)
    {
      if (points->size() < params->min_points)
      {
        // 2b) Terminate because remaining cloud is too small
        std::shared_ptr<pcl::PointCloud<T>> cloud = std::make_shared<pcl::PointCloud<T>>();
        cloud = points;
        outliers.push_back(cloud);
        outlier_normals.push_back(Eigen::Vector3f::Zero());
        outlier_is_obstacle.push_back(false);  // Default to not an obstacle
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
        // 2a) Terminate because this plane is parallel enough to the ground
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud(points);
        // Extract outliers into a new outlier candidate
        std::shared_ptr<pcl::PointCloud<T>> cloud = std::make_shared<pcl::PointCloud<T>>();
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
        outliers.push_back(cloud);
        // This outlier has no normal, cannot be turned back into ground points
        outlier_normals.push_back(Eigen::Vector3f::Zero());
        outlier_is_obstacle.push_back(true);
        // Then remove the outliers from the ground points
        extract.setNegative(false);
        extract.filter(*points);
        return;
      }
      else
      {
        // This is a vertical plane - add to obstacles
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud(points);
        extract.setIndices(inliers);
        extract.setNegative(false);
        std::shared_ptr<pcl::PointCloud<T>> cloud = std::make_shared<pcl::PointCloud<T>>();
        extract.filter(*cloud);
        outliers.push_back(cloud);
        outlier_normals.push_back(normal);
        outlier_is_obstacle.push_back(true);
        // Then remove these points from the ground plane
        extract.setNegative(true);
        extract.filter(*points);
      }
    }
  }

  // Bin ID
  size_t ring, sector;

  // The points in the bin that may be part of ground plane
  std::shared_ptr<pcl::PointCloud<T>> points;

  // The points in this bin that are obstacles
  std::vector<std::shared_ptr<pcl::PointCloud<T>>> outliers;
  std::vector<Eigen::Vector3f> outlier_normals;
  std::vector<bool> outlier_is_obstacle;

  // Specification of the point cloud
  Eigen::Vector3f normal;

  // Parameters to be used for processing
  std::shared_ptr<BinParams> params;
};

template <typename T>
using BinPtr = std::shared_ptr<Bin<T>>;

/**
 * @brief Bins are organized in rings around the robot.
 */
template <typename T>
class BinModel : public GroundModel<T>
{
public:
  /**
   * @brief Create a new bin model
   * @param num_rings Number of rings around the robot.
   * @param margins The start/end ranges for the rings.
   * @param sectors Number of sectors in each ring.
   */
  BinModel(size_t num_rings, std::vector<double> margins, std::vector<long> sectors)
  {
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

  /** @brief Set the bin parameters. */
  void setBinParams(std::shared_ptr<BinParams> params, double scaling)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        size_t ring = bin->ring;
        double dist = std::min(1.0, (ring_margins[ring] + ring_margins[ring + 1]) / 2.0);
        bin->params->min_points = params->min_points;
        bin->params->planar_tolerance = params->planar_tolerance * (1 + dist * scaling);
        bin->params->vertical_tolerance = params->vertical_tolerance;
      }
    }
  }

  /** @brief Add points from a cloud. */
  void addPoints(pcl::PointCloud<T> & cloud)
  {
    // Insert points into bins
    for (size_t i = 0; i < cloud.size(); ++i)
    {
      T point = cloud.points[i];
      if (point.intensity < 0.001)
      {
        // Filter out points that are erroneous
        continue;
      }
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

    // Ensure consistency between bin
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        // Check bin inside
        if (bin->ring > 0)
        {
          int this_level = bins[bin->ring].size();
          int lower_level = bins[bin->ring].size();
          int s = bin->ring / (this_level / lower_level);
          apply_consistency(bin, bins[bin->ring - 1][s]);
        }

        // Check bin to left/right
        int l = (static_cast<int>(bin->sector) - 1) % bins[bin->ring].size();
        int r = (static_cast<int>(bin->sector) + 1) % bins[bin->ring].size();
        apply_consistency(bin, bins[bin->ring][l]);
        apply_consistency(bin, bins[bin->ring][r]);
      }
    }
  }

  /** @brief Get a point cloud of all ground points. */
  bool getGroundCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        // All points still in the "points" are ground points
        cloud += *(bin->points);
        // Outliers that are not obstacles are also ground points
        for (size_t i = 0; i < bin->outliers.size(); ++i)
        {
          if (!bin->outlier_is_obstacle[i])
          {
            cloud += *(bin->outliers[i]);
          }
        }
      }
    }
    return true;
  }

  /** @brief Get a point cloud of all obstacle points. */
  bool getObstacleCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & ring : bins)
    {
      for (auto & bin : ring)
      {
        for (size_t i = 0; i < bin->outliers.size(); ++i)
        {
          if (bin->outlier_is_obstacle[i])
          {
            cloud += *(bin->outliers[i]);
          }
        }
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
        for (auto & outlier_cloud : bin->outliers)
        {
          for (auto & point : *outlier_cloud)
          {
            cloud.points.emplace_back(point.x, point.y, point.z, r, g, b);
          }
        }
        // Shift colors
        std::swap(b, g);
        std::swap(r, g);
      }
    }

    return true;
  }

private:
  /** @brief Apply local consistency constraints. */
  void apply_consistency(BinPtr<T> a, BinPtr<T> b)
  {
    if (b->normal.norm() < 0.1)
    {
      // Not a valid normal - bin B has no reference ground plane.
      return;
    }

    for (size_t i = 0; i < a->outliers.size(); ++i)
    {
      auto n = a->outlier_normals[i];
      if (n.norm() < 0.1)
      {
        // Not a valid normal
        // TODO: use adjacent bin to see if we can make these not outliers
      }
      else
      {
        float angle = acos(n.dot(b->normal));
        if (angle < a->params->vertical_tolerance)
        {
          /*
          if (a->outlier_is_obstacle[i])
          {
            std::cout << "Bin " << a->ring << "," << a->sector
                      << " reverted outlier when aligned with "
                      << b->ring << "," << b->sector << std::endl;
          }
          */
          a->outlier_is_obstacle[i] = false;
        }
      }
    }
  }

  // Vector of rings, which are a vector of sector bins
  std::vector<std::vector<BinPtr<T>>> bins;
  std::vector<double> ring_margins;
  std::vector<double> sector_margins;
};

#endif  // ROBOMAGELLAN__GROUND_BIN_MODEL_HPP_
