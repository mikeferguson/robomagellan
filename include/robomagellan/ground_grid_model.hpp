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

#ifndef ROBOMAGELLAN__GROUND_GRID_MODEL_HPP_
#define ROBOMAGELLAN__GROUND_GRID_MODEL_HPP_

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

struct GridParams
{
  // Minimum number of points to build internal model
  size_t min_points = 8;

  // Maximum thickness of ground surface
  double planar_tolerance = 0.05;
};

template <typename T>
struct GridCell
{
  GridCell()
  {
    points = std::make_shared<pcl::PointCloud<T>>();
    obstacles = std::make_shared<pcl::PointCloud<T>>();
    params = std::make_shared<GridParams>();
  }

  void clear()
  {
    points->clear();
    obstacles->clear();
    is_ground = false;
    features_valid = false;
    // Reset normal to all zeros
    normal = Eigen::Vector3f::Zero();
  }

  void process()
  {
    if (points->size() < params->min_points)
    {
      // Cannot make determination at this time
      return;
    }

    // Extract best fit plane
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<T> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params->planar_tolerance);
    seg.setInputCloud(points);
    seg.segment(*inliers, *coef);

    // Store the normal for the plane
    normal(0) = coef->values[0];
    normal(1) = coef->values[1];
    normal(2) = coef->values[2];

    // Extract outliers into obstacles
    pcl::ExtractIndices<T> extract;
    extract.setInputCloud(points);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    // And then remove outliers from the cloud
    extract.setNegative(false);
    extract.filter(*points);
    
    // Compute mean and covariance of remaining points
    pcl::computeMeanAndCovarianceMatrix(*points, covariance, mean);

    // Mark features (mean, cov, and normal) as valid
    features_valid = true;
  }

  // The points in the cell that may be part of ground plane
  std::shared_ptr<pcl::PointCloud<T>> points;
  bool is_ground;
  bool features_valid;

  // The points in this cell that are obstacles
  std::shared_ptr<pcl::PointCloud<T>> obstacles;

  // Features of the point cloud
  Eigen::Vector4f mean;
  Eigen::Vector3f normal;
  Eigen::Matrix3f covariance;

  // Parameters to be used for processing
  std::shared_ptr<GridParams> params;
};

template <typename T>
using GriDCellPtr = std::shared_ptr<GridCell<T>>;

/**
 * @brief Segmentation of a point cloud into ground and obstacles.
 *
 * General algorithm:
 *  1) Split the point cloud into a grid of cells in XY plane.
 *  2) Compute the features for each cell:
 *     a) Estimated height in the X axis.
 *     b) Estimated plane, normal, and covariance.
 *  3) Remove any points which are outliers of the plane:
 *     a) If they form a cluster, add to obstacles.
 *     b) If not, they are noise and discarded.
 *  4) Classify the remaining points in each cell as ground or obstacle:
 *     a) Select the non-empty cell as close to the front of the robot as the seed.
 *     b) The seed cell is estimated to be ground solely by the height and normal.
 *     c) Each adjacent cell of the seed is added to the candidate list.
 *     d) Mark a candidate as obstacle if any of the following:
 *        i)   There is a significant increase in height compared to adjacent
 *             cells that are marked as ground.
 *        ii)  There is a signficant change in the normal compared to adjacent
 *             cells that are marked as ground.
 *        iii) The covariance indicates that the ground is "rough"
 *        iv)  The cell is empty (and close enough to the sensor for this
 *             condition to apply).
 *     e) If the candidate is classified as ground - add all adjacent, unvisited
 *        cells to the candidate list and continue processing the list per (d).
 *
 * Future Work:
 *  1) Can we learn weights to apply to the features of each cell?
 */
template <typename T>
class GridModel : public GroundModel<T>
{
public:
  /**
   * @brief Create a new grid model.
   * @param cell_size Size of cells in x & y (meters).
   * @param max_extent Maximum distance cells span in x & y (meters).
   */
  GridModel(double cell_size, double max_extent)
  {
    // Store sizes
    cell_size_ = cell_size;
    x_cells_ = max_extent / cell_size_;
    y_cells_ = max_extent / cell_size_;

    std::cout << "Grid model: " << cell_size_ << ", " << x_cells_ << ", " << y_cells_ << std::endl;

    // Allocate storage
    this->cells_.resize(2 * x_cells_ * 2 * y_cells_);
  }

  /** @brief Clear all cells. */
  void clear()
  {
    for (auto & cell : cells_)
    {
      cell.clear();
    }
  }

  /** @brief Set the cell parameters. */
  void setGridParams(std::shared_ptr<GridParams> params)
  {
    for (auto & cell : cells_)
    {
      cell.params = params;
    }
  }

  /** @brief Add points from a cloud. */
  void addPoints(pcl::PointCloud<T> & cloud)
  {
    // Insert points into bins
    for (size_t i = 0; i < cloud.size(); ++i)
    {
      T point = cloud.points[i];

      int x = (point.x / cell_size_) + x_cells_;
      int y = (point.y / cell_size_) + y_cells_;

      if (x >= 0 && x < (2 * x_cells_) &&
          y >= 0 && y < (2 * y_cells_))
      {
        // Index is valid
        cells_[x * (2 * x_cells_) + y].points->push_back(point);
      }
    }

    // Compute features in each cell
    for (auto & cell : cells_)
    {
      cell.process();
    }

    // Do classification of ground/obstacles
    // TODO
  }

  /** @brief Get a point cloud of all ground points. */
  bool getGroundCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & cell : cells_)
    {
      if (cell.is_ground)
      {
        cloud += *(cell.points);
      }
    }
    return true;
  }

  /** @brief Get a point cloud of all obstacle points. */
  bool getObstacleCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & cell : cells_)
    {
      if (!cell.is_ground)
      {
        cloud += *(cell.points);
      }
      cloud += *(cell.obstacles);
    }
    return true;
  }

  /** @brief Get a colorized representation of the point cloud. */
  bool getColorCloud(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
  {
    cloud.points.clear();

    // Setup colorization
    uint8_t r = 255;
    uint8_t g = 0;
    uint8_t b = 0;

    for (auto & cell : cells_)
    {
      for (auto & point : *(cell.points))
      {
        cloud.points.emplace_back(point.x, point.y, point.z, r, g, b);
      }
      for (auto & point : *(cell.obstacles))
      {
        cloud.points.emplace_back(point.x, point.y, point.z, r, g, b);
      }
      // Shift colors
      std::swap(b, g);
      std::swap(r, g);
    }

    return true;
  }

private:
  std::vector<GridCell<T>> cells_;
  double cell_size_;
  size_t x_cells_, y_cells_;
};

#endif  // ROBOMAGELLAN__GROUND_GRID_MODEL_HPP_
