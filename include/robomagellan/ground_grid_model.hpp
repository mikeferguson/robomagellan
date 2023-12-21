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
#include <queue>
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

  // Maximum distance from vertical plane can be and still be ground
  // (default is 15 degrees)
  double vertical_tolerance = 0.2617;
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
  }

  bool compute()
  {
    if (points->size() < params->min_points)
    {
      // Cannot compute features on so few points
      features_valid = false;
      return false;
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

    if (inliers->indices.empty())
    {
      // Not able to extract plane
      features_valid = false;
      return false;
    }

    // Store the parameters for the plane
    normal(0) = coef->values[0];
    normal(1) = coef->values[1];
    normal(2) = coef->values[2];
    d = coef->values[3];

    // Extract outliers into obstacles cloud
    pcl::ExtractIndices<T> extract;
    extract.setInputCloud(points);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    // And then remove outliers from the points cloud
    extract.setNegative(false);
    extract.filter(*points);

    // Compute mean and covariance of remaining points
    pcl::computeMeanAndCovarianceMatrix(*points, covariance, mean);

    // Mark features (mean, cov, and normal) as valid
    features_valid = true;
    return true;
  }

  bool computeViaAdjacent(GridCell<T> & adj)
  {
    // Make sure adjacent cell is valid
    if (!adj.features_valid) return false;
    if (!adj.is_ground) return false;

    // Split cloud into ground/nonground based on planar fit
    pcl::PointCloud<T> ground, nonground;
    for (auto point : *points)
    {
      double v = point.x * adj.normal(0) + point.y * adj.normal(1) + point.z * adj.normal(2) + adj.d;
      if (fabs(v) < params->planar_tolerance)
      {
        ground.push_back(point);
      }
      else
      {
        nonground.push_back(point);
      }
    }

    if (ground.size() < nonground.size())
    {
      // Reject this filtering
      return false;
    }

    // Accept filtering
    *points = ground;
    *obstacles = nonground;
    // Copy the plane parameters from adjacent
    normal = adj.normal;
    d = adj.d;
    // Compute mean and covariance of remaining points
    pcl::computeMeanAndCovarianceMatrix(*points, covariance, mean);
    // Mark features (mean, cov, and normal) as valid
    features_valid = true;
    return true;
  }

  // Indices of this cell
  size_t x, y;

  // The points in the cell that may be part of ground plane
  std::shared_ptr<pcl::PointCloud<T>> points;
  bool is_ground;
  bool features_valid;

  // The points in this cell that are obstacles
  std::shared_ptr<pcl::PointCloud<T>> obstacles;

  // Features of the point cloud
  Eigen::Vector4f mean;
  Eigen::Matrix3f covariance;
  Eigen::Vector3f normal;
  double d;

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
 *     c) Each adjacent cell of the seed is added to the candidate queue.
 *     d) Mark a candidate as obstacle if any of the following:
 *        i)   There is a significant increase in height compared to adjacent
 *             cells that are marked as ground.
 *        ii)  There is a signficant change in the normal compared to adjacent
 *             cells that are marked as ground.
 *        iii) The covariance indicates that the ground is "rough".
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
   * @param grid_size Maximum distance cells span in x & y (meters).
   */
  GridModel(double cell_size, double grid_size)
  {
    // Store sizes
    cell_size_ = cell_size;
    x_cells_ = grid_size / cell_size_;
    y_cells_ = grid_size / cell_size_;

    std::cout << "Grid model has " << x_cells_ << "x" << y_cells_ << " cells of " << cell_size_ << "m" << std::endl;

    // Allocate storage
    this->cells_.resize(x_cells_ * y_cells_);
    this->processed_.resize(x_cells_ * y_cells_);

    // Set indices of cells
    for (size_t x = 0; x < x_cells_; ++x)
    {
      for (size_t y = 0; y < y_cells_; ++y)
      {
        this->cells_[getIndex(x, y)].x = x;
        this->cells_[getIndex(x, y)].y = y;
      }
    }
  }

  /** @brief Clear all cells. */
  void clear()
  {
    // Clear all points from all cells
    for (auto & cell : cells_)
    {
      cell.clear();
    }
    // Reset all processed flags
    std::fill(processed_.begin(), processed_.end(), false);
  }

  /** @brief Set the cell parameters. */
  void setParams(std::shared_ptr<GridParams> params, double scaling)
  {
    std::cout << "Setting grid params to:" << std::endl;
    std::cout << "  Min Points:         " << params->min_points << std::endl;
    std::cout << "  Planar Tolerance:   " << params->planar_tolerance << std::endl;
    std::cout << "  Vertical Tolerance: " << params->vertical_tolerance << std::endl;
    for (auto & cell : cells_)
    {
      double d = std::hypot((static_cast<int>(cell.x) - static_cast<int>(x_cells_ / 2)) * cell_size_,
                            (static_cast<int>(cell.y) - static_cast<int>(y_cells_ / 2)) * cell_size_);
      d = std::max(1.0, d);
      cell.params->min_points = params->min_points;
      cell.params->planar_tolerance = params->planar_tolerance * (1.0 + d * scaling);
      cell.params->vertical_tolerance = params->vertical_tolerance;
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

      int x = (point.x / cell_size_) + (x_cells_ / 2);
      int y = (point.y / cell_size_) + (y_cells_ / 2);

      if (x >= 0 && x < x_cells_ && y >= 0 && y < y_cells_)
      {
        // Index is valid
        cells_[getIndex(x, y)].points->push_back(point);
      }
    }

    // Compute features in each cell
    for (auto & cell : cells_)
    {
      cell.compute();
    }

    // Queue of cells to be processed - entry is index
    std::queue<size_t> cell_queue;

    // Find the cell most in front of the robot
    // Start at center of robot/grid and work forward
    size_t x_seed = x_cells_ / 2;
    size_t y_seed = y_cells_ / 2;
    while (true)
    {
      auto & cell = cells_[getIndex(x_seed, y_seed)];
      if (cell.features_valid)
      {
        // TODO: Make sure points are close enough to the ground (need to do calibration first)
        // Make sure we are vertical enough
        if (isHorizontal(cell))
        {
          // Mark as ground
          cell.is_ground = true;
          // Mark processed
          processed_[getIndex(x_seed, y_seed)] = true;
          // Insert adjacent cells
          for (auto adj : getAdjacent(cell))
          {
            cell_queue.push(adj);
          }
          break;
        }
      }

      // This is not a valid seed, move forward
      if (++x_seed >= x_cells_)
      {
        if (++y_seed >= y_cells_)
        {
          std::cerr << "No valid seed found!" << std::endl;
          return;
        }
      }
    }

    size_t processed_count = 1;

    // Do classification of ground/obstacles
    while (!cell_queue.empty())
    {
      // Get cell index - mark cell as processed
      size_t i = cell_queue.front(); cell_queue.pop();
      if (processed_[i]) continue;
      processed_[i] = true;
      ++processed_count;

      // Get the cell
      auto & cell = cells_[i];

      // Determine adjacent cells
      std::vector<size_t> adjacent = getAdjacent(cell);

      // Classification for seed depends on no adjacent cells
      if (cell.features_valid)
      {
        // Various conditions to check
        bool is_horizontal = false;
        // TODO: compute is_step
        // TODO: compute is_rough

        // Determine if plane is vertical enough
        is_horizontal = isHorizontal(cell);
        if (!is_horizontal)
        {
          // Compare to neighbors
          for (auto idx : adjacent)
          {
            auto & adj = cells_[idx];
            if (isHorizontalToAdjacent(cell, adj))
            {
              is_horizontal = true;
              break;
            }
          }
        }

        // Assign is_ground
        if (is_horizontal)
        {
          cell.is_ground = true;
        }
      }
      else if (cell.points->empty())
      {
        // TODO: add drop detection for empty cells near robot
      }
      else
      {
        // Attempt to connect these points to an adjacent plane
        for (auto idx : adjacent)
        {
          auto & adj = cells_[idx];
          if (cell.computeViaAdjacent(adj))
          {
            cell.is_ground = true;
            break;
          }
        }
      }

      // Add unprocessed, adjacent cells to the queue
      if (cell.is_ground)
      {
        for (auto idx : adjacent)
        {
          if (!processed_[idx]) cell_queue.push(idx);
        }
      }
    }

    std::cout << "  processed " << processed_count << " cells"  << std::endl;

    // Mark all remaining unprocessed as invalid
    // TODO: is this actually safe to do?
    for (size_t i = 0; i < cells_.size(); ++i)
    {
      if (!processed_[i]) cells_[i].features_valid = false;
    }
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
      if ((!cell.is_ground) && cell.features_valid)
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
  size_t getIndex(size_t x, size_t y)
  {
    return (x * x_cells_) + y;
  }

  std::vector<size_t> getAdjacent(GridCell<T> & cell)
  {
    std::vector<size_t> adj;
    if (cell.x > 0)
    {
      adj.push_back(getIndex(cell.x - 1, cell.y));
    }
    if (cell.y > 0)
    {
      adj.push_back(getIndex(cell.x, cell.y - 1));
    }
    if (cell.x < x_cells_ - 1)
    {
      adj.push_back(getIndex(cell.x + 1, cell.y));
    }
    if (cell.y < y_cells_ - 1)
    {
      adj.push_back(getIndex(cell.x, cell.y + 1));
    }
    return adj;
  }

  // @brief Determine if points in cell form a horizontal plane
  bool isHorizontal(GridCell<T> & cell)
  {
    float angle = acos(Eigen::Vector3f::UnitZ().dot(cell.normal));
    return (angle < cell.params->vertical_tolerance);
  }

  // @brief Determine if points in "cell" are horizontal enough to "adj"
  bool isHorizontalToAdjacent(GridCell<T> & cell, GridCell<T> & adj)
  {
    if (!adj.features_valid) return false;
    if (!adj.is_ground) return false;
    float angle = acos(adj.normal.dot(cell.normal));
    return (angle < cell.params->vertical_tolerance);
  }

  std::vector<GridCell<T>> cells_;
  std::vector<bool> processed_;
  double cell_size_;
  size_t x_cells_, y_cells_;
};

#endif  // ROBOMAGELLAN__GROUND_GRID_MODEL_HPP_
