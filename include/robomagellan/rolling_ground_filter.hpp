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

#ifndef ROBOMAGELLAN__ROLLING_GROUND_FILTER_HPP_
#define ROBOMAGELLAN__ROLLING_GROUND_FILTER_HPP_

#include <cmath>
#include <memory>
#include <queue>
#include <vector>
#include <utility>
#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct RollingGridParams
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
struct RollingGridCell
{
  RollingGridCell()
  {
    points = std::make_shared<pcl::PointCloud<T>>();
    obstacles = std::make_shared<pcl::PointCloud<T>>();
    params = std::make_shared<RollingGridParams>();
    cycles = 0;
    reset();
  }

  void operator=(const RollingGridCell& other)
  {
    if (this != &other)
    {
      this->points = other.points;
      this->obstacles = other.obstacles;
      this->params = other.params;
      this->is_ground = other.is_ground;
      this->n = other.n;
      this->mean = other.mean;
      this->covariance = other.covariance;
      this->correlation = other.correlation;
      this->valid = other.valid;
      this->cycles = other.cycles;
    }
  }

  // @brief Called at the beginning of each update cycle
  void clear()
  {
    points->clear();
    obstacles->clear();
    is_ground = false;
  }

  // @brief Called when a cell wraps around
  void reset()
  {
    clear();
    n = 0;
    mean = Eigen::Vector3f::Zero();
    covariance = Eigen::Matrix3f::Zero();
    correlation = Eigen::Matrix3f::Zero();
    valid = false;
  }

  void addPoint(T point)
  {
    // Store point in cloud
    points->push_back(point);
    // Update mean and correlation
    Eigen::Vector3f p(point.x, point.y, point.z);
    mean = (mean * n + p) / (n + 1);
    for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        correlation(i, j) = (correlation(i, j) * n + p(i) * p(j)) / (n + 1);
      }
    }
    n += 1;
    valid = false;
  }

  bool compute()
  {
    ++cycles;
    if (valid || n < 3)
    {
      return false;
    }

    const double scale = n / (n - 1);
    for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = i; j < 3; ++j)
      {
        covariance(i, j) = (correlation(i, j) - (mean(i) * mean(j))) * scale;
        covariance(j, i) = covariance(i, j);
      }
    }

    valid = true;
    return true;
  }

  // TODO REMOVE
  int cycles;

  // The points in the cell that may be part of ground plane
  std::shared_ptr<pcl::PointCloud<T>> points;
  bool is_ground;

  // The points in this cell that are obstacles
  std::shared_ptr<pcl::PointCloud<T>> obstacles;

  // Are features valid
  bool valid;

  // Number of points that have fallen into this grid cell
  // Technically this should be size_t - but for Eigen math, needs to be double
  double n;

  // Features of the point cloud
  Eigen::Vector3f mean;
  Eigen::Matrix3f covariance;
  Eigen::Matrix3f correlation;

  // Parameters to be used for processing
  std::shared_ptr<RollingGridParams> params;
};

template <typename T>
class RollingGridModel
{
public:
  /**
   * @brief Create a new grid model.
   * @param cell_size Size of cells in x & y (meters).
   * @param grid_size Maximum distance cells span in x & y (meters).
   */
  RollingGridModel(double cell_size, double grid_size)
  {
    // Store sizes
    cell_size_ = cell_size;
    x_cells_ = grid_size / cell_size_;
    y_cells_ = grid_size / cell_size_;
    x_origin_ = 0.0;
    y_origin_ = 0.0;

    std::cout << "Rolling grid model has " << x_cells_ << "x" << y_cells_ << " cells of " << cell_size_ << "m" << std::endl;

    // Allocate storage
    this->cells_.resize(x_cells_ * y_cells_);
  }

  /** @brief Set the robot pose. */
  void setRobotPose(double x, double y, double /* yaw */)
  {
    // Get x/y origin in discrete cells
    int x0_disc = (x / cell_size_) - (x_cells_ / 2);
    int y0_disc = (y / cell_size_) - (y_cells_ / 2);

    // Get y/x in continuous space, rounded to cell_size_
    double x0_cont = x0_disc * cell_size_;
    double y0_cont = y0_disc * cell_size_; 

    // How far to shift cells
    int x_delta = (x0_cont - x_origin_) / cell_size_;
    int y_delta = (y0_cont - y_origin_) / cell_size_;

    if (x_delta != 0 || y_delta != 0)
    {
      // Actually shift cells
      std::vector<RollingGridCell<T>> scratch;
      scratch.resize(cells_.size());
      for (int i = 0; i < x_cells_; ++i)
      {
        for (int j = 0; j < y_cells_; ++j)
        {
          int xx = i + x_delta;
          int yy = j + y_delta;
          if (xx >= 0 && xx < x_cells_ && yy >= 0 && yy < y_cells_)
          {
            scratch[getIndex(i, j)] = cells_[getIndex(xx, yy)];
          }
        }
      }
      std::swap(scratch, cells_);
    }

    // Store x/y origin
    x_origin_ = x0_cont;
    y_origin_ = y0_cont;
  }

  /** @brief Clear all cells. */
  void clear()
  {
    for (auto & cell : cells_)
    {
      cell.clear();
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

      int x = (point.x - x_origin_)/ cell_size_;
      int y = (point.y - y_origin_)/ cell_size_;

      if (x >= 0 && x < x_cells_ && y >= 0 && y < y_cells_)
      {
        // Index is valid
        cells_[getIndex(x, y)].addPoint(point);
      }
    }

    // Compute features in each cell
    for (auto & cell : cells_)
    {
      cell.compute();
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
      if ((!cell.is_ground) && cell.points->size() > cell.params->min_points)
      {
        cloud += *(cell.points);
      }
      if (cell.obstacles->size() > cell.params->min_points)
      {
      cloud += *(cell.obstacles);
      }
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

  bool getMeanCloud(pcl::PointCloud<T> & cloud)
  {
    for (auto & cell : cells_)
    {
      if (cell.n > 0)
      {
        T point;
        point.x = cell.mean(0);
        point.y = cell.mean(1);
        point.z = cell.mean(2);
        point.intensity = static_cast<float>(cell.cycles) / 50.0;
        cloud.push_back(point);
      }
    }
    return true;
  }

private:
  size_t getIndex(size_t x, size_t y)
  {
    return (x * x_cells_) + y;
  }

  std::vector<size_t> getAdjacent(RollingGridCell<T> & cell)
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
  bool isHorizontal(RollingGridCell<T> & cell)
  {
    float angle = acos(Eigen::Vector3f::UnitZ().dot(cell.normal));
    return (angle < cell.params->vertical_tolerance);
  }

  bool isVertical(RollingGridCell<T> & cell)
  {
    float angle = acos(Eigen::Vector3f::UnitZ().dot(cell.normal));
    return (angle > (1.57 - cell.params->vertical_tolerance));
  }

  // @brief Determine if points in "cell" are horizontal enough to "adj"
  bool isHorizontalToAdjacent(RollingGridCell<T> & cell, RollingGridCell<T> & adj)
  {
    if (!adj.features_valid) return false;
    if (!adj.is_ground) return false;
    float angle = acos(adj.normal.dot(cell.normal));
    return (angle < cell.params->vertical_tolerance);
  }

  std::vector<RollingGridCell<T>> cells_;
  // Size of the grid
  double cell_size_;
  size_t x_cells_, y_cells_;
  // Lower corner pose in fixed frame
  double x_origin_, y_origin_;
};

template <typename T>
class RollingGroundFilter : public rclcpp::Node
{
public:
  RollingGroundFilter()
  : rclcpp::Node("ground_filter"),
    logger_(rclcpp::get_logger("ground_filter"))
  {

    // Parameters
    debug_topics_ = this->declare_parameter<bool>("debug_topics", false);

    double cell_size = this->declare_parameter<double>("cell_size", 0.25);
    double grid_size = this->declare_parameter<double>("grid_size", 16.0);
    model_ = std::make_unique<RollingGridModel<T>>(cell_size, grid_size);

    robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.2);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Publish the points that correspond to the ground plane
    auto qos = rclcpp::SystemDefaultsQoS();
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground", qos);

    // Publish the points that correspond to obstacles
    obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", qos);

    if (debug_topics_)
    {
      colored_cell_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cells", qos);
      mean_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mean", qos);
    }

    // Subscribe the incoming point cloud message
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                   "cloud", qos, std::bind(&RollingGroundFilter<T>::cloudCb,
                                           this, std::placeholders::_1));
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    pcl::PointCloud<T> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Transform cloud to fixed_frame_
    try
    {
      tf2_buffer_->canTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp,
                                tf2::durationFromSec(transform_timeout_));
      if (!pcl_ros::transformPointCloud(fixed_frame_, cloud, cloud, *tf2_buffer_))
      {
        // Error message gets printed inside transformPointCloud
        return;
      }
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                   msg->header.frame_id.c_str(), fixed_frame_.c_str());
      return;
    }

    // Lookup robot_frame_ to fixed_frame_ transform
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.stamp = msg->header.stamp;
    robot_pose.header.frame_id = robot_frame_;
    robot_pose.pose.orientation.w = 1.0;
    try
    {
      tf2_buffer_->transform(robot_pose, robot_pose, fixed_frame_,
                             tf2::durationFromSec(transform_timeout_));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                   robot_frame_.c_str(), fixed_frame_.c_str());
      return;
    }

    RCLCPP_INFO(logger_, "Start process");
    model_->setRobotPose(robot_pose.pose.position.x,
                         robot_pose.pose.position.y,
                         tf2::getYaw(robot_pose.pose.orientation));
    model_->clear();
    model_->addPoints(cloud);
    RCLCPP_INFO(logger_, "End process");

    pcl::PointCloud<T> ground_cloud, obstacle_cloud;
    model_->getGroundCloud(ground_cloud);
    model_->getObstacleCloud(obstacle_cloud);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(ground_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = fixed_frame_;
    ground_pub_->publish(cloud_msg);

    pcl::toROSMsg(obstacle_cloud, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = fixed_frame_;
    obstacle_pub_->publish(cloud_msg);

    if (debug_topics_)
    {
      pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
      model_->getColorCloud(color_cloud);
      sensor_msgs::msg::PointCloud2 color_msg;
      pcl::toROSMsg(color_cloud, color_msg);
      color_msg.header.stamp = msg->header.stamp;
      color_msg.header.frame_id = fixed_frame_;
      colored_cell_pub_->publish(color_msg);

      pcl::PointCloud<T> mean_cloud;
      model_->getMeanCloud(mean_cloud);
      pcl::toROSMsg(mean_cloud, cloud_msg);
      cloud_msg.header.stamp = msg->header.stamp;
      cloud_msg.header.frame_id = fixed_frame_;
      mean_pub_->publish(cloud_msg);
    }
  }

  // Organization
  std::unique_ptr<RollingGridModel<T>> model_;

  // Parameters
  bool debug_topics_;
  std::string fixed_frame_;
  std::string robot_frame_;
  double transform_timeout_;

  // ROS Interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cell_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mean_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

#endif  // ROBOMAGELLAN__ROLLING_GROUND_FILTER_HPP_
