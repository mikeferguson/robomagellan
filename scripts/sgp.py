#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2024 Michael Ferguson
# Copyright (c) 2022 Mahmoud Ali
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Dependencies to install:
#  python3 -m pip install tensorflow gpflow

import time
import numpy as np
from math import atan2, cos, sin

# Tensorflow and friends
import tensorflow as tf
import gpflow

# ROS2
import cv2
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import ros2_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# TODO: sort out issues with using float32 in tensorflow
# TODO: remove high variance (non obstacle) points from output point clouds
# TODO: subscribe to actual goal, setup TF transformer for goal

""" @brief: class to construct a 2D Sparse GP using gpflow. """
class SGP2D:

    """ @brief Create model, initialize RQ kernel parameters. """
    def __init__(self, ls1, ls2, var, alpha, noise, noise_var):
        self.kernel1 = gpflow.kernels.RationalQuadratic(lengthscales=[ls1, ls2])
        self.kernel1.variance.assign(var)
        self.kernel1.alpha.assign(alpha)
        self.kernel2 = gpflow.kernels.White(noise)
        self.kernel2.variance.assign(noise_var)
        self.kernel = self.kernel1 + self.kernel2
        self.meanf = gpflow.mean_functions.Constant(0)

    """ @brief Initialize training data as tensors. """
    def set_training_data(self, d_in, d_out):
        mdl_in = tf.Variable(d_in, dtype=tf.float64)
        mdl_out = tf.Variable(d_out, dtype=tf.float64)
        self.data = (mdl_in, mdl_out)

    """ @brief Initialize inducing points from the training data. """
    def set_indpts_from_training_data(self, indpts_size, in_data):
        data_size = np.shape(in_data)[0]
        pts_idx = range(0, data_size, int(data_size / indpts_size))
        self.indpts = in_data[[idx for idx in pts_idx], :]

    """ @brief Initialize the SGP model using gpflow lib. """
    def set_sgp_model(self):
        self.model = gpflow.models.SGPR(self.data,
                                        self.kernel,
                                        self.indpts,
                                        mean_function=self.meanf)

    """ @brief Select params set to trainable/fixed. """
    def select_trainable_param(self):
        gpflow.set_trainable(self.kernel1.variance, False)
        gpflow.set_trainable(self.kernel1.lengthscales, False)
        gpflow.set_trainable(self.kernel2.variance, False)
        gpflow.set_trainable(self.model.likelihood.variance, False)

    """ @brief Minimize the loss during training. """
    def minimize_loss(self):
        self.model.training_loss_closure() 

    """ @brief Setup optimizer: here we are using adam. """ 
    def adam_optimize_param(self):
        optimizer = tf.optimizers.Adam()
        optimizer.minimize(self.model.training_loss,
                           self.model.trainable_variables)


class SparseGaussianProcessGoalFinder(Node):

    def __init__(self):
        super().__init__('sgp_goal_finder')

        # Downsampling will divide number of points by this ratio
        self.declare_parameter('downsampling_ratio', 3)
        self.pc_downsample = self.get_parameter('downsampling_ratio').value

        # Parameters for the sparse gaussian process
        self.declare_parameter('ls1', 0.09)
        self.declare_parameter('ls2', 0.11)
        self.declare_parameter('var', 0.7)
        self.declare_parameter('alpha', 10)
        self.declare_parameter('noise', 10)
        self.declare_parameter('noise_var', 0.005)
        self.declare_parameter('num_inducing_pts', 400)
        self.num_inducing_pts = self.get_parameter('num_inducing_pts').value

        # Sampling surface characteristics
        self.declare_parameter('surface_radius', 5)
        self.surface_radius = self.get_parameter('surface_radius').value
        # Theta angles will be sampled from min to max, at this resolution
        self.declare_parameter('surface_theta_resolution', 0.05)
        self.declare_parameter('surface_theta_min', -np.pi * (2 / 3))
        self.declare_parameter('surface_theta_max', np.pi * (2 / 3))
        # Alpha angles will be sampled from min to max, at this resolution
        self.declare_parameter('surface_alpha_min', -0.1)
        self.declare_parameter('surface_alpha_max', 0.35)
        self.declare_parameter('surface_alpha_resolution', 0.025)
        self.setup_sample_grid()

        # Scoring of goals
        self.declare_parameter('k_dist', 5)
        self.declare_parameter('k_heading', 4)
        self.k_dist = self.get_parameter('k_dist').value
        self.k_heading = self.get_parameter('k_heading').value

        self.declare_parameter('visualization_radius', 5.0)
        self.viz_radius = self.get_parameter('visualization_radius').value

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.surface_pub = self.create_publisher(PointCloud2, 'surface_viz', 1)
        self.surface_mean_pub = self.create_publisher(PointCloud2, 'surface_mean', 1)
        self.surface_var_pub = self.create_publisher(PointCloud2, 'surface_var', 1)
        self.cloud_sub = self.create_subscription(PointCloud2, 'spherical_cloud',
                                                  self.cloud_cb, 1)

    def cloud_cb(self, msg):
        self.header = msg.header
        self.fields = msg.fields

        pc = ros2_numpy.point_cloud2.pointcloud2_to_array(msg, squeeze=True)
        pc = np.round(np.array(pc.tolist(), dtype='float'), 4)
        print("Initial shape: ", pc.shape)

        start = time.time()
        # Downsample point cloud and assign thetas, alphas, ocs, and rds variables
        self.downsample(pc)
        print("Downsample time: ", time.time() - start)

        # Train on input and output data
        self.sgp_din = np.column_stack((self.pc_thetas, self.pc_alphas))
        self.sgp_dout = np.array(self.pc_oc, dtype='float').reshape(-1, 1)
        self.sgp_fit(self.get_parameter('ls1').value,
                     self.get_parameter('ls2').value,
                     self.get_parameter('var').value,
                     self.get_parameter('alpha').value,
                     self.get_parameter('noise').value,
                     self.get_parameter('noise_var').value)
        print("Fit time: ", time.time() - start)

        # Sample along a fixed grid pattern
        grid_mean, grid_var = self.sgp.model.predict_f(self.sample_grid)
        self.grid_var = grid_var.numpy()
        self.grid_mean = grid_mean.numpy()
        self.grid_rds = self.surface_radius - self.grid_mean
        print("Sample time: ", time.time() - start)

        # TODO: fill in with actual goal from a ROS topic/action
        #       transform the goals into the local frame at each step
        self.goal_x = 8.0
        self.goal_y = 0.0

        # Compute frontiers
        self.update_variance_threshold()
        frontiers = self.compute_frontiers()
        best_frontier = self.score_frontiers(frontiers)

        # Publish best frontier
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = best_frontier[0]
        ps.pose.position.y = best_frontier[1]
        ps.pose.orientation.z = sin(best_frontier[2] / 2.0)
        ps.pose.orientation.w = cos(best_frontier[2] / 2.0)
        self.goal_pub.publish(ps)

        # (Lazy) publish various debugging topics
        self.publish_surface_viz()
        self.publish_surface_mean_viz()
        self.publish_surface_var_viz()

    def downsample(self, pc):
        # Sort the cloud by increasing thetas
        pc = pc[np.argsort(pc[:, 0])]

        # Pull out just the theta values 
        thetas = pc.transpose()[:][0].reshape(-1, 1)
        unique_thetas = np.array(sorted(set(thetas.flatten())))

        # Take the first measurement at each theta
        keep_theta_ids = [t for t in range(0, np.shape(unique_thetas)[0], self.pc_downsample)]
        ids = []
        for t in keep_theta_ids:
            ids = ids + list(np.where(thetas == unique_thetas[t])[0])
        pc = pc[ids]
        pc = pc.transpose()
        self.pc_thetas = np.round(pc[:][0].reshape(-1, 1), 4)
        self.pc_alphas = np.round(pc[:][1].reshape(-1, 1), 4)
        self.pc_rds = np.round(pc[:][2].reshape(-1, 1), 4)
        self.pc_oc = np.round(pc[:][3].reshape(-1, 1), 4)
        self.pc_sz = np.shape(self.pc_thetas)[0]

    """ @brief Initiate and train 2D SGP using the SGP2D class. """
    def sgp_fit(self, ls1, ls2, var, alpha, noise, noise_var):
        self.sgp = SGP2D(ls1, ls2, var, alpha, noise, noise_var)
        self.sgp.set_training_data(self.sgp_din, self.sgp_dout)
        self.sgp.set_indpts_from_training_data(self.num_inducing_pts, self.sgp_din)
        self.sgp.set_sgp_model()
        self.sgp.select_trainable_param()
        self.sgp.minimize_loss()

    def update_variance_threshold(self):
        # Compute variance distribution statistics
        sgp_var_mean = np.mean(self.grid_var)
        sgp_var_var = np.var(self.grid_var)
        self.var_thresh = 0.1 * (sgp_var_mean - 3 * sgp_var_var)
        print("Variance distribution mean and variance: ", sgp_var_mean, sgp_var_var)
        print("Variance threshold: ", self.var_thresh)

    def compute_frontiers(self):
        # Normalize variance
        normalized_var = self.grid_var / np.linalg.norm(self.grid_var)

        # Create an image
        img = np.zeros((self.sample_grid_h, self.sample_grid_w), np.uint8)
        bw_var = (255 / normalized_var[normalized_var.argmax(axis=0)]) * normalized_var
        img[0:self.sample_grid_h, 0:self.sample_grid_w] = \
            bw_var.reshape(self.sample_grid_w, self.sample_grid_h).T

        # Grey to binary
        img_threshold = 0.5 * int(np.mean(bw_var) + self.var_thresh * np.var(bw_var))
        _, bw_img = cv2.threshold(img, img_threshold, 255, cv2.THRESH_BINARY)

        # Resize image
        scale_factor = 10
        width = int(bw_img.shape[1] * scale_factor)
        height = int(bw_img.shape[0] * scale_factor)
        scaled_img = cv2.resize(bw_img, (width, height))
        contours, hierarchy = cv2.findContours(scaled_img.copy(),
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        # Frontier search works relatively well in confined spaces but
        # leads to erratic behavior when in more wide open spaces.
        # Add an additional "frontier" that is a beeline to the goal,
        # if said frontier falls within an existing frontier
        goal_theta = atan2(self.goal_y, self.goal_x)
        goal_alpha = np.pi / 2.0
        # Convert alpha/theta into cx/cy
        goal_cx = int(((self.grid_thetas - goal_theta)**2).argmin(axis=0))
        goal_cy = int(((self.grid_alphas - goal_alpha)**2).argmin(axis=0))

        frontier_centers = []
        frontier_areas = []
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Convert centers into possible goals
                cx = int(cx / scale_factor)
                cy = int(cy / scale_factor)
                frontier_centers.append([self.grid_thetas[cx], self.grid_alphas[cy]])
                frontier_areas.append(cv2.contourArea(i))
                if cv2.pointPolygonTest(i, (goal_cx, goal_cy), False):
                    frontier_centers.append([goal_theta, goal_alpha])

        return np.array(frontier_centers).reshape(-1, 2)

    def score_frontiers(self, frontiers):
        num_frontiers = np.shape(frontiers)[0]
        gx = self.goal_x
        gy = self.goal_y

        # Convert frontiers from theta/alpha to x/y
        rds = np.ones(num_frontiers, dtype='float32').reshape(-1, 1)
        rds *= self.surface_radius
        x, y, z = self.convert_spherical_2_cartesian(
                    frontiers.T[0].reshape(-1, 1),
                    frontiers.T[1].reshape(-1, 1),
                    rds)
        xy = np.hstack((x, y))

        # Determine cost based on distance to goal + change in heading
        cost = self.k_dist * np.sqrt((gx - xy.T[0])**2 + (gy - xy.T[1])**2) + \
               self.k_heading * (frontiers.T[0]**2)
        # Find lowest cost goal
        index = cost.argmin(axis=0)

        # Return the goal in local x,y,yaw
        return [float(x) for x in [xy[index][0], xy[index][1], frontiers[index][0]]]

    """ @brief Configure grid used to reconstruct sampling surface. """
    def setup_sample_grid(self):
        theta_res = self.get_parameter('surface_theta_resolution').value
        theta_min = self.get_parameter('surface_theta_min').value
        theta_max = self.get_parameter('surface_theta_max').value
        self.grid_thetas = np.arange(theta_min, theta_max, theta_res, dtype='float64')

        alpha_res = self.get_parameter('surface_alpha_resolution').value
        alpha_min = self.get_parameter('surface_alpha_min').value
        alpha_max = self.get_parameter('surface_alpha_max').value
        self.grid_alphas = np.arange(np.pi / 2 - alpha_max, np.pi / 2 - alpha_min, alpha_res,
                                     dtype='float64')

        self.sample_grid = np.array(np.meshgrid(self.grid_thetas, self.grid_alphas)).T.reshape(-1, 2)
        self.sample_grid_w = np.shape(self.grid_thetas)[0]
        self.sample_grid_h = np.shape(self.grid_alphas)[0]

    """ @brief Used for publishing results/visualization. """
    def convert_spherical_2_cartesian(self, theta, alpha, dist):
        x = np.array(dist * np.sin(alpha) * np.cos(theta),
                     dtype='float32').reshape(-1, 1)
        y = np.array(dist * np.sin(alpha) * np.sin(theta),
                     dtype='float32').reshape(-1, 1)
        z = np.array(dist * np.cos(alpha), dtype='float32').reshape(-1, 1)
        return x, y, z

    """ @brief Publish occupancy surface mean for visualization. """
    def publish_surface_mean_viz(self):
        if self.surface_mean_pub.get_subscription_count() > 0:
            rds = self.viz_radius * np.ones(np.shape(self.grid_rds)[0],
                                            dtype='float32').reshape(-1, 1)
            x, y, z = self.convert_spherical_2_cartesian(
                        self.sample_grid.T[:][0].reshape(-1, 1),
                        self.sample_grid.T[:][1].reshape(-1, 1),
                        rds)
            intensity = np.array(self.grid_rds,
                                 dtype='float32').reshape(-1, 1)
            surface_pc = np.column_stack((x, y, z, intensity))
            surface_pc2 = point_cloud2.create_cloud(self.header, self.fields, surface_pc)
            self.surface_mean_pub.publish(surface_pc2)

    """ @brief Publish occupancy surface variance for visualization. """
    def publish_surface_var_viz(self):
        if self.surface_var_pub.get_subscription_count() > 0:
            rds = self.viz_radius * np.ones(np.shape(self.grid_rds)[0],
                                            dtype='float32').reshape(-1, 1)
            x, y, z = self.convert_spherical_2_cartesian(
                        self.sample_grid.T[:][0].reshape(-1, 1),
                        self.sample_grid.T[:][1].reshape(-1, 1),
                        rds)
            intensity = np.array(self.grid_var,
                                 dtype='float32').reshape(-1, 1)
            surface_pc = np.column_stack((x, y, z, intensity))
            surface_pc2 = point_cloud2.create_cloud(self.header, self.fields, surface_pc)
            self.surface_var_pub.publish(surface_pc2)

    """ @brief Publish reconstruction of the 3d surface. """
    def publish_surface_viz(self):
        if self.surface_pub.get_subscription_count() > 0:
            x, y, z = self.convert_spherical_2_cartesian(
                        self.sample_grid.T[:][0].reshape(-1, 1),
                        self.sample_grid.T[:][1].reshape(-1, 1),
                        self.grid_rds)
            intensity = np.array(self.grid_rds,
                                 dtype='float32').reshape(-1, 1)
            surface_pc = np.column_stack((x, y, z, intensity))
            surface_pc2 = point_cloud2.create_cloud(self.header, self.fields, surface_pc)
            self.surface_pub.publish(surface_pc2)


if __name__ == '__main__':
    rclpy.init()
    try:
        node = SparseGaussianProcessGoalFinder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
