# RoboMagellan

This is a work in progress.

## Setup (ROS 2 Iron, 22.04)

 * Setup Livox Time Synchronization. The Livox supports
   [IEEE1588](https://github.com/Livox-SDK/Livox-SDK/wiki/livox-device-time-synchronization-manual)
   ```
   sudo apt install linuxptp (this automatically works for eth0)
   ```
 * Pair the PS4 controller following [these instructions](https://www.robotandchisel.com/2020/04/05/ps4-controller-and-ros/)
 * Build things in workspace:
   * https://github.com/mikeferguson/livox_ros_driver2
   * https://github.com/mikeferguson/robomagellan
   * https://github.com/mikeferguson/robot_controllers
   * https://github.com/mikeferguson/um7
   * https://github.com/mikeferguson/serial
 * The livox is not yet in the URDF (intentionally, since I lack a calibration routine). This
   is a halfway decent stand in:
   ```
   ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.3 --pitch 0.44 --frame-id base_link --child-frame-id livox_frame
   ```

## Launch Files

 * robot.launch - everything needed to run the robot. Setting the offline_mode
   argument to true will only run the processing pipelines. This is useful for
   running against bagfiles.
 * raw_imu_odom_gps.launch - starts a minimal set of drivers and a ROS bag.
   These bags can be used with the offline_mode.

## Recording Bagfiles

Raw data bagfile for analysis and development:
```
ros2 bag record /base_controller/odom /gps/nmea_sentence /imu_um7/data /imu_um7/mag /imu_um7/rpy /joint_states /livox/imu /livox/lidar /robot_description /tf /tf_static
```

## Hardware Details

 * Switch: TP-Link Litewave 5. Runs off 5V, max 600mA.
