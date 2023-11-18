# RoboMagellan

This is a work in progress.

The first generation of this robot was actually laser cut at Willow Garage in 2012 and used a
4WD differential drive base. While this allowed easy carrying of a fairly large 12V sealed
lead acid battery and easy upgrades over time, the robot ground speed was too slow to
really be competitive. Numerous sensor packages evolved over time on this platform.

The second generation robot is based on a 1/10th scale RC stadium truck and uses a Livox MID-360
sensor for obstacle avoidance.

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

 * Computer: Intel NUC, 19V @ 60W.
 * Switch: TP-Link Litewave 5.  5V @ 3.7W.
 * Lidar: Livox MID-360, 12V (9-27V) @ 6.5W. Comms over Ethernet.
 * Camera: TBD.
 * GPS: TBD. Comms over USB.
 * IMU: UM7. Comms over USB.
 * Custom integrated power and control board. Comms over Ethernet.
 * Wifi Access Point: GL-AR300M16.
