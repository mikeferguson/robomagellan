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
   * https://github.com/mikeferguson/robot_controllers (branch: odom_covariance)
   * https://github.com/mikeferguson/um7
   * https://github.com/mikeferguson/serial
   * https://github.com/ros-perception/imu_pipeline
 * The livox is not yet in the URDF (intentionally, since I lack a calibration routine). This
   is a halfway decent stand in:
   ```
   ros2 run tf2_ros static_transform_publisher --x 0.1 --z 0.3 --pitch 0.44 --frame-id base_link --child-frame-id livox_frame
   ```

## Launch Files

 * robot.launch.py - everything needed to run the robot. Can be run in offline
   mode to run just the pipelines, with use_sim_time set. This is useful for
   running against bagfiles:
   ```
   ros2 launch robomagellan robot.launch.py offline:=true
   ```
 * drivers.launch.py - this is just the hardware drivers - useful for collecting
   bagfiles. Normally is included as part of robot.launch.py when running the
   RoboMagellan application

## Event Configuration

 * Edit line 34 of fixed_lag_global.yaml to set the initial heading of the robot
 * TODO: setup breadcrumbs and goal poses

## Recording Bagfiles

Raw data bagfile for analysis and development:
```
ros2 bag record /base_controller/odom /gps/nmea_sentence /imu_um7/data /imu_um7/mag /imu_um7/rpy /joint_states /livox/imu /livox/lidar /robot_description /tf /tf_static
```

## rviz_satellite

This fork works on Iron: https://github.com/nobleo/rviz_satellite/tree/ros2

To test the plugin:

```
ros2 topic pub -r 10 /gps/fix sensor_msgs/msg/NavSatFix "{header: {frame_id: "map"}, status: {service: 1}, latitude: VALUE, longitude: VALUE}"
```

## Hardware Details (gen2)

 * Computer: Intel NUC, 19V @ 60W.
 * Switch: TP-Link Litewave 5.  5V @ 3.7W.
 * Lidar: Livox MID-360, 12V (9-27V) @ 6.5W. Comms over Ethernet.
 * Camera: TBD.
 * GPS: Sparkfun GPS-RTK-SMA w/ UBlox ZED-F9P. Comms over USB.
 * IMU: UM7. Comms over USB.
 * Custom integrated power and control board. Comms over Ethernet.
 * Wifi Access Point: GL-AR300M16.
 * Motor: Hobbywing 30404310 - 13.5T Sensored. KV: 2850. 0.0289 Ohm. 2-3S, 2 Pole. Max Power: 180W @ 50A
