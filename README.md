# RoboMagellan

This is a work in progress.

## Setup (22.04)

 * Setup Livox Time Synchronization. The Livox supports
   [IEEE1588](https://github.com/Livox-SDK/Livox-SDK/wiki/livox-device-time-synchronization-manual)
   ```
   sudo apt install linuxptp (this automatically works for eth0)
   ```
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
