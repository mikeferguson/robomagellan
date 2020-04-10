# RoboMagellan

This is a work in progress.

## Setup Notes (18.04)

I was unable to get the Realsense ROS debians to work reliably, so I ended up
building from source against the driver debians. I also had issues with some
of the newest releases of librealsense, so we are setting the version manually

    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
    sudo apt-get install librealsense2-dkms=1.3.13-0ubuntu1
    sudo apt-get install librealsense2-dev=2.32.1-0~realsense0.1913

Then we can build the actual ROS workspace:

    mkdir ~/melodic/src; cd ~/melodic/src
    git clone git@github.com:mikeferguson/etherbotix_python.git
    git clone git@github.com:mikeferguson/robomagellan.git
    git clone git@github.com:gareth-cross/rviz_satellite.git
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ~/melodic
    rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
    source /opt/ros/melodic/setup.bash
    catkin_make

## Launch Files

 * robot.launch - everything needed to run the robot. Setting the offline_mode
   argument to true will only run the processing pipelines. This is useful for
   running against bagfiles.
 * raw_imu_odom_gps.launch - starts a minimal set of drivers and a ROS bag.
   These bags can be used with the offline_mode.
