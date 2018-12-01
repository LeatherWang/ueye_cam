
**Lastest update date: 2018/12/1**

I have removed the software extrigger for the camera, and add the hareware time sync with a xsens IMU or a odometry.

The IMU provides one GPIO to output the sync singal (named `SyncOut`) with the clock 400Hz, so I use it to trigger the camera, for details, you can see:[LeatherWang/ethzasl_xsens_driver](https://github.com/LeatherWang/ethzasl_xsens_driver)

As for the sync with the wheel odometry, I use extra a single chip to resample the odometry data and count, if have received 5 frame data, send one trigger singal to the camera, for the details, you can see: [LeatherWang/slam_car](https://github.com/LeatherWang/slam_car)


---
Thanks to Chang Liu's work, I add software external trigger mode, which can save some external trigger circuit, but ROS1.0 is seriously lacking in real-time, so it is better to use ROS2.0 or hardware external trigger mode.

---

**DISCLAMER:**

This project was created within an academic research setting, and thus should
be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
code, so please adjust expectations accordingly. With that said, we are
intrinsically motivated to ensure its correctness (and often its performance).
Please use the corresponding web repository tool (e.g. github, bitbucket, etc)
to file bugs, suggestions, pull requests; we will do our best to address them
in a timely manner.

**ADDITIONAL NOTE:**
This repo is the developed based on the merge of https://github.com/anqixu/ueye_cam and https://github.com/ProjectArtemis/ueye_cam.
It is aimming to enable updated version of the Ueye camera driver, with the capability of hardware syncronisation with px4 through mavros (default enabled). It also has adaptive exposure control to opimise the exposure setting for slam in realtime while avoid over exposure to clash with the sync signal. It remains the capabilities of auto standby mode with external trigger.

px4 setup:

    1.GPIO trigger mode

    2.TRIG_INTERVAL: 33.33 ms

    3.TRIG_POLARITY: 0 (active low)

    4.TRIG_ACT_TIME: 0.5 ms. The manual specifies it only has to be a minimum of 1 microsecond.

    5.TRIG_MODE: 1, because we want our camera driver to be ready to receive images before starting to trigger. This is essential to properly process sequence numbers.


    6.TRIG_PINS: 56, Leave default.



**LAYOUT:**
- ueye_cam/
  - cfg/:                 dynamic_reconfigure configuration files
  - include/:             header files
  - launch/:              roslaunch files
  - src/:                 source files
  - CMakeLists.txt:       CMake project configuration file
  - LICENSES:             license agreement
  - package.xml:          ROS/Catkin package file
  - nodelet_plugins.xml:  ROS nodelet specification file
  - README.md:            this file
- ~/.ros/camera_info/:    camera calibration yaml files
                          (see documentation for camera_calibration ROS package
                          for more details)
- ~/.ros/camera_conf/:    UEye camera parameter configuration files
                          (generatable using ueyedemo executable:
                          File -> save parameter -> to file...)


**DOCUMENTATION:**

www.ros.org/wiki/ueye_cam



Copyright (c) 2013-2016, Chang Liu, Kabir Mohammed, Anqi Xu and contributors

All rights reserved.

BSD3 license: see LICENSE file
