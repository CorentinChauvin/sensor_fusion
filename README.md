KTH Formula Student task - Localization
=======================================

Set up of **robot_localization** for a given Matlab dataset of speedometer, IMU and GNSS.


Install
-------
To install robot_localization on ROS Kinetic:
```
sudo apt install ros-kinetic-robot-localization
```

Run
---
```
roslaunch sensor_fusion main.launch
```

The main topics to look at are:
- **/odom**: data of the speedometer
- **/imu**: data of the IMU
- **/gnss**: data of the GNSS
- **/odom_filtered**: output of the first Kalman filter
- **/odom_final**: final output of the second Kalman filter
