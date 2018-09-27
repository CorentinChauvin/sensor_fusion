#!/usr/bin/env python

"""
    Node to publish the given data as ROS messages
"""

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
import rospy
import scipy.io as sio
from math import atan2

if __name__ == '__main__':
    rospy.init_node('data_publisher')

    # ROS parameters
    file_path = rospy.get_param('~file_path')
    minimal_speed = rospy.get_param('~minimal_speed')   # minimal speed for taking the heading into account
    cov_gyro = rospy.get_param('~cov_gyro')
    cov_acc = rospy.get_param('~cov_acc')
    cov_gnss = rospy.get_param('~cov_gnss')
    cov_odom = rospy.get_param('~cov_odom')

    # ROS publishers
    pub_imu = rospy.Publisher('/imu', Imu, queue_size=10)
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
    pub_gnss = rospy.Publisher('/gnss', Odometry, queue_size=10)

    # Load IMU data
    mat = sio.loadmat(file_path)
    base_time = rospy.get_rostime()

    imu_data = []
    for k in range(len(mat['in_data']['IMU'][0][0]['t'][0][0])):
        msg = Imu()

        t = mat['in_data']['IMU'][0][0]['t'][0][0][k][0]
        msg.header.stamp = base_time + rospy.Duration.from_sec(t)
        msg.header.frame_id = 'base_link'

        msg.linear_acceleration.x = mat['in_data']['IMU'][0][0]['acc'][0][0][0][k]
        msg.linear_acceleration.y = mat['in_data']['IMU'][0][0]['acc'][0][0][1][k]
        msg.linear_acceleration.z = mat['in_data']['IMU'][0][0]['acc'][0][0][2][k]
        msg.linear_acceleration_covariance[0] = cov_acc
        msg.linear_acceleration_covariance[4] = cov_acc
        msg.linear_acceleration_covariance[8] = cov_acc
        msg.angular_velocity.x = mat['in_data']['IMU'][0][0]['gyro'][0][0][0][k]
        msg.angular_velocity.y = mat['in_data']['IMU'][0][0]['gyro'][0][0][1][k]
        msg.angular_velocity.z = mat['in_data']['IMU'][0][0]['gyro'][0][0][2][k]
        msg.angular_velocity_covariance[0] = cov_gyro
        msg.angular_velocity_covariance[4] = cov_gyro
        msg.angular_velocity_covariance[8] = cov_gyro

        imu_data.append(msg)

    # Load GNSS data
    gnss_data = []
    for k in range(len(mat['in_data']['GNSS'][0][0]['t'][0][0])):
        msg = Odometry()

        t = mat['in_data']['GNSS'][0][0]['t'][0][0][k][0]
        msg.header.stamp = base_time + rospy.Duration.from_sec(t)
        msg.header.frame_id = 'map'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = mat['in_data']['GNSS'][0][0]['pos_ned'][0][0][0][k]  # north
        msg.pose.pose.position.y = mat['in_data']['GNSS'][0][0]['pos_ned'][0][0][1][k]  # east
        msg.pose.pose.position.z = -mat['in_data']['GNSS'][0][0]['pos_ned'][0][0][2][k] # down
        msg.pose.covariance[0] = cov_gnss
        msg.pose.covariance[7] = cov_gnss
        msg.pose.covariance[14] = cov_gnss

        # Heading
        if k > 0:
            dx = msg.pose.pose.position.x - gnss_data[k-1].pose.pose.position.x
            dy = msg.pose.pose.position.y - gnss_data[k-1].pose.pose.position.y
            heading = atan2(dy, dx)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, heading)
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]

            d = dx**2 + dy**2
            dt = msg.header.stamp.to_sec() - gnss_data[k-1].header.stamp.to_sec()

            if float(d)/dt > minimal_speed:
                msg.pose.covariance[35] = (cov_gnss*dx)**2 + (cov_gnss*dy)**2 / d
            else:
                msg.pose.covariance[35] = 10**9
        else:
            msg.pose.covariance[35] = 10**9

        gnss_data.append(msg)

    # Load odometry data
    odom_data = []
    for k in range(len(mat['in_data']['SPEEDOMETER'][0][0]['t'][0][0])):
        msg = Odometry()

        t = mat['in_data']['SPEEDOMETER'][0][0]['t'][0][0][k][0]
        msg.header.stamp = base_time + rospy.Duration.from_sec(t)
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.twist.twist.linear.x = mat['in_data']['SPEEDOMETER'][0][0]['speed'][0][0][0][k]
        msg.twist.covariance[0] = cov_odom * (msg.twist.twist.linear.x)**2

        odom_data.append(msg)


    # Main loop
    k_imu = 0   # current index in the data lists
    k_odom = 0
    k_gnss = 0

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()

        # IMU
        if k_imu < len(imu_data) and imu_data[k_imu].header.stamp <= current_time:
            pub_imu.publish(imu_data[k_imu])
            k_imu += 1

        # GNSS
        if k_gnss < len(gnss_data) and gnss_data[k_gnss].header.stamp <= current_time:
            pub_gnss.publish(gnss_data[k_gnss])
            k_gnss += 1

        # Odom
        if k_odom < len(odom_data) and odom_data[k_odom].header.stamp <= current_time:
            pub_odom.publish(odom_data[k_odom])
            k_odom += 1

        rate.sleep()
