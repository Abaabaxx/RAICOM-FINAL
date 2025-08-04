#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ackermann_msgs.msg import AckermannDrive

def main():
    rospy.init_node('ackermann_drive_forward_node')
    pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=10)


    rate = rospy.Rate(10)  # 10Hz
    drive_msg = AckermannDrive()
    drive_msg.speed = 3.5  # 单位：m/s，按你要求
    drive_msg.steering_angle = 0.0  # 直行

    rospy.loginfo("Publishing constant speed commands...")

    while not rospy.is_shutdown():
        pub.publish(drive_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
