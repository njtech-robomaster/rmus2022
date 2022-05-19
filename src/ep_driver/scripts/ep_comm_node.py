#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
import tf
import sys
import time
import robomaster
from robomaster import robot
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class epRobot(object):

    def __init__(self) -> None:
        pass

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f',
                      data.linear.x, data.linear.y, data.angular.z)

        data.linear.x = max(-1.0, data.linear.x)
        data.linear.x = min(1.0, data.linear.x)
        data.linear.y = max(-1.0, data.linear.y)
        data.linear.y = min(1.0, data.linear.y)
        data.angular.z = max(-1.0, data.angular.z)
        data.angular.z = min(1.0, data.angular.z)

        self.ep_chassis.move(x=data.linear.x, y=-data.linear.y,
                             z=-data.angular.z*57.2).wait_for_completed()

    def callback_position(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f',
                      data.linear.x, data.linear.y, data.angular.z)

        data.linear.x = max(-0.5, data.linear.x)
        data.linear.x = min(0.5, data.linear.x)
        data.linear.y = max(-0.5, data.linear.y)
        data.linear.y = min(0.5, data.linear.y)
        data.angular.z = max(-900.0, data.angular.z)
        data.angular.z = min(900.0, data.angular.z)

        self.ep_chassis.move(x=data.linear.x, y=-data.linear.y,
                             z=-data.angular.z).wait_for_completed()

    def callback_arm(self, data):

        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f',
                      data.position.x, data.position.y)

        if abs(data.position.x) < 1e-10 and abs(data.position.y) < 1e-10:
            self.ep_arm.recenter().wait_for_completed()
            # cmd = 'robotic_arm recenter;'
        else:
            self.ep_arm.moveto(
                data.position.x, data.position.y).wait_for_completed()

    def callback_gripper(self, data):

        rospy.loginfo(rospy.get_caller_id() + 'I heard %f', data.x)

        if abs(data.x - 1.0) < 1e-10:
            self.ep_gripper.open(power=50)
            time.sleep(1)
            self.ep_gripper.pause()

        if abs(data.x) < 1e-10:
            self.ep_gripper.close(power=50)
            time.sleep(1)
            self.ep_gripper.pause()

    def sub_position_handler(self, position_info):
        x, y, z = position_info
        self.pos_x = x
        self.pos_y = y
        if rospy.is_shutdown():
            sys.exit(1)

    def sub_attitude_info_handler(self, attitude_info):
        yaw, pitch, roll = attitude_info
        self.ang_z = yaw
        if rospy.is_shutdown():
            sys.exit(1)

    def sub_imu_info_handler(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        self.ang_vel_z = gyro_z
        imu = Imu()

        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"

        q = tf.transformations.quaternion_from_euler(0, 0, self.ang_z)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]

        imu.linear_acceleration.x = acc_x
        imu.linear_acceleration.y = acc_y
        imu.linear_acceleration.z = acc_z

        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = gyro_y
        imu.angular_velocity.z = gyro_z

        try:
            self.imu_publisher.publish(imu)
        except Exception as e:
            print(e)
        if rospy.is_shutdown():
            sys.exit(1)

    def sub_velocity_info_handler(self, imu_info):
        vgx, vgy, vgz, vbx, vby, vbz = imu_info
        self.vel_x = vbx
        self.vel_y = vby

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.ang_z)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(
            Point(self.pos_x, self.pos_y, 0.), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(self.vel_x, self.vel_y, 0), Vector3(0, 0, self.ang_vel_z))

        self.odom_publisher.publish(odom)
        if rospy.is_shutdown():
            sys.exit(1)

    def connect(self):
        robomaster.config.LOCAL_IP_STR = "192.168.42.1"

        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type='rndis')

        self.ep_chassis = self.ep_robot.chassis
        self.ep_arm = self.ep_robot.robotic_arm
        self.ep_gripper = self.ep_robot.gripper

        rospy.init_node('epsdk', anonymous=True)

        self.imu_publisher = rospy.Publisher("/ep/imu", Imu, queue_size=10)
        self.odom_publisher = rospy.Publisher("/ep/odom", Odometry, queue_size=10)

        self.pos_x = 0
        self.pos_y = 0
        self.ang_z = 0
        self.vel_x = 0
        self.vel_y = 0
        self.ang_vel_z = 0

        self.ep_chassis.sub_position(freq=10, callback=self.sub_position_handler)
        self.ep_chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)
        self.ep_chassis.sub_imu(freq=10, callback=self.sub_imu_info_handler)
        self.ep_chassis.sub_velocity(freq=10, callback=self.sub_velocity_info_handler)

        rospy.Subscriber('cmd_vel', Twist, self.callback)
        rospy.Subscriber('cmd_position', Twist, self.callback_position)
        rospy.Subscriber('arm_position', Pose, self.callback_arm)
        rospy.Subscriber('arm_gripper', Point, self.callback_gripper)

    def disconnect(self):
        self.ep_chassis.unsub_position()
        self.ep_chassis.unsub_attitude()
        self.ep_chassis.unsub_imu()
        self.ep_chassis.unsub_velocity()
        self.ep_robot.close()


if __name__ == '__main__':
    ep = epRobot()
    ep.connect()
    rospy.spin()
    ep.disconnect()
