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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import robomaster
from robomaster import robot

import socket
import sys
import time

# 直连模式下，机器人默认 IP 地址为 192.168.2.1, 控制命令端口号为 40923


class epRobot(object):

    def __init__(self) -> None:
        self.listener()
        self.turn_off_connect()

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.linear.x, data.linear.y, data.angular.z)

        data.linear.x = max(-1.0, data.linear.x)
        data.linear.x = min(1.0, data.linear.x)
        data.linear.y = max(-1.0, data.linear.y)
        data.linear.y = min(1.0, data.linear.y)
        data.angular.z = max(-1.0, data.angular.z)
        data.angular.z = min(1.0, data.angular.z)


        self.ep_chassis.move(x=data.linear.x,y=-data.linear.y,z= -data.angular.z*57.2).wait_for_completed()
        #cmd = 'chassis speed x %f y %f z %f;' %(data.linear.x, -data.linear.y, -data.angular.z*57.2)


    def callback_position(self,data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.linear.x, data.linear.y, data.angular.z)

        data.linear.x = max(-0.5, data.linear.x)
        data.linear.x = min(0.5, data.linear.x)
        data.linear.y = max(-0.5, data.linear.y)
        data.linear.y = min(0.5, data.linear.y)
        data.angular.z = max(-900.0, data.angular.z)
        data.angular.z = min(900.0, data.angular.z)
        

        self.ep_chassis.move(x=data.linear.x,y=-data.linear.y,z= -data.angular.z).wait_for_completed()
        #cmd = 'chassis move x %f y %f z %f;' %(data.linear.x, -data.linear.y, -data.angular.z)


    def callback_arm(self,data):

        rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f', data.position.x, data.position.y)

        #data.position.x = max(90, data.position.x)
        #data.position.x = min(250, data.position.x)
        #data.position.y = max(40, data.position.y)
        #data.position.y = min(-100, data.position.y)

        if abs(data.position.x) < 1e-10 and abs(data.position.y) < 1e-10:
            self.ep_arm.recenter().wait_for_completed()
            # cmd = 'robotic_arm recenter;'
        else:
            self.ep_arm.moveto(data.position.x, data.position.y).wait_for_completed()
            # cmd = 'robotic_arm moveto x %f y %f;' %(data.position.x, data.position.y)
            # cmd = 'robotic_arm move x %f y %f;' %(data.position.x, data.position.y)

    def callback_gripper(self,data):

        rospy.loginfo(rospy.get_caller_id() + 'I heard %f', data.x)

        #data.position.x = max(90, data.position.x)
        #data.position.x = min(250, data.position.x)
        #data.position.y = max(40, data.position.y)
        #data.position.y = min(-100, data.position.y)

        if abs(data.x - 1.0) < 1e-10:
            self.ep_gripper.open(power=50)
            time.sleep(1)
            self.ep_gripper.pause()
            # cmd = 'robotic_gripper close 1;'
            
        if abs(data.x) < 1e-10:
            self.ep_gripper.close(power=50)
            time.sleep(1)
            self.ep_gripper.pause()
            # cmd = 'robotic_gripper open 1;'


        # 发送控制命令给机器人
        # s.send(cmd.encode('utf-8'))
        # print(cmd)

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta")
        self.ep_chassis = self.ep_robot.chassis
        self.ep_arm = self.ep_robot.robotic_arm
        self.ep_gripper=self.ep_robot.gripper

        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('cmd_vel', Twist, self.callback)
        rospy.Subscriber('cmd_position', Twist, self.callback_position)
        rospy.Subscriber('arm_position', Pose, self.callback_arm)
        rospy.Subscriber('arm_gripper', Point, self.callback_gripper)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def turn_off_connect(self):
        self.ep_robot.close()

if __name__ == '__main__':
    ep=epRobot()    

