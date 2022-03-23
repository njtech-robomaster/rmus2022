#!/usr/bin/env python
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

import socket
import sys

# 直连模式下，机器人默认 IP 地址为 192.168.2.1, 控制命令端口号为 40923
host = "192.168.2.1"
port = 40923
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
spd_lin = 0.5
spd_ang = 2.0

def callback(data):
    data.linear.x = max(-spd_lin, data.linear.x)
    data.linear.x = min(spd_lin, data.linear.x)
    data.angular.z = max(-spd_ang, data.angular.z)
    data.angular.z = min(spd_ang, data.angular.z)

    rospy.loginfo(rospy.get_caller_id() + 'I heard %f %f %f', data.linear.x, data.linear.y, data.angular.z)
    cmd = 'chassis speed x %f y %f z %f;' %(data.linear.x, -data.linear.y, -data.angular.z*57.2)

    # 发送控制命令给机器人
    
    
    s.send(cmd.encode('utf-8'))
    print(cmd)

def listener():

    address = (host, int(port))

    # 与机器人控制命令端口建立 TCP 连接

    print("Connecting...")

    s.connect(address)

    print("Connected!")

    msg = 'command;'

    # 发送控制命令给机器人
    s.send(msg.encode('utf-8'))
    try:
        # 等待机器人返回执行结果
        buf = s.recv(1024)

        print(buf.decode('utf-8'))
    except socket.error as e:
        print("Error receiving :", e)
        sys.exit(1)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def turn_off_connect():
    msg = 'quit;'

    # 发送控制命令给机器人
    s.send(msg.encode('utf-8'))
    try:
        # 等待机器人返回执行结果
        buf = s.recv(1024)

        if buf.decode('utf-8') == 'ok;':
            print("")
            print("Disable SDK")
    except socket.error as e:
        print("Error receiving :", e)
        sys.exit(1)
    # 关闭端口连接
    s.shutdown(socket.SHUT_WR)
    s.close()

if __name__ == '__main__':
    listener()
    turn_off_connect()

