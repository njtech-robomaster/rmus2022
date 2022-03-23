#!/bin/python3
import threading
import roslib
import rospy
import re
import rospy
import numpy as np
import cv2
import math
import os
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import sys
import time
from geometry_msgs.msg import Twist, Pose, Point

import sys, select, termios, tty

class graspAruco:
    def __init__(self):
        self.base_move_position_pub = rospy.Publisher("cmd_position", Twist)
        self.base_move_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose)
        self.image_sub = rospy.Subscriber("/aruco_pose", Pose, self.graspCallback, queue_size = 1)

        self.grasp_success = False

        self.base_vel = 0.11
        self.execution_cycle = 10.0

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        print("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        print("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self,t_vector):
        print("move_arm", t_vector)
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.2       # TODO
        move_arm_msg.position.y = -0.02
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        print("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    # def reset_arm(self):            
    #     reset_arm_msg = Pose()
    #     reset_arm_msg.position.x = 0
    #     reset_arm_msg.position.y = 0
    #     reset_arm_msg.position.z = 0.0
    #     reset_arm_msg.orientation.x = 0.0
    #     reset_arm_msg.orientation.y = 0.0
    #     reset_arm_msg.orientation.z = 0.0
    #     reset_arm_msg.orientation.w = 0.0
    #     print("reset the arm")
    #     self.arm_position_pub.publish(reset_arm_msg)
    def reset_arm(self):            
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        print("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)
        rospy.sleep(0.1)
        self.arm_position_pub.publish(reset_arm_msg)

    def move_base_x(self, t_vector):
        move_base_msg_x = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        # goal = [0.03, 0.0, 0.115]
        # distance_x = t_vector[2]-goal[2]
        # # x_move = 0.07
        # if distance_x <= 0.05:
        #     x_move = 0
        # else:
        x_move = 0.05
            # x_move = 0.7 * distance_x
        # print("x_movement", x_move)
        move_base_msg_x.linear.x = x_move
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        print("move the base in x direction")
        self.base_move_position_pub.publish(move_base_msg_x)

    def forward_test_01m(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.01
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(20)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_zero(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        
    def forward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = self.base_vel
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def backward_minimum_x(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = -self.base_vel
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.1)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_right(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = -self.base_vel
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def forward_minimum_y_left(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = self.base_vel
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)
        rospy.sleep(0.05)
        # motor STOP
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.x = 0.0
        vel_cmd.angular.y = 0.0
        vel_cmd.angular.z = 0.0
        self.base_move_vel_pub.publish(vel_cmd)

    def move_base_velocity_x(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        for t in range(int(duration)):
            n_forward_minimun = b_vector * self.execution_cycle
            n_forward_zero = (1-b_vector) * self.execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_back(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        for t in range(int(duration)):
            n_forward_minimun = b_vector * self.execution_cycle
            n_forward_zero = (1-b_vector) * self.execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.backward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_right(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        for t in range(int(duration)):
            n_forward_minimun = b_vector * self.execution_cycle
            n_forward_zero = (1-b_vector) * self.execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_right()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_left(self,b_vector = 0.5, duration = 10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        for t in range(int(duration)):
            n_forward_minimun = b_vector * self.execution_cycle
            n_forward_zero = (1-b_vector) * self.execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_left()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    
    def move_base_y(self):
        move_base_msg_y = Twist()
        # unit in [m]
        # in the car frame
        # t_vector is the position of the marker in the camera frame
        # goal = [0.03, 0.0, 0.115]
        y_move = 0.05
        move_base_msg_y.linear.x = 0.0
        # move_base_msg_y.linear.y = goal[0] - t_vector[0]
        move_base_msg_y.linear.y = y_move
        move_base_msg_y.linear.z = 0.0
        move_base_msg_y.angular.x = 0.0
        move_base_msg_y.angular.y = 0.0
        move_base_msg_y.angular.z = 0.0
        print("move_base_y 5.0 cm")
        self.base_move_position_pub.publish(move_base_msg_y)

    def graspCallback(self,data):

        if self.grasp_success == True:
            return

        # count = 0
        # time_re = 1/30 # time resolution in second, 30 Hz
        # self.reset_arm()
        # rospy.sleep(1)
        gama_x = 0.01
        gama_y = 0.01
        
        tvec = [0,0,0]
        tvec[0] = data.position.x
        tvec[1] = data.position.y
        tvec[2] = data.position.z

        quat = [0,0,0,0]
        quat[0] = data.orientation.x
        quat[1] = data.orientation.y
        quat[2] = data.orientation.z
        quat[3] = data.orientation.w
   
        # goal = [0.03, 0.0, 0.07]
        goal = [0.03, 0.0, 0.13]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = abs(tvec[0] - goal[0])
        if (tvec[0]-goal[0]) > 0:
            move_y_right = True
        else:
            move_y_right = False # move to left
        print("distance in x", distance_in_x)
        print("distance in y", distance_in_y)
        # print("move y to right", move_y_right)

    
        if (distance_in_x <= gama_x) and (distance_in_y <= gama_y):
            print("===== start to grasp ====")
            # self.open_gripper()
            # rospy.sleep(1)
            self.move_arm(tvec)
            rospy.sleep(1)
            self.close_gripper()
            rospy.sleep(1)
            self.reset_arm()
            rospy.sleep(1)
            self.forward_zero()
            rospy.sleep(1)
            print("===== finish graspping ====")

            self.grasp_success = True
            self.move_base_velocity_back(b_vector = 0.4, duration = 8)
            
        else:
            # self.move_base_x(tvec)
            # self.move_base_velocity()
            # int(distance_in_x / 0.5)
            if distance_in_x > gama_x:
                
                self.move_base_velocity_x(b_vector = 0.4, duration = 1)
                print("move in x")
                # time.sleep(1)
            if distance_in_y > gama_y:
                #self.move_base_y()
                if move_y_right:
                    
                    self.move_base_velocity_y_right(b_vector = 0.3, duration = 1)
                    print("move y to right")
                else:
                    
                    self.move_base_velocity_y_left(b_vector = 0.3, duration = 1)
                    print("move y to left")


def main():

    rospy.init_node('grasp_aruco_node', anonymous=True)
    ap = graspAruco()
    print("=====init=====")
    ap.reset_arm()
    rospy.sleep(2)
    print("=====reset arm at beginning=====")
    ap.open_gripper()
    rospy.sleep(2)
    print("=====open gripper at beginning=====")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ap.forward_zero()
        # ap.reset_arm()


if __name__=="__main__":
    main()
    
