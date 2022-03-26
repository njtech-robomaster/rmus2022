#!/bin/python3
# Software License Agreement (BSD License)

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, Point
from apriltag_msgs.msg import ApriltagMarkerArray


def quaternion_to_rotation_matrix(quat):
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], 0.0],
         [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], 0.0],
            [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], 0.0],
            [0.0, 0.0, 0.0, 1.0]],
        dtype=q.dtype)
    return rot_matrix


def get_wp_from_pose(aruco_pose_msg):
    rotation_quaternion = np.asarray([aruco_pose_msg.orientation.w,
                                      aruco_pose_msg.orientation.x,
                                      aruco_pose_msg.orientation.y,
                                      aruco_pose_msg.orientation.z])
    rot_ = quaternion_to_rotation_matrix(rotation_quaternion)
    pos_ = np.asarray([0, 0, 0.1, 0])
    pos_marker = np.asarray([aruco_pose_msg.position.x,
                             aruco_pose_msg.position.y,
                             aruco_pose_msg.position.z,
                             0])
    pos_wp = np.matmul(pos_, rot_)
    pos_wp += pos_marker
    wp_ = pos_wp[:3]
    rospy.loginfo("marker : " + str(pos_marker))
    rospy.loginfo("waypoint : " + str(wp_))
    return wp_


class graspAruco:
    def __init__(self):
        self.base_move_position_pub = rospy.Publisher("cmd_position", Twist)
        self.base_move_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose)
        self.sinknum_sub = rospy.Subscriber(
            '/sinknum', String, self.sinknumCallback, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "/markers", ApriltagMarkerArray, self.sinkCallback, queue_size=1)
        self.state = rospy.Subscriber(
            "state", String, self.statecallback)
        self.place_success = False
        self.onPlane = False
        self.sinknum = 5

    def sinknumCallback(self, data):
        self.sinknum = data.sinknum

    def statecallback(self, state):
        if state.data:
            self.onPlane = True
        else:
            self.onPlane = False

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        rospy.loginfo("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        rospy.loginfo("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def move_arm(self, t_vector):
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.2      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        rospy.loginfo("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def move_arm0(self, t_vector):
        move_arm_msg = Pose()
        # unit in [cm]
        # in the gripper base frame
        move_arm_msg.position.x = 0.90      # TODO
        move_arm_msg.position.y = 0.12
        move_arm_msg.position.z = 0
        move_arm_msg.orientation.x = 0.0
        move_arm_msg.orientation.y = 0.0
        move_arm_msg.orientation.z = 0.0
        move_arm_msg.orientation.w = 0.0
        rospy.loginfo("move the arm to the grasping pose")
        self.arm_position_pub.publish(move_arm_msg)

    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.09
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        rospy.loginfo("reset the arm")
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
        # rospy.loginfo("x_movement", x_move)
        move_base_msg_x.linear.x = x_move
        move_base_msg_x.linear.y = 0.0
        move_base_msg_x.linear.z = 0.0
        move_base_msg_x.angular.x = 0.0
        move_base_msg_x.angular.y = 0.0
        move_base_msg_x.angular.z = 0.0
        rospy.loginfo("move the base in x direction")
        self.base_move_position_pub.publish(move_base_msg_x)

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
        vel_cmd.linear.x = 0.11
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
        vel_cmd.linear.x = -0.11
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

    def forward_minimum_y_right(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = -0.11
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
        vel_cmd.linear.y = 0.11
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

    def move_base_velocity_x(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_back(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.backward_minimum_x()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_right(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
            for tt in range(int(n_forward_minimun)):
                self.forward_minimum_y_right()
            for tt in range(int(n_forward_zero)):
                self.forward_zero()

    def move_base_velocity_y_left(self, b_vector=0.5, duration=10):
        # b_vector : x_bandwidth
        # duration : multiple of execution_cycle
        execution_cycle = 10.0
        for t in range(int(duration)):
            n_forward_minimun = b_vector * execution_cycle
            n_forward_zero = (1-b_vector) * execution_cycle
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
        rospy.loginfo("move_base_y 5.0 cm")
        self.base_move_position_pub.publish(move_base_msg_y)

    def sinkCallback(self, data):

        if self.place_success == True or self.onPlane == False:
            return

        gama_x = 0.01
        gama_y = 0.01

        data = data.markers
        for x in data:
            if x.id == self.sinknum:
                data = x
                rospy.loginfo("Find "+str(x.id))
                break
        else:
            return

        tvec = [0, 0, 0]
        tvec[0] = data.pose.position.x
        tvec[1] = data.pose.position.y
        tvec[2] = data.pose.position.z

        quat = [0, 0, 0, 0]
        quat[0] = data.pose.orientation.x
        quat[1] = data.pose.orientation.y
        quat[2] = data.pose.orientation.z
        quat[3] = data.pose.orientation.w

        goal = [0.03, 0.0, 0.2]
        distance_in_x = tvec[2] - goal[2]
        distance_in_y = abs(tvec[0] - goal[0])
        if (tvec[0]-goal[0]) > 0:
            move_y_right = True
        else:
            move_y_right = False  # move to left
        print("distance in x", distance_in_x)
        print("distance in y", distance_in_y)

        if (distance_in_x <= gama_x) and (distance_in_y <= gama_y):
            # step forward
            self.move_base_velocity_x(b_vector=0.2, duration=16)

            rospy.loginfo("===== start placing ====")
            self.reset_arm()
            rospy.sleep(1)
            self.move_arm0(tvec)
            rospy.sleep(1)
            self.move_arm(tvec)
            rospy.sleep(1)
            self.open_gripper()
            rospy.sleep(1)
            self.reset_arm()
            rospy.sleep(1)
            self.close_gripper()
            self.forward_zero()
            rospy.sleep(1)
            rospy.loginfo("===== finish ====")

            self.place_success = True
            rospy.sleep(5)
            self.place_success = False

        else:
            # self.move_base_x(tvec)
            # self.move_base_velocity()
            # int(distance_in_x / 0.5)
            if distance_in_x > gama_x:
                self.move_base_velocity_x(b_vector=0.4, duration=1)
                rospy.loginfo("move in x")
                # time.sleep(1)
            if distance_in_y > gama_y:
                # self.move_base_y()
                if move_y_right:

                    self.move_base_velocity_y_right(b_vector=0.6, duration=1)
                    rospy.loginfo("move y to right")
                else:

                    self.move_base_velocity_y_left(b_vector=0.6, duration=1)
                    rospy.loginfo("move y to left")


def main():
    rospy.init_node('plane_aruco_node', anonymous=True)
    ap = graspAruco()
    rospy.loginfo("=====init=====")
    # ap.reset_arm()
    rospy.sleep(1)
    rospy.loginfo("=====reset arm at beginning=====")
    # ap.open_gripper()
    rospy.sleep(1)
    rospy.loginfo("=====open gripper at beginning=====")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        ap.forward_zero()
        # ap.reset_arm()


if __name__ == '__main__':
    main()
