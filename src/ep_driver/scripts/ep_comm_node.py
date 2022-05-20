#!/usr/bin/env python3

from math import cos, sin
import rospy
import tf
import robomaster
from robomaster import robot
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class epRobot(object):

    def __init__(self) -> None:
        pass

    def callback(self, data):
        data.linear.x = max(-1.0, data.linear.x)
        data.linear.x = min(1.0, data.linear.x)
        data.linear.y = max(-1.0, data.linear.y)
        data.linear.y = min(1.0, data.linear.y)
        data.angular.z = max(-1.0, data.angular.z)
        data.angular.z = min(1.0, data.angular.z)

        self.ep_chassis.drive_speed(x=data.linear.x, y=-data.linear.y, z=-data.angular.z*57.3, timeout=0.5)

    def callback_arm(self, data):
        if abs(data.position.x) < 1e-10 and abs(data.position.y) < 1e-10:
            self.ep_arm.recenter()
        else:
            self.ep_arm.moveto(data.position.x, data.position.y)

    def callback_gripper(self, data):
        if abs(data.x - 1.0) < 1e-10:
            self.ep_gripper.open(power=50)
        elif abs(data.x) < 1e-10:
            self.ep_gripper.close(power=50)

    def sub_position_handler(self, position_info):
        x, y, z = position_info
        self.pos_x = x
        self.pos_y = y

    def sub_attitude_info_handler(self, attitude_info):
        yaw, pitch, roll = attitude_info
        self.ang_z = yaw / 57.3

    def sub_imu_info_handler(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        self.ang_vel_z = gyro_z
        imu = Imu()

        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"

        imu.orientation_covariance[0] = -1

        imu.linear_acceleration.x = acc_x*9.8
        imu.linear_acceleration.y = -acc_y*9.8
        imu.linear_acceleration.z = -acc_z*9.8

        imu.angular_velocity.x = gyro_x
        imu.angular_velocity.y = -gyro_y
        imu.angular_velocity.z = -gyro_z

        self.imu_publisher.publish(imu)

    def sub_velocity_info_handler(self, imu_info):
        vgx, vgy, vgz, vbx, vby, vbz = imu_info
        self.vel_x = vbx
        self.vel_y = vby

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, -self.ang_z)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = -self.pos_y

        self.odom_publisher.publish(odom)

    def connect(self):
        robomaster.config.LOCAL_IP_STR = "192.168.42.3"

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

        self.ep_chassis.sub_position(cs=1, freq=10, callback=self.sub_position_handler)
        self.ep_chassis.sub_attitude(freq=10, callback=self.sub_attitude_info_handler)
        self.ep_chassis.sub_imu(freq=10, callback=self.sub_imu_info_handler)
        self.ep_chassis.sub_velocity(freq=10, callback=self.sub_velocity_info_handler)

        rospy.Subscriber('cmd_vel', Twist, self.callback)
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
