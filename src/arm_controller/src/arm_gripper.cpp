#include "arm_gripper.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

ArmGripper::ArmGripper() {
	arm_gripper_pub = nh.advertise<geometry_msgs::Point>("/arm_gripper", 1);
	arm_position_pub = nh.advertise<geometry_msgs::Pose>("/arm_position", 1);
}

void ArmGripper::open_gripper() {
	geometry_msgs::Point msg;
	msg.x = 0;
	arm_gripper_pub.publish(msg);
	ROS_INFO("Open gripper");
}

void ArmGripper::close_gripper() {
	geometry_msgs::Point msg;
	msg.x = 1;
	arm_gripper_pub.publish(msg);
	ROS_INFO("Close gripper");
}

void ArmGripper::move_to(double x, double y) {
	x = std::clamp(x, 0.09, 0.24);
	if (0.09 <= x && x <= 0.18) {
		y = std::max(y, 0.08);
	} else if (x > 0.18) {
		y = std::max(y, -0.02);
	}

	geometry_msgs::Pose msg;
	msg.position.x = x;
	msg.position.y = y;
	arm_position_pub.publish(msg);
	ROS_INFO_STREAM("Move arm to x=" << x << " y=" << y);
}

void ArmGripper::reset_position() {
	move_to(0.1, 0.12);
}
