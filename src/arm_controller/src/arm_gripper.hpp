#pragma once

#include <ros/ros.h>

class ArmGripper {
  public:
	ArmGripper();
	void open_gripper();
	void close_gripper();
	void reset_position();
	void move_to(double x, double y);

  private:
	ros::NodeHandle nh;
	ros::Publisher arm_gripper_pub;
	ros::Publisher arm_position_pub;
};
