#include "chassis_move.hpp"
#include "grasp_place_action.hpp"
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "arm_controller");

	GraspPlace as;

	ros::spin();
}
