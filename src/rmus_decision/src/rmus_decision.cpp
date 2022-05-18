#include "decision_node.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "rmus_decision");

	DecisionNode node;

	ros::NodeHandle nh;
	auto timer = nh.createTimer(
	    ros::Duration(0.1), [&](auto) { node.start(); }, true);

	ros::spin();
}
