#include "utils.hpp"
#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "print_2d_pose");

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	ros::NodeHandle nh;
	auto timer = nh.createTimer(ros::Duration(1.0), [&](auto) {
		geometry_msgs::TransformStamped tf;
		try {
			tf = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
		} catch (tf2::TransformException &ex) {
			std::cout << ex.what() << std::endl;
			return;
		}
		std::cout << tf.transform.translation.x << ", "
		          << tf.transform.translation.y << ", "
		          << get_yaw(tf.transform.rotation) << std::endl;
	});

	ros::spin();
}
