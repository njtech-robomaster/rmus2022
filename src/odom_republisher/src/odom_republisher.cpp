#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "odom_republisher");

	ros::NodeHandle nh;
	tf2_ros::TransformBroadcaster tf_broadcaster;

	auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
	    "odom", 10, [&](const nav_msgs::Odometry::ConstPtr &msg) {
		    geometry_msgs::TransformStamped tf;
		    tf.header = msg->header;
		    tf.child_frame_id = msg->child_frame_id;
		    tf.transform.translation.x = msg->pose.pose.position.x;
		    tf.transform.translation.y = msg->pose.pose.position.y;
		    tf.transform.translation.z = msg->pose.pose.position.z;
		    tf.transform.rotation = msg->pose.pose.orientation;
		    tf_broadcaster.sendTransform(tf);
	    });

	ros::spin();
}
