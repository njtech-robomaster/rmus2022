#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static double normalize_angle(double x) {
	double a = fmod(x + M_PI, 2 * M_PI);
	return a >= 0 ? (a - M_PI) : (a + M_PI);
}

static double get_yaw(const geometry_msgs::Quaternion &q) {
	double roll, pitch, yaw;
	tf2::Matrix3x3{{q.x, q.y, q.z, q.w}}.getRPY(roll, pitch, yaw);
	return normalize_angle(yaw);
}

static geometry_msgs::Quaternion quaternionMsgFromYaw(double yaw) {
	geometry_msgs::Quaternion msg;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	msg = tf2::toMsg(q);
	return msg;
}
