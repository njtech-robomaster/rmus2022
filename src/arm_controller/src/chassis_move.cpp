#include "chassis_move.hpp"
#include "utils.hpp"
#include <nav_msgs/Odometry.h>
#include <tuple>

static constexpr double X_TOLERANCE = 0.01;
static constexpr double Y_TOLERANCE = 0.01;
static constexpr double YAW_TOLERANCE = 0.01;

static constexpr double sign(double x) {
	return x < 0 ? -1 : 1;
}

ChassisMove::ChassisMove() : tf_listener(tf_buffer) {
	chassis_move_target_pub =
	    nh.advertise<geometry_msgs::PoseStamped>("chassis_move_target", 1);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(
	    "/ep/odom", 1, [this](const nav_msgs::Odometry::ConstPtr &odom) {
		    if (current_callback == nullptr)
			    return;

		    double odom_x = odom->pose.pose.position.x;
		    double odom_y = odom->pose.pose.position.y;
		    double odom_yaw = get_yaw(odom->pose.pose.orientation);

		    if (goal.state == MoveFeedback::State::INIT) {
			    idle_ticks = 0;
			    goal.state = MoveFeedback::State::FIXING_YAW;
		    }

		    double ex = goal.goal_x - odom_x;
		    double ey = goal.goal_y - odom_y;
		    goal.error_x = ex * std::cos(odom_yaw) + ey * std::sin(odom_yaw);
		    goal.error_y = -ex * std::sin(odom_yaw) + ey * std::cos(odom_yaw);
		    goal.error_theta = normalize_angle(goal.goal_theta - odom_yaw);

		    switch (goal.state) {

		    case MoveFeedback::State::FIXING_YAW: {

			    bool moving = false;
			    geometry_msgs::Twist twist;
			    if (std::abs(goal.error_theta) > YAW_TOLERANCE) {
				    twist.angular.z = 0.2 * sign(goal.error_theta);
				    moving = true;
			    }
			    cmd_vel_pub.publish(twist);
			    if (moving) {
				    current_callback(goal);
			    } else {
				    ROS_INFO_STREAM("Goal fixing_xy done dx="
				                    << goal.error_x << " dy=" << goal.error_y
				                    << " dtheta=" << goal.error_theta);
				    goal.state = MoveFeedback::State::FIXING_XY;
				    current_callback(goal);
			    }
		    } break;

		    case MoveFeedback::State::FIXING_XY: {

			    bool perform_control = idle_ticks == 0;
			    idle_ticks++;
			    idle_ticks %= 3;

			    if (!perform_control) {
				    geometry_msgs::Twist twist;
				    cmd_vel_pub.publish(twist);
				    return;
			    }

			    bool moving = false;
			    geometry_msgs::Twist twist;
			    if (std::abs(goal.error_x) > X_TOLERANCE) {
				    twist.linear.x = 0.1 * sign(goal.error_x);
				    moving = true;
			    }
			    if (std::abs(goal.error_y) > Y_TOLERANCE) {
				    twist.linear.y = 0.2 * sign(goal.error_y);
				    moving = true;
			    }
			    cmd_vel_pub.publish(twist);
			    if (moving) {
				    current_callback(goal);
			    } else {
				    goal.state = MoveFeedback::State::SUCCESS;
				    ROS_INFO_STREAM("Goal success dx="
				                    << goal.error_x << " dy=" << goal.error_y
				                    << " dtheta=" << goal.error_theta);
				    auto cb = current_callback;
				    current_callback = nullptr;
				    cb(goal);
			    }

		    } break;

		    default:
			    ROS_ERROR("Bad state!");
			    break;
		    }
	    });
}

void ChassisMove::execute(double dx, double dy, double dtheta,
                          ros::Time timestamp,
                          std::function<void(const MoveFeedback &)> callback) {

	if (current_callback != nullptr) {
		// cancel current goal
		goal.state = MoveFeedback::State::FAIL;
		current_callback(goal);
	}

	current_callback = callback;
	goal = MoveFeedback();
	goal.state = MoveFeedback::State::INIT;
	goal.goal_dx = dx;
	goal.goal_dy = dy;
	goal.goal_dtheta = dtheta;
	goal.goal_timestamp = timestamp;

	geometry_msgs::PoseStamped goal_in_base_link;
	goal_in_base_link.header.frame_id = "base_link";
	goal_in_base_link.header.stamp = timestamp;
	goal_in_base_link.pose.position.x = dx;
	goal_in_base_link.pose.position.y = dy;
	goal_in_base_link.pose.orientation = quaternionMsgFromYaw(dtheta);
	geometry_msgs::PoseStamped goal_in_odom =
	    tf_buffer.transform(goal_in_base_link, "odom", ros::Duration(0.5));
	goal.goal_x = goal_in_odom.pose.position.x;
	goal.goal_y = goal_in_odom.pose.position.y;
	goal.goal_theta = get_yaw(goal_in_odom.pose.orientation);
	chassis_move_target_pub.publish(goal_in_odom);
}
