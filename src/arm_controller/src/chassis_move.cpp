#include "chassis_move.hpp"
#include "utils.hpp"
#include <nav_msgs/Odometry.h>
#include <tuple>

constexpr double STATIC_LINEAR_VELOCITY = 0.005;
constexpr double STATIC_ANGULAR_VELOCITY = 0.005;
constexpr double IDLE_TIME = 0.3;
constexpr double X_TOLERANCE = 0.15;
constexpr double Y_TOLERANCE = 0.15;
constexpr double THETA_TOLERANCE = 0.15;
constexpr int MAX_RETRY_COUNT = 5;
constexpr double X_TOLERANCE2 = 0.01;
constexpr double Y_TOLERANCE2 = 0.01;
constexpr double YAW_TOLERANCE2 = 0.01;

constexpr double sign(double x) {
	return x < 0 ? -1 : 1;
}

ChassisMove::ChassisMove() {
	postion_pub = nh.advertise<geometry_msgs::Twist>("/cmd_position", 1);
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
		    double odom_vx = odom->twist.twist.linear.x;
		    double odom_vy = odom->twist.twist.linear.y;
		    double odom_vtheta = odom->twist.twist.angular.z;

		    auto send_cmd_position = [&] {
			    geometry_msgs::Twist pos_cmd;

			    if (std::abs(goal.error_x) > X_TOLERANCE ||
			        std::abs(goal.error_y) > Y_TOLERANCE) {
				    pos_cmd.linear.x = goal.error_x;
				    pos_cmd.linear.y = goal.error_y;
				    ROS_INFO("Adjusting x,y");
			    } else {
				    pos_cmd.angular.z = goal.error_theta;
				    ROS_INFO("Adjusting yaw");
			    }

			    postion_pub.publish(pos_cmd);
		    };

		    switch (goal.state) {

		    case MoveFeedback::State::INIT: {
			    goal.state = MoveFeedback::State::MOVING;
			    goal.goal_x = odom_x + goal.goal_dx * std::cos(odom_yaw) -
			                  goal.goal_dy * std::sin(odom_yaw);
			    goal.goal_y = odom_y + goal.goal_dx * std::sin(odom_yaw) +
			                  goal.goal_dy * std::cos(odom_yaw);
			    goal.goal_theta = normalize_angle(odom_yaw + goal.goal_dtheta);

			    double ex = goal.goal_x - odom_x;
			    double ey = goal.goal_y - odom_y;
			    goal.error_x =
			        ex * std::cos(odom_yaw) + ey * std::sin(odom_yaw);
			    goal.error_y =
			        -ex * std::sin(odom_yaw) + ey * std::cos(odom_yaw);
			    goal.error_theta = normalize_angle(goal.goal_theta - odom_yaw);
			    goal.idle_since = ros::Time::now();

			    geometry_msgs::PoseStamped debug_msg;
			    debug_msg.header.frame_id = "base_link";
			    debug_msg.header.stamp = odom->header.stamp;
			    debug_msg.pose.position.x = goal.goal_dx;
			    debug_msg.pose.position.y = goal.goal_dy;
			    debug_msg.pose.orientation =
			        quaternionMsgFromYaw(goal.goal_dtheta);
			    chassis_move_target_pub.publish(debug_msg);

			    ROS_INFO_STREAM("Goal dx=" << goal.error_x
			                               << " dy=" << goal.error_y
			                               << " dtheta=" << goal.error_theta);

			    send_cmd_position();

			    current_callback(goal);

		    } break;

		    case MoveFeedback::State::MOVING: {

			    double ex = goal.goal_x - odom_x;
			    double ey = goal.goal_y - odom_y;
			    goal.error_x =
			        ex * std::cos(odom_yaw) + ey * std::sin(odom_yaw);
			    goal.error_y =
			        -ex * std::sin(odom_yaw) + ey * std::cos(odom_yaw);
			    goal.error_theta = normalize_angle(goal.goal_theta - odom_yaw);

			    double linear_vel =
			        std::sqrt(odom_vx * odom_vx + odom_vy * odom_vy);
			    double angular_vel = odom_vtheta;
			    if (linear_vel > STATIC_LINEAR_VELOCITY ||
			        angular_vel > STATIC_ANGULAR_VELOCITY) {
				    goal.idle_since = ros::Time::now();
			    }

			    if ((ros::Time::now() - goal.idle_since).toSec() > IDLE_TIME) {

				    if (std::abs(goal.error_x) > X_TOLERANCE ||
				        std::abs(goal.error_y) > Y_TOLERANCE ||
				        std::abs(goal.error_theta) > THETA_TOLERANCE) {

					    if (goal.retry_count >= MAX_RETRY_COUNT) {
						    ROS_INFO_STREAM("Goal failed dx="
						                    << goal.error_x
						                    << " dy=" << goal.error_y
						                    << " dtheta=" << goal.error_theta);
						    goal.state = MoveFeedback::State::FAIL;
						    current_callback(goal);
						    current_callback = nullptr;

					    } else {
						    goal.idle_since = ros::Time::now();
						    goal.retry_count++;

						    ROS_INFO_STREAM("Fix goal dx="
						                    << goal.error_x
						                    << " dy=" << goal.error_y
						                    << " dtheta=" << goal.error_theta);

						    send_cmd_position();

						    current_callback(goal);
					    }

				    } else {

					    ROS_INFO_STREAM("Goal moving done dx="
					                    << goal.error_x
					                    << " dy=" << goal.error_y
					                    << " dtheta=" << goal.error_theta);
					    goal.state = MoveFeedback::State::FIXING_XY;
					    current_callback(goal);
				    }
			    }

		    } break;

		    case MoveFeedback::State::FIXING_XY: {

			    double ex = goal.goal_x - odom_x;
			    double ey = goal.goal_y - odom_y;
			    goal.error_x =
			        ex * std::cos(odom_yaw) + ey * std::sin(odom_yaw);
			    goal.error_y =
			        -ex * std::sin(odom_yaw) + ey * std::cos(odom_yaw);
			    goal.error_theta = normalize_angle(goal.goal_theta - odom_yaw);

			    bool moving = false;
			    geometry_msgs::Twist twist;
			    if (std::abs(goal.error_x) > X_TOLERANCE2) {
				    twist.linear.x = 0.1 * sign(goal.error_x);
				    moving = true;
			    }
			    if (std::abs(goal.error_y) > Y_TOLERANCE2) {
				    twist.linear.y = 0.1 * sign(goal.error_y);
				    moving = true;
			    }
			    cmd_vel_pub.publish(twist);
			    if (!moving) {
				    ROS_INFO_STREAM("Goal fixing_xy done dx="
				                    << goal.error_x << " dy=" << goal.error_y
				                    << " dtheta=" << goal.error_theta);
				    goal.state = MoveFeedback::State::FIXING_YAW;
			    }
			    current_callback(goal);

		    } break;

		    case MoveFeedback::State::FIXING_YAW: {

			    double ex = goal.goal_x - odom_x;
			    double ey = goal.goal_y - odom_y;
			    goal.error_x =
			        ex * std::cos(odom_yaw) + ey * std::sin(odom_yaw);
			    goal.error_y =
			        -ex * std::sin(odom_yaw) + ey * std::cos(odom_yaw);
			    goal.error_theta = normalize_angle(goal.goal_theta - odom_yaw);

			    bool moving = false;
			    geometry_msgs::Twist twist;
			    if (std::abs(goal.error_theta) > YAW_TOLERANCE2) {
				    twist.angular.z = 0.1 * sign(goal.error_theta);
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
				    current_callback(goal);
				    current_callback = nullptr;
			    }
		    } break;

		    default:
			    ROS_ERROR("Bad state!");
			    break;
		    }
	    });
}

void ChassisMove::execute(double dx, double dy, double dtheta,
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
}
