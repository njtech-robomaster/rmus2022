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

ChassisMove::ChassisMove() {
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
			    goal.goal_x = odom_x + goal.goal_dx * std::cos(odom_yaw) -
			                  goal.goal_dy * std::sin(odom_yaw);
			    goal.goal_y = odom_y + goal.goal_dx * std::sin(odom_yaw) +
			                  goal.goal_dy * std::cos(odom_yaw);
			    goal.goal_theta = normalize_angle(odom_yaw + goal.goal_dtheta);

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
				    twist.angular.z = 0.1 * sign(goal.error_theta);
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

			    bool moving = false;
			    geometry_msgs::Twist twist;
			    if (std::abs(goal.error_x) > X_TOLERANCE) {
				    twist.linear.x = 0.1 * sign(goal.error_x);
				    moving = true;
			    }
			    if (std::abs(goal.error_y) > Y_TOLERANCE) {
				    twist.linear.y = 0.1 * sign(goal.error_y);
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
