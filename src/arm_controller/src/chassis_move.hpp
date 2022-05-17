#pragma once

#include <ros/ros.h>

class MoveFeedback {
  public:
	enum class State { INIT, MOVING, FIXING_YAW, FIXING_XY, SUCCESS, FAIL };

	State state = State::INIT;
	double goal_dx = NAN;
	double goal_dy = NAN;
	double goal_dtheta = NAN;
	double goal_x = NAN;
	double goal_y = NAN;
	double goal_theta = NAN;
	double error_x = NAN;
	double error_y = NAN;
	double error_theta = NAN;
	int retry_count = 0;
	ros::Time idle_since = ros::Time::now();
};

class ChassisMove {
  public:
	ChassisMove();
	void execute(double dx, double dy, double dtheta,
	             std::function<void(const MoveFeedback &)> callback);

  private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Publisher postion_pub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher chassis_move_target_pub;

	std::optional<std::tuple<ros::Time, double, double>> last_odom_pos;

	std::function<void(const MoveFeedback &)> current_callback;
	MoveFeedback goal;
};
