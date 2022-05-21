#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class MoveFeedback {
  public:
	enum class State { INIT, FIXING_YAW, FIXING_XY, SUCCESS, FAIL };

	State state = State::INIT;
	ros::Time goal_timestamp;
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
	void execute(double dx, double dy, double dtheta, ros::Time timestamp,
	             std::function<void(const MoveFeedback &)> callback);

  private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher chassis_move_target_pub;
	int idle_ticks;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

	std::function<void(const MoveFeedback &)> current_callback;
	MoveFeedback goal;
};
