#include "arm_gripper.hpp"
#include "async_wait.hpp"
#include "chassis_move.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <apriltag_msgs/ApriltagMarkerArray.h>
#include <arm_controller_srvs/GraspPlaceAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <optional>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>

class TaskDetails {
  public:
	geometry_msgs::PoseStamped target;
	bool pick;
	double ideal_seperation;
	double back_distance;
	double ideal_observing_distance;
	std::optional<double> min_arm_x;
};

class GraspPlace {
  public:
	GraspPlace();

	void goalCallback();

	void
	onFiducialMarkers(const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg);

	std::optional<TaskDetails>
	get_task_details(const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg);

	void aiming_done();

	actionlib::SimpleActionServer<arm_controller_srvs::GraspPlaceAction>
	    action_server;

  private:
	enum class State {
		IDLE,
		OBSERVING1,
		AIMING1,
		OBSERVING2,
		AIMING2,
		ACTING,
		MOVING_BACK
	};
	enum class Action { GRASP_ORE, PLACE_ORE, STACK_ORE };

	ros::NodeHandle nh;
	ros::Publisher arm_target_pub;
	ros::Subscriber fiducial_markers_sub;
	ros::Subscriber odom_sub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base;

	ChassisMove chassis_move;
	ArmGripper arm;
	AsyncWaiter async_wait;

	State state = State::IDLE;
	int marker_id;
	Action action;
	TaskDetails task;
	int observe_retries;

	ros::Time last_moving_time;
	std::optional<nav_msgs::Odometry> last_odom;

	std::tuple<double, double, double>
	compute_goal(const geometry_msgs::PoseStamped &target, double seperation);
};
