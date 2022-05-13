#include "arm_gripper.hpp"
#include "async_wait.hpp"
#include "chassis_move.hpp"
#include <actionlib/server/simple_action_server.h>
#include <apriltag_msgs/ApriltagMarkerArray.h>
#include <arm_controller/GraspPlaceAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <optional>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TaskDetails {
  public:
	geometry_msgs::PoseStamped target;
	bool pick;
	double ideal_seperation;
	double back_distance;
};

class GraspPlace {
  public:
	GraspPlace();

	void goalCallback();

	void
	onFiducialMarkers(const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg);

	std::optional<TaskDetails>
	get_task_details(const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg);

	actionlib::SimpleActionServer<arm_controller::GraspPlaceAction>
	    action_server;

  private:
	enum class State { IDLE, OBSERVING, AIMING, ACTING, MOVING_BACK };
	enum class Action { GRASP_ORE, PLACE_ORE, STACK_ORE };

	ros::NodeHandle nh;
	ros::Publisher arm_target_pub;
	ros::Subscriber fiducial_markers_sub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

	ChassisMove chassis_move;
	ArmGripper arm;
	AsyncWaiter async_wait;

	State state = State::IDLE;
	int marker_id;
	Action action;
	TaskDetails task;
	int observe_retries;
};
