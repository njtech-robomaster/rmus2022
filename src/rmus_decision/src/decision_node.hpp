#pragma once

#include <actionlib/client/simple_action_client.h>
#include <apriltag_msgs/ApriltagMarkerArray.h>
#include <arm_controller_srvs/GraspPlaceAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class DecisionNode {
  public:
	DecisionNode();

	void start();

  private:
	enum class State {
		INIT,
		DETECT_EXCHANGE,
		NAV_ORE,
		GRASP_ORE,
		NAV_EXCHANGE,
		PLACE_ORE,
		DONE
	};

	ros::NodeHandle nh;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_action;
	actionlib::SimpleActionClient<arm_controller_srvs::GraspPlaceAction>
	    arm_action;
	ros::Subscriber markers_sub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;

	geometry_msgs::Pose exchange_detection_location;
	std::array<geometry_msgs::Pose, 5> ore_locations;
	std::array<geometry_msgs::Pose, 3> exchange_locations;
	std::array<int, 5> first_ore_preference;
	std::optional<std::array<int, 3>> task_ores_override;

	State state;
	std::array<int, 3> task_ores;
	std::array<bool, 3> task_finished;
	int current_task;

	void on_markers_detected(
	    const apriltag_msgs::ApriltagMarkerArray::ConstPtr &markers);
	void to_detect_exchange();
	void to_nav_ore(int task_idx);
	void to_grasp_ore();
	void to_nav_exchange();
	void to_place_ore();
	void to_end();
	int choose_first_task();
	std::optional<int> choose_next_task();
	void navigate_to(const geometry_msgs::Pose &pose,
	                 std::function<void(bool)> cb);
};
