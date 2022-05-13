#include <actionlib/client/simple_action_client.h>
#include <arm_controller/GraspPlaceAction.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "arm_controller_test");

	actionlib::SimpleActionClient<arm_controller::GraspPlaceAction> client(
	    "grasp_place", true);
	client.waitForServer();

	ROS_INFO("Server started");

	int id = ros::param::param("~id", 0);
	int action = ros::param::param("~action", 1);
	arm_controller::GraspPlaceGoal goal;
	goal.marker_id = id;
	goal.action_type = action;
	ROS_INFO_STREAM("Sending goal id=" << id << " action=" << action);

	auto state = client.sendGoalAndWait(goal);

	ROS_INFO_STREAM(state.toString() << ": " << state.getText());
}
