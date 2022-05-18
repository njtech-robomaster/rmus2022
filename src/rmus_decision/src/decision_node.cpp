#include "decision_node.hpp"
#include "utils.hpp"

template <typename Tp, size_t Nm>
static std::array<Tp, Nm> retrive_array_param(const ros::NodeHandle &nh,
                                              const std::string &param) {
	std::vector<Tp> values;
	if (!nh.getParam(param, values)) {
		throw std::runtime_error("Parameter " + param + " is not set!");
	}
	if (values.size() != Nm) {
		throw std::runtime_error("Size of parameter " + param + " is not " +
		                         std::to_string(Nm));
	}
	std::array<Tp, Nm> array;
	for (size_t i = 0; i < Nm; i++) {
		array[i] = values[i];
	}
	return array;
}

static geometry_msgs::Pose retrive_pose_param(const ros::NodeHandle &nh,
                                              const std::string &param) {
	auto values = retrive_array_param<double, 3>(nh, param);
	double x = values[0];
	double y = values[1];
	double yaw = values[2];
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = 0;
	pose.orientation = quaternionMsgFromYaw(yaw);
	return pose;
}

DecisionNode::DecisionNode()
    : move_action("move_base", true), arm_action("grasp_place", true),
      tf_listener(tf_buffer), state(State::INIT) {
	markers_sub = nh.subscribe<apriltag_msgs::ApriltagMarkerArray>(
	    "markers", 1,
	    std::bind(&DecisionNode::on_markers_detected, this,
	              std::placeholders::_1));

	ros::NodeHandle nh_private("~");
	exchange_detection_location =
	    retrive_pose_param(nh_private, "locations/exchange_detection");
	ore_locations[0] = retrive_pose_param(nh_private, "locations/ore_1");
	ore_locations[1] = retrive_pose_param(nh_private, "locations/ore_2");
	ore_locations[2] = retrive_pose_param(nh_private, "locations/ore_3");
	ore_locations[3] = retrive_pose_param(nh_private, "locations/ore_4");
	ore_locations[4] = retrive_pose_param(nh_private, "locations/ore_5");
	exchange_locations[0] =
	    retrive_pose_param(nh_private, "locations/exchange_B");
	exchange_locations[1] =
	    retrive_pose_param(nh_private, "locations/exchange_O");
	exchange_locations[2] =
	    retrive_pose_param(nh_private, "locations/exchange_X");
	first_ore_preference =
	    retrive_array_param<int, 5>(nh_private, "first_ore_preference");
	for (int &ore_id : first_ore_preference) {
		ore_id--; // convert 1~5 to 0~4
	}

	move_action.waitForServer();
	arm_action.waitForServer();
}

void DecisionNode::start() {
	std::fill(task_finished.begin(), task_finished.end(), false);
	to_detect_exchange();
}

void DecisionNode::on_markers_detected(
    const apriltag_msgs::ApriltagMarkerArray::ConstPtr &markers) {

	if (state != State::DETECT_EXCHANGE) {
		return;
	}

	using ExchangeTag = std::tuple<int, geometry_msgs::PoseStamped>;
	std::vector<ExchangeTag> exchange_tags;
	for (const auto &marker : markers->markers) {
		if (marker.id >= 5) {
			// B, O, X
			continue;
		}
		geometry_msgs::PoseStamped pose;
		pose.header = marker.header;
		pose.pose = marker.pose;
		if (pose.header.frame_id != "camera_color_optical_frame") {
			ROS_WARN_STREAM("Marker is in "
			                << pose.header.frame_id
			                << ", not camera_color_optical_frame!");
			continue;
		}
		geometry_msgs::PoseStamped pose_in_base_link;
		try {
			pose_in_base_link = tf_buffer.transform(pose, "base_link");
		} catch (tf2::TransformException &ex) {
			ROS_WARN_STREAM(
			    "Can't transform exchange marker pose: " << ex.what());
			continue;
		}
		if (pose_in_base_link.pose.position.z < 0.200) {
			continue;
		}
		exchange_tags.push_back({marker.id, pose});
	}

	if (exchange_tags.size() < 3) {
		return;
	} else if (exchange_tags.size() > 3) {
		ROS_WARN_STREAM("Detected " << exchange_tags.size()
		                            << " exchange tags!");
		return;
	}

	std::sort(exchange_tags.begin(), exchange_tags.end(),
	          [](const ExchangeTag &a, const ExchangeTag &b) {
		          const auto &pose_a = std::get<1>(a);
		          const auto &pose_b = std::get<1>(b);
		          return pose_a.pose.position.x < pose_b.pose.position.x;
	          });

	for (int i = 0; i < 3; i++) {
		task_ores[i] = std::get<0>(exchange_tags[i]);
	}

	ROS_INFO("Ores: %d, %d, %d", task_ores[0] + 1, task_ores[1] + 1,
	         task_ores[2] + 1);

	to_nav_ore(choose_first_task());
}

void DecisionNode::to_detect_exchange() {
	state = State::DETECT_EXCHANGE;

	navigate_to(exchange_detection_location, [this](bool success) {});
}

void DecisionNode::to_nav_ore(int task_idx) {
	state = State::NAV_ORE;
	current_task = task_idx;

	int ore = task_ores[current_task];
	ROS_INFO_STREAM("Going to ore " << (ore + 1));

	navigate_to(ore_locations[ore], [this](bool success) { to_grasp_ore(); });
}

void DecisionNode::to_grasp_ore() {
	state = State::GRASP_ORE;

	int ore = task_ores[current_task];
	ROS_INFO_STREAM("Grasping ore " << (ore + 1));

	arm_controller_srvs::GraspPlaceGoal goal;
	goal.action_type = arm_controller_srvs::GraspPlaceGoal::ACTION_GRASP_ORE;
	goal.marker_id = ore;
	arm_action.sendGoal(
	    goal,
	    [this](const actionlib::SimpleClientGoalState &state,
	           const arm_controller_srvs::GraspPlaceResult::ConstPtr &result) {
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			    ROS_INFO("Grasp succeeded");
			    to_nav_exchange();

		    } else {
			    ROS_ERROR_STREAM("Grasp failed, will retry. "
			                     << state.toString() << ": "
			                     << state.getText());
			    to_nav_ore(current_task);
		    }
	    });
}

void DecisionNode::to_nav_exchange() {
	state = State::NAV_EXCHANGE;

	ROS_INFO_STREAM("Navigating to exchange #" << current_task);
	navigate_to(exchange_locations[current_task],
	            [this](bool success) { to_place_ore(); });
}

void DecisionNode::to_place_ore() {
	state = State::PLACE_ORE;

	ROS_INFO_STREAM("Place ore to exchange #" << current_task);
	int exchanage_marker_id = 5 + current_task;

	arm_controller_srvs::GraspPlaceGoal goal;
	goal.action_type = arm_controller_srvs::GraspPlaceGoal::ACTION_PLACE_ORE;
	goal.marker_id = exchanage_marker_id;
	arm_action.sendGoal(
	    goal,
	    [this](const actionlib::SimpleClientGoalState &state,
	           const arm_controller_srvs::GraspPlaceResult::ConstPtr &result) {
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			    ROS_INFO("Place succeeded");

			    task_finished[current_task] = true;

			    auto next_task = choose_next_task();
			    if (next_task.has_value()) {
				    to_nav_ore(*next_task);
			    } else {
				    to_end();
			    }

		    } else {
			    ROS_ERROR_STREAM("Place failed, will retry. "
			                     << state.toString() << ": "
			                     << state.getText());
			    to_nav_exchange();
		    }
	    });
}

void DecisionNode::to_end() {
	state = State::DONE;
}

int DecisionNode::choose_first_task() {
	std::array<int, 5> w;
	std::fill(w.begin(), w.end(), -1);
	for (int i = 0; i < 5; i++) {
		w[first_ore_preference[i]] = i;
	}
	int task_idx = 0;
	for (int i = 1; i < 3; i++) {
		if (w[task_ores[i]] < w[task_ores[task_idx]]) {
			task_idx = i;
		}
	}
	return task_idx;
}

std::optional<int> DecisionNode::choose_next_task() {
	for (int i = 0; i < 3; i++) {
		if (!task_finished[i]) {
			return i;
		}
	}
	return std::nullopt;
}

void DecisionNode::navigate_to(const geometry_msgs::Pose &pose,
                               std::function<void(bool)> cb) {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose = pose;
	ROS_INFO_STREAM("Navigating to x=" << pose.position.x
	                                   << ", y=" << pose.position.y << ", yaw="
	                                   << get_yaw(pose.orientation));
	move_action.sendGoal(
	    goal, [cb](const actionlib::SimpleClientGoalState &state,
	               const move_base_msgs::MoveBaseResult::ConstPtr &result) {
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			    ROS_INFO("Navigation succeeded");
			    cb(true);
		    } else {
			    ROS_ERROR_STREAM("Navigation failed. " << state.toString()
			                                           << ": "
			                                           << state.getText());
			    cb(false);
		    }
	    });
}
