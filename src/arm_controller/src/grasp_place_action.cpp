#include "grasp_place_action.hpp"
#include "utils.hpp"

constexpr double BASE_LINK_HEIGHT = 0.050;

constexpr bool isOreMarker(int id) {
	return id >= 0 && id <= 4;
}

GraspPlace::GraspPlace()
    : action_server("grasp_place", false), tf_buffer(ros::Duration(60)),
      tf_listener(tf_buffer) {
	arm_target_pub = nh.advertise<geometry_msgs::PoseStamped>("arm_target", 1);
	fiducial_markers_sub = nh.subscribe<apriltag_msgs::ApriltagMarkerArray>(
	    "markers", 1,
	    std::bind(&GraspPlace::onFiducialMarkers, this, std::placeholders::_1));
	action_server.registerGoalCallback(
	    std::bind(&GraspPlace::goalCallback, this));
	action_server.start();
}

void GraspPlace::goalCallback() {
	if (state != State::IDLE) {
		ROS_WARN("state is not idle");
		return;
	}
	auto goal = action_server.acceptNewGoal();
	this->marker_id = goal->marker_id;
	switch (goal->action_type) {
	case arm_controller_srvs::GraspPlaceGoal::ACTION_GRASP_ORE:
		if (!isOreMarker(marker_id)) {
			action_server.setAborted(arm_controller_srvs::GraspPlaceResult(),
			                         "Can only grasp 0~4");
			return;
		}
		this->action = Action::GRASP_ORE;
		break;
	case arm_controller_srvs::GraspPlaceGoal::ACTION_PLACE_ORE:
		if (isOreMarker(marker_id)) {
			action_server.setAborted(arm_controller_srvs::GraspPlaceResult(),
			                         "Can only place 5~7");
			return;
		}
		this->action = Action::PLACE_ORE;
		break;
	case arm_controller_srvs::GraspPlaceGoal::ACTION_STACK_ORE:
		if (!isOreMarker(marker_id)) {
			action_server.setAborted(arm_controller_srvs::GraspPlaceResult(),
			                         "Can only stack 0~4");
			return;
		}
		this->action = Action::STACK_ORE;
		break;
	default:
		action_server.setAborted(arm_controller_srvs::GraspPlaceResult(),
		                         "Unknown action");
		return;
	}

	if (this->action == Action::GRASP_ORE) {
		arm.open_gripper();
	}
	arm.reset_position();

	this->state = State::OBSERVING;
	observe_retries = 0;
}

void GraspPlace::onFiducialMarkers(
    const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg) {
	if (this->state != State::OBSERVING)
		return;

	auto task_opt = get_task_details(msg);

	if (!task_opt.has_value()) {
		observe_retries++;

		if (observe_retries > 10) {
			action_server.setAborted(arm_controller_srvs::GraspPlaceResult(),
			                         "No marker detected");
			this->state = State::IDLE;
			return;
		} else {
			return;
		}
	}
	this->task = *task_opt;

	arm_target_pub.publish(task.target);

	geometry_msgs::PoseStamped target_relative =
	    tf_buffer.transform(task.target, "base_link");
	double target_x = target_relative.pose.position.x;
	double target_y = target_relative.pose.position.y;
	double target_yaw = get_yaw(target_relative.pose.orientation);
	double goal_x = target_x + std::cos(target_yaw) * task.ideal_seperation;
	double goal_y = target_y + std::sin(target_yaw) * task.ideal_seperation;
	double goal_yaw = normalize_angle(target_yaw + M_PI);

	if (task.pick) {
		arm.open_gripper();
	}

	state = State::AIMING;
	chassis_move.execute(
	    goal_x, goal_y, goal_yaw, [this](const MoveFeedback &feedback) {
		    if (state != State::AIMING)
			    return;

		    if (feedback.state == MoveFeedback::State::FAIL) {
			    action_server.setAborted(
			        arm_controller_srvs::GraspPlaceResult(), "Failed to aim");
			    this->state = State::IDLE;

		    } else if (feedback.state == MoveFeedback::State::SUCCESS) {
			    this->state = State::ACTING;
			    geometry_msgs::PoseStamped target_relative =
			        tf_buffer.transform(task.target, "arm_link", ros::Time(0),
			                            "odom");
			    arm.move_to(target_relative.pose.position.x,
			                target_relative.pose.position.y);

			    async_wait.wait(ros::Duration(2), [this] {
				    if (task.pick) {
					    arm.close_gripper();
				    } else {
					    arm.open_gripper();
				    }

				    async_wait.wait(ros::Duration(2), [this] {
					    arm.reset_position();
					    this->state = State::MOVING_BACK;

					    chassis_move.execute(
					        -task.back_distance, 0, 0,
					        [this](const MoveFeedback &feedback) {
						        if (feedback.state ==
						            MoveFeedback::State::SUCCESS) {
							        this->state = State::IDLE;
							        action_server.setSucceeded();
						        } else if (feedback.state ==
						                   MoveFeedback::State::FAIL) {
							        this->state = State::IDLE;
							        action_server.setAborted(
							            arm_controller_srvs::GraspPlaceResult(),
							            "Moving back failed");
						        }
					        });
				    });
			    });
		    }
	    });
}

static geometry_msgs::PoseStamped
get_marker_pose(const apriltag_msgs::ApriltagMarker &marker) {
	geometry_msgs::PoseStamped result;
	result.header = marker.header;
	result.pose = marker.pose;
	return result;
}

static void offset_pose(geometry_msgs::PoseStamped &pose,
                        const tf2::Vector3 &offset) {
	tf2::Quaternion q;
	tf2::fromMsg(pose.pose.orientation, q);
	auto d = tf2::Matrix3x3(q) * offset;
	pose.pose.position.x += d.x();
	pose.pose.position.y += d.y();
	pose.pose.position.z += d.z();
}

std::optional<TaskDetails> GraspPlace::get_task_details(
    const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg) {

	if (action == Action::GRASP_ORE || action == Action::STACK_ORE) {
		// cube marker
		std::vector<geometry_msgs::PoseStamped> filtered;
		for (const auto &marker : msg->markers) {
			if (marker.id != this->marker_id)
				continue;
			auto pose = get_marker_pose(marker);
			offset_pose(pose, {-0.045 / 2 + 0.050, 0, 0});
			pose = tf_buffer.transform(pose, "base_link");

			double cube_height = pose.pose.position.z + BASE_LINK_HEIGHT;
			ROS_INFO_STREAM("Cube height: " << cube_height);

			if (action == Action::GRASP_ORE) {
				if (cube_height > 0.045 && cube_height < 0.100) {
					filtered.push_back(pose);
				}
			} else if (action == Action::STACK_ORE) {
				if (cube_height > 0.000 && cube_height < 0.050) {
					offset_pose(pose, {0, 0, 0.06});
					filtered.push_back(pose);
				}
			}
		}

		if (filtered.empty())
			return std::nullopt;

		std::sort(filtered.begin(), filtered.end(),
		          [](const geometry_msgs::PoseStamped &a,
		             const geometry_msgs::PoseStamped &b) {
			          double angle_a = std::abs(
			              normalize_angle(get_yaw(a.pose.orientation) + M_PI));
			          double angle_b = std::abs(
			              normalize_angle(get_yaw(b.pose.orientation) + M_PI));
			          return angle_a < angle_b;
		          });

		TaskDetails t;
		t.target = filtered[0];
		t.back_distance = 0.2;
		t.ideal_seperation = 0.20;
		if (action == Action::GRASP_ORE) {
			t.pick = true;
		} else if (action == Action::STACK_ORE) {
			t.pick = false;
		}

		return t;

	} else if (action == Action::PLACE_ORE) {
		// exchange marker
		return std::nullopt;
	}

	return std::nullopt;
}
