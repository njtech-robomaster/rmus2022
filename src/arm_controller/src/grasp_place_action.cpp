#include "grasp_place_action.hpp"
#include "utils.hpp"

constexpr double BASE_LINK_HEIGHT = 0.050;

constexpr double STATIC_LINEAR_VELOCITY = 0.01;
constexpr double STATIC_ANGULAR_VELOCITY = 0.02;

constexpr double STATIC_TIME = 0.5;

constexpr bool isOreMarker(int id) {
	return id >= 0 && id <= 4;
}

GraspPlace::GraspPlace()
    : action_server("grasp_place", false), tf_buffer(ros::Duration(60)),
      tf_listener(tf_buffer), move_base("move_base", true) {
	arm_target_pub = nh.advertise<geometry_msgs::PoseStamped>("arm_target", 1);
	fiducial_markers_sub = nh.subscribe<apriltag_msgs::ApriltagMarkerArray>(
	    "markers", 1,
	    std::bind(&GraspPlace::onFiducialMarkers, this, std::placeholders::_1));
	action_server.registerGoalCallback(
	    std::bind(&GraspPlace::goalCallback, this));

	odom_sub = nh.subscribe<nav_msgs::Odometry>(
	    "/ep/odom", 1, [this](const nav_msgs::Odometry::ConstPtr &odom) {
		    if (last_odom.has_value()) {
			    double dt =
			        (odom->header.stamp - last_odom->header.stamp).toSec();
			    double dx = odom->pose.pose.position.x -
			                last_odom->pose.pose.position.x;
			    double dy = odom->pose.pose.position.y -
			                last_odom->pose.pose.position.y;
			    double v = std::sqrt(dx * dx + dy * dy) / dt;
			    double w = std::abs(odom->twist.twist.angular.z);
			    if (v > STATIC_LINEAR_VELOCITY || w > STATIC_ANGULAR_VELOCITY) {
				    last_moving_time = ros::Time::now();
			    }

		    } else {
			    last_moving_time = ros::Time::now();
		    }
		    last_odom = *odom;
	    });

	move_base.waitForServer();

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

	this->state = State::OBSERVING1;
	observe_retries = 0;
}

std::tuple<double, double, double>
GraspPlace::compute_goal(const geometry_msgs::PoseStamped &target,
                         double seperation) {
	geometry_msgs::PoseStamped target_relative =
	    tf_buffer.transform(target, "base_link");
	double target_x = target_relative.pose.position.x;
	double target_y = target_relative.pose.position.y;
	double target_yaw = get_yaw(target_relative.pose.orientation);
	double goal_x = target_x + std::cos(target_yaw) * seperation;
	double goal_y = target_y + std::sin(target_yaw) * seperation;
	double goal_yaw = normalize_angle(target_yaw + M_PI);
	return {goal_x, goal_y, goal_yaw};
}

void GraspPlace::onFiducialMarkers(
    const apriltag_msgs::ApriltagMarkerArray::ConstPtr &msg) {

	if (!last_odom.has_value() ||
	    (ros::Time::now() - last_moving_time).toSec() < STATIC_TIME) {
		return;
	}

	switch (this->state) {
	case State::OBSERVING1: {

		auto task_opt = get_task_details(msg);
		if (task_opt.has_value()) {
			this->task = *task_opt;

		} else {
			observe_retries++;
			if (observe_retries <= 10) {
				return;
			} else {
				action_server.setAborted(
				    arm_controller_srvs::GraspPlaceResult(),
				    "No marker detected");
				this->state = State::IDLE;
				return;
			}
		}

		arm_target_pub.publish(task.target);

		this->state = State::AIMING1;

		auto [goal_x, goal_y, goal_yaw] =
		    compute_goal(task.target, task.ideal_observing_distance);

		if (ros::param::param("~use_move_base", true)) {

			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "base_link";
			goal.target_pose.header.stamp = task.target.header.stamp;
			goal.target_pose.pose.position.x = goal_x;
			goal.target_pose.pose.position.y = goal_y;
			goal.target_pose.pose.orientation = quaternionMsgFromYaw(goal_yaw);
			ROS_INFO_STREAM("Send goal to move_base: "
			                << goal_x << ", " << goal_y << ", " << goal_yaw);
			move_base.sendGoal(
			    goal,
			    [this](const actionlib::SimpleClientGoalState &state,
			           const move_base_msgs::MoveBaseResult::ConstPtr &result) {
				    if (this->state != State::AIMING1)
					    return;

				    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
					    ROS_INFO("move_base action succeeded");
					    observe_retries = 0;
					    this->state = State::OBSERVING2;

				    } else {
					    ROS_ERROR_STREAM(state.toString()
					                     << ": " << state.getText());
					    action_server.setAborted(
					        arm_controller_srvs::GraspPlaceResult(),
					        "Failed to aim");
					    this->state = State::IDLE;
				    }
			    });

		} else {

			chassis_move.execute(
			    goal_x, goal_y, goal_yaw, task.target.header.stamp,
			    [this](const MoveFeedback &feedback) {
				    if (this->state != State::AIMING1)
					    return;

				    if (feedback.state == MoveFeedback::State::FAIL) {
					    action_server.setAborted(
					        arm_controller_srvs::GraspPlaceResult(),
					        "Failed to aim");
					    this->state = State::IDLE;

				    } else if (feedback.state == MoveFeedback::State::SUCCESS) {
					    observe_retries = 0;
					    this->state = State::OBSERVING2;
				    }
			    });
		}

	} break;

	case State::OBSERVING2: {

		auto task_opt = get_task_details(msg);
		if (task_opt.has_value()) {
			this->task = *task_opt;

		} else {
			observe_retries++;
			if (observe_retries <= 10) {
				return;
			} else {
				ROS_WARN("No marker detected in Observing #2 stage, using old "
				         "information");
			}
		}

		arm_target_pub.publish(task.target);

		this->state = State::AIMING2;

		auto [goal_x, goal_y, goal_yaw] =
		    compute_goal(task.target, task.ideal_seperation);

		chassis_move.execute(
		    goal_x, goal_y, goal_yaw, task.target.header.stamp,
		    [this](const MoveFeedback &feedback) {
			    if (this->state != State::AIMING2)
				    return;

			    if (feedback.state == MoveFeedback::State::FAIL) {
				    action_server.setAborted(
				        arm_controller_srvs::GraspPlaceResult(),
				        "Failed to aim");
				    this->state = State::IDLE;

			    } else if (feedback.state == MoveFeedback::State::SUCCESS) {
				    aiming_done();
			    }
		    });

	} break;

	default:
		break;
	}
}

void GraspPlace::aiming_done() {
	this->state = State::ACTING;

	geometry_msgs::PoseStamped target_relative =
	    tf_buffer.transform(task.target, "arm_link", ros::Time(0), "odom");

	double arm_x = target_relative.pose.position.x;
	double arm_y = task.arm_y;
	if (arm_x < task.min_arm_x) {
		arm_x = *task.min_arm_x;
	}

	arm.move_to(arm_x, arm_y);

	async_wait.wait(ros::Duration(1), [this] {
		if (task.pick) {
			arm.close_gripper();
		} else {
			arm.open_gripper();
		}

		async_wait.wait(ros::Duration(1), [this] {
			arm.reset_position();
			arm.reset_position();
			this->state = State::MOVING_BACK;

			bool skip_moving_back =
			    ros::param::param("~skip_moving_back", true);
			if (skip_moving_back) {
				this->state = State::IDLE;
				action_server.setSucceeded();

			} else {
				chassis_move.execute(
				    -task.back_distance, 0, 0, ros::Time(0),
				    [this](const MoveFeedback &feedback) {
					    if (feedback.state == MoveFeedback::State::SUCCESS) {
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
			}
		});
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
			offset_pose(pose, {-0.045 / 2 + 0.010, 0, 0});
			offset_pose(pose, {0.125, 0, 0});
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
		t.back_distance = 0.10;
		t.ideal_seperation = 0.20;
		t.ideal_observing_distance = 0.30;
		t.arm_y = -0.02;
		t.min_arm_x = 0.181;
		if (action == Action::GRASP_ORE) {
			t.pick = true;
		} else if (action == Action::STACK_ORE) {
			t.pick = false;
		}

		return t;

	} else if (action == Action::PLACE_ORE) {
		// exchange marker
		for (const auto &marker : msg->markers) {
			if (marker.id == this->marker_id) {

				auto pose = get_marker_pose(marker);
				offset_pose(pose, {-0.080, 0, 0});
				offset_pose(pose, {0.125, 0, 0});

				TaskDetails t;
				t.target = pose;
				t.back_distance = 0.10;
				t.ideal_seperation = 0.20;
				t.pick = false;
				t.ideal_observing_distance = 0.30;
				t.min_arm_x = 0.181;
				t.arm_y = 0.12;
				return t;
			}
		}
		return std::nullopt;
	} else {
		return std::nullopt;
	}
}
