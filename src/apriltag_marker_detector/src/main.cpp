#include "async_executor.hpp"
#include "detector.hpp"
#include <apriltag_msgs/ApriltagMarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <thread>

#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/core/eigen.hpp>

using namespace apriltag_marker_detector;

using Executor =
    AsyncExecutor<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>;

const std::string preprocess_window_name = "Marker Preprocess";
const std::string result_window_name = "Marker Detections";
const std::array<std::string, 8> tag_id_name_mapping{"1", "2", "3", "4",
                                                     "5", "B", "O", "X"};

double get_tag_size(int id) {
	switch (id) {
	case 0: // 1
	case 1: // 2
	case 2: // 3
	case 3: // 4
	case 4: // 5
		return 0.040;

	case 5: // B
	case 6: // O
	case 7: // X
		return 0.050;
	default:
		ROS_ERROR_STREAM("Unknown tag id " << id);
		return 1;
	}
}

bool detection_only_mode;
bool enable_pose_array;
bool preview_preprocess;
bool preview_results;
bool needs_wait_key;
std::unique_ptr<Detector> detector;
std::unique_ptr<Executor> executor;
ros::Publisher markers_pub;
ros::Publisher pose_array_pub;
image_transport::CameraSubscriber color_camera_sub;
image_transport::Subscriber color_image_sub;

void publish_pose_array(const apriltag_msgs::ApriltagMarkerArray &markers,
                        const std_msgs::Header &header) {
	geometry_msgs::PoseArray pose_array;
	pose_array.header = header;
	for (const auto &marker : markers.markers) {
		pose_array.poses.push_back(marker.pose);
	}
	pose_array_pub.publish(pose_array);
}

cv::Mat preprocess(const cv::Mat &image) {
	cv::Mat hsv;
	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	cv::Mat nonred;
	cv::inRange(hsv, cv::Scalar(6, 0, 0), cv::Scalar(176, 255, 255), nonred);

	cv::Mat split[3];
	cv::split(hsv, split);
	cv::Mat grayscale;
	grayscale = split[1];
	// grayscale.setTo(0, nonred);
	cv::bitwise_not(grayscale, grayscale);

	if (preview_preprocess) {
		cv::imshow(preprocess_window_name, grayscale);
	}

	return grayscale;
}

void show_results(const cv::Mat &image,
                  const std::vector<TagDetection> &detections) {
	cv::Mat display;
	image.copyTo(display);
	for (const auto &detection : detections) {
		const auto &corners = detection.corners;

		cv::line(display, corners[0], corners[1], {0, 0xff, 0}, 2);
		cv::line(display, corners[0], corners[3], {0, 0, 0xff}, 2);
		cv::line(display, corners[1], corners[2], {0xff, 0, 0}, 2);
		cv::line(display, corners[2], corners[3], {0xff, 0, 0}, 2);

		auto text = tag_id_name_mapping[detection.id];
		auto fontface = cv::FONT_HERSHEY_PLAIN;
		double fontscale = 1.5;
		int thickness = 2;
		int baseline;
		auto textsize =
		    cv::getTextSize(text, fontface, fontscale, thickness, &baseline);
		putText(display, text,
		        {static_cast<int>(detection.center.x) - textsize.width / 2,
		         static_cast<int>(detection.center.y) + textsize.height / 2},
		        fontface, fontscale, {0xff, 0x7f, 0}, thickness);
	}
	cv::imshow(result_window_name, display);
}

apriltag_msgs::ApriltagMarkerArray
solve_markers_pose(const sensor_msgs::ImageConstPtr &image_msg,
                   const sensor_msgs::CameraInfoConstPtr &camera_info_msg,
                   const std::vector<TagDetection> &detections) {
	auto k = camera_info_msg->K;
	cv::Mat camera_matrix(3, 3, CV_64FC1, k.data());

	apriltag_msgs::ApriltagMarkerArray markers;

	for (auto &detection : detections) {

		apriltag_msgs::ApriltagMarker marker;
		marker.header = image_msg->header;
		marker.tag_family = detection.family;
		marker.id = detection.id;

		std::vector<cv::Point2d> image_points = {
		    detection.corners[3], // top-left
		    detection.corners[2], // top-right
		    detection.corners[1], // bottom-right
		    detection.corners[0], // bottom-left
		};

		double tag_size = get_tag_size(marker.id);
		std::vector<cv::Point3d> object_points = {
		    {-tag_size / 2, tag_size / 2, 0},
		    {tag_size / 2, tag_size / 2, 0},
		    {tag_size / 2, -tag_size / 2, 0},
		    {-tag_size / 2, -tag_size / 2, 0},
		};

		cv::Vec3d rvec, tvec;
		if (!cv::solvePnP(object_points, image_points, camera_matrix,
		                  camera_info_msg->D, rvec, tvec, false,
		                  cv::SOLVEPNP_IPPE_SQUARE)) {
			continue;
		}

		cv::Matx33d cv_rotation_matrix;
		cv::Rodrigues(rvec, cv_rotation_matrix);
		Eigen::Matrix3d rotation_matrix;
		cv::cv2eigen(cv_rotation_matrix, rotation_matrix);

		Eigen::Matrix3d t;
		t << 0, 1, 0, 0, 0, 1, 1, 0, 0;
		rotation_matrix = rotation_matrix * t;

		marker.pose.position.x = tvec[0];
		marker.pose.position.y = tvec[1];
		marker.pose.position.z = tvec[2];
		marker.pose.orientation =
		    tf2::toMsg(Eigen::Quaterniond(rotation_matrix));

		markers.markers.push_back(marker);
	}
	return markers;
}

void process_frame(const sensor_msgs::ImageConstPtr &image_msg,
                   const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {

	auto cv_image = cv_bridge::cvtColor(cv_bridge::toCvShare(image_msg),
	                                    sensor_msgs::image_encodings::BGR8);
	auto grayscale = preprocess(cv_image->image);
	auto detections = detector->detect(grayscale);

	if (preview_results) {
		show_results(cv_image->image, detections);
	}

	if (!detection_only_mode) {
		auto markers =
		    solve_markers_pose(image_msg, camera_info_msg, detections);
		markers_pub.publish(markers);

		if (enable_pose_array) {
			publish_pose_array(markers, image_msg->header);
		}
	}

	if (needs_wait_key) {
		cv::waitKey(1);
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "apriltag_marker_detector");

	detection_only_mode = ros::param::param("~detection_only_mode", false);
	enable_pose_array = ros::param::param("~publish_pose_array", true);

	preview_preprocess = ros::param::param("~preview_preprocess", false);
	preview_results = ros::param::param("~preview_results", false);
	needs_wait_key = preview_preprocess || preview_results;

	DetectorConfig cfg;
	cfg.family = ros::param::param("~family", std::string("tagRmus2022"));
	cfg.nthreads = ros::param::param(
	    "~nthreads", static_cast<int>(std::thread::hardware_concurrency()));
	detector = std::make_unique<Detector>(cfg);

	executor = std::make_unique<Executor>(&process_frame);

	ros::NodeHandle nh;
	markers_pub =
	    nh.advertise<apriltag_msgs::ApriltagMarkerArray>("markers", 10);
	if (enable_pose_array) {
		pose_array_pub =
		    nh.advertise<geometry_msgs::PoseArray>("markers/pose_array", 10);
	}
	image_transport::ImageTransport it(nh);

	if (detection_only_mode) {
		color_image_sub =
		    it.subscribe("camera/color/image_raw", 1,
		                 [&](const sensor_msgs::ImageConstPtr &image_msg) {
			                 executor->feed(image_msg, nullptr);
		                 });
	} else {
		color_camera_sub = it.subscribeCamera(
		    "camera/color/image_raw", 1,
		    [&](const sensor_msgs::ImageConstPtr &image_msg,
		        const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
			    executor->feed(image_msg, camera_info_msg);
		    });
	}

	ros::spin();
}
