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

double tag_size;
bool enable_pose_array;
std::unique_ptr<Detector> detector;
std::unique_ptr<Executor> executor;
ros::Publisher markers_pub;
ros::Publisher pose_array_pub;
image_transport::CameraSubscriber color_image_sub;

void publish_pose_array(const apriltag_msgs::ApriltagMarkerArray &markers,
                        const std_msgs::Header &header) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header = header;
  for (const auto &marker : markers.markers) {
    pose_array.poses.push_back(marker.pose);
  }
  pose_array_pub.publish(pose_array);
}

const std::string window_detection_name = "Object Detection";

void process_frame(const sensor_msgs::ImageConstPtr &image_msg,
                   const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
  auto cv_image = cv_bridge::cvtColor(cv_bridge::toCvShare(image_msg),
                                      sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_image->image;
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat split[3];
  cv::split(hsv, split);
  cv::Mat grayscale;
  grayscale = split[1];
  cv::bitwise_not(grayscale, grayscale);

  cv::imshow(window_detection_name, grayscale);
  cv::waitKey(1);

  auto detections = detector->detect(grayscale);

  auto k = camera_info_msg->K;
  cv::Mat camera_matrix(3, 3, CV_64FC1, k.data());

  std::vector<cv::Point3d> object_points = {
      {-tag_size / 2, tag_size / 2, 0},
      {tag_size / 2, tag_size / 2, 0},
      {tag_size / 2, -tag_size / 2, 0},
      {-tag_size / 2, -tag_size / 2, 0},
  };

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

    marker.pose.position.x = tvec[0];
    marker.pose.position.y = tvec[1];
    marker.pose.position.z = tvec[2];
    marker.pose.orientation = tf2::toMsg(Eigen::Quaterniond(rotation_matrix));

    markers.markers.push_back(marker);
  }

  markers_pub.publish(markers);
  if (enable_pose_array) {
    publish_pose_array(markers, image_msg->header);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "apriltag_marker_detector");

  tag_size = ros::param::param("~tag_size", 0.050);
  enable_pose_array = ros::param::param("~publish_pose_array", true);

  DetectorConfig cfg;
  cfg.family = ros::param::param("~family", std::string("tagRmus2022"));
  cfg.nthreads = ros::param::param(
      "~nthreads", static_cast<int>(std::thread::hardware_concurrency()));
  detector = std::make_unique<Detector>(cfg);

  executor = std::make_unique<Executor>(&process_frame);

  ros::NodeHandle nh;
  markers_pub = nh.advertise<apriltag_msgs::ApriltagMarkerArray>("markers", 10);
  if (enable_pose_array) {
    pose_array_pub =
        nh.advertise<geometry_msgs::PoseArray>("markers/pose_array", 10);
  }
  image_transport::ImageTransport it(nh);
  color_image_sub = it.subscribeCamera(
      "camera/color/image_raw", 1,
      [&](const sensor_msgs::ImageConstPtr &image_msg,
          const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
        executor->feed(image_msg, camera_info_msg);
      });

  ros::spin();
}
