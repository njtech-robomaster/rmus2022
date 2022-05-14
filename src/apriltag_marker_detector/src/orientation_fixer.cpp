#include "orientation_fixer.hpp"

namespace apriltag_marker_detector {

static double angle(const cv::Vec2d &v1, const cv::Vec2d &v2) {
	double cosAngle = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
	if (cosAngle > 1.0)
		return 0.0;
	else if (cosAngle < -1.0)
		return M_PI;
	return std::acos(cosAngle);
}

static double marker_angle_topleft(const TagDetection &marker, int corner) {
	cv::Vec2d v1 = marker.center - marker.corners[corner];
	cv::Vec2d v2{1, 1};
	return std::abs(angle(v1, v2));
}

void fix_marker_orientation(TagDetection &marker) {
	if (marker.family == "tagRmus2022" && (marker.id == 6 || marker.id == 7)) {
		// O, X
		int min_idx = 0;
		double min_angle = marker_angle_topleft(marker, 0);
		for (int i = 1; i < 4; i++) {
			double angle = marker_angle_topleft(marker, i);
			if (angle < min_angle) {
				min_angle = angle;
				min_idx = i;
			}
		}
		if (min_idx != 3) {
			auto origin = marker.corners;
			for (int i = 0; i < 4; i++) {
				marker.corners[i] = origin[(i + min_idx + 1) % 4];
			}
		}
	}
}

} // namespace apriltag_marker_detector
