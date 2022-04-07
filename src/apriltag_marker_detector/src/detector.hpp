#pragma once

#include <opencv2/core/mat.hpp>
#include <thread>

namespace apriltag_marker_detector {

class DetectorConfig {
  public:
	// Tag family to use
	std::string family = "tag36h11";

	// Use this many CPU threads
	int nthreads = std::thread::hardware_concurrency();

	// Decimate input image by this factor
	double quad_decimate = 1.0;

	// Apply low-pass blur to input
	double quad_sigma = 0.0;

	// Spend more time trying to align edges of tags
	bool refine_edges = true;
};

class TagDetection {
  public:
	// Tag family
	std::string family;

	// The decoded ID of the tag
	int id;

	// How many error bits were corrected? Note: accepting large numbers of
	// corrected errors leads to greatly increased false positive rates.
	// NOTE: As of this implementation, the detector cannot detect tags with
	// a hamming distance greater than 2.
	int hamming;

	// A measure of the quality of the binary decoding process: the
	// average difference between the intensity of a data bit versus
	// the decision threshold. Higher numbers roughly indicate better
	// decodes. This is a reasonable measure of detection accuracy
	// only for very small tags-- not effective for larger tags (where
	// we could have sampled anywhere within a bit cell and still
	// gotten a good detection.)
	float decision_margin;

	// The 3x3 homography matrix describing the projection from an
	// "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
	// -1)) to pixels in the image.
	cv::Matx33d homography_matrix;

	// The center of the detection in image pixel coordinates.
	cv::Point2d center;

	// The corners of the tag in image pixel coordinates. These always
	// wrap counter-clock wise around the tag.
	std::array<cv::Point2d, 4> corners;
};

class Detector {
  public:
	Detector(const DetectorConfig &cfg = DetectorConfig());
	Detector(const Detector &) = delete;
	~Detector();

	std::vector<TagDetection> detect(const cv::Mat &grayscale_image);

  private:
	class Impl;
	std::unique_ptr<Impl> impl;
};

} // namespace apriltag_marker_detector
