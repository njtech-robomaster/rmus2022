#include "detector.hpp"
#include "tag_family.hpp"
#include <apriltag/apriltag.h>

namespace apriltag_marker_detector {

class Detector::Impl {
  public:
	apriltag_detector_t *td;
	std::unique_ptr<TagFamily> tf;

	Impl(const DetectorConfig &cfg) {
		auto created_family = create_tag_family(cfg.family);
		if (!created_family.has_value()) {
			throw std::invalid_argument("Unknown tag family");
		}
		tf = std::move(*created_family);

		td = apriltag_detector_create();
		td->quad_decimate = cfg.quad_decimate;
		td->quad_sigma = cfg.quad_sigma;
		td->nthreads = cfg.nthreads;
		td->debug = 0;
		td->refine_edges = cfg.refine_edges;

		apriltag_detector_add_family_bits(td, tf->impl, tf->bits_corrected);
	}

	Impl(const Impl &) = delete;

	~Impl() {
		apriltag_detector_destroy(td);
	}
};

Detector::Detector(const DetectorConfig &cfg)
    : impl(std::make_unique<Impl>(cfg)){};

Detector::~Detector() = default;

std::vector<TagDetection> Detector::detect(const cv::Mat &gray) {
	if (gray.type() != CV_8UC1) {
		throw std::invalid_argument("Input image type must be CV_8UC1");
	}

	image_u8_t im = {.width = gray.cols,
	                 .height = gray.rows,
	                 .stride = gray.cols,
	                 .buf = gray.data};
	zarray_t *detections = apriltag_detector_detect(impl->td, &im);
	int detections_size = zarray_size(detections);

	std::vector<TagDetection> results(detections_size);
	for (int i = 0; i < detections_size; i++) {
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		TagDetection &result = results[i];

		result.family = impl->tf->impl->name;
		result.id = det->id;
		result.hamming = det->hamming;
		result.decision_margin = det->decision_margin;
		memcpy(result.homography_matrix.val, det->H->data, 9 * sizeof(double));
		result.center.x = det->c[0];
		result.center.y = det->c[1];
		result.corners[0] = {det->p[0][0], det->p[0][1]};
		result.corners[1] = {det->p[1][0], det->p[1][1]};
		result.corners[2] = {det->p[2][0], det->p[2][1]};
		result.corners[3] = {det->p[3][0], det->p[3][1]};
	}

	apriltag_detections_destroy(detections);
	return results;
}

} // namespace apriltag_marker_detector
