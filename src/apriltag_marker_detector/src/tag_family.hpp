#pragma once

extern "C" {
#include "apriltag/apriltag.h"
}
#include <memory>
#include <optional>

namespace apriltag_marker_detector {

class TagFamily {
public:
  apriltag_family_t *impl;
  int bits_corrected;

  TagFamily(apriltag_family_t *(*create_func)(void),
            void (*destroy_func)(apriltag_family_t *), int bits_corrected) {
    this->destroy_func = destroy_func;
    this->impl = create_func();
    this->bits_corrected = bits_corrected;
  }

  TagFamily(const TagFamily &) = delete;

  ~TagFamily() {
    destroy_func(impl);
    impl = nullptr;
  }

private:
  void (*destroy_func)(apriltag_family_t *);
};

std::optional<std::unique_ptr<TagFamily>>
create_tag_family(const std::string &name);

} // namespace apriltag_marker_detector
