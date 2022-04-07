#include "tag_family.hpp"
#include <unordered_map>

extern "C" {
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagRmus2022.h>
#include <apriltag/tagRobomaster2021.h>
#include <apriltag/tagRobomaster2022.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
}

namespace apriltag_marker_detector {

static std::unordered_map<std::string,
                          std::tuple<apriltag_family_t *(*)(void),
                                     void (*)(apriltag_family_t *), int>>
    tag_families = {
        {"tag16h5", {&tag16h5_create, &tag16h5_destroy, 2}},
        {"tag25h9", {&tag25h9_create, &tag25h9_destroy, 2}},
        {"tag36h10", {&tag36h10_create, &tag36h10_destroy, 2}},
        {"tag36h11", {&tag36h11_create, &tag36h11_destroy, 2}},
        {"tagCircle21h7", {&tagCircle21h7_create, &tagCircle21h7_destroy, 2}},
        {"tagCircle49h12",
         {&tagCircle49h12_create, &tagCircle49h12_destroy, 2}},
        {"tagCustom48h12",
         {&tagCustom48h12_create, &tagCustom48h12_destroy, 2}},
        {"tagStandard41h12",
         {&tagStandard41h12_create, &tagStandard41h12_destroy, 2}},
        {"tagStandard52h13",
         {&tagStandard52h13_create, &tagStandard52h13_destroy, 2}},
        {"tagRobomaster2021",
         {&tagRobomaster2021_create, &tagRobomaster2021_destroy, 0}},
        {"tagRobomaster2022",
         {&tagRobomaster2022_create, &tagRobomaster2022_destroy, 0}},
        {"tagRmus2022", {&tagRmus2022_create, &tagRmus2022_destroy, 0}},
};

std::optional<std::unique_ptr<TagFamily>>
create_tag_family(const std::string &name) {
  auto entry = tag_families.find(name);
  if (entry == tag_families.end()) {
    return std::nullopt;
  }

  auto value = entry->second;
  return std::make_unique<TagFamily>(std::get<0>(value), std::get<1>(value),
                                     std::get<2>(value));
}

}; // namespace apriltag_marker_detector
