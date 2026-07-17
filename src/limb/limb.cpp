#include "limbnav/limb.hpp"

namespace limbnav {
namespace limb {

std::vector<LimbPoint> extract(const cv::Mat & /*preprocessed*/,
                               const cv::Point2d & /*seed_center*/,
                               double /*seed_radius_px*/,
                               const LimbConfig & /*cfg*/,
                               const ClassifyConfig & /*classify*/,
                               std::vector<std::string> &flags) {
  // P1 (§16): radial rays + sub-pixel localizers land here after the G0
  // synthetic harness exists. Refusing loudly beats guessing quietly (§4.7).
  flags.push_back(std::string(flags::kStageNotImplemented) + ":limb");
  return {};
}

} // namespace limb
} // namespace limbnav
