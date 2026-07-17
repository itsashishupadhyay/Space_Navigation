#ifndef LIMBNAV_LIMB_HPP
#define LIMBNAV_LIMB_HPP

#include "limbnav/types.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace limbnav {
namespace limb {

// One sub-pixel limb candidate from a radial ray (§6.1 stage 3).
struct LimbPoint {
  double x_px = 0.0;
  double y_px = 0.0;
  double gradient = 0.0;   // edge strength at the localized position
  double ray_angle_rad = 0.0;
  bool is_limb = false;    // false ⇒ classified terminator/ambiguous
};

// P1 implements the radial-ray sub-pixel localizers (parabola_grad,
// zernike, erf_fit). Until then this returns an empty vector and appends
// flags::kStageNotImplemented + ":limb" to `flags` — callers must REFUSE.
std::vector<LimbPoint> extract(const cv::Mat &preprocessed,
                               const cv::Point2d &seed_center,
                               double seed_radius_px, const LimbConfig &cfg,
                               const ClassifyConfig &classify,
                               std::vector<std::string> &flags);

} // namespace limb
} // namespace limbnav

#endif // LIMBNAV_LIMB_HPP
