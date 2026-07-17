#ifndef LIMBNAV_FIT_HPP
#define LIMBNAV_FIT_HPP

#include "limbnav/limb.hpp"
#include "limbnav/types.hpp"

#include <vector>

namespace limbnav {
namespace fit {

// P1 implements RANSAC + direct LS ellipse fitting (§6.1 stage 4). Until
// then this returns a default (invalid) result and appends
// flags::kStageNotImplemented + ":fit" — callers must REFUSE.
struct FitResult {
  EllipseParams ellipse;
  double rms_px = 0.0;
  int n_inliers = 0;
  bool valid = false;
};

FitResult robust_ellipse(const std::vector<limb::LimbPoint> &points,
                         const FitConfig &cfg,
                         std::vector<std::string> &flags);

} // namespace fit
} // namespace limbnav

#endif // LIMBNAV_FIT_HPP
