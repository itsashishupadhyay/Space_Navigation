#include "limbnav/fit.hpp"

namespace limbnav {
namespace fit {

FitResult robust_ellipse(const std::vector<limb::LimbPoint> & /*points*/,
                         const FitConfig & /*cfg*/,
                         std::vector<std::string> &flags) {
  // P1 (§16): RANSAC + fitEllipseDirect/AMS land here after G0 exists.
  flags.push_back(std::string(flags::kStageNotImplemented) + ":fit");
  return {};
}

} // namespace fit
} // namespace limbnav
