#ifndef LIMBNAV_TYPES_HPP
#define LIMBNAV_TYPES_HPP

#include <string>
#include <vector>

namespace limbnav {

// Honest-fail verdict, same trichotomy as the submodule's NavDecision.
enum class Verdict { NOMINAL, DEGRADED, REFUSED };

std::string to_string(Verdict v);

// Canonical flag strings (§5, §15). Flags are open-ended; these are the
// ones the pipeline and eval harness key on.
namespace flags {
inline constexpr const char *kStageNotImplemented = "STAGE_NOT_IMPLEMENTED";
inline constexpr const char *kPolarDegenerate = "POLAR_DEGENERATE";
inline constexpr const char *kTerminatorAmbiguous = "TERMINATOR_AMBIGUOUS";
inline constexpr const char *kRingOcclusion = "RING_OCCLUSION";
inline constexpr const char *kLimbSaturated = "LIMB_SATURATED";
inline constexpr const char *kGateNoDetection = "GATE_NO_DETECTION";
inline constexpr const char *kGateWrongBody = "GATE_WRONG_BODY";
} // namespace flags

// Verified instrument record, loaded from the submodule's
// artifacts/instruments.csv. Never hardcode camera numbers (§3).
struct InstrumentRecord {
  std::string mission;
  std::string instrument;
  int array_px_x = 0;
  int array_px_y = 0;
  double pixel_pitch_um = 0.0;
  double focal_length_mm = 0.0;
  double fov_deg_x = 0.0;
  double fov_deg_y = 0.0;
  double ifov_arcsec_per_px = 0.0; // as recorded; cross-check vs pitch/f
  std::string boresight_frame;
  std::string source_citation;
  std::string verified_at;
  bool verified = false; // true only if the row parsed cleanly
};

// Fitted ellipse in (distortion-corrected) pixel coordinates.
struct EllipseParams {
  double a_px = 0.0;   // semi-major
  double b_px = 0.0;   // semi-minor
  double phi_rad = 0.0; // major-axis angle in pixel coords
  double cx_px = 0.0;
  double cy_px = 0.0;
};

// Result of the full measurement pipeline for one image (§6.1 stage 5).
struct LimbMeasurement {
  std::string image_id;
  EllipseParams ellipse;
  double fit_rms_px = 0.0;
  int n_limb_points = 0;
  int n_inliers = 0;
  Verdict verdict = Verdict::REFUSED;
  std::vector<std::string> flags;
};

// ---- Pipeline configuration (Appendix B schema; searched by the loop) ----

struct DenoiseConfig {
  // one of: none | gaussian | median | bilateral | nlmeans
  std::string type = "none";
  double sigma = 1.0;      // gaussian
  int ksize = 3;           // median
  int d = 9;               // bilateral neighborhood
  double sigma_color = 30; // bilateral
  double sigma_space = 5;  // bilateral
  double h = 10.0;         // nlmeans strength
};

struct ContrastConfig {
  // one of: none | clahe   (⚠ nonlinear stretch can shift the limb — §6.1)
  std::string type = "none";
  double clip = 2.0;
  int grid = 8;
};

struct CoarseConfig {
  // one of: otsu | adaptive | canny | scharr
  std::string type = "otsu";
  double lo = 40.0; // canny
  double hi = 120.0;
};

struct LimbConfig {
  int rays = 720;
  // one of: parabola_grad | zernike | erf_fit
  std::string localizer = "erf_fit";
  double search_px = 12.0;
};

struct ClassifyConfig {
  bool use_sun_vector = true;
  double min_grad_ratio = 2.0;
};

struct FitConfig {
  // one of: fitEllipseDirect | fitEllipseAMS | taubin_circle
  std::string type = "fitEllipseDirect";
  int ransac_iters = 500;
  double ransac_tol_px = 0.75;
  double min_arc_deg = 100.0;
};

struct PipelineConfig {
  DenoiseConfig denoise;
  ContrastConfig contrast;
  CoarseConfig coarse;
  LimbConfig limb;
  ClassifyConfig classify;
  FitConfig fit;
  double gate_conf_min = 0.5;
};

} // namespace limbnav

#endif // LIMBNAV_TYPES_HPP
