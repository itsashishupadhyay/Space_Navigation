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
  double radius_px = 0.0;  // distance from the ray origin (seed center)
  double gradient = 0.0;   // |d I/d s| at the localized position
  double ray_angle_rad = 0.0;
  bool is_limb = false;    // false ⇒ classified terminator/ambiguous/invalid
};

// Stage-2 coarse disk (§6.1): threshold/edge → largest contour → seed
// geometry for the radial rays. Center/axes are integer-grade estimates —
// they are never the measurement.
struct CoarseDisk {
  bool ok = false;
  double cx_px = 0.0;
  double cy_px = 0.0;
  double r_px = 0.0;             // min-enclosing-circle radius
  EllipseParams ellipse;         // contour fitEllipse when ≥ 5 points
  bool has_ellipse = false;
};

CoarseDisk coarse_disk(const cv::Mat &preprocessed, const CoarseConfig &cfg,
                       std::vector<std::string> &flags);

// Per-ray radial profile sampling + sub-pixel edge localization
// (parabola_grad | zernike | erf_fit), then limb/terminator classification
// against the Sun hint and gradient statistics. `seed` supplies the per-ray
// expected radius: the coarse ellipse when available, else the circle.
std::vector<LimbPoint> extract(const cv::Mat &preprocessed,
                               const CoarseDisk &seed, const LimbConfig &cfg,
                               const ClassifyConfig &classify,
                               const SunHint &sun,
                               std::vector<std::string> &flags,
                               double search_scale = 1.0);

// --- exposed for unit testing (bias budgets are gate-critical, §5.2) ---

// Sub-pixel edge on a 1-D profile I(s), s increasing outward (bright→dark).
// Returns s* or a negative value on failure. `grad_out` gets |dI/ds| there.
double localize_parabola(const std::vector<double> &s,
                         const std::vector<double> &I, double &grad_out);

// Physical limb-profile model, fitted by variable projection: the pre-blur
// profile (H + Cs·√d + M·d)·1_{d>0} (d = s0−s, distance inside the limb)
// convolved exactly/numerically with a Gaussian PSF of width σ. The √d term
// is Lambert grazing shading; H is the true step (high-phase or
// Lommel-Seeliger limbs); M absorbs slow interior trends. Linear in
// (B, H, Cs, M) — solved in closed form — with only (s0, σ) nonlinear.
struct ErfFitResult {
  bool ok = false;
  double s0 = 0.0;
  double sigma = 0.0;
  double H = 0.0;      // step height at the cutoff
  double Csqrt = 0.0;  // sqrt (Lambert) coefficient
  double M = 0.0;      // interior linear slope
  double B = 0.0;      // background (space) level
  double rms = 0.0;
};
// R_lambert > 0 replaces the plain √d basis with the exact sphere-Lambert
// grazing profile for a body of that pixel radius (used by the consensus
// stage, where the radius is known from the running ellipse).
// force_family: -1 auto (BIC), 0 step-only, 1 sqrt-only, 2 both. Freezing
// the family across the consensus rounds of one image prevents family
// flips on smeared pooled profiles (a misaligned reference ellipse blurs
// the √ kink until both families fit equally well — the s0 estimates then
// differ by >1 px).
ErfFitResult localize_erf(const std::vector<double> &s,
                          const std::vector<double> &I, double s0_init,
                          double sigma_init, double R_lambert = 0.0,
                          int force_family = -1);

// Ghosal–Mehrotra Zernike-moment edge operator on a 7×7 patch around
// (x, y); returns sub-pixel edge point in image coordinates.
bool localize_zernike(const cv::Mat &img32f, double x, double y,
                      double &edge_x, double &edge_y);

// Joint consensus refinement (§6.1 pass 3). Every ray of an image crosses
// the same physical limb, so:
//  1. the profile family/σ/shape are fitted ONCE on the pooled,
//     contrast-normalized ray windows (effective SNR ≈ ray-SNR·√n_rays);
//  2. each ray is re-localized with a single offset DOF against the frozen
//     shape (per-ray mix of step/√ columns — the ratio varies along a
//     crescent limb);
//  3. the ellipse-parameter corrections (δa, δb, δφ, δcx, δcy) are solved
//     as one Huber-weighted linear LS of those offsets on the analytic
//     perturbation basis ∂r/∂(a,b,φ,cx,cy) — bounded influence instead of
//     inlier selection, because selecting inliers from a skewed offset
//     distribution re-biases the scale;
//  4. the common-mode radial term is overridden by the pooled fit's s0
//     (validated unbiased), removing the per-ray nonlinear-bias residue.
// Iterated K times by the caller via repeated invocation.
struct JointRefineResult {
  bool ok = false;
  EllipseParams ellipse;
  int n_rays = 0;
  double residual_rms_px = 0.0; // Huber-weighted per-ray offset residual
  int family = -1;              // pooled profile family used (0/1/2)
  double arc_deg = 0.0;         // angular coverage of the rays actually used
};
JointRefineResult joint_refine(const cv::Mat &img32f,
                               const EllipseParams &ellipse,
                               const LimbConfig &cfg,
                               const ClassifyConfig &classify,
                               const SunHint &sun,
                               std::vector<std::string> &flags,
                               int force_family = -1);

// Pooled-profile radial scale calibration: fit the physical limb profile to
// ALL contrast-normalized ray windows sampled around `ellipse` (effective
// SNR ≈ ray-SNR·√n_rays; validated unbiased in 1-D with the exact-Lambert
// basis). delta_px is the signed common-mode offset of the true limb
// relative to the ellipse — robust point fits own the SHAPE, this owns the
// SCALE (robust inlier selection on skewed per-ray errors is biased; the
// pooled estimator is not).
struct ScaleCalib {
  bool ok = false;
  double delta_px = 0.0;
  double sigma_px = 0.0; // pooled PSF-width estimate
  int n_rays = 0;
  int family = -1; // profile family the pooled fit chose (0/1/2)
};
ScaleCalib pooled_scale_calibration(const cv::Mat &img32f,
                                    const EllipseParams &ellipse,
                                    const LimbConfig &cfg,
                                    const ClassifyConfig &classify,
                                    const SunHint &sun,
                                    int force_family = -1);

} // namespace limb
} // namespace limbnav

#endif // LIMBNAV_LIMB_HPP
