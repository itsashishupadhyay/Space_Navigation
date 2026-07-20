#include "limbnav/measure.hpp"

#include "limbnav/fit.hpp"
#include "limbnav/limb.hpp"
#include "limbnav/preprocess.hpp"

#include <algorithm>
#include <cmath>

namespace limbnav {

namespace {

// Re-seed the ray extraction from a fitted ellipse (pass 2).
limb::CoarseDisk seed_from_ellipse(const EllipseParams &e) {
  limb::CoarseDisk s;
  s.ok = true;
  s.cx_px = e.cx_px;
  s.cy_px = e.cy_px;
  s.r_px = e.a_px;
  s.ellipse = e;
  s.has_ellipse = true;
  return s;
}

bool has_flag(const std::vector<std::string> &flags, const char *f) {
  return std::find(flags.begin(), flags.end(), f) != flags.end();
}

} // namespace

LimbMeasurement measure(const cv::Mat &gray, const PipelineConfig &cfg,
                        const SunHint &sun, const std::string &image_id) {
  LimbMeasurement m;
  m.image_id = image_id;
  m.verdict = Verdict::REFUSED;

  if (gray.empty() || gray.channels() != 1) {
    m.flags.push_back("IMAGE_INVALID");
    return m;
  }

  cv::Mat pre;
  try {
    pre = preprocess::apply(gray, cfg.denoise, cfg.contrast);
  } catch (const std::exception &) {
    m.flags.push_back("PREPROCESS_FAILED");
    return m;
  }

  const limb::CoarseDisk coarse = limb::coarse_disk(pre, cfg.coarse, m.flags);
  if (!coarse.ok) return m;

  // Pass 1: wide search (coarse radii can be ~15% low on limb-darkened
  // disks), robust fit.
  const double wide_scale =
      std::max(1.0, 0.25 * coarse.r_px / std::max(cfg.limb.search_px, 1.0));
  std::vector<limb::LimbPoint> pts1 = limb::extract(
      pre, coarse, cfg.limb, cfg.classify, sun, m.flags, wide_scale);
  int n1 = 0;
  for (const auto &p : pts1) n1 += p.is_limb ? 1 : 0;
  if (n1 < 8) return m;
  fit::FitResult f1 = fit::robust_ellipse(pts1, cfg.fit, m.flags);
  if (!f1.valid) return m;

  // Pass 2: narrow search re-seeded from the pass-1 ellipse.
  std::vector<limb::LimbPoint> pts2 =
      limb::extract(pre, seed_from_ellipse(f1.ellipse), cfg.limb, cfg.classify,
                    sun, m.flags, 1.0);
  int n2 = 0;
  for (const auto &p : pts2) n2 += p.is_limb ? 1 : 0;
  fit::FitResult f2;
  if (n2 >= 8) f2 = fit::robust_ellipse(pts2, cfg.fit, m.flags);

  fit::FitResult best = f2.valid ? f2 : f1;
  int n_best = f2.valid ? n2 : n1;
  if (!f2.valid) m.flags.push_back("PASS2_FAILED");

  // Pass 3 (§6.1): joint consensus refinement — pooled profile (family
  // frozen after the first round: refitting it on smeared pooled data lets
  // it flip, and step↔√ differ by >1 px in implied s0), per-ray offsets,
  // Huber LS on the ellipse-perturbation basis, pooled common mode.
  bool joint_ok = false;
  int family = -1;
  double n_joint = 0.0, rms_joint = 0.0, arc_joint = 360.0;
  for (int round = 0; round < 4; ++round) {
    const limb::JointRefineResult jr = limb::joint_refine(
        pre, best.ellipse, cfg.limb, cfg.classify, sun, m.flags, family);
    if (!jr.ok) break;
    const double da = jr.ellipse.a_px - best.ellipse.a_px;
    const double db = jr.ellipse.b_px - best.ellipse.b_px;
    best.ellipse = jr.ellipse;
    family = jr.family; // freeze after round 0
    n_joint = jr.n_rays;
    rms_joint = jr.residual_rms_px;
    arc_joint = jr.arc_deg;
    joint_ok = true;
    if (std::fabs(da) < 5e-4 && std::fabs(db) < 5e-4) break;
  }
  if (joint_ok) {
    m.flags.push_back("JOINT_REFINED");
    n_best = static_cast<int>(n_joint);
    best.rms_px = rms_joint;
    best.n_inliers = static_cast<int>(n_joint);
  } else {
    m.flags.push_back("JOINT_UNAVAILABLE");
  }

  m.n_limb_points = n_best;
  m.n_inliers = best.n_inliers;
  m.ellipse = best.ellipse;
  m.fit_rms_px = best.rms_px;
  if (joint_ok) {
    // Verdict from the FINAL stage's own coverage — SHORT_ARC flags from
    // the seeding passes describe superseded geometry.
    m.verdict = arc_joint < 180.0 ? Verdict::DEGRADED : Verdict::NOMINAL;
    if (arc_joint < 180.0) m.flags.push_back("JOINT_SHORT_ARC");
  } else if (f2.valid) {
    m.verdict = has_flag(m.flags, "SHORT_ARC") ? Verdict::DEGRADED
                                               : Verdict::NOMINAL;
  } else {
    // Pass-2 failure after a valid pass-1 is suspicious: degrade, don't
    // pretend (§4.7).
    m.verdict = Verdict::DEGRADED;
  }
  return m;
}

} // namespace limbnav
