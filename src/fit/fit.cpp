#include "limbnav/fit.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <random>

namespace limbnav {
namespace fit {

namespace {

EllipseParams from_rotated_rect(const cv::RotatedRect &rr) {
  EllipseParams e;
  const double w = rr.size.width / 2.0, h = rr.size.height / 2.0;
  if (w >= h) {
    e.a_px = w;
    e.b_px = h;
    e.phi_rad = rr.angle * M_PI / 180.0;
  } else {
    e.a_px = h;
    e.b_px = w;
    e.phi_rad = rr.angle * M_PI / 180.0 + M_PI / 2.0;
  }
  e.cx_px = rr.center.x;
  e.cy_px = rr.center.y;
  return e;
}

bool valid_ellipse(const EllipseParams &e) {
  return std::isfinite(e.a_px) && std::isfinite(e.b_px) &&
         std::isfinite(e.phi_rad) && std::isfinite(e.cx_px) &&
         std::isfinite(e.cy_px) && e.a_px > 1.0 && e.b_px > 1.0 &&
         e.a_px < 1e5 && e.a_px / e.b_px < 20.0;
}

// First-order (Sampson) distance from a point to the ellipse — a very good
// geometric-distance approximation near the curve, exact on it.
double sampson_distance(const EllipseParams &e, double x, double y) {
  const double c = std::cos(e.phi_rad), s = std::sin(e.phi_rad);
  const double dx = x - e.cx_px, dy = y - e.cy_px;
  const double xp = c * dx + s * dy;
  const double yp = -s * dx + c * dy;
  const double a2 = e.a_px * e.a_px, b2 = e.b_px * e.b_px;
  const double F = xp * xp / a2 + yp * yp / b2 - 1.0;
  const double gx = 2.0 * xp / a2 * c - 2.0 * yp / b2 * s;
  const double gy = 2.0 * xp / a2 * s + 2.0 * yp / b2 * c;
  const double g = std::hypot(gx, gy);
  return g > 1e-12 ? F / g : 1e9;
}

// Taubin circle fit (double precision, closed form).
bool taubin_circle(const std::vector<cv::Point2d> &pts, EllipseParams &e) {
  const size_t n = pts.size();
  if (n < 3) return false;
  double mx = 0.0, my = 0.0;
  for (const auto &p : pts) {
    mx += p.x;
    my += p.y;
  }
  mx /= n;
  my /= n;
  double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
  for (const auto &p : pts) {
    const double X = p.x - mx, Y = p.y - my, Z = X * X + Y * Y;
    Mxx += X * X;
    Myy += Y * Y;
    Mxy += X * Y;
    Mxz += X * Z;
    Myz += Y * Z;
    Mzz += Z * Z;
  }
  Mxx /= n;
  Myy /= n;
  Mxy /= n;
  Mxz /= n;
  Myz /= n;
  Mzz /= n;
  const double Mz = Mxx + Myy;
  const double Cov_xy = Mxx * Myy - Mxy * Mxy;
  const double Var_z = Mzz - Mz * Mz;
  const double A3 = 4.0 * Mz, A2 = -3.0 * Mz * Mz - Mzz;
  const double A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  const double A0 =
      Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) -
      Var_z * Cov_xy;
  double x = 0.0;
  for (int i = 0; i < 99; ++i) { // Newton on the characteristic poly
    const double f = A0 + x * (A1 + x * (A2 + x * A3));
    const double df = A1 + x * (2.0 * A2 + 3.0 * x * A3);
    if (std::fabs(df) < 1e-14) break;
    const double xn = x - f / df;
    if (!std::isfinite(xn) || std::fabs(xn - x) < 1e-12) {
      x = xn;
      break;
    }
    x = xn;
  }
  const double det = x * x - x * Mz + Cov_xy;
  if (std::fabs(det) < 1e-14) return false;
  const double cx = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
  const double cy = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;
  const double r = std::sqrt(cx * cx + cy * cy + Mz);
  e.a_px = e.b_px = r;
  e.phi_rad = 0.0;
  e.cx_px = cx + mx;
  e.cy_px = cy + my;
  return std::isfinite(r) && r > 1.0;
}

bool final_fit(const std::vector<cv::Point2d> &pts, const std::string &type,
               EllipseParams &e) {
  if (type == "taubin_circle") return taubin_circle(pts, e);
  if (pts.size() < 5) return false;
  std::vector<cv::Point2f> f;
  f.reserve(pts.size());
  for (const auto &p : pts) f.emplace_back(static_cast<float>(p.x),
                                           static_cast<float>(p.y));
  cv::RotatedRect rr;
  if (type == "fitEllipseAMS") {
    rr = cv::fitEllipseAMS(f);
  } else { // fitEllipseDirect (default)
    rr = cv::fitEllipseDirect(f);
  }
  e = from_rotated_rect(rr);
  return valid_ellipse(e);
}

double arc_span_deg(const EllipseParams &e,
                    const std::vector<cv::Point2d> &pts) {
  std::vector<double> ang;
  ang.reserve(pts.size());
  const double c = std::cos(e.phi_rad), s = std::sin(e.phi_rad);
  for (const auto &p : pts) {
    const double dx = p.x - e.cx_px, dy = p.y - e.cy_px;
    ang.push_back(std::atan2((-s * dx + c * dy) / e.b_px,
                             (c * dx + s * dy) / e.a_px));
  }
  std::sort(ang.begin(), ang.end());
  double max_gap = 2.0 * M_PI - (ang.back() - ang.front());
  for (size_t i = 1; i < ang.size(); ++i)
    max_gap = std::max(max_gap, ang[i] - ang[i - 1]);
  return (2.0 * M_PI - max_gap) * 180.0 / M_PI;
}

} // namespace

FitResult robust_ellipse(const std::vector<limb::LimbPoint> &points,
                         const FitConfig &cfg,
                         std::vector<std::string> &flags) {
  FitResult out;
  std::vector<cv::Point2d> P;
  P.reserve(points.size());
  for (const auto &p : points)
    if (p.is_limb) P.emplace_back(p.x_px, p.y_px);
  if (P.size() < 8) {
    flags.push_back("TOO_FEW_INLIERS");
    return out;
  }

  // RANSAC over minimal 6-point ellipse hypotheses (deterministic seed, §8).
  std::mt19937 rng(0xC0FFEEu);
  std::uniform_int_distribution<size_t> pick(0, P.size() - 1);
  std::vector<char> best_mask(P.size(), 0);
  int best_count = -1;
  for (int it = 0; it < cfg.ransac_iters; ++it) {
    size_t idx[6];
    for (int k = 0; k < 6; ++k) {
      bool fresh = true;
      do {
        idx[k] = pick(rng);
        fresh = true;
        for (int j = 0; j < k; ++j) fresh = fresh && idx[j] != idx[k];
      } while (!fresh);
    }
    std::vector<cv::Point2f> sample6;
    for (size_t k : idx)
      sample6.emplace_back(static_cast<float>(P[k].x),
                           static_cast<float>(P[k].y));
    cv::RotatedRect rr;
    try {
      rr = cv::fitEllipseDirect(sample6);
    } catch (const cv::Exception &) {
      continue;
    }
    const EllipseParams h = from_rotated_rect(rr);
    if (!valid_ellipse(h)) continue;
    int count = 0;
    std::vector<char> mask(P.size(), 0);
    for (size_t i = 0; i < P.size(); ++i) {
      if (std::fabs(sampson_distance(h, P[i].x, P[i].y)) <= cfg.ransac_tol_px) {
        mask[i] = 1;
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_mask = mask;
    }
  }
  if (best_count < 8) {
    flags.push_back("RANSAC_NO_CONSENSUS");
    return out;
  }

  // Final LS fit on the consensus set, re-selecting inliers once.
  for (int round = 0; round < 2; ++round) {
    std::vector<cv::Point2d> in;
    for (size_t i = 0; i < P.size(); ++i)
      if (best_mask[i]) in.push_back(P[i]);
    EllipseParams e;
    if (!final_fit(in, cfg.type, e)) {
      flags.push_back("FINAL_FIT_FAILED");
      return out;
    }
    out.ellipse = e;
    for (size_t i = 0; i < P.size(); ++i)
      best_mask[i] =
          std::fabs(sampson_distance(e, P[i].x, P[i].y)) <= cfg.ransac_tol_px
              ? 1
              : 0;
  }

  std::vector<cv::Point2d> in;
  for (size_t i = 0; i < P.size(); ++i)
    if (best_mask[i]) in.push_back(P[i]);
  if (in.size() < 8) {
    flags.push_back("TOO_FEW_INLIERS");
    return out;
  }
  double ss = 0.0;
  for (const auto &p : in) {
    const double d = sampson_distance(out.ellipse, p.x, p.y);
    ss += d * d;
  }
  out.rms_px = std::sqrt(ss / in.size());
  out.n_inliers = static_cast<int>(in.size());

  const double span = arc_span_deg(out.ellipse, in);
  if (span < cfg.min_arc_deg) {
    flags.push_back("ARC_TOO_SHORT");
    return out;
  }
  if (span < 180.0) flags.push_back("SHORT_ARC");
  out.valid = true;
  return out;
}

} // namespace fit
} // namespace limbnav
