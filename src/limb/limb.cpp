#include "limbnav/limb.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace limbnav {
namespace limb {

namespace {

// Bilinear sample on CV_32F; clamps to the border.
double sample(const cv::Mat &img, double x, double y) {
  const int w = img.cols, h = img.rows;
  x = std::min(std::max(x, 0.0), w - 1.001);
  y = std::min(std::max(y, 0.0), h - 1.001);
  const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
  const double fx = x - x0, fy = y - y0;
  const float *r0 = img.ptr<float>(y0);
  const float *r1 = img.ptr<float>(std::min(y0 + 1, h - 1));
  const int x1 = std::min(x0 + 1, w - 1);
  return (1 - fy) * ((1 - fx) * r0[x0] + fx * r0[x1]) +
         fy * ((1 - fx) * r1[x0] + fx * r1[x1]);
}

double normal_cdf(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }

// Solve the n×n system A·x = b in place (Gaussian elimination, partial
// pivoting). Returns false if singular. n ≤ 5 here.
bool solve_inplace(std::vector<std::vector<double>> &A, std::vector<double> &b) {
  const int n = static_cast<int>(b.size());
  for (int c = 0; c < n; ++c) {
    int piv = c;
    for (int r = c + 1; r < n; ++r) {
      if (std::fabs(A[r][c]) > std::fabs(A[piv][c])) piv = r;
    }
    if (std::fabs(A[piv][c]) < 1e-14) return false;
    std::swap(A[c], A[piv]);
    std::swap(b[c], b[piv]);
    for (int r = c + 1; r < n; ++r) {
      const double f = A[r][c] / A[c][c];
      for (int k = c; k < n; ++k) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int c = n - 1; c >= 0; --c) {
    for (int k = c + 1; k < n; ++k) b[c] -= A[c][k] * b[k];
    b[c] /= A[c][c];
  }
  return true;
}

// ---- Variable-projection limb-profile fit ----
//
// Pre-blur physical profile at distance d = s0 − s inside the limb:
//   I₀(d) = (H + Cs·√d + M·d) · 1_{d>0}
// observed through a Gaussian PSF of width σ. The step and linear columns
// blur in closed form via E[max(0, X+d)] for X ~ N(0,σ²); the √ column is
// blurred by discrete Gaussian quadrature (kink at d=0 has no elementary
// closed form). For fixed (s0, σ) the model is LINEAR in (B, H, Cs, M):
// solve that 4×4 LS exactly, and search only (s0, σ) with Nelder–Mead.
struct ProfileDesign {
  // columns evaluated for one (s0, σ) over the sample vector. When
  // R_lambert > 0 the √ column becomes the exact sphere-Lambert grazing
  // profile √(2d/R − (d/R)²) instead of plain √d — the plain form is ~1%
  // low at window depth, which an M ≥ 0 model can only absorb by biasing
  // s0 inward (measured −1.2 px at R=300, σ=2).
  std::vector<double> step, sqrtc, lin;
  void build(const std::vector<double> &s, double s0, double sigma,
             double R_lambert = 0.0) {
    const int n = static_cast<int>(s.size());
    step.assign(n, 0.0);
    sqrtc.assign(n, 0.0);
    lin.assign(n, 0.0);
    const double gnorm = 1.0 / (sigma * std::sqrt(2.0 * M_PI));
    for (int i = 0; i < n; ++i) {
      const double d = s0 - s[i];
      const double z = d / sigma;
      const double Phi = normal_cdf(z);
      const double phi = std::exp(-0.5 * z * z) / std::sqrt(2.0 * M_PI);
      step[i] = Phi;
      lin[i] = sigma * (z * Phi + phi); // exact E[max(0, X+d)]
      // ∫G_σ(t)·f(d−t)dt with t = d−u², f(x) = √x·(shape):
      // 2∫₀^∞ u²·shape·G_σ(d−u²)du — smooth integrand (the √ kink is
      // absorbed by the substitution), so a uniform grid converges cleanly
      // at any σ.
      const double U = std::sqrt(std::max(d, 0.0) + 6.0 * sigma);
      const int K = 48;
      const double du = U / K;
      double acc = 0.0;
      for (int j = 0; j < K; ++j) {
        const double u = (j + 0.5) * du;
        const double dd = u * u; // depth inside the cutoff
        const double x = d - dd;
        double shape = 1.0;
        if (R_lambert > 0.0) {
          shape = std::sqrt(std::max(0.0, 2.0 / R_lambert -
                                              dd / (R_lambert * R_lambert))) *
                  std::sqrt(R_lambert / 2.0); // normalized: →√d as R→∞
        }
        acc += u * u * shape * std::exp(-0.5 * x * x / (sigma * sigma));
      }
      sqrtc[i] = 2.0 * gnorm * acc * du;
    }
  }
};

// Least squares over the active column set; β entries for dropped columns
// are zero. Returns SSE (large on failure).
double profile_ls(const std::vector<double> &I, const ProfileDesign &D,
                  bool use_step, bool use_sqrt, bool use_lin, double beta[4]) {
  const int n = static_cast<int>(I.size());
  const std::vector<double> *cols[4] = {nullptr, &D.step, &D.sqrtc, &D.lin};
  bool active[4] = {true, use_step, use_sqrt, use_lin};
  std::vector<int> idx;
  for (int k = 0; k < 4; ++k)
    if (active[k]) idx.push_back(k);
  const int m = static_cast<int>(idx.size());
  std::vector<std::vector<double>> AtA(m, std::vector<double>(m, 0.0));
  std::vector<double> Atb(m, 0.0);
  auto col = [&](int k, int i) -> double {
    return k == 0 ? 1.0 : (*cols[k])[i];
  };
  for (int i = 0; i < n; ++i) {
    for (int a = 0; a < m; ++a) {
      const double va = col(idx[a], i);
      Atb[a] += va * I[i];
      for (int b = a; b < m; ++b) AtA[a][b] += va * col(idx[b], i);
    }
  }
  for (int a = 0; a < m; ++a)
    for (int b = 0; b < a; ++b) AtA[a][b] = AtA[b][a];
  std::vector<double> x = Atb;
  if (!solve_inplace(AtA, x)) return 1e30;
  for (int k = 0; k < 4; ++k) beta[k] = 0.0;
  for (int a = 0; a < m; ++a) beta[idx[a]] = x[a];
  double sse = 0.0;
  for (int i = 0; i < n; ++i) {
    double f = beta[0];
    for (int k = 1; k < 4; ++k)
      if (active[k]) f += beta[k] * col(k, i);
    const double r = I[i] - f;
    sse += r * r;
  }
  return sse;
}

// Physicality: H, Cs AND M must all be non-negative — near-limb interior
// brightness never decreases inward, and allowing M < 0 lets "√ − linear"
// mimic a shifted step (measured +0.6…1.5 px outward bias on noisy step
// rays). Best feasible subset within the allowed family.
double fit_family(const std::vector<double> &I, const ProfileDesign &D,
                  bool allow_step, bool allow_sqrt, double beta[4]) {
  double best = 1e30;
  for (int h = allow_step ? 1 : 0; h >= 0; --h) {
    for (int c = allow_sqrt ? 1 : 0; c >= 0; --c) {
      for (int m = 1; m >= 0; --m) {
        if (!h && !c && !m) continue; // background-only fits nothing
        double bb[4];
        const double sse = profile_ls(I, D, h != 0, c != 0, m != 0, bb);
        if (sse < best && bb[1] >= 0.0 && bb[2] >= 0.0 && bb[3] >= 0.0) {
          best = sse;
          std::copy(bb, bb + 4, beta);
        }
      }
    }
  }
  return best;
}

// Frozen-family variant: the named columns are REQUIRED (a frozen family
// that silently degenerates to a subset re-opens the family-flip bias this
// mechanism exists to prevent); only the nuisance M toggles.
double fit_family_required(const std::vector<double> &I,
                           const ProfileDesign &D, bool req_step,
                           bool req_sqrt, double beta[4]) {
  double best = 1e30;
  for (int m = 1; m >= 0; --m) {
    if (!req_step && !req_sqrt && !m) continue;
    double bb[4];
    const double sse = profile_ls(I, D, req_step, req_sqrt, m != 0, bb);
    const bool feasible =
        bb[3] >= 0.0 && (!req_step || bb[1] > 0.0) && (!req_sqrt || bb[2] > 0.0);
    if (sse < best && feasible) {
      best = sse;
      std::copy(bb, bb + 4, beta);
    }
  }
  return best;
}

// 7×7 Zernike masks over the unit disk (raw integrals — the shared
// normalization cancels in l = A20/A11′, verified analytically:
// A20_raw(l) ≡ l·A11_raw(l) for a step edge). Built once, 8×8 subsampled.
struct ZernikeMasks {
  static constexpr int N = 7;
  double m11r[N][N] = {}, m11i[N][N] = {}, m20[N][N] = {};
  ZernikeMasks() {
    const double R = N / 2.0; // patch radius in px
    const int ss = 8;
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < N; ++j) {
        for (int a = 0; a < ss; ++a) {
          for (int b = 0; b < ss; ++b) {
            const double x = (j - (N - 1) / 2.0 + (b + 0.5) / ss - 0.5) / R;
            const double y = (i - (N - 1) / 2.0 + (a + 0.5) / ss - 0.5) / R;
            if (x * x + y * y > 1.0) continue;
            const double w = 1.0 / (ss * ss);
            m11r[i][j] += w * x;
            m11i[i][j] += w * y;
            m20[i][j] += w * (2.0 * (x * x + y * y) - 1.0);
          }
        }
      }
    }
  }
};
const ZernikeMasks &zernike_masks() {
  static const ZernikeMasks m;
  return m;
}

} // namespace

double localize_parabola(const std::vector<double> &s,
                         const std::vector<double> &I, double &grad_out) {
  const int n = static_cast<int>(s.size());
  grad_out = 0.0;
  if (n < 5) return -1.0;
  // s increases outward, intensity drops bright→dark: look for the most
  // negative dI/ds.
  int m = -1;
  double best = 0.0;
  std::vector<double> g(n, 0.0);
  for (int i = 1; i + 1 < n; ++i) {
    g[i] = (I[i + 1] - I[i - 1]) / (s[i + 1] - s[i - 1]);
    if (g[i] < best) {
      best = g[i];
      m = i;
    }
  }
  if (m < 1 || best >= 0.0) return -1.0;
  const double gm1 = g[std::max(1, m - 1)], g0 = g[m],
               gp1 = g[std::min(n - 2, m + 1)];
  const double den = gm1 - 2.0 * g0 + gp1;
  double delta = 0.0;
  if (std::fabs(den) > 1e-12) {
    delta = 0.5 * (gm1 - gp1) / den;
    delta = std::min(std::max(delta, -1.0), 1.0);
  }
  const double h = s[1] - s[0];
  grad_out = std::fabs(best);
  return s[m] + delta * h;
}

ErfFitResult localize_erf(const std::vector<double> &s,
                          const std::vector<double> &I, double s0_init,
                          double sigma_init, double R_lambert,
                          int force_family) {
  ErfFitResult out;
  const int n = static_cast<int>(s.size());
  if (n < 8) return out;

  const bool forced = force_family >= 0 && force_family <= 2;
  const bool f_step = force_family != 1; // required when forced
  const bool f_sqrt = force_family != 0;
  ProfileDesign D;
  double beta_scratch[4];
  auto objective = [&](double s0, double log_sigma, double beta[4]) {
    const double sigma = std::min(std::max(std::exp(log_sigma), 0.25), 8.0);
    D.build(s, s0, sigma, R_lambert);
    return forced ? fit_family_required(I, D, f_step, f_sqrt, beta)
                  : fit_family(I, D, true, true, beta);
  };

  // Nelder–Mead on (s0, log σ) — deterministic, derivative-free (the sqrt
  // quadrature makes analytic derivatives unpleasant).
  struct Vertex {
    double x[2];
    double f;
  };
  const double ls0 = std::log(std::min(std::max(sigma_init, 0.3), 6.0));
  Vertex vx[3] = {{{s0_init, ls0}, 0.0},
                  {{s0_init + 0.75, ls0}, 0.0},
                  {{s0_init, ls0 + 0.35}, 0.0}};
  for (auto &v : vx) v.f = objective(v.x[0], v.x[1], beta_scratch);

  for (int iter = 0; iter < 90; ++iter) {
    std::sort(std::begin(vx), std::end(vx),
              [](const Vertex &a, const Vertex &b) { return a.f < b.f; });
    if (std::fabs(vx[2].x[0] - vx[0].x[0]) < 1e-5 &&
        std::fabs(vx[2].x[1] - vx[0].x[1]) < 1e-4)
      break;
    const double cx = (vx[0].x[0] + vx[1].x[0]) / 2.0;
    const double cy = (vx[0].x[1] + vx[1].x[1]) / 2.0;
    Vertex refl{{cx + (cx - vx[2].x[0]), cy + (cy - vx[2].x[1])}, 0.0};
    refl.f = objective(refl.x[0], refl.x[1], beta_scratch);
    if (refl.f < vx[0].f) {
      Vertex exp_{{cx + 2.0 * (cx - vx[2].x[0]), cy + 2.0 * (cy - vx[2].x[1])},
                  0.0};
      exp_.f = objective(exp_.x[0], exp_.x[1], beta_scratch);
      vx[2] = exp_.f < refl.f ? exp_ : refl;
    } else if (refl.f < vx[1].f) {
      vx[2] = refl;
    } else {
      Vertex ctr{{cx + 0.5 * (vx[2].x[0] - cx), cy + 0.5 * (vx[2].x[1] - cy)},
                 0.0};
      ctr.f = objective(ctr.x[0], ctr.x[1], beta_scratch);
      if (ctr.f < vx[2].f) {
        vx[2] = ctr;
      } else { // shrink toward best
        for (int k = 1; k < 3; ++k) {
          vx[k].x[0] = vx[0].x[0] + 0.5 * (vx[k].x[0] - vx[0].x[0]);
          vx[k].x[1] = vx[0].x[1] + 0.5 * (vx[k].x[1] - vx[0].x[1]);
          vx[k].f = objective(vx[k].x[0], vx[k].x[1], beta_scratch);
        }
      }
    }
  }
  std::sort(std::begin(vx), std::end(vx),
            [](const Vertex &a, const Vertex &b) { return a.f < b.f; });

  // Family selection at the shared (s0, σ): the full model's step and √
  // columns go collinear at large σ, and the ≥0 constraints then convert
  // noise into a systematic s0 pull. BIC picks the simplest family the
  // data decisively supports (a marginal √ win on a noisy step ray is a
  // false positive that biases s0 outward — the penalty must carry margin).
  const double s0_nm = vx[0].x[0];
  const double lsig_nm = vx[0].x[1];
  const double sigma_nm = std::min(std::max(std::exp(lsig_nm), 0.25), 8.0);
  D.build(s, s0_nm, sigma_nm, R_lambert);
  struct Family {
    bool use_step, use_sqrt;
    int k; // parameters: B, M, (H), (Cs), s0, σ
  };
  const Family fams[3] = {{true, false, 5}, {false, true, 5}, {true, true, 6}};
  int best_fam = -1;
  if (force_family >= 0 && force_family <= 2) {
    best_fam = force_family;
  } else {
    double best_bic = 1e300;
    for (int fi = 0; fi < 3; ++fi) {
      double b[4];
      const double sse =
          fit_family(I, D, fams[fi].use_step, fams[fi].use_sqrt, b);
      if (sse >= 1e29) continue;
      const int k = fams[fi].k;
      if (n - k - 1 <= 0) continue;
      const double bic =
          n * std::log(std::max(sse, 1e-12) / n) + k * std::log(1.0 * n);
      if (bic < best_bic) {
        best_bic = bic;
        best_fam = fi;
      }
    }
  }
  if (best_fam < 0) return out;

  // 1-D golden-section re-polish of s0 under the chosen family (σ fixed).
  // The chosen columns are REQUIRED from here on — subset fallback would
  // re-open the family-flip bias mid-polish.
  const double gr = 0.6180339887498949;
  double lo = s0_nm - 0.75, hi = s0_nm + 0.75;
  auto fam_sse = [&](double s0v) {
    double b[4];
    D.build(s, s0v, sigma_nm, R_lambert);
    return fit_family_required(I, D, fams[best_fam].use_step,
                               fams[best_fam].use_sqrt, b);
  };
  double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
  double f1 = fam_sse(x1), f2 = fam_sse(x2);
  for (int it = 0; it < 34; ++it) {
    if (f1 < f2) {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - gr * (hi - lo);
      f1 = fam_sse(x1);
    } else {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + gr * (hi - lo);
      f2 = fam_sse(x2);
    }
    if (hi - lo < 1e-5) break;
  }
  const double s0 = (lo + hi) / 2.0;
  double beta[4];
  D.build(s, s0, sigma_nm, R_lambert);
  const double sse = fit_family_required(I, D, fams[best_fam].use_step,
                                         fams[best_fam].use_sqrt, beta);
  if (sse >= 1e29) return out;
  // The edge must sit inside the sampled window to be trustworthy.
  if (s0 <= s[0] || s0 >= s[n - 1]) return out;

  out.ok = true;
  out.s0 = s0;
  out.sigma = sigma_nm;
  out.B = beta[0];
  out.H = beta[1];
  out.Csqrt = beta[2];
  out.M = beta[3];
  out.rms = std::sqrt(sse / n);
  return out;
}

bool localize_zernike(const cv::Mat &img32f, double x, double y,
                      double &edge_x, double &edge_y) {
  const ZernikeMasks &Z = zernike_masks();
  const int N = ZernikeMasks::N, half = N / 2;
  const int xc = static_cast<int>(std::lround(x));
  const int yc = static_cast<int>(std::lround(y));
  if (xc - half < 0 || yc - half < 0 || xc + half >= img32f.cols ||
      yc + half >= img32f.rows)
    return false;
  double a11r = 0.0, a11i = 0.0, a20 = 0.0;
  for (int i = 0; i < N; ++i) {
    const float *row = img32f.ptr<float>(yc - half + i);
    for (int j = 0; j < N; ++j) {
      const double v = row[xc - half + j];
      a11r += v * Z.m11r[i][j];
      a11i += v * Z.m11i[i][j];
      a20 += v * Z.m20[i][j];
    }
  }
  const double phi = std::atan2(a11i, a11r);
  const double a11p = a11r * std::cos(phi) + a11i * std::sin(phi);
  if (std::fabs(a11p) < 1e-9) return false;
  const double l = a20 / a11p; // unit-disk distance (A20_raw = l·A11_raw)
  if (std::fabs(l) > 1.0) return false;
  const double R = N / 2.0;
  edge_x = xc + l * R * std::cos(phi);
  edge_y = yc + l * R * std::sin(phi);
  return true;
}

namespace {

// One ray's profile window sampled relative to the running ellipse.
struct ProfileRay {
  double theta = 0.0, r_exp = 0.0, A = 0.0, B = 0.0;
  double cospsi = 0.0;     // cos(angle to sun direction), 0 if no sun hint
  std::vector<double> s;  // offset relative to r_exp
  std::vector<double> I;
};

std::vector<ProfileRay> build_profile_rays(const cv::Mat &img,
                                           const EllipseParams &ellipse,
                                           const LimbConfig &cfg,
                                           const ClassifyConfig &classify,
                                           const SunHint &sun, double win,
                                           double step) {
  std::vector<ProfileRay> rays;
  const bool use_sun = classify.use_sun_vector && sun.available &&
                       sun.phase_deg >= 5.0 &&
                       std::hypot(sun.x, sun.y) > 1e-6;
  double sx = 0.0, sy = 0.0, tan2a = 0.0;
  if (use_sun) {
    const double norm = std::hypot(sun.x, sun.y);
    sx = sun.x / norm;
    sy = sun.y / norm;
    const double ta = std::tan(sun.phase_deg * M_PI / 180.0);
    tan2a = ta * ta;
  }
  rays.reserve(cfg.rays);
  for (int k = 0; k < cfg.rays; ++k) {
    const double th = 2.0 * M_PI * k / cfg.rays;
    const double cth = std::cos(th), sth = std::sin(th);
    const double ca = std::cos(th - ellipse.phi_rad);
    const double sa = std::sin(th - ellipse.phi_rad);
    const double r_exp = ellipse.a_px * ellipse.b_px /
                         std::hypot(ellipse.b_px * ca, ellipse.a_px * sa);
    if (use_sun) {
      // Illumination test for a point ON THE LIMB at ray azimuth θ: the
      // local outward normal there is n = (cosθ, sinθ, 0) in image-plane
      // coords (grazing incidence — zero component along the view axis),
      // so n·sun = sinα·cospsi. This is > 0 — the limb point is genuinely
      // lit — for EVERY cospsi > 0, at every phase angle α, not just
      // α ≤ 90°: a thin crescent's horns are still the true limb, just
      // faint (brightness ∝ sinα·cospsi → 0 as cospsi → 0), and the
      // per-ray contrast cut downstream already rejects those on its own.
      // Only the anti-sun side (cospsi < 0) needs an explicit cut: there
      // the limb itself is dark and the visible bright/dark edge (when one
      // exists) is the terminator, offset inward by the depth below.
      const double cospsi = cth * sx + sth * sy;
      if (cospsi < 0.0 && 0.5 * r_exp * tan2a * cospsi * cospsi > 0.4)
        continue;
    }
    ProfileRay R;
    R.theta = th;
    R.r_exp = r_exp;
    R.cospsi = use_sun ? (cth * sx + sth * sy) : 0.0;
    for (double s = -win; s <= win; s += step) {
      R.s.push_back(s);
      R.I.push_back(sample(img, ellipse.cx_px + (r_exp + s) * cth,
                           ellipse.cy_px + (r_exp + s) * sth));
    }
    const int n = static_cast<int>(R.s.size());
    const int tail = std::max(3, n / 7);
    std::vector<double> head(R.I.begin(), R.I.begin() + tail);
    std::vector<double> back(R.I.end() - tail, R.I.end());
    std::nth_element(head.begin(), head.begin() + head.size() / 2, head.end());
    std::nth_element(back.begin(), back.begin() + back.size() / 2, back.end());
    R.A = head[head.size() / 2];
    R.B = back[back.size() / 2];
    if (R.A - R.B <= 1e-9) continue;
    rays.push_back(std::move(R));
  }
  return rays;
}

// Pooled, contrast-normalized profile → one decisive family/σ/shape fit.
ErfFitResult pooled_profile_fit(const std::vector<ProfileRay> &rays,
                                double R_hat, int force_family) {
  std::vector<double> ps, pv;
  const size_t stride = std::max<size_t>(1, rays.size() * 49 / 3000);
  size_t counter = 0;
  for (const auto &R : rays) {
    for (size_t i = 0; i < R.s.size(); ++i) {
      if (counter++ % stride) continue;
      ps.push_back(R.s[i]);
      pv.push_back((R.I[i] - R.B) / (R.A - R.B));
    }
  }
  // Sort by s for the tail-median init inside localize_erf (it assumes
  // inner→outer ordering).
  std::vector<size_t> ord(ps.size());
  for (size_t i = 0; i < ord.size(); ++i) ord[i] = i;
  std::sort(ord.begin(), ord.end(),
            [&](size_t a, size_t b) { return ps[a] < ps[b]; });
  std::vector<double> ps2(ps.size()), pv2(ps.size());
  for (size_t i = 0; i < ord.size(); ++i) {
    ps2[i] = ps[ord[i]];
    pv2[i] = pv[ord[i]];
  }
  ErfFitResult shared;
  if (ps2.size() >= 64)
    shared = localize_erf(ps2, pv2, 0.0, 1.5, R_hat, force_family);
  if (shared.ok && std::getenv("LIMBNAV_DEBUG")) {
    std::fprintf(stderr,
                 "[consensus] pooled: s0=%+.3f sigma=%.3f H=%.4f Cs=%.4f "
                 "M=%.5f rms=%.4f n=%zu rays=%zu R_hat=%.1f\n",
                 shared.s0, shared.sigma, shared.H, shared.Csqrt, shared.M,
                 shared.rms, ps2.size(), rays.size(), R_hat);
  }
  return shared;
}

} // namespace

// Bilinear pooled profile fit: shared shape (H, Cs, M at fixed s0, σ) with
// per-ray affine (gain, offset) profiled out by alternation. Normalizing by
// per-ray median levels instead injects ~7% amplitude jitter (the "interior"
// median of a Lambert profile is depth-dependent and noisy), which raises
// the pooled residual floor above the step-vs-√ shape difference and makes
// the family degenerate — measured as two minima ~1 px apart with Δrms
// 4e-4. Profiling the affine per ray drops the floor to the noise level.
struct PooledShape {
  bool ok = false;
  double s0 = 0.0, sigma = 0.0;
  double H = 0.0, Cs = 0.0, M = 0.0;
  double rms = 0.0;
};

namespace {

double bilinear_sse(const std::vector<ProfileRay> &rays, double s0,
                    double sigma, double R_hat, bool req_step, bool req_sqrt,
                    double shape_out[3]) {
  if (rays.empty()) return 1e30;
  ProfileDesign D;
  D.build(rays[0].s, s0, sigma, R_hat); // shared s-grid across rays
  const int ns = static_cast<int>(rays[0].s.size());
  const size_t nr = rays.size();

  std::vector<double> g(nr), c(nr);
  for (size_t i = 0; i < nr; ++i) {
    g[i] = std::max(rays[i].A - rays[i].B, 1e-6);
    c[i] = rays[i].B;
  }
  double H = 0.0, Cs = 0.0, M = 0.0;

  for (int alt = 0; alt < 4; ++alt) {
    // (b) shared shape LS with per-ray gains fixed: columns scaled by g_i.
    // Non-negativity cascade honoring the required columns; M optional.
    double best = 1e30, bh = 0, bc = 0, bm = 0;
    for (int m_on = 1; m_on >= 0; --m_on) {
      // build normal equations over active columns {step?, sqrt?, lin?}
      std::vector<int> cols;
      if (req_step) cols.push_back(0);
      if (req_sqrt) cols.push_back(1);
      if (m_on) cols.push_back(2);
      if (cols.empty()) continue;
      const int P = static_cast<int>(cols.size());
      std::vector<std::vector<double>> AtA(P, std::vector<double>(P, 0.0));
      std::vector<double> Atb(P, 0.0);
      for (size_t i = 0; i < nr; ++i) {
        for (int j = 0; j < ns; ++j) {
          const double base[3] = {D.step[j], D.sqrtc[j], D.lin[j]};
          const double y = rays[i].I[j] - c[i];
          for (int a = 0; a < P; ++a) {
            const double va = g[i] * base[cols[a]];
            Atb[a] += va * y;
            for (int b = a; b < P; ++b)
              AtA[a][b] += va * g[i] * base[cols[b]];
          }
        }
      }
      for (int a = 0; a < P; ++a)
        for (int b = 0; b < a; ++b) AtA[a][b] = AtA[b][a];
      std::vector<double> x = Atb;
      if (!solve_inplace(AtA, x)) continue;
      double th = 0, tc = 0, tm = 0;
      for (int a = 0; a < P; ++a) {
        if (cols[a] == 0) th = x[a];
        if (cols[a] == 1) tc = x[a];
        if (cols[a] == 2) tm = x[a];
      }
      if (th < 0.0 || tc < 0.0 || tm < 0.0) continue;
      if ((req_step && th <= 0.0) || (req_sqrt && tc <= 0.0)) continue;
      // quick SSE for this candidate
      double sse = 0.0;
      for (size_t i = 0; i < nr; ++i) {
        for (int j = 0; j < ns; ++j) {
          const double mval = th * D.step[j] + tc * D.sqrtc[j] + tm * D.lin[j];
          const double r = rays[i].I[j] - c[i] - g[i] * mval;
          sse += r * r;
        }
      }
      if (sse < best) {
        best = sse;
        bh = th;
        bc = tc;
        bm = tm;
      }
    }
    if (best >= 1e29) return 1e30;
    H = bh;
    Cs = bc;
    M = bm;

    // (a) per-ray affine with the shape fixed.
    std::vector<double> mv(ns);
    double msum = 0, mss = 0;
    for (int j = 0; j < ns; ++j) {
      mv[j] = H * D.step[j] + Cs * D.sqrtc[j] + M * D.lin[j];
      msum += mv[j];
      mss += mv[j] * mv[j];
    }
    const double det = ns * mss - msum * msum;
    if (std::fabs(det) < 1e-12) return 1e30;
    for (size_t i = 0; i < nr; ++i) {
      double is = 0, ims = 0;
      for (int j = 0; j < ns; ++j) {
        is += rays[i].I[j];
        ims += rays[i].I[j] * mv[j];
      }
      const double gi = (ns * ims - msum * is) / det;
      if (gi <= 1e-9) continue; // keep previous gain: contrast must be > 0
      g[i] = gi;
      c[i] = (is - gi * msum) / ns;
    }
  }

  double sse = 0.0;
  for (size_t i = 0; i < nr; ++i) {
    for (int j = 0; j < ns; ++j) {
      const double mval = H * D.step[j] + Cs * D.sqrtc[j] + M * D.lin[j];
      const double r = rays[i].I[j] - c[i] - g[i] * mval;
      sse += r * r;
    }
  }
  shape_out[0] = H;
  shape_out[1] = Cs;
  shape_out[2] = M;
  return sse;
}

// ---- Physically-exact composite shape for Lambert crescents ----
//
// The intensity along ray θ at depth d = s0 − s inside the limb is, exactly
// (not a small-d approximation — see the derivation in the review notes):
//   μ(d,θ) = sinα·cospsi(θ)·(1 − d/R) + cosα·√(2d/R − (d/R)²),   d ≥ 0
// where α is the phase angle and cospsi(θ) the cosine to the sun direction
// — both flight-legitimate (§6.2), and R the body radius (self-estimated
// from the running ellipse, never ground truth). The step:√ MIXING RATIO
// varies with cospsi(θ) — a fixed shared (H, Cs, M) shape cannot represent
// every ray at once, and forcing one (as the earlier free-parameter fit
// did) measured up to +1.4% bias at α=120°, growing with phase angle. With
// α and cospsi(θ) known, there is no shape left to fit: only a per-ray
// affine (gain, offset) against this fully-determined composite column.
double composite_col(const ProfileDesign &D, int j, double sinA, double cosA,
                     double cospsi, double invR) {
  return sinA * cospsi * (D.step[j] - D.lin[j] * invR) + cosA * D.sqrtc[j];
}

// NOTE: a real detector integrates over a finite pixel aperture before any
// further optical blur, and the exact composite shape has an
// infinite-slope (√d) kink at d=0 — pixel-aperture pre-averaging of a
// slope-singular function is NOT equivalent to widening a pure-Gaussian
// blur (verified numerically against the renderer: matching the
// aperture-averaged physics closes most of a multi-DN gap a pure-Gaussian
// composite leaves at α~60°, measured as a systematic 0.3–0.7% residual
// bias in the fitted radius). A per-ray free empirical linear-in-d term
// was tried to absorb this and made the estimator SIGNIFICANTLY less
// robust (the extra 2-column LS goes ill-conditioned across large swaths
// of the (s0,σ) search space, stalling the golden-section search) for a
// modest accuracy gain — reverted; tracked as a known G0 gap (see DECK).
double bilinear_sse_physical(const std::vector<ProfileRay> &rays, double s0,
                             double sigma, double R_hat, double sinA,
                             double cosA) {
  if (rays.empty() || R_hat <= 0.0) return 1e30;
  ProfileDesign D;
  D.build(rays[0].s, s0, sigma, R_hat);
  const int ns = static_cast<int>(rays[0].s.size());
  double sse = 0.0;
  for (const auto &R : rays) {
    double sm = 0, si = 0, smm = 0, smi = 0;
    for (int j = 0; j < ns; ++j) {
      const double m = composite_col(D, j, sinA, cosA, R.cospsi, 1.0 / R_hat);
      sm += m;
      si += R.I[j];
      smm += m * m;
      smi += m * R.I[j];
    }
    const double det = ns * smm - sm * sm;
    if (std::fabs(det) < 1e-12) return 1e30;
    const double g = (ns * smi - sm * si) / det;
    if (g <= 0.0) return 1e30; // contrast must be positive
    const double c = (si - g * sm) / ns;
    for (int j = 0; j < ns; ++j) {
      const double m = composite_col(D, j, sinA, cosA, R.cospsi, 1.0 / R_hat);
      const double r = R.I[j] - (c + g * m);
      sse += r * r;
    }
  }
  return sse;
}

PooledShape pooled_physical_fit(const std::vector<ProfileRay> &rays,
                                double R_hat, double phase_deg) {
  PooledShape out;
  if (rays.size() < 16 || R_hat <= 0.0) return out;
  const size_t npts = rays.size() * rays[0].s.size();
  const double sinA = std::sin(phase_deg * M_PI / 180.0);
  const double cosA = std::cos(phase_deg * M_PI / 180.0);

  auto sse_at = [&](double s0v, double sigv) {
    return bilinear_sse_physical(rays, s0v, sigv, R_hat, sinA, cosA);
  };
  auto golden1d = [](auto f, double lo, double hi, int iters) {
    const double gr = 0.6180339887498949;
    double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
    double f1 = f(x1), f2 = f(x2);
    for (int it = 0; it < iters; ++it) {
      if (f1 < f2) {
        hi = x2;
        x2 = x1;
        f2 = f1;
        x1 = hi - gr * (hi - lo);
        f1 = f(x1);
      } else {
        lo = x1;
        x1 = x2;
        f1 = f2;
        x2 = lo + gr * (hi - lo);
        f2 = f(x2);
      }
    }
    return (lo + hi) / 2.0;
  };
  double sig = 1.8, s0 = 0.0;
  for (int cycle = 0; cycle < 3; ++cycle) {
    s0 = golden1d([&](double v) { return sse_at(v, sig); }, s0 - 3.0, s0 + 3.0,
                  28);
    sig = golden1d([&](double v) { return sse_at(s0, v); },
                   std::max(0.3, sig * 0.5), std::min(6.0, sig * 2.0), 22);
  }
  const double sse = sse_at(s0, sig);
  if (sse >= 1e29) return out;
  out.ok = true;
  out.s0 = s0;
  out.sigma = sig;
  out.rms = std::sqrt(sse / npts);
  if (std::getenv("LIMBNAV_DEBUG")) {
    std::fprintf(stderr,
                 "[physical] s0=%+.3f sigma=%.3f rms=%.4f rays=%zu "
                 "phase=%.1f\n",
                 out.s0, out.sigma, out.rms, rays.size(), phase_deg);
  }
  return out;
}

PooledShape pooled_bilinear_fit(const std::vector<ProfileRay> &rays,
                                double R_hat, int force_family) {
  PooledShape out;
  if (rays.size() < 16) return out;
  const size_t npts = rays.size() * rays[0].s.size();

  struct Fam {
    bool step, sq;
  };
  const Fam fams[3] = {{true, false}, {false, true}, {true, true}};
  int lo_f = 0, hi_f = 2;
  if (force_family >= 0 && force_family <= 2) lo_f = hi_f = force_family;

  double best_bic = 1e300;
  for (int fi = lo_f; fi <= hi_f; ++fi) {
    // golden over s0 at three fixed σ seeds, then golden over σ.
    double shape[3];
    auto sse_at = [&](double s0v, double sigv) {
      double sh[3];
      return bilinear_sse(rays, s0v, sigv, R_hat, fams[fi].step, fams[fi].sq,
                          sh);
    };
    auto golden1d = [](auto f, double lo, double hi, int iters) {
      const double gr = 0.6180339887498949;
      double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
      double f1 = f(x1), f2 = f(x2);
      for (int it = 0; it < iters; ++it) {
        if (f1 < f2) {
          hi = x2;
          x2 = x1;
          f2 = f1;
          x1 = hi - gr * (hi - lo);
          f1 = f(x1);
        } else {
          lo = x1;
          x1 = x2;
          f1 = f2;
          x2 = lo + gr * (hi - lo);
          f2 = f(x2);
        }
      }
      return (lo + hi) / 2.0;
    };
    double sig = 1.8;
    double s0 = 0.0;
    for (int cycle = 0; cycle < 3; ++cycle) {
      s0 = golden1d([&](double v) { return sse_at(v, sig); }, s0 - 3.0,
                    s0 + 3.0, 28);
      sig = golden1d([&](double v) { return sse_at(s0, v); },
                     std::max(0.3, sig * 0.5), std::min(6.0, sig * 2.0), 22);
    }
    const double sse =
        bilinear_sse(rays, s0, sig, R_hat, fams[fi].step, fams[fi].sq, shape);
    if (sse >= 1e29) continue;
    const int k = (fi == 2 ? 3 : 2) + 2; // shape params + (s0, σ)
    const double bic = npts * std::log(std::max(sse, 1e-12) / npts) +
                       k * std::log(1.0 * npts);
    if (bic < best_bic) {
      best_bic = bic;
      out.ok = true;
      out.s0 = s0;
      out.sigma = sig;
      out.H = shape[0];
      out.Cs = shape[1];
      out.M = shape[2];
      out.rms = std::sqrt(sse / npts);
    }
  }
  if (out.ok && std::getenv("LIMBNAV_DEBUG")) {
    std::fprintf(stderr,
                 "[bilinear] s0=%+.3f sigma=%.3f H=%.4f Cs=%.4f M=%.5f "
                 "rms=%.4f rays=%zu\n",
                 out.s0, out.sigma, out.H, out.Cs, out.M, out.rms,
                 rays.size());
  }
  return out;
}

} // namespace

ScaleCalib pooled_scale_calibration(const cv::Mat &img32f,
                                    const EllipseParams &ellipse,
                                    const LimbConfig &cfg,
                                    const ClassifyConfig &classify,
                                    const SunHint &sun, int force_family) {
  ScaleCalib out;
  cv::Mat img;
  if (img32f.depth() == CV_32F) {
    img = img32f;
  } else {
    img32f.convertTo(img, CV_32F);
  }
  const std::vector<ProfileRay> rays =
      build_profile_rays(img, ellipse, cfg, classify, sun, 12.0, 0.5);
  if (rays.size() < 16) return out;
  const double R_hat = (ellipse.a_px + ellipse.b_px) / 2.0;
  const ErfFitResult shared = pooled_profile_fit(rays, R_hat, force_family);
  if (!shared.ok) return out;
  out.ok = true;
  out.delta_px = shared.s0;
  out.sigma_px = shared.sigma;
  out.n_rays = static_cast<int>(rays.size());
  out.family = (shared.H > 1e-9 && shared.Csqrt > 1e-9)
                   ? 2
                   : (shared.Csqrt > 1e-9 ? 1 : 0);
  return out;
}

JointRefineResult joint_refine(const cv::Mat &img32f,
                               const EllipseParams &ellipse,
                               const LimbConfig &cfg,
                               const ClassifyConfig &classify,
                               const SunHint &sun,
                               std::vector<std::string> &flags,
                               int force_family) {
  JointRefineResult res;
  cv::Mat img;
  if (img32f.depth() == CV_32F) {
    img = img32f;
  } else {
    img32f.convertTo(img, CV_32F);
  }

  const double win = 12.0;
  const std::vector<ProfileRay> rays =
      build_profile_rays(img, ellipse, cfg, classify, sun, win, 0.5);
  if (rays.size() < 16) {
    flags.push_back("CONSENSUS_TOO_FEW_RAYS");
    return res;
  }
  const double R_hat = (ellipse.a_px + ellipse.b_px) / 2.0;

  // Physical mode: sun direction + phase angle known (§6.2, flight-legitimate)
  // → the step:√ mixing ratio per ray is fully determined by geometry, no
  // free shape left to fit (see bilinear_sse_physical). Falls back to the
  // free-shape fit only for phase-free/no-sun-hint imagery (pure geometric
  // ellipses, or α≈0 disks where the mixing is uniformly all-√ anyway).
  // DISABLED (see composite_col / bilinear_sse_physical for the derivation
  // and DECK.md decision log for the measurement): the physically-exact
  // step:√ mixing this composite shape encodes was VERIFIED correct in the
  // unblurred limit (exact match to the renderer's own point-sampled
  // physics), but a pure-Gaussian 1D-radial blur of it is NOT what the
  // rendered (or any real detector's) image actually contains — pixel-
  // aperture pre-averaging of the √d term's infinite-slope kink at d=0
  // measurably changes the blurred shape, and 2D tangential mixing near
  // the curved limb adds a further correction neither closed-form column
  // captures. Enabling this path raised G0 mean error 0.165%→0.24% at
  // α≥60°. A correct fix needs the aperture/2D effects modeled (or the
  // shape empirically calibrated per-image, which was tried and found to
  // make the (s0,σ) search ill-conditioned) — real future work, not a
  // quick patch. The free-shape (H,Cs,M) fallback below is what actually
  // ships; physical composite is kept for the next session to build on.
  const bool physical = false && classify.use_sun_vector && sun.available &&
                        sun.phase_deg >= 5.0 &&
                        std::hypot(sun.x, sun.y) > 1e-6;
  const double sinA = physical ? std::sin(sun.phase_deg * M_PI / 180.0) : 0.0;
  const double cosA = physical ? std::cos(sun.phase_deg * M_PI / 180.0) : 0.0;

  PooledShape shared;
  if (physical) {
    shared = pooled_physical_fit(rays, R_hat, sun.phase_deg);
  } else {
    shared = pooled_bilinear_fit(rays, R_hat, force_family);
  }
  if (!shared.ok) {
    flags.push_back("CONSENSUS_POOLED_FIT_FAILED");
    return res;
  }
  res.family = physical ? 2
                        : ((shared.H > 1e-9 && shared.Cs > 1e-9)
                               ? 2
                               : (shared.Cs > 1e-9 ? 1 : 0));

  struct RayOffset {
    double theta = 0.0, t = 0.0, w = 0.0;
  };
  std::vector<RayOffset> obs;
  obs.reserve(rays.size());

  // Per-ray relocalization: 1 nonlinear DOF (offset), σ from the pooled
  // fit. In physical mode the shape has no free parameters left (single
  // per-ray affine gain against the known composite); otherwise a
  // non-negative {step, √}-mix cascade, as before.
  ProfileDesign D;
  // Localization information for a blurred cutoff lives within ~±3σ of the
  // edge; interior samples only leak shape mismatch into s0. First pass on
  // the full window, second on an edge-restricted window.
  const double edge_win = std::max(3.0 * shared.sigma + 1.0, 4.0);
  for (const auto &R : rays) {
    auto make_sse = [&](double center, double half) {
      std::vector<double> es, eI;
      for (size_t i = 0; i < R.s.size(); ++i) {
        if (R.s[i] >= center - half && R.s[i] <= center + half) {
          es.push_back(R.s[i]);
          eI.push_back(R.I[i]);
        }
      }
      return [es = std::move(es), eI = std::move(eI), &D, &shared, R_hat,
              physical, sinA, cosA, cospsi = R.cospsi](double s0) mutable
                 -> double {
        if (es.size() < 8) return 1e30;
        D.build(es, s0, shared.sigma, R_hat);
        if (physical) {
          const int ns = static_cast<int>(es.size());
          double sm = 0, si = 0, smm = 0, smi = 0;
          for (int j = 0; j < ns; ++j) {
            const double m =
                composite_col(D, j, sinA, cosA, cospsi, 1.0 / R_hat);
            sm += m;
            si += eI[j];
            smm += m * m;
            smi += m * eI[j];
          }
          const double det = ns * smm - sm * sm;
          if (std::fabs(det) < 1e-12) return 1e30;
          const double g = (ns * smi - sm * si) / det;
          if (g <= 0.0) return 1e30;
          const double c = (si - g * sm) / ns;
          double sse = 0.0;
          for (int j = 0; j < ns; ++j) {
            const double m =
                composite_col(D, j, sinA, cosA, cospsi, 1.0 / R_hat);
            const double r = eI[j] - (c + g * m);
            sse += r * r;
          }
          return sse / ns;
        }
        double beta[4];
        // Non-negative {step, √}-mix cascade, no linear column per ray.
        double best = 1e30;
        const bool combos[3][2] = {{true, true}, {true, false}, {false, true}};
        for (const auto &c : combos) {
          double bb[4];
          const double sse = profile_ls(eI, D, c[0], c[1], false, bb);
          if (sse < best && bb[1] >= 0.0 && bb[2] >= 0.0) {
            best = sse;
            std::copy(bb, bb + 4, beta);
          }
        }
        if (best >= 1e29) return 1e30;
        if (beta[1] + beta[2] <= 1e-12) return 1e30; // no contrast at all
        return best / es.size(); // normalized: window sizes differ
      };
    };
    auto golden = [&](auto &sse_at, double lo, double hi) {
      const double gr = 0.6180339887498949;
      double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
      double f1 = sse_at(x1), f2 = sse_at(x2);
      for (int it = 0; it < 30; ++it) {
        if (f1 < f2) {
          hi = x2;
          x2 = x1;
          f2 = f1;
          x1 = hi - gr * (hi - lo);
          f1 = sse_at(x1);
        } else {
          lo = x1;
          x1 = x2;
          f1 = f2;
          x2 = lo + gr * (hi - lo);
          f2 = sse_at(x2);
        }
        if (hi - lo < 1e-4) break;
      }
      return (lo + hi) / 2.0;
    };

    auto sse_full = make_sse(0.0, win);
    const double s0_coarse = golden(sse_full, -4.0, 4.0);
    if (std::fabs(s0_coarse) > 3.5) continue; // pinned at bracket: distrust
    auto sse_edge = make_sse(s0_coarse, edge_win);
    const double s0 = golden(sse_edge, s0_coarse - 2.0, s0_coarse + 2.0);
    const double sse = sse_edge(s0);
    if (sse >= 1e29) continue;

    // Faint-edge cut against the per-ray residual noise (sse is per-sample).
    const double rms = std::sqrt(sse);
    if (R.A - R.B < 6.0 * rms) continue;
    RayOffset o;
    o.theta = R.theta;
    o.t = s0;
    // Information-proxy weight: bright, sharp rays localize better.
    o.w = (R.A - R.B) * (R.A - R.B) / std::max(sse, 1e-12);
    obs.push_back(o);
  }
  if (obs.size() < 16) {
    flags.push_back("CONSENSUS_TOO_FEW_POINTS");
    return res;
  }

  // Huber-IRLS weighted LS of offsets on the ellipse-perturbation basis:
  //   t(θ) ≈ δa·∂r/∂a + δb·∂r/∂b + δφ·∂r/∂φ + δcx·cosθ + δcy·sinθ
  // (a center shifted by δ moves the limb crossing along ray d̂ by +δ·d̂ to
  // first order) with ∂r/∂a = r·b²cos²u/(aD), ∂r/∂b = r·a²sin²u/(bD),
  // ∂r/∂φ = r·(a²−b²)·sinu·cosu/D, u = θ−φ, D = b²cos²u + a²sin²u.
  //
  // Near-circular ellipses (a≈b — the common case for airless-moon and all
  // Lambert-sphere geometry, since a true sphere's limb IS a circle) make
  // ∂r/∂a and ∂r/∂b nearly collinear (cos²u vs sin²u riding on the same
  // near-1 prefactor), and φ is undefined. Fitting them as two independent
  // parameters on a short crescent arc lets noise split an isotropic radius
  // shift into wildly different, partially-cancelling da/db (measured
  // da=-0.44, db=-0.43 vs a well-conditioned isotropic fit of -0.42 on the
  // same data — individually noisy, coincidentally similar here, but not
  // reliably so). Collapse a,b into ONE isotropic-radius column instead;
  // ∂r/∂a + ∂r/∂b ≡ r/a·(cos²u+sin²u) = r/a ≈ 1 for a≈b, exactly right.
  const bool fit_phi = (ellipse.a_px - ellipse.b_px) > 1e-3 * ellipse.a_px;
  const int P = fit_phi ? 5 : 3;
  std::vector<std::array<double, 5>> X(obs.size());
  for (size_t i = 0; i < obs.size(); ++i) {
    const double u = obs[i].theta - ellipse.phi_rad;
    const double cu = std::cos(u), su = std::sin(u);
    const double a = ellipse.a_px, b = ellipse.b_px;
    const double Dq = b * b * cu * cu + a * a * su * su;
    const double r = a * b / std::sqrt(Dq);
    if (fit_phi) {
      X[i][0] = r * b * b * cu * cu / (a * Dq);
      X[i][1] = r * a * a * su * su / (b * Dq);
      X[i][2] = std::cos(obs[i].theta);
      X[i][3] = std::sin(obs[i].theta);
      X[i][4] = r * (a * a - b * b) * su * cu / Dq;
    } else {
      X[i][0] = r * b * b * cu * cu / (a * Dq) + r * a * a * su * su / (b * Dq);
      X[i][1] = std::cos(obs[i].theta);
      X[i][2] = std::sin(obs[i].theta);
    }
  }
  std::vector<double> hub(obs.size(), 1.0);
  std::vector<double> sol(P, 0.0);
  for (int round = 0; round < 3; ++round) {
    std::vector<std::vector<double>> AtA(P, std::vector<double>(P, 0.0));
    std::vector<double> Atb(P, 0.0);
    for (size_t i = 0; i < obs.size(); ++i) {
      const double w = obs[i].w * hub[i];
      for (int aa = 0; aa < P; ++aa) {
        Atb[aa] += w * X[i][aa] * obs[i].t;
        for (int bb = aa; bb < P; ++bb)
          AtA[aa][bb] += w * X[i][aa] * X[i][bb];
      }
    }
    for (int aa = 0; aa < P; ++aa)
      for (int bb = 0; bb < aa; ++bb) AtA[aa][bb] = AtA[bb][aa];
    sol = Atb;
    if (!solve_inplace(AtA, sol)) {
      flags.push_back("CONSENSUS_SINGULAR");
      return res;
    }
    // Huber reweight (k = 1.345 of the robust scale).
    std::vector<double> absr(obs.size());
    for (size_t i = 0; i < obs.size(); ++i) {
      double pred = 0.0;
      for (int aa = 0; aa < P; ++aa) pred += sol[aa] * X[i][aa];
      absr[i] = std::fabs(obs[i].t - pred);
    }
    std::vector<double> tmp = absr;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    const double scale = std::max(1.4826 * tmp[tmp.size() / 2], 1e-3);
    for (size_t i = 0; i < obs.size(); ++i) {
      const double k = 1.345 * scale;
      hub[i] = absr[i] <= k ? 1.0 : k / absr[i];
    }
  }

  // Common-mode override: the pooled fit owns the mean radial offset (the
  // per-ray nonlinear bias at low SNR survives even Huber averaging) — but
  // ONLY in non-physical mode. In physical mode the per-ray fit is a
  // single gain against a fully-determined shape (§composite_col) and
  // carries no such bias; forcing agreement with shared.s0 there measured
  // as actively CANCELING the correct isotropic radius correction whenever
  // shared.s0 happened to be near zero (it usually is — shared.s0 answers
  // "does the CURRENT ellipse need a uniform shift", da answers "how far
  // is the current ellipse's radius from the data", and after any prior
  // shift the two need not agree at all).
  double common_shift = 0.0;
  if (!physical) {
    double mean_pred = 0.0;
    for (size_t i = 0; i < obs.size(); ++i) {
      double pred = 0.0;
      for (int aa = 0; aa < P; ++aa) pred += sol[aa] * X[i][aa];
      mean_pred += pred;
    }
    mean_pred /= obs.size();
    common_shift = shared.s0 - mean_pred;
  }

  const double da = sol[0] + common_shift;
  const double db = (fit_phi ? sol[1] : sol[0]) + common_shift; // isotropic: db tracks da
  const double dcx = fit_phi ? sol[2] : sol[1];
  const double dcy = fit_phi ? sol[3] : sol[2];
  const double dphi = fit_phi ? sol[4] : 0.0;
  if (std::fabs(da) > 5.0 || std::fabs(db) > 5.0 || std::fabs(dcx) > 5.0 ||
      std::fabs(dcy) > 5.0 || std::fabs(dphi) > 0.2) {
    flags.push_back("CONSENSUS_CORRECTION_IMPLAUSIBLE");
    return res;
  }

  res.ellipse = ellipse;
  res.ellipse.a_px += da;
  res.ellipse.b_px += db;
  res.ellipse.cx_px += dcx;
  res.ellipse.cy_px += dcy;
  res.ellipse.phi_rad += dphi;
  if (res.ellipse.b_px > res.ellipse.a_px) {
    std::swap(res.ellipse.a_px, res.ellipse.b_px);
    res.ellipse.phi_rad += M_PI / 2.0;
  }
  res.n_rays = static_cast<int>(obs.size());
  double wr2 = 0.0, wsum = 0.0;
  for (size_t i = 0; i < obs.size(); ++i) {
    double pred = 0.0;
    for (int aa = 0; aa < P; ++aa) pred += sol[aa] * X[i][aa];
    const double e = obs[i].t - pred;
    wr2 += obs[i].w * hub[i] * e * e;
    wsum += obs[i].w * hub[i];
  }
  res.residual_rms_px = wsum > 0.0 ? std::sqrt(wr2 / wsum) : 0.0;
  {
    std::vector<double> th;
    th.reserve(obs.size());
    for (const auto &o : obs) th.push_back(o.theta);
    std::sort(th.begin(), th.end());
    double max_gap = 2.0 * M_PI - (th.back() - th.front());
    for (size_t i = 1; i < th.size(); ++i)
      max_gap = std::max(max_gap, th[i] - th[i - 1]);
    res.arc_deg = (2.0 * M_PI - max_gap) * 180.0 / M_PI;
  }
  res.ok = true;
  if (std::getenv("LIMBNAV_DEBUG")) {
    std::fprintf(stderr,
                 "[joint] n=%d da=%+.3f db=%+.3f dc=(%+.3f,%+.3f) dphi=%+.4f "
                 "pooled_s0=%+.3f rms=%.3f fam=%d\n",
                 res.n_rays, da, db, dcx, dcy, dphi, shared.s0,
                 res.residual_rms_px, res.family);
  }
  return res;
}

CoarseDisk coarse_disk(const cv::Mat &preprocessed, const CoarseConfig &cfg,
                       std::vector<std::string> &flags) {
  CoarseDisk out;
  cv::Mat u8;
  if (preprocessed.depth() == CV_8U) {
    u8 = preprocessed;
  } else {
    cv::normalize(preprocessed, u8, 0, 255, cv::NORM_MINMAX, CV_8U);
  }

  cv::Mat mask;
  if (cfg.type == "otsu") {
    cv::threshold(u8, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  } else if (cfg.type == "adaptive") {
    cv::adaptiveThreshold(u8, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, 51, -2);
  } else if (cfg.type == "canny") {
    cv::Canny(u8, mask, cfg.lo, cfg.hi);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  } else if (cfg.type == "scharr") {
    cv::Mat gx, gy, mag;
    cv::Scharr(u8, gx, CV_32F, 1, 0);
    cv::Scharr(u8, gy, CV_32F, 0, 1);
    cv::magnitude(gx, gy, mag);
    cv::normalize(mag, mag, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::threshold(mag, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  } else {
    flags.push_back("COARSE_UNKNOWN_TYPE:" + cfg.type);
    return out;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (contours.empty()) {
    flags.push_back("NO_COARSE_DISK");
    return out;
  }
  size_t best = 0;
  double best_area = -1.0;
  for (size_t i = 0; i < contours.size(); ++i) {
    const double a = cv::contourArea(contours[i]);
    if (a > best_area) {
      best_area = a;
      best = i;
    }
  }
  if (best_area < 50.0) {
    flags.push_back("NO_COARSE_DISK");
    return out;
  }

  cv::Point2f c;
  float r = 0.f;
  cv::minEnclosingCircle(contours[best], c, r);
  out.cx_px = c.x;
  out.cy_px = c.y;
  out.r_px = r;
  if (contours[best].size() >= 5) {
    const cv::RotatedRect e = cv::fitEllipse(contours[best]);
    out.ellipse.a_px = std::max(e.size.width, e.size.height) / 2.0;
    out.ellipse.b_px = std::min(e.size.width, e.size.height) / 2.0;
    out.ellipse.phi_rad = e.angle * M_PI / 180.0 +
                          (e.size.width >= e.size.height ? 0.0 : M_PI / 2.0);
    out.ellipse.cx_px = e.center.x;
    out.ellipse.cy_px = e.center.y;
    // Trust the contour ellipse only when it broadly agrees with the
    // enclosing circle (a crescent's contour ellipse collapses inward).
    out.has_ellipse = out.ellipse.a_px > 0.55 * r && out.ellipse.a_px < 1.2 * r &&
                      out.ellipse.b_px > 0.3 * r;
  }
  out.ok = true;
  return out;
}

namespace {

// Robust per-ray edge init: outermost downward crossing of the 30%-contrast
// level on a lightly smoothed profile. Gradient-max seeding fails on
// limb-darkened (Lambert) disks — interior slope ≈ 5 DN/px vs
// finite-difference noise ≈ 13 DN/px at SNR 20 — while a level crossing of
// the blurred cutoff stays within ~2σ of the limb for every profile family.
struct RayInit {
  bool ok = false;
  double s_init = 0.0;
  double grad = 0.0;     // |dI/ds| at the crossing (smoothed)
  double contrast = 0.0; // inner-level − outer-level
};

RayInit ray_init(const std::vector<double> &s, const std::vector<double> &I) {
  RayInit out;
  const int n = static_cast<int>(s.size());
  if (n < 12) return out;
  std::vector<double> sm(n);
  sm[0] = I[0];
  sm[n - 1] = I[n - 1];
  for (int i = 1; i + 1 < n; ++i)
    sm[i] = 0.25 * I[i - 1] + 0.5 * I[i] + 0.25 * I[i + 1];

  auto median_range = [&](int lo, int hi) { // [lo, hi)
    std::vector<double> v(sm.begin() + lo, sm.begin() + hi);
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  };
  const int tail = std::max(3, n / 7);
  const double A = median_range(0, tail);         // innermost (bright)
  const double B = median_range(n - tail, n);     // outermost (space)
  out.contrast = A - B;
  if (out.contrast <= 1e-9) return out;
  const double T = B + 0.3 * out.contrast;

  int cross = -1; // last downward crossing of T
  for (int i = 0; i + 1 < n; ++i)
    if (sm[i] >= T && sm[i + 1] < T) cross = i;
  if (cross < 0) return out;
  const double f = (sm[cross] - T) / std::max(sm[cross] - sm[cross + 1], 1e-12);
  out.s_init = s[cross] + f * (s[cross + 1] - s[cross]);
  const int g0 = std::max(1, cross), g1 = std::min(n - 2, cross + 1);
  out.grad = std::fabs((sm[g1 + 1] - sm[g0 - 1]) / (s[g1 + 1] - s[g0 - 1]));
  out.ok = true;
  return out;
}

} // namespace

std::vector<LimbPoint> extract(const cv::Mat &preprocessed,
                               const CoarseDisk &seed, const LimbConfig &cfg,
                               const ClassifyConfig &classify,
                               const SunHint &sun,
                               std::vector<std::string> &flags,
                               double search_scale) {
  std::vector<LimbPoint> pts;
  if (!seed.ok || seed.r_px < 8.0) {
    flags.push_back("SEED_INVALID");
    return pts;
  }
  cv::Mat img;
  if (preprocessed.depth() == CV_32F) {
    img = preprocessed;
  } else {
    preprocessed.convertTo(img, CV_32F);
  }

  const double step = 0.5;
  const bool use_sun = classify.use_sun_vector && sun.available &&
                       sun.phase_deg >= 5.0 &&
                       std::hypot(sun.x, sun.y) > 1e-6;
  double sx = 0.0, sy = 0.0, tan2a = 0.0;
  if (use_sun) {
    const double norm = std::hypot(sun.x, sun.y);
    sx = sun.x / norm;
    sy = sun.y / norm;
    const double ta = std::tan(sun.phase_deg * M_PI / 180.0);
    tan2a = ta * ta; // |tan α|² — same for α and 180°−α, as needed
  }

  pts.reserve(cfg.rays);
  for (int k = 0; k < cfg.rays; ++k) {
    const double th = 2.0 * M_PI * k / cfg.rays;
    const double cth = std::cos(th), sth = std::sin(th);

    // Geometric terminator cut (§5.2, exact small-depth model): on the
    // anti-sun side the visible boundary is the terminator, lying
    // ≈ R·tan²α·cos²ψ/2 inside the true limb. Drop rays whose expected
    // terminator depth exceeds a fraction of the RANSAC tolerance; the
    // near-junction gray zone stays and RANSAC prunes it. (At α > 90° the
    // whole anti-sun half fails this test, recovering the semicircle rule.)
    if (use_sun) {
      // Only the anti-sun side needs a cut — see build_profile_rays for
      // why the sunward side (cospsi > 0) is the true limb at every phase.
      const double cospsi = cth * sx + sth * sy;
      if (cospsi < 0.0) {
        const double depth = 0.5 * seed.r_px * tan2a * cospsi * cospsi;
        if (depth > 0.4) continue;
      }
    }

    // Per-ray expected radius from the coarse ellipse when trusted.
    double r_exp = seed.r_px;
    if (seed.has_ellipse) {
      const double ca = std::cos(th - seed.ellipse.phi_rad);
      const double sa = std::sin(th - seed.ellipse.phi_rad);
      const double a = seed.ellipse.a_px, b = seed.ellipse.b_px;
      r_exp = a * b / std::hypot(b * ca, a * sa);
    }
    const double search = std::max(cfg.search_px, cfg.search_px * search_scale);
    const double s_lo = std::max(2.0, r_exp - search);
    const double s_hi = r_exp + search;

    std::vector<double> ss, II;
    ss.reserve(static_cast<size_t>((s_hi - s_lo) / step) + 2);
    for (double s = s_lo; s <= s_hi; s += step) {
      ss.push_back(s);
      II.push_back(sample(img, seed.cx_px + s * cth, seed.cy_px + s * sth));
    }

    const RayInit init = ray_init(ss, II);
    if (!init.ok) continue;

    LimbPoint p;
    p.ray_angle_rad = th;
    p.gradient = init.grad;

    // Local window around the init for all localizers (±4σ at the largest
    // PSF the G0 sweep covers).
    const double win = 12.0;
    std::vector<double> ws, wI;
    for (size_t i = 0; i < ss.size(); ++i) {
      if (ss[i] >= init.s_init - win && ss[i] <= init.s_init + win) {
        ws.push_back(ss[i]);
        wI.push_back(II[i]);
      }
    }

    if (cfg.localizer == "parabola_grad") {
      double grad = 0.0;
      const double s_par = localize_parabola(ws, wI, grad);
      if (s_par < 0.0) continue;
      p.radius_px = s_par;
      p.gradient = grad;
    } else if (cfg.localizer == "erf_fit") {
      const ErfFitResult f = localize_erf(ws, wI, init.s_init, 1.5);
      if (!f.ok) continue;
      // Faint-edge cut: measured contrast must clear the fit residual by a
      // decisive margin, or the localization is noise-driven (crescent
      // near-junction rays at low SNR).
      if (init.contrast < 6.0 * f.rms) continue;
      p.radius_px = f.s0;
      // p.gradient stays the crossing-measured slope: a consistent
      // sharpness measure even when H≈0 (Lambert ramp limbs).
    } else if (cfg.localizer == "zernike") {
      double ex = 0.0, ey = 0.0;
      if (!localize_zernike(img, seed.cx_px + init.s_init * cth,
                            seed.cy_px + init.s_init * sth, ex, ey))
        continue;
      // Radial component along this ray.
      p.radius_px = (ex - seed.cx_px) * cth + (ey - seed.cy_px) * sth;
    } else {
      flags.push_back("LIMB_UNKNOWN_LOCALIZER:" + cfg.localizer);
      return {};
    }
    if (p.radius_px <= 2.0) continue;
    p.x_px = seed.cx_px + p.radius_px * cth;
    p.y_px = seed.cy_px + p.radius_px * sth;
    p.is_limb = true;
    pts.push_back(p);
  }

  // Gradient-ratio cut against the median of surviving candidates
  // (terminator/haze edges are soft; §5.2 criterion b).
  std::vector<double> gs;
  for (const auto &p : pts)
    if (p.is_limb) gs.push_back(p.gradient);
  if (gs.size() >= 8 && classify.min_grad_ratio > 1.0) {
    std::nth_element(gs.begin(), gs.begin() + gs.size() / 2, gs.end());
    const double med = gs[gs.size() / 2];
    for (auto &p : pts)
      if (p.is_limb && p.gradient < med / classify.min_grad_ratio)
        p.is_limb = false;
  }

  int n_limb = 0;
  for (const auto &p : pts) n_limb += p.is_limb ? 1 : 0;
  if (n_limb < 8) flags.push_back("TOO_FEW_LIMB_POINTS");
  return pts;
}

} // namespace limb
} // namespace limbnav
