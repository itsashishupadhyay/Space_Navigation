#include "limbnav/limb.hpp"
#include "synthetic/renderer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

using limbnav::limb::ErfFitResult;
using limbnav::limb::localize_erf;
using limbnav::limb::localize_parabola;
using limbnav::limb::localize_zernike;

namespace {

double normal_cdf(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }

// Analytic blurred ideal step: sharp cutoff at s0, interior level A,
// space level B, PSF sigma.  I(s) = B + (A−B)·Φ((s0−s)/σ).
void step_profile(double s0, double sigma, double A, double B,
                  std::vector<double> &s, std::vector<double> &I) {
  s.clear();
  I.clear();
  for (double x = 0.0; x <= 24.0; x += 0.5) {
    s.push_back(x);
    I.push_back(B + (A - B) * normal_cdf((s0 - x) / sigma));
  }
}

// Numerically blurred EXACT Lambert limb at zero phase: sharp cutoff at s0
// with grazing profile μ(d) = √(2d/R − (d/R)²) — the hardest physical edge
// (§5.2). (An earlier version of this test used plain √(d/R) and passed
// while the pipeline was biased: the reference physics must be exact.)
void lambert_profile(double s0, double sigma, double R, double A, double B,
                     std::vector<double> &s, std::vector<double> &I) {
  s.clear();
  I.clear();
  const double h = 0.02;
  for (double x = 0.0; x <= 24.0; x += 0.5) {
    double acc = 0.0, wsum = 0.0;
    for (double t = -6.0 * sigma; t <= 6.0 * sigma; t += h) {
      const double w = std::exp(-t * t / (2.0 * sigma * sigma));
      const double d = s0 - (x + t);
      const double f =
          d > 0.0 ? std::sqrt(std::max(0.0, 2.0 * d / R - (d / R) * (d / R)))
                  : 0.0;
      acc += w * f;
      wsum += w;
    }
    s.push_back(x);
    I.push_back(B + (A - B) * acc / wsum);
  }
}

} // namespace

TEST(Localizers, ErfExactOnBlurredStep) {
  for (double sigma : {0.5, 1.0, 2.0, 3.0}) {
    std::vector<double> s, I;
    step_profile(12.3, sigma, 200.0, 12.0, s, I);
    const ErfFitResult f = localize_erf(s, I, 11.0, 1.5);
    ASSERT_TRUE(f.ok) << "sigma=" << sigma;
    EXPECT_NEAR(f.s0, 12.3, 0.02) << "sigma=" << sigma;
    EXPECT_NEAR(f.sigma, sigma, 0.15 * sigma) << "sigma=" << sigma;
    EXPECT_LT(std::fabs(f.M), 1.0); // no interior ramp on a step
  }
}

TEST(Localizers, ErfRampAwareOnLambertLimb) {
  // The gate-critical case: zero-phase Lambert limb (√ ramp, no plateau).
  // With the exact-Lambert basis (R known — the consensus stage) the bias
  // budget is the gate-level 0.05 px; with the plain √ basis (per-ray
  // seeding, R unknown) a documented ≲1.2 px inward bias is accepted and
  // later removed by the consensus refit.
  for (double sigma : {0.5, 1.5, 3.0}) {
    std::vector<double> s, I;
    lambert_profile(12.3, sigma, 200.0, 200.0, 12.0, s, I);
    const ErfFitResult exact = localize_erf(s, I, 11.5, 1.5, 200.0);
    ASSERT_TRUE(exact.ok) << "sigma=" << sigma;
    EXPECT_NEAR(exact.s0, 12.3, 0.05) << "sigma=" << sigma;

    const ErfFitResult plain = localize_erf(s, I, 11.5, 1.5);
    ASSERT_TRUE(plain.ok) << "sigma=" << sigma;
    EXPECT_NEAR(plain.s0, 12.3, 1.2) << "sigma=" << sigma;
  }
}

TEST(Localizers, ErfNoisyStepStaysWithinBudget) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, (200.0 - 12.0) / 10.0); // SNR 10
  double worst = 0.0, mean = 0.0;
  const int trials = 64;
  for (int trial = 0; trial < trials; ++trial) {
    std::vector<double> s, I;
    step_profile(12.3, 1.5, 200.0, 12.0, s, I);
    for (auto &v : I) v += noise(rng);
    const ErfFitResult f = localize_erf(s, I, 11.0, 1.5);
    ASSERT_TRUE(f.ok);
    worst = std::max(worst, std::fabs(f.s0 - 12.3));
    mean += f.s0 - 12.3;
  }
  mean /= trials;
  // The ellipse fit averages hundreds of rays: BIAS is what breaks the
  // gate; single-ray scatter shrinks as 1/√N and RANSAC clips the tails.
  EXPECT_LT(std::fabs(mean), 0.06);
  EXPECT_LT(worst, 1.6);
}

TEST(Localizers, ParabolaCentersBlurredStep) {
  for (double sigma : {1.0, 2.0}) {
    std::vector<double> s, I;
    step_profile(12.3, sigma, 200.0, 12.0, s, I);
    double g = 0.0;
    const double s0 = localize_parabola(s, I, g);
    ASSERT_GT(s0, 0.0);
    EXPECT_NEAR(s0, 12.3, 0.15) << "sigma=" << sigma;
    EXPECT_GT(g, 0.0);
  }
}

TEST(Localizers, ZernikeLocatesStraightEdge) {
  // Vertical bright|dark edge at x = 31.7 rendered with AA + blur σ1 via a
  // huge-radius disk (locally straight at this scale).
  limbnav_test::RenderParams p;
  p.width = 64;
  p.height = 64;
  p.cx = 31.7 - 5000.0; // limb crosses x=31.7 moving right
  p.cy = 32.0;
  p.a = p.b = 5000.0;
  p.psf_sigma = 1.0;
  p.snr = 0.0;
  const limbnav_test::Rendered r = limbnav_test::render(p);
  double ex = 0.0, ey = 0.0;
  ASSERT_TRUE(localize_zernike(r.img, 32.0, 32.0, ex, ey));
  EXPECT_NEAR(ex, 31.7, 0.15);
  EXPECT_NEAR(ey, 32.0, 0.5); // edge is vertical: y is unconstrained-ish
}
