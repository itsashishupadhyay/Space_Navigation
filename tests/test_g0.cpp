// G0 gate (§2): on rendered scenes — swept PSF σ 0.5–3 px, SNR 5–100,
// phase-angle crescents 0–120° — mean |axis error| ≤ 0.05% and
// p99 ≤ 0.2%. Coverage rule applies: every case must measure (REFUSED is a
// gate failure here). Writes build/G0_report.csv for the Deck.

#include "limbnav/measure.hpp"
#include "synthetic/renderer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

struct CaseResult {
  std::string name;
  double err_a_pct = 0.0;
  double err_b_pct = 0.0;
  std::string verdict;
};

void run_case(const limbnav_test::RenderParams &p, const std::string &name,
              std::vector<CaseResult> &results) {
  const limbnav_test::Rendered r = limbnav_test::render(p);
  limbnav::PipelineConfig cfg; // defaults: none/none, otsu, erf_fit, Direct
  limbnav::SunHint sun;
  if (p.lambert && p.phase_deg >= 5.0) {
    sun.available = true;
    sun.x = r.sun_x;
    sun.y = r.sun_y;
    sun.phase_deg = r.phase_deg;
  }
  const limbnav::LimbMeasurement m = limbnav::measure(r.img, cfg, sun, name);

  CaseResult cr;
  cr.name = name;
  cr.verdict = limbnav::to_string(m.verdict);
  if (m.verdict == limbnav::Verdict::REFUSED) {
    cr.err_a_pct = cr.err_b_pct = 100.0; // coverage rule: refusal = failure
  } else {
    cr.err_a_pct = (m.ellipse.a_px - r.a) / r.a * 100.0;
    cr.err_b_pct = (m.ellipse.b_px - r.b) / r.b * 100.0;
  }
  results.push_back(cr);
}

} // namespace

TEST(G0, SyntheticGate) {
  std::vector<CaseResult> results;
  std::uint32_t seed = 1000;

  // Uniform ellipses: sizes × ratio × orientation × PSF × SNR.
  for (double radius : {120.0, 320.0}) {
    for (double ratio : {1.0, 0.9}) {
      for (double phi : {0.0, 30.0}) {
        for (double sigma : {0.5, 1.5, 3.0}) {
          for (double snr : {10.0, 100.0}) {
            limbnav_test::RenderParams p;
            p.width = p.height = 768;
            p.cx = 384.3;
            p.cy = 383.6;
            p.a = radius;
            p.b = radius * ratio;
            p.phi_deg = phi;
            p.psf_sigma = sigma;
            p.snr = snr;
            p.seed = seed++;
            char name[128];
            std::snprintf(name, sizeof(name),
                          "ellipse_r%.0f_q%.2f_phi%.0f_s%.1f_snr%.0f", radius,
                          ratio, phi, sigma, snr);
            run_case(p, name, results);
          }
        }
      }
    }
  }

  // Lambert spheres: full disk (hardest edge profile) + crescents 20–120°.
  for (double radius : {150.0, 300.0}) {
    for (double phase : {0.0, 20.0, 60.0, 90.0, 120.0}) {
      for (double sun_az : {0.0, 130.0}) {
        if (phase == 0.0 && sun_az != 0.0) continue;
        for (double sigma : {1.0, 2.0}) {
          for (double snr : {20.0, 100.0}) {
            limbnav_test::RenderParams p;
            p.width = p.height = 768;
            p.cx = 383.7;
            p.cy = 384.4;
            p.a = p.b = radius;
            p.lambert = true;
            p.phase_deg = phase;
            p.sun_az_deg = sun_az;
            p.psf_sigma = sigma;
            p.snr = snr;
            p.seed = seed++;
            char name[128];
            std::snprintf(name, sizeof(name),
                          "lambert_r%.0f_a%.0f_az%.0f_s%.1f_snr%.0f", radius,
                          phase, sun_az, sigma, snr);
            run_case(p, name, results);
          }
        }
      }
    }
  }

  // Aggregate gate metrics over both axes of every case.
  std::vector<double> abs_errs;
  int refused = 0;
  for (const auto &c : results) {
    abs_errs.push_back(std::fabs(c.err_a_pct));
    abs_errs.push_back(std::fabs(c.err_b_pct));
    if (c.verdict == "REFUSED") ++refused;
  }
  double mean = 0.0;
  for (double e : abs_errs) mean += e;
  mean /= abs_errs.size();
  std::vector<double> sorted = abs_errs;
  std::sort(sorted.begin(), sorted.end());
  const double p99 =
      sorted[std::min(sorted.size() - 1,
                      static_cast<size_t>(std::ceil(0.99 * sorted.size())) - 1)];
  const double worst = sorted.back();

  // Report for the Deck.
  if (std::FILE *f = std::fopen("G0_report.csv", "w")) {
    std::fprintf(f, "case,err_a_pct,err_b_pct,verdict\n");
    for (const auto &c : results)
      std::fprintf(f, "%s,%.5f,%.5f,%s\n", c.name.c_str(), c.err_a_pct,
                   c.err_b_pct, c.verdict.c_str());
    std::fprintf(f, "AGGREGATE,mean=%.5f,p99=%.5f,worst=%.5f\n", mean, p99,
                 worst);
    std::fclose(f);
  }
  std::printf("[G0] cases=%zu refused=%d mean|err|=%.4f%% p99=%.4f%% "
              "worst=%.4f%%\n",
              results.size(), refused, mean, p99, worst);
  for (const auto &c : results) {
    const double w = std::max(std::fabs(c.err_a_pct), std::fabs(c.err_b_pct));
    if (w > 0.2)
      std::printf("[G0]   offender %-40s a=%+.4f%% b=%+.4f%% %s\n",
                  c.name.c_str(), c.err_a_pct, c.err_b_pct, c.verdict.c_str());
  }

  EXPECT_EQ(refused, 0) << "coverage rule: no refusals on synthetics";
  EXPECT_LE(mean, 0.05) << "G0 mean |axis error| gate";
  EXPECT_LE(p99, 0.2) << "G0 p99 gate";
}
