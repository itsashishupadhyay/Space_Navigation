#ifndef LIMBNAV_TESTS_RENDERER_HPP
#define LIMBNAV_TESTS_RENDERER_HPP

// Synthetic scene renderer for the G0 harness (§8). Test scaffolding only —
// never part of the flight library.

#include <opencv2/core.hpp>

#include <cstdint>

namespace limbnav_test {

struct RenderParams {
  int width = 640;
  int height = 640;
  double cx = 320.0, cy = 320.0;
  double a = 200.0, b = 200.0;   // semi-axes, px
  double phi_deg = 0.0;          // major-axis angle
  double psf_sigma = 1.0;        // Gaussian PSF, px (0 = none)
  double snr = 0.0;              // (fg-bg)/noise_sigma; <=0 = noiseless
  bool lambert = false;          // Lambert-lit sphere (requires a == b)
  double phase_deg = 0.0;        // solar phase angle (lambert only)
  double sun_az_deg = 0.0;       // image-plane sun azimuth (0 = +x)
  double fg = 200.0, bg = 12.0;  // intensity levels
  std::uint32_t seed = 42;
  int supersample = 8;
};

struct Rendered {
  cv::Mat img;        // CV_32F
  double a = 0.0, b = 0.0, phi_rad = 0.0, cx = 0.0, cy = 0.0; // truth
  double sun_x = 0.0, sun_y = 0.0; // image-plane sun direction (unit)
  double phase_deg = 0.0;
};

Rendered render(const RenderParams &p);

} // namespace limbnav_test

#endif // LIMBNAV_TESTS_RENDERER_HPP
