#include "renderer.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace limbnav_test {

Rendered render(const RenderParams &p) {
  if (p.lambert && std::fabs(p.a - p.b) > 1e-9) {
    throw std::invalid_argument("lambert rendering requires a sphere (a == b)");
  }
  Rendered out;
  out.a = p.a;
  out.b = p.b;
  out.phi_rad = p.phi_deg * M_PI / 180.0;
  out.cx = p.cx;
  out.cy = p.cy;
  out.phase_deg = p.phase_deg;
  const double az = p.sun_az_deg * M_PI / 180.0;
  out.sun_x = std::cos(az);
  out.sun_y = std::sin(az);

  cv::Mat img(p.height, p.width, CV_32F, cv::Scalar(p.bg));

  // Sun direction in 3D: image-plane azimuth az, out-of-plane by phase.
  const double alpha = p.phase_deg * M_PI / 180.0;
  const double s3[3] = {std::sin(alpha) * std::cos(az),
                        std::sin(alpha) * std::sin(az), std::cos(alpha)};

  const double cphi = std::cos(out.phi_rad), sphi = std::sin(out.phi_rad);
  const int margin = static_cast<int>(std::ceil(4.0 * p.psf_sigma + 4.0));
  const int x0 = std::max(0, static_cast<int>(p.cx - p.a - margin));
  const int x1 = std::min(p.width - 1, static_cast<int>(p.cx + p.a + margin));
  const int y0 = std::max(0, static_cast<int>(p.cy - p.a - margin));
  const int y1 = std::min(p.height - 1, static_cast<int>(p.cy + p.a + margin));
  const int ss = std::max(1, p.supersample);
  const double w = 1.0 / (ss * ss);

  for (int yi = y0; yi <= y1; ++yi) {
    float *row = img.ptr<float>(yi);
    for (int xi = x0; xi <= x1; ++xi) {
      double acc = 0.0;
      for (int sy = 0; sy < ss; ++sy) {
        for (int sx = 0; sx < ss; ++sx) {
          const double x = xi + (sx + 0.5) / ss - 0.5 - p.cx;
          const double y = yi + (sy + 0.5) / ss - 0.5 - p.cy;
          const double xr = cphi * x + sphi * y;
          const double yr = -sphi * x + cphi * y;
          const double q2 =
              xr * xr / (p.a * p.a) + yr * yr / (p.b * p.b);
          if (q2 > 1.0) continue;
          if (p.lambert) {
            const double nz = std::sqrt(std::max(0.0, 1.0 - q2));
            const double mu = (x / p.a) * s3[0] + (y / p.a) * s3[1] + nz * s3[2];
            acc += std::max(0.0, mu);
          } else {
            acc += 1.0;
          }
        }
      }
      row[xi] = static_cast<float>(p.bg + (p.fg - p.bg) * acc * w);
    }
  }

  if (p.psf_sigma > 0.0) {
    const int k = 2 * static_cast<int>(std::ceil(4.0 * p.psf_sigma)) + 1;
    cv::GaussianBlur(img, img, cv::Size(k, k), p.psf_sigma, p.psf_sigma,
                     cv::BORDER_REPLICATE);
  }
  if (p.snr > 0.0) {
    cv::Mat noise(img.size(), CV_32F);
    cv::RNG rng(p.seed);
    rng.fill(noise, cv::RNG::NORMAL, 0.0, (p.fg - p.bg) / p.snr);
    img += noise;
  }
  out.img = img;
  return out;
}

} // namespace limbnav_test
