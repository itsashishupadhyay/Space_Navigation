// measure_limb — single image → measurement JSON on stdout (§6.1).
//
// P0 status: plumbing only. Loads the image and the verified instrument
// record, runs the preprocess stage, then REFUSES with
// STAGE_NOT_IMPLEMENTED flags because the limb/fit stages land in P1.
// The binary never sees ground truth or SPICE range (§4.3).

#include "limbnav/fit.hpp"
#include "limbnav/geometry.hpp"
#include "limbnav/io.hpp"
#include "limbnav/limb.hpp"
#include "limbnav/preprocess.hpp"
#include "limbnav/types.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <iostream>
#include <string>

namespace {

void usage(const char *argv0) {
  std::fprintf(stderr,
               "usage: %s --image <path> --instruments <instruments.csv>\n"
               "          [--mission cassini] [--instrument issna]\n"
               "          [--image-id <id>]\n",
               argv0);
}

} // namespace

int main(int argc, char **argv) {
  std::string image_path, instruments_csv, image_id;
  std::string mission = "cassini", instrument = "issna";

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char * {
      return (i + 1 < argc) ? argv[++i] : nullptr;
    };
    if (a == "--image") {
      const char *v = next();
      if (v) image_path = v;
    } else if (a == "--instruments") {
      const char *v = next();
      if (v) instruments_csv = v;
    } else if (a == "--mission") {
      const char *v = next();
      if (v) mission = v;
    } else if (a == "--instrument") {
      const char *v = next();
      if (v) instrument = v;
    } else if (a == "--image-id") {
      const char *v = next();
      if (v) image_id = v;
    } else {
      usage(argv[0]);
      return 2;
    }
  }
  if (image_path.empty() || instruments_csv.empty()) {
    usage(argv[0]);
    return 2;
  }

  limbnav::LimbMeasurement m;
  m.image_id = image_id.empty() ? image_path : image_id;
  const limbnav::PipelineConfig cfg; // defaults; config files land with the loop (P3)

  limbnav::InstrumentRecord instr;
  if (!limbnav::io::load_instrument_record(instruments_csv, mission, instrument,
                                           instr)) {
    m.verdict = limbnav::Verdict::REFUSED;
    m.flags.push_back("INSTRUMENT_RECORD_UNVERIFIED");
    std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
              << std::endl;
    return 1;
  }
  if (limbnav::geometry::ifov_cross_check_rel(instr) > 1e-3) {
    m.verdict = limbnav::Verdict::REFUSED;
    m.flags.push_back("IFOV_CROSS_CHECK_FAILED");
    std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
              << std::endl;
    return 1;
  }

  const cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    m.verdict = limbnav::Verdict::REFUSED;
    m.flags.push_back("IMAGE_UNREADABLE");
    std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
              << std::endl;
    return 1;
  }

  // NOTE: detection gate (submodule ONNX, §4.4) is wired in P1 together with
  // the coarse-disk stage that consumes its bbox seed.
  const cv::Mat pre = limbnav::preprocess::apply(img, cfg.denoise, cfg.contrast);

  auto points = limbnav::limb::extract(pre, {0.0, 0.0}, 0.0, cfg.limb,
                                       cfg.classify, m.flags);
  const auto fit = limbnav::fit::robust_ellipse(points, cfg.fit, m.flags);
  m.n_limb_points = static_cast<int>(points.size());
  m.n_inliers = fit.n_inliers;
  m.verdict = fit.valid ? limbnav::Verdict::NOMINAL : limbnav::Verdict::REFUSED;
  if (fit.valid) {
    m.ellipse = fit.ellipse;
    m.fit_rms_px = fit.rms_px;
  }

  std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
            << std::endl;
  return m.verdict == limbnav::Verdict::REFUSED ? 1 : 0;
}
