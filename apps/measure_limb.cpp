// measure_limb — single image → measurement JSON on stdout (§6.1).
//
// Runs preprocess → coarse → radial-ray sub-pixel limb → RANSAC ellipse.
// The binary never sees ground truth or SPICE range (§4.3); the optional
// --sun-dir/--phase-deg hint is flight-legitimate context for
// limb/terminator classification (§6.2). Detection gate (§4.4) runs
// upstream for real Cassini batches (P2).

#include "limbnav/geometry.hpp"
#include "limbnav/io.hpp"
#include "limbnav/measure.hpp"
#include "limbnav/types.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void usage(const char *argv0) {
  std::fprintf(stderr,
               "usage: %s --image <path> --instruments <instruments.csv>\n"
               "          [--mission cassini] [--instrument issna]\n"
               "          [--image-id <id>] [--sun-dir sx,sy]\n"
               "          [--phase-deg <deg>]\n",
               argv0);
}

} // namespace

int main(int argc, char **argv) {
  std::string image_path, instruments_csv, image_id;
  std::string mission = "cassini", instrument = "issna";
  limbnav::SunHint sun;

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
    } else if (a == "--sun-dir") {
      const char *v = next();
      if (v && std::sscanf(v, "%lf,%lf", &sun.x, &sun.y) == 2) {
        sun.available = true;
      }
    } else if (a == "--phase-deg") {
      const char *v = next();
      if (v) sun.phase_deg = std::atof(v);
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

  const cv::Mat img =
      cv::imread(image_path, cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
  if (img.empty()) {
    m.verdict = limbnav::Verdict::REFUSED;
    m.flags.push_back("IMAGE_UNREADABLE");
    std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
              << std::endl;
    return 1;
  }

  m = limbnav::measure(img, cfg, sun, m.image_id);

  std::cout << limbnav::io::measurement_to_json(m, limbnav::io::config_hash(cfg))
            << std::endl;
  return m.verdict == limbnav::Verdict::REFUSED ? 1 : 0;
}
