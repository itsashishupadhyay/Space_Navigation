#ifndef LIMBNAV_MEASURE_HPP
#define LIMBNAV_MEASURE_HPP

#include "limbnav/types.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace limbnav {

// Full measurement pipeline, stages 1–5 of §6.1 (the stage-0 detection gate
// runs upstream on real Cassini frames; synthetic G0 images bypass it by
// design — G0 separates algorithm error from Cassini systematics, §8).
//
// Two deterministic passes: coarse disk → rays → robust fit, then rays
// re-seeded from the pass-1 ellipse → final fit. Truth-blind: takes only
// the image, the pipeline config, and the flight-legitimate Sun hint.
LimbMeasurement measure(const cv::Mat &gray, const PipelineConfig &cfg,
                        const SunHint &sun, const std::string &image_id);

} // namespace limbnav

#endif // LIMBNAV_MEASURE_HPP
