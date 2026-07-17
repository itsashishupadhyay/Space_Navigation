#ifndef LIMBNAV_PREPROCESS_HPP
#define LIMBNAV_PREPROCESS_HPP

#include "limbnav/types.hpp"

#include <opencv2/core.hpp>

namespace limbnav {
namespace preprocess {

// Stage-1 preprocessing (§6.1): denoise then optional contrast stretch.
// Input must be single-channel 8-bit or 16-bit. Throws std::invalid_argument
// on an unknown config type — an experiment config typo must fail loudly,
// not silently measure with the wrong chain.
cv::Mat apply(const cv::Mat &gray, const DenoiseConfig &denoise,
              const ContrastConfig &contrast);

} // namespace preprocess
} // namespace limbnav

#endif // LIMBNAV_PREPROCESS_HPP
