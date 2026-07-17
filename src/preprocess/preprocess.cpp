#include "limbnav/preprocess.hpp"

#include <opencv2/imgproc.hpp>

#include <stdexcept>

namespace limbnav {
namespace preprocess {

cv::Mat apply(const cv::Mat &gray, const DenoiseConfig &denoise,
              const ContrastConfig &contrast) {
  if (gray.empty() || gray.channels() != 1) {
    throw std::invalid_argument("preprocess: expected non-empty single-channel image");
  }

  cv::Mat out;
  if (denoise.type == "none") {
    out = gray.clone();
  } else if (denoise.type == "gaussian") {
    cv::GaussianBlur(gray, out, cv::Size(0, 0), denoise.sigma, denoise.sigma,
                     cv::BORDER_REPLICATE);
  } else if (denoise.type == "median") {
    if (denoise.ksize < 3 || denoise.ksize % 2 == 0) {
      throw std::invalid_argument("preprocess: median ksize must be odd >= 3");
    }
    cv::medianBlur(gray, out, denoise.ksize);
  } else if (denoise.type == "bilateral") {
    cv::Mat src8 = gray;
    // bilateralFilter supports 8U/32F; 16-bit inputs are filtered in float.
    if (gray.depth() == CV_16U) {
      gray.convertTo(src8, CV_32F);
    }
    cv::Mat filtered;
    cv::bilateralFilter(src8, filtered, denoise.d, denoise.sigma_color,
                        denoise.sigma_space);
    if (gray.depth() == CV_16U) {
      filtered.convertTo(out, CV_16U);
    } else {
      out = filtered;
    }
  } else {
    throw std::invalid_argument("preprocess: unknown denoise type '" +
                                denoise.type + "'");
  }

  if (contrast.type == "none") {
    return out;
  }
  if (contrast.type == "clahe") {
    auto clahe = cv::createCLAHE(contrast.clip,
                                 cv::Size(contrast.grid, contrast.grid));
    cv::Mat stretched;
    clahe->apply(out, stretched);
    return stretched;
  }
  throw std::invalid_argument("preprocess: unknown contrast type '" +
                              contrast.type + "'");
}

} // namespace preprocess
} // namespace limbnav
