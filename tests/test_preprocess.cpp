#include "limbnav/preprocess.hpp"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include <stdexcept>

using namespace limbnav;

namespace {

cv::Mat impulse_image() {
  cv::Mat img = cv::Mat::zeros(64, 64, CV_8UC1);
  img.at<uint8_t>(32, 32) = 255;
  return img;
}

} // namespace

TEST(Preprocess, NoneIsIdentity) {
  const cv::Mat img = impulse_image();
  DenoiseConfig d; // none
  ContrastConfig c; // none
  const cv::Mat out = preprocess::apply(img, d, c);
  EXPECT_EQ(cv::countNonZero(img != out), 0);
}

TEST(Preprocess, GaussianSpreadsImpulse) {
  const cv::Mat img = impulse_image();
  DenoiseConfig d;
  d.type = "gaussian";
  d.sigma = 1.5;
  ContrastConfig c;
  const cv::Mat out = preprocess::apply(img, d, c);
  double max_val = 0.0;
  cv::minMaxLoc(out, nullptr, &max_val);
  EXPECT_LT(max_val, 255.0);           // peak spread out
  EXPECT_GT(cv::countNonZero(out), 1); // energy went somewhere
}

TEST(Preprocess, MedianRemovesImpulse) {
  const cv::Mat img = impulse_image();
  DenoiseConfig d;
  d.type = "median";
  d.ksize = 3;
  ContrastConfig c;
  const cv::Mat out = preprocess::apply(img, d, c);
  EXPECT_EQ(cv::countNonZero(out), 0); // isolated speckle killed
}

TEST(Preprocess, ClaheProducesSameGeometry) {
  const cv::Mat img = impulse_image();
  DenoiseConfig d;
  ContrastConfig c;
  c.type = "clahe";
  const cv::Mat out = preprocess::apply(img, d, c);
  EXPECT_EQ(out.size(), img.size());
  EXPECT_EQ(out.type(), img.type());
}

TEST(Preprocess, UnknownConfigFailsLoudly) {
  const cv::Mat img = impulse_image();
  DenoiseConfig d;
  d.type = "definitely_not_a_filter";
  ContrastConfig c;
  EXPECT_THROW(preprocess::apply(img, d, c), std::invalid_argument);

  DenoiseConfig d2;
  ContrastConfig c2;
  c2.type = "gamma"; // not implemented
  EXPECT_THROW(preprocess::apply(img, d2, c2), std::invalid_argument);

  EXPECT_THROW(preprocess::apply(cv::Mat(), d2, ContrastConfig{}),
               std::invalid_argument);
}
