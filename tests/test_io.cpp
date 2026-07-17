#include "limbnav/geometry.hpp"
#include "limbnav/io.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace limbnav;

namespace {
const std::string kInstrumentsCsv =
    std::string(LIMBNAV_REPO_ROOT) +
    "/third_party/object_detection_opencv_cpp/artifacts/instruments.csv";
} // namespace

TEST(Io, LoadsVerifiedNacRecordFromSubmodule) {
  InstrumentRecord r;
  ASSERT_TRUE(io::load_instrument_record(kInstrumentsCsv, "cassini", "issna", r))
      << "expected verified row in " << kInstrumentsCsv;
  EXPECT_TRUE(r.verified);
  EXPECT_EQ(r.array_px_x, 1024);
  EXPECT_EQ(r.array_px_y, 1024);
  EXPECT_DOUBLE_EQ(r.pixel_pitch_um, 12.0);
  EXPECT_DOUBLE_EQ(r.focal_length_mm, 2003.44);
  EXPECT_NEAR(r.ifov_arcsec_per_px, 1.2357, 1e-4);
  EXPECT_EQ(r.boresight_frame, "CASSINI_ISS_NAC");
  // Quoted citation field with embedded commas must survive CSV parsing.
  EXPECT_NE(r.source_citation.find("Porco"), std::string::npos);
  // Recorded IFOV and pitch/f derivation agree (§3).
  EXPECT_LT(geometry::ifov_cross_check_rel(r), 1e-3);
}

TEST(Io, FailsClosedOnMissingFileOrUnknownInstrument) {
  InstrumentRecord r;
  r.verified = false;
  EXPECT_FALSE(io::load_instrument_record("/nonexistent.csv", "cassini",
                                          "issna", r));
  EXPECT_FALSE(io::load_instrument_record(kInstrumentsCsv, "cassini",
                                          "no_such_camera", r));
  EXPECT_FALSE(r.verified);
}

TEST(Io, MeasurementJsonCarriesVerdictFlagsAndProvenance) {
  LimbMeasurement m;
  m.image_id = "co-iss-n999";
  m.verdict = Verdict::REFUSED;
  m.flags = {"STAGE_NOT_IMPLEMENTED:limb"};

  PipelineConfig cfg;
  const std::string j = io::measurement_to_json(m, io::config_hash(cfg));
  EXPECT_NE(j.find("\"verdict\": \"REFUSED\""), std::string::npos);
  EXPECT_NE(j.find("STAGE_NOT_IMPLEMENTED:limb"), std::string::npos);
  EXPECT_NE(j.find("\"git_sha\""), std::string::npos);
  EXPECT_NE(j.find("\"config_hash\""), std::string::npos);
}

TEST(Io, ConfigHashSeparatesConfigs) {
  PipelineConfig a, b;
  b.denoise.type = "gaussian";
  b.denoise.sigma = 1.5;
  EXPECT_NE(io::config_hash(a), io::config_hash(b));
  EXPECT_EQ(io::config_hash(a), io::config_hash(PipelineConfig{}));
}
