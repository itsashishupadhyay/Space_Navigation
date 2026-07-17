#include "limbnav/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace limbnav;
namespace geo = limbnav::geometry;

namespace {

InstrumentRecord nac_like() {
  // Values as recorded in instruments.csv; the io test asserts the real file
  // matches these. Tests are eval-side and may hold known values (§4.3 binds
  // the pipeline binary, not the test suite).
  InstrumentRecord r;
  r.mission = "cassini";
  r.instrument = "issna";
  r.array_px_x = 1024;
  r.array_px_y = 1024;
  r.pixel_pitch_um = 12.0;
  r.focal_length_mm = 2003.44;
  r.ifov_arcsec_per_px = 1.2357;
  r.verified = true;
  return r;
}

} // namespace

TEST(Geometry, IfovFromPitchAndFocalLength) {
  const double ifov = geo::ifov_rad(nac_like());
  EXPECT_NEAR(ifov, 5.9897e-6, 1e-9); // §3: ≈ 5.9897 µrad/px
  EXPECT_LT(geo::ifov_cross_check_rel(nac_like()), 1e-3);
}

TEST(Geometry, IfovRejectsUnphysicalRecord) {
  InstrumentRecord r = nac_like();
  r.focal_length_mm = 0.0;
  EXPECT_LT(geo::ifov_rad(r), 0.0);
  EXPECT_GE(geo::ifov_cross_check_rel(r), 1.0);
}

TEST(Geometry, ExactRulerRoundTrip) {
  const double ifov = geo::ifov_rad(nac_like());
  // Rhea sanity example (§5.3): R ≈ 764 km at 400,000 km.
  const double R = 764.0, r = 400000.0;
  const double theta = std::asin(R / r);
  const double rho_px = theta / ifov;
  EXPECT_NEAR(rho_px, 319.0, 1.0); // §5.3: radius ≈ 319 px

  // Phase-1 ruler recovers R exactly; Phase-2 ruler recovers r exactly.
  EXPECT_NEAR(geo::radius_km_from_range(r, rho_px, ifov), R, 1e-9);
  EXPECT_NEAR(geo::range_km_from_radius(R, rho_px, ifov), r, 1e-6);
}

TEST(Geometry, RangeRefusesNonPositiveInputs) {
  const double ifov = geo::ifov_rad(nac_like());
  EXPECT_LT(geo::range_km_from_radius(764.0, 0.0, ifov), 0.0);
  EXPECT_LT(geo::range_km_from_radius(764.0, -5.0, ifov), 0.0);
}

TEST(Geometry, ApparentAxesLimits) {
  // Saturn-like oblate spheroid (values from CLAUDE.md Appendix A, test-only).
  const double a = 60268.0, c = 54364.0;

  // Equatorial view (B = 0): full flattening visible.
  auto eq = geo::apparent_semi_axes(a, c, 0.0);
  EXPECT_DOUBLE_EQ(eq.a_km, a);
  EXPECT_DOUBLE_EQ(eq.b_km, c);

  // Pole-on (B = ±90°): circle of radius a.
  auto pole = geo::apparent_semi_axes(a, c, M_PI / 2.0);
  EXPECT_NEAR(pole.b_km, a, 1e-6);

  // Intermediate B: minor axis strictly between c and a.
  auto mid = geo::apparent_semi_axes(a, c, M_PI / 6.0);
  EXPECT_GT(mid.b_km, c);
  EXPECT_LT(mid.b_km, a);
}

TEST(Geometry, PolarDegenerateThreshold) {
  EXPECT_FALSE(geo::polar_degenerate(59.0 * M_PI / 180.0));
  EXPECT_TRUE(geo::polar_degenerate(61.0 * M_PI / 180.0));
  EXPECT_TRUE(geo::polar_degenerate(-75.0 * M_PI / 180.0));
}

TEST(Geometry, ModeBInversionRoundTrip) {
  const double a = 60268.0, c = 54364.0;
  for (double B_deg : {0.0, 15.0, 30.0, 45.0, 59.0}) {
    const double B = B_deg * M_PI / 180.0;
    const double b_app = geo::apparent_semi_axes(a, c, B).b_km;
    EXPECT_NEAR(geo::polar_radius_from_apparent(b_app, a, B), c, 1e-6)
        << "B = " << B_deg;
  }
}

TEST(Geometry, ModeBInversionRefusesDegenerateGeometry) {
  const double a = 60268.0, c = 54364.0;
  const double B = M_PI / 2.0; // pole-on
  const double b_app = geo::apparent_semi_axes(a, c, B).b_km;
  EXPECT_LT(geo::polar_radius_from_apparent(b_app, a, B), 0.0);
  // Inconsistent measurement: apparent minor axis below the geometric floor.
  EXPECT_LT(geo::polar_radius_from_apparent(100.0, a, M_PI / 4.0), 0.0);
}
