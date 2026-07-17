#include "limbnav/geometry.hpp"

#include <cmath>

namespace limbnav {

std::string to_string(Verdict v) {
  switch (v) {
  case Verdict::NOMINAL:
    return "NOMINAL";
  case Verdict::DEGRADED:
    return "DEGRADED";
  case Verdict::REFUSED:
    return "REFUSED";
  }
  return "REFUSED";
}

namespace geometry {

double ifov_rad(const InstrumentRecord &instr) {
  if (instr.pixel_pitch_um <= 0.0 || instr.focal_length_mm <= 0.0) {
    return -1.0;
  }
  return (instr.pixel_pitch_um * 1e-6) / (instr.focal_length_mm * 1e-3);
}

double ifov_cross_check_rel(const InstrumentRecord &instr) {
  constexpr double kArcsecPerRad = 180.0 * 3600.0 / M_PI;
  const double derived = ifov_rad(instr);
  if (derived <= 0.0 || instr.ifov_arcsec_per_px <= 0.0) {
    return 1.0; // maximally suspicious
  }
  const double recorded_rad = instr.ifov_arcsec_per_px / kArcsecPerRad;
  return std::fabs(derived - recorded_rad) / recorded_rad;
}

double angular_radius_rad(double rho_px, double ifov) { return rho_px * ifov; }

double radius_km_from_range(double range_km, double rho_px, double ifov) {
  return range_km * std::sin(angular_radius_rad(rho_px, ifov));
}

double range_km_from_radius(double radius_km, double rho_px, double ifov) {
  const double s = std::sin(angular_radius_rad(rho_px, ifov));
  if (rho_px <= 0.0 || s <= 0.0) {
    return -1.0;
  }
  return radius_km / s;
}

ApparentAxes apparent_semi_axes(double a_km, double c_km, double B_rad) {
  const double sb = std::sin(B_rad);
  const double cb = std::cos(B_rad);
  ApparentAxes out;
  out.a_km = a_km;
  out.b_km = std::sqrt(a_km * a_km * sb * sb + c_km * c_km * cb * cb);
  return out;
}

bool polar_degenerate(double B_rad, double threshold_deg) {
  return std::fabs(B_rad) > threshold_deg * M_PI / 180.0;
}

double polar_radius_from_apparent(double b_app_km, double a_km, double B_rad) {
  const double sb = std::sin(B_rad);
  const double cb = std::cos(B_rad);
  if (std::fabs(cb) < 1e-9) {
    return -1.0; // pole-on: polar diameter unobservable (§5.2)
  }
  const double num = b_app_km * b_app_km - a_km * a_km * sb * sb;
  if (num < 0.0) {
    return -1.0;
  }
  return std::sqrt(num) / std::fabs(cb);
}

} // namespace geometry
} // namespace limbnav
