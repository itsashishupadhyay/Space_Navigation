#ifndef LIMBNAV_GEOMETRY_HPP
#define LIMBNAV_GEOMETRY_HPP

#include "limbnav/types.hpp"

namespace limbnav {
namespace geometry {

// Angle subtended by one pixel, in radians, derived from the verified
// instrument record (never hardcoded): pitch [µm] / focal length [mm].
double ifov_rad(const InstrumentRecord &instr);

// Relative disagreement between the record's ifov_arcsec_per_px column and
// the pitch/focal-length derivation. Used as a load-time sanity check.
double ifov_cross_check_rel(const InstrumentRecord &instr);

// θ = ρ_px × IFOV. Exact forms everywhere (§5.1): sin/asin, never the
// small-angle shortcut.
double angular_radius_rad(double rho_px, double ifov);

// Phase 1 ruler: R_est = r × sin(θ). r is the SPICE center distance.
double radius_km_from_range(double range_km, double rho_px, double ifov);

// Phase 2 ruler: r_est = R_known / sin(θ). Returns <0 if rho_px or the
// resulting sin is non-positive (caller must REFUSE).
double range_km_from_radius(double radius_km, double rho_px, double ifov);

// Orthographic apparent semi-axes of an oblate spheroid (equatorial radius
// a_km, polar radius c_km) viewed from planetocentric sub-observer latitude
// B (§5.2):  a_app = a,  b_app = sqrt(a²sin²B + c²cos²B).
struct ApparentAxes {
  double a_km = 0.0;
  double b_km = 0.0;
};
ApparentAxes apparent_semi_axes(double a_km, double c_km, double B_rad);

// |B| > threshold (default 60°) ⇒ polar diameter unobservable (§5.2).
bool polar_degenerate(double B_rad, double threshold_deg = 60.0);

// Mode B inversion (§6.3): recover polar radius c from the apparent minor
// semi-axis b_app (km), the equatorial radius a (km) and B. Returns <0 when
// the geometry is degenerate (|B| near 90°) or the argument goes negative
// (measurement inconsistent with a ≥ b oblate model).
double polar_radius_from_apparent(double b_app_km, double a_km, double B_rad);

} // namespace geometry
} // namespace limbnav

#endif // LIMBNAV_GEOMETRY_HPP
