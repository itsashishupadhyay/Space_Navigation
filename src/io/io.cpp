#include "limbnav/io.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <vector>

#ifndef LIMBNAV_GIT_SHA
#define LIMBNAV_GIT_SHA "unknown"
#endif

namespace limbnav {
namespace io {

namespace {

// RFC-4180-ish CSV line splitter: handles quoted fields containing commas
// and doubled quotes (instruments.csv citations contain both).
std::vector<std::string> split_csv_line(const std::string &line) {
  std::vector<std::string> fields;
  std::string cur;
  bool in_quotes = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (in_quotes) {
      if (c == '"') {
        if (i + 1 < line.size() && line[i + 1] == '"') {
          cur.push_back('"');
          ++i;
        } else {
          in_quotes = false;
        }
      } else {
        cur.push_back(c);
      }
    } else if (c == '"') {
      in_quotes = true;
    } else if (c == ',') {
      fields.push_back(cur);
      cur.clear();
    } else if (c != '\r') {
      cur.push_back(c);
    }
  }
  fields.push_back(cur);
  return fields;
}

bool parse_double(const std::string &s, double &out) {
  try {
    size_t pos = 0;
    out = std::stod(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

bool parse_int(const std::string &s, int &out) {
  try {
    size_t pos = 0;
    out = std::stoi(s, &pos);
    return pos == s.size();
  } catch (...) {
    return false;
  }
}

std::string json_escape(const std::string &s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      out.push_back(c);
    }
  }
  return out;
}

} // namespace

bool load_instrument_record(const std::string &csv_path,
                            const std::string &mission,
                            const std::string &instrument,
                            InstrumentRecord &out) {
  std::ifstream f(csv_path);
  if (!f.is_open()) {
    return false;
  }

  std::string header_line;
  if (!std::getline(f, header_line)) {
    return false;
  }
  const std::vector<std::string> header = split_csv_line(header_line);

  auto col = [&header](const std::string &name) -> int {
    for (size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  const int c_mission = col("mission");
  const int c_instrument = col("instrument");
  const int c_ax = col("array_px_x");
  const int c_ay = col("array_px_y");
  const int c_pitch = col("pixel_pitch_um");
  const int c_focal = col("focal_length_mm");
  const int c_fovx = col("fov_deg_x");
  const int c_fovy = col("fov_deg_y");
  const int c_ifov = col("ifov_arcsec_per_px");
  const int c_frame = col("boresight_frame");
  const int c_cite = col("source_citation");
  const int c_verified = col("verified_at");
  if (c_mission < 0 || c_instrument < 0 || c_ax < 0 || c_ay < 0 ||
      c_pitch < 0 || c_focal < 0 || c_ifov < 0) {
    return false;
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> v = split_csv_line(line);
    const size_t need = static_cast<size_t>(
        std::max({c_mission, c_instrument, c_ax, c_ay, c_pitch, c_focal,
                  c_ifov}));
    if (v.size() <= need) {
      continue;
    }
    if (v[c_mission] != mission || v[c_instrument] != instrument) {
      continue;
    }

    InstrumentRecord r;
    r.mission = v[c_mission];
    r.instrument = v[c_instrument];
    bool ok = parse_int(v[c_ax], r.array_px_x) &&
              parse_int(v[c_ay], r.array_px_y) &&
              parse_double(v[c_pitch], r.pixel_pitch_um) &&
              parse_double(v[c_focal], r.focal_length_mm) &&
              parse_double(v[c_ifov], r.ifov_arcsec_per_px);
    if (c_fovx >= 0 && static_cast<size_t>(c_fovx) < v.size()) {
      ok = ok && parse_double(v[c_fovx], r.fov_deg_x);
    }
    if (c_fovy >= 0 && static_cast<size_t>(c_fovy) < v.size()) {
      ok = ok && parse_double(v[c_fovy], r.fov_deg_y);
    }
    if (!ok || r.array_px_x <= 0 || r.pixel_pitch_um <= 0.0 ||
        r.focal_length_mm <= 0.0) {
      return false; // matching row that fails to parse is a hard failure
    }
    if (c_frame >= 0 && static_cast<size_t>(c_frame) < v.size()) {
      r.boresight_frame = v[c_frame];
    }
    if (c_cite >= 0 && static_cast<size_t>(c_cite) < v.size()) {
      r.source_citation = v[c_cite];
    }
    if (c_verified >= 0 && static_cast<size_t>(c_verified) < v.size()) {
      r.verified_at = v[c_verified];
    }
    r.verified = true;
    out = r;
    return true;
  }
  return false;
}

std::string git_sha() { return LIMBNAV_GIT_SHA; }

std::string config_hash(const PipelineConfig &cfg) {
  std::ostringstream ss;
  ss << cfg.denoise.type << ',' << cfg.denoise.sigma << ',' << cfg.denoise.ksize
     << ',' << cfg.denoise.d << ',' << cfg.denoise.sigma_color << ','
     << cfg.denoise.sigma_space << ',' << cfg.denoise.h << '|'
     << cfg.contrast.type << ',' << cfg.contrast.clip << ',' << cfg.contrast.grid
     << '|' << cfg.coarse.type << ',' << cfg.coarse.lo << ',' << cfg.coarse.hi
     << '|' << cfg.limb.rays << ',' << cfg.limb.localizer << ','
     << cfg.limb.search_px << '|' << cfg.classify.use_sun_vector << ','
     << cfg.classify.min_grad_ratio << '|' << cfg.fit.type << ','
     << cfg.fit.ransac_iters << ',' << cfg.fit.ransac_tol_px << ','
     << cfg.fit.min_arc_deg << '|' << cfg.gate_conf_min;
  const std::string s = ss.str();

  uint64_t h = 1469598103934665603ull; // FNV-1a 64
  for (unsigned char c : s) {
    h ^= c;
    h *= 1099511628211ull;
  }
  std::ostringstream hex;
  hex << std::hex << h;
  return hex.str();
}

std::string measurement_to_json(const LimbMeasurement &m,
                                const std::string &cfg_hash) {
  std::ostringstream j;
  j.precision(10);
  j << "{\n";
  j << "  \"image_id\": \"" << json_escape(m.image_id) << "\",\n";
  j << "  \"verdict\": \"" << to_string(m.verdict) << "\",\n";
  j << "  \"ellipse\": {\"a_px\": " << m.ellipse.a_px
    << ", \"b_px\": " << m.ellipse.b_px << ", \"phi_rad\": " << m.ellipse.phi_rad
    << ", \"cx_px\": " << m.ellipse.cx_px << ", \"cy_px\": " << m.ellipse.cy_px
    << "},\n";
  j << "  \"fit_rms_px\": " << m.fit_rms_px << ",\n";
  j << "  \"n_limb_points\": " << m.n_limb_points << ",\n";
  j << "  \"n_inliers\": " << m.n_inliers << ",\n";
  j << "  \"flags\": [";
  for (size_t i = 0; i < m.flags.size(); ++i) {
    j << (i ? ", " : "") << '"' << json_escape(m.flags[i]) << '"';
  }
  j << "],\n";
  j << "  \"git_sha\": \"" << json_escape(git_sha()) << "\",\n";
  j << "  \"config_hash\": \"" << json_escape(cfg_hash) << "\"\n";
  j << "}";
  return j.str();
}

} // namespace io
} // namespace limbnav
