#ifndef LIMBNAV_IO_HPP
#define LIMBNAV_IO_HPP

#include "limbnav/types.hpp"

#include <string>

namespace limbnav {
namespace io {

// Load the row for (mission, instrument) from the submodule's
// artifacts/instruments.csv. Handles quoted CSV fields. Returns false and
// leaves `out` untouched on any parse/read failure (§3: single source of
// camera truth; §4.7: honest failure).
bool load_instrument_record(const std::string &csv_path,
                            const std::string &mission,
                            const std::string &instrument,
                            InstrumentRecord &out);

// Hand-written JSON for a LimbMeasurement (no external JSON library in the
// flight path). Embeds git_sha and library versions (§8).
std::string measurement_to_json(const LimbMeasurement &m,
                                const std::string &config_hash);

// Build-time git SHA (configure-time capture; "unknown" outside a repo).
std::string git_sha();

// FNV-1a hash of the pipeline config's canonical serialization — the
// config_hash embedded in every JSON output (§8).
std::string config_hash(const PipelineConfig &cfg);

} // namespace io
} // namespace limbnav

#endif // LIMBNAV_IO_HPP
