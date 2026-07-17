// batch_eval — manifest → per-image measurement JSON files + a summary line.
//
// P0 status: plumbing skeleton. Iterates manifest rows and reports counts;
// every image REFUSES until P1 implements limb/fit. Coverage rule (§10.4):
// the denominator is always the manifest — this tool never drops rows.

#include "limbnav/io.hpp"
#include "limbnav/types.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  std::string manifest_path, instruments_csv;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char * {
      return (i + 1 < argc) ? argv[++i] : nullptr;
    };
    if (a == "--manifest") {
      const char *v = next();
      if (v) manifest_path = v;
    } else if (a == "--instruments") {
      const char *v = next();
      if (v) instruments_csv = v;
    }
  }
  if (manifest_path.empty()) {
    std::fprintf(stderr,
                 "usage: %s --manifest <csv> [--instruments <csv>]\n"
                 "P0 skeleton: counts manifest rows; measurement lands in P1.\n",
                 argv[0]);
    return 2;
  }

  std::ifstream f(manifest_path);
  if (!f.is_open()) {
    std::fprintf(stderr, "batch_eval: cannot open %s\n", manifest_path.c_str());
    return 1;
  }
  std::string line;
  long rows = -1; // header
  while (std::getline(f, line)) {
    if (!line.empty()) {
      ++rows;
    }
  }

  std::cout << "{\"manifest\": \"" << manifest_path << "\", \"images\": " << rows
            << ", \"measured\": 0, \"refused\": " << rows
            << ", \"note\": \"P0 skeleton — limb/fit stages land in P1\", "
            << "\"git_sha\": \"" << limbnav::io::git_sha() << "\"}"
            << std::endl;
  return 0;
}
