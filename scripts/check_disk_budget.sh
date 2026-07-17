#!/usr/bin/env bash
# Disk-budget guard (§4.5): data/ must stay < 10 GB; stop and ask at 9 GB.
# Runs before every download batch. Symlinked pre-existing caches (e.g.
# data/spice -> ICES spice_cache) are reported separately: they predate this
# repo and are not additions against the budget, but they are shown so the
# owner sees the whole footprint.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA_DIR="$REPO_ROOT/data"
WARN_GB=9
HARD_GB=10

if [ ! -d "$DATA_DIR" ]; then
  echo "data/ does not exist yet — 0 GB used"
  exit 0
fi

phys_kb=$(du -sk "$DATA_DIR" | awk '{print $1}')
phys_gb=$(echo "$phys_kb" | awk '{printf "%.2f", $1 / 1048576}')

echo "data/ physical usage (symlinks not followed): ${phys_gb} GB (budget: ${HARD_GB} GB, warn: ${WARN_GB} GB)"

for link in "$DATA_DIR"/*; do
  if [ -L "$link" ]; then
    target=$(readlink "$link")
    tgt_size=$(du -sh "$target" 2>/dev/null | awk '{print $1}' || echo "?")
    echo "  external (pre-existing, not counted): $(basename "$link") -> $target ($tgt_size)"
  fi
done

awk -v g="$phys_gb" -v w="$WARN_GB" -v h="$HARD_GB" 'BEGIN {
  if (g >= h) { print "ERROR: data/ exceeds the hard " h " GB budget — stop."; exit 2 }
  if (g >= w) { print "WARNING: data/ at " g " GB >= " w " GB — stop and ask the owner (§4.5)."; exit 1 }
  print "OK: within budget."
}'
