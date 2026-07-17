#!/usr/bin/env python3
"""Generate ground-truth radii from the PCK — never hand-entered (§6.2).

Reads BODYnnn_RADII via SPICE from pck00011.tpc (the kernel already recorded
in the submodule's spacecraft_state.csv) and writes
data/ground_truth/radii.json with provenance. The appendix table in
CLAUDE.md is illustrative; this output is authoritative.

Usage:
  .venv/bin/python scripts/make_ground_truth.py [--kernels data/spice]
                                                [--out data/ground_truth/radii.json]
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import datetime, timezone

import spiceypy as spice

BODIES = [
    # primaries (§9)
    "RHEA", "DIONE", "TETHYS", "ENCELADUS", "MIMAS", "IAPETUS",
    # Jupiter-flyby candidates
    "EUROPA", "GANYMEDE", "CALLISTO", "IO",
    # secondaries (documented filter-dependent limb corrections, §5.2)
    "SATURN", "JUPITER",
    # special study, excluded from G1
    "TITAN",
]


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_sha(repo_root: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short=12", "HEAD"], cwd=repo_root, text=True
        ).strip()
    except Exception:
        return "unknown"


def main() -> int:
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser()
    ap.add_argument("--kernels", default=os.path.join(repo_root, "data/spice"))
    ap.add_argument(
        "--out", default=os.path.join(repo_root, "data/ground_truth/radii.json")
    )
    args = ap.parse_args()

    pck = os.path.join(args.kernels, "pck00011.tpc")
    if not os.path.exists(pck):
        print(f"ERROR: {pck} not found — link or download the kernel set first",
              file=sys.stderr)
        return 1

    spice.furnsh(pck)
    out = {
        "schema_version": 1,
        "generated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "generator_git_sha": git_sha(repo_root),
        "pck": {"file": "pck00011.tpc", "sha256": sha256_file(pck)},
        "note": "radii_km = BODYnnn_RADII (a, b, c). Truth source per §6.2; "
                "gas-giant radii are the 1-bar reference surface; Titan's "
                "visible limb is haze, offset from these surface radii (§5.2).",
        "bodies": {},
    }
    for body in BODIES:
        _, radii = spice.bodvrd(body, "RADII", 3)
        a, b, c = (float(radii[0]), float(radii[1]), float(radii[2]))
        out["bodies"][body.lower()] = {
            "radii_km": [a, b, c],
            "mean_radius_km": (a + b + c) / 3.0,
            "naif_id": int(spice.bodn2c(body)),
        }
    spice.kclear()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    tmp = args.out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, indent=2)
    os.replace(tmp, args.out)
    print(f"wrote {args.out} ({len(out['bodies'])} bodies)")
    for k, v in out["bodies"].items():
        r = v["radii_km"]
        print(f"  {k:10s} a={r[0]:9.2f} b={r[1]:9.2f} c={r[2]:9.2f} km")
    return 0


if __name__ == "__main__":
    sys.exit(main())
