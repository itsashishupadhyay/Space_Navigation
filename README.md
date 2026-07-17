# limbnav — Cassini ISS-NAC Limb Metrology & Optical Navigation

Measure the diameter of planets and moons from Cassini ISS-NAC images using
C++ classical image processing with **sub-pixel limb fitting**, to better than
1% error in km against SPICE-derived truth — then invert the ruler into range
and relative position. Precision successor to
[`object_detection_opencv_cpp`](https://github.com/itsashishupadhyay/object_detection_opencv_cpp)
(vendored at `third_party/`), whose YOLO bounding boxes honestly bottom out at
a ~20% range-error floor; this project replaces the bbox ruler with limb-arc
geometry while keeping that repo's verified instrument records, SPICE-grounded
truth, and honest-fail verdicts.

**Status (P0 bootstrap, 2026-07-17):** scaffold + geometry/i-o/preprocess
modules building and tested on macOS (`cmake && ctest`: 17/17 green). The
limb-localization and ellipse-fit stages are honest `REFUSED +
STAGE_NOT_IMPLEMENTED` stubs until the G0 synthetic harness lands (P1) — this
repo does not pretend to measure yet. SPICE geometry sidecars (range,
sub-observer latitude, phase, Sun vector, and the `limbpt`-projected expected
limb per image) generate from the shared kernel cache; see
`research/DECK.md` for live status, gates, and the decision log.

## Layout

- `include/limbnav/`, `src/` — C++17 static library (`core`/`imgproc` only in
  the flight path; `dnn` for the detection gate). CPU-only, aarch64-portable,
  no `-ffast-math`.
- `apps/measure_limb` — single image → measurement JSON (truth-blind by
  construction: it receives image + instrument record, never SPICE range or
  known radii).
- `apps/batch_eval` — manifest → per-image JSON + metrics (P0: skeleton).
- `scripts/` — Python scaffolding (never in the flight path):
  `make_geometry_sidecar.py` (per-image SPICE truth via `limbpt`),
  `make_ground_truth.py` (PCK radii), `check_disk_budget.sh`.
- `research/` — the Deck (`DECK.md`, `STATE.json`, `RESEARCH_LOG.md`,
  `SESSION_LOG.md`), literature program, experiment history.
- `third_party/` — the YOLO/SPICE predecessor repo + karpathy/autoresearch
  (submodules).
- `data/` — gitignored, < 10 GB: kernels (symlinked pre-existing cache),
  sidecars, ground truth, future image downloads. Manifests are committed.

## Build

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build && ctest --test-dir build
```

Requires OpenCV (Homebrew on macOS) and CMake ≥ 3.21.

## Python side

```sh
python3.12 -m venv .venv && .venv/bin/pip install spiceypy numpy
ln -s <path-to-cassini-kernel-cache> data/spice   # ~3.4 GB, see DECK.md
.venv/bin/python scripts/make_ground_truth.py
.venv/bin/python scripts/make_geometry_sidecar.py
```

## Honesty contract

REFUSED is always a legal answer; the eval denominator is always the manifest
(no silent drops); truth lives only in sidecars; gates are judged by the eval
harness, never by eyeballing. See `CLAUDE.md` (project constitution) §2 for
the G0–G5 gate definitions.
