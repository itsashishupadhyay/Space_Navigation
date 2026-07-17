# DECK — limbnav status dashboard

**Status:** P0 Bootstrap **complete** (2026-07-17). Build + 17/17 tests green on macOS; 154 geometry sidecars generated and QA'd; literature log seeded. Next: P1 synthetic estimator (G0).

## Gates

| Gate | Status | Evidence |
|---|---|---|
| G0 synthetic | pending | tests/synthetic not built yet (P1) |
| G1 real ≤1% | pending | — |
| G2 stretch ≤0.1% | pending | — |
| G3 novelty | pending | SEARCH_LOG.md seeded, audit not started |
| G4 holdout | pending | — |
| G5 nav preview | pending | — |

## Best so far
none — experiment 0000 (naive baseline) not yet run (P3).

## Last experiments
none.

## Data / disk budget
- `data/` budget: **< 10 GB hard, warn at 9 GB** (§4.5). Currently ≈ 5 MB physical (sidecars + ground truth; kernels are a symlink).
- SPICE kernels: **reusing existing cache** at `/Users/upadhyay/dev/ICES/object_detection_opencv_cpp/spice_cache/` (3.4 GB, already on disk, sha256-verified against `spacecraft_state.csv` on every sidecar run) via symlink `data/spice`. No re-download.
- Legacy images: 2,479 PNGs (421 MB) already at `/Users/upadhyay/dev/ICES/object_detection_opencv_cpp/data/`; submodule manifest `local_path` points there and paths are valid. New downloads (P2) go under `data/images/`.
- Images with verified SPICE state: 1,582. Primary-body counts: enceladus 58, iapetus 41, rhea 38, dione 33, tethys 31, mimas 27.
- **CK attitude coverage gap (found 2026-07-17):** only 269/1,582 state rows have attitude — the cached reconstructed CKs end early-2004, so **zero Saturn-tour moon frames have pointing**. Covered: jupiter 98, saturn(2001 approach) 98 (but those lack planetary SPK coverage → no sidecars), io 12, europa/ganymede/callisto/himalia/moon 11 each. **P2 must download tour CKs targeted to curated frames** (per-image selection keeps it small; full-tour CKs would be GBs).
- **Sidecars: 154 generated** (`data/sidecar/`): jupiter 98, io 12, callisto 11, europa 11, ganymede 11, moon 11 — includes the constitution's named Jupiter-flyby G1 candidates. 98 Saturn-2001 frames skipped (SPKINSUFFDATA, logged).

## Sidecar QA (2026-07-17, all 154)
| check | result |
|---|---|
| ellipse-fit RMS of projected limb | ≤ 6×10⁻³ px worst (Moon @ 768 px radius); ≤ 2.5×10⁻⁶ px for Jupiter — projection chain exact |
| Jupiter apparent b/a vs §5.2 oblate model, 98 frames | mean dev 0.006%, max 0.007% (perspective vs orthographic residual — expected) |
| Moon apparent radius vs PCK 1737.4 km | +0.012% ≈ stellar aberration (β≈10⁻⁴) on the apparent limb; cancels in Mode-A px-vs-px; footnote for Mode-B error table |
| near-spherical sphere-check dev | callisto/ganymede 0.004%; europa 0.10%; io 0.41% (real tri-axiality) |

## Token-spend notes
- 2026-07-17: P0 done by main session (Fable 5, owner-selected). No subagents spawned — scaffold is mechanical; roster defined in `.claude/agents/` for later phases.

## Needs review queue
0 items (see `review/NEEDS_REVIEW.md`).

## Next actions
see `research/STATE.json` next_actions (kept in sync).

## Decision log
- 2026-07-17 **Submodule path**: constitution §7 says `third_party/`; repo had them at `lib/`. Relocated via `git mv` to match the constitution.
- 2026-07-17 **autoresearch**: kept as a committed submodule (already wired by owner) rather than an uncommitted local clone; same content, better reproducibility.
- 2026-07-17 **SPICE kernels**: symlink `data/spice` → ICES checkout's `spice_cache/` instead of re-downloading 3.4 GB. Kernel SHA256s must match `spacecraft_state.csv` records (spot-checked by sidecar generator, which records hashes per sidecar).
- 2026-07-17 **CLAUDE.md is gitignored** by owner's `.gitignore` ("AI Helpers"). Conflicts with §7 layout (CLAUDE.md in tree) and §18.5 fresh-clone reproducibility. NOT changed unilaterally — flagged to owner in review queue.
- 2026-07-17 **Focal length**: sidecars + C++ use **2003.44 mm (NAIF IK `cas_iss_v10.ti`)** = IFOV 5.98971e-6 rad/px, matching `instruments.csv` (single source of truth; loaded, never hardcoded). The in-flight 2002.70 mm value differs by 0.037%; revisit only if G1 residuals show a systematic scale bias.
- 2026-07-17 **Sidecar pixel convention**: `predicted_center_px`/limb projection use IK boresight + focal-plane axes via pinhole model, origin = CCD center from `instruments.csv` (documented in `scripts/make_geometry_sidecar.py`). Line/sample orientation vs PDS display convention is UNVALIDATED until P1 real-image cross-check — flagged `PIXEL_CONVENTION_UNVALIDATED` in every sidecar until then.
- 2026-07-17 **Sidecar truth is the *apparent* limb** (LT+S, includes stellar aberration ≈ 10⁻⁴ relative): correct for Mode-A px comparison (measurement sees the same photons); the ~0.01% apparent-vs-geometric term goes in the Mode-B error table.
- 2026-07-17 **G1 candidate pivot option**: with tour CKs absent, the Jupiter-flyby set (europa/ganymede/callisto, 33 frames, but only ≤ 47 px radius — **below the 150 px curation floor**) cannot be the primary G1 set either. G1 still requires Saturn-tour moon frames → targeted CK download is a **P2 hard prerequisite**, budgeted per-frame.
