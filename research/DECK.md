# DECK — limbnav status dashboard

**Status:** P0 Bootstrap complete. P1 synthetic estimator **built and tested, G0 not yet passing** (2026-07-20): renderer, all three localizers (parabola/erf/zernike), coarse disk, RANSAC ellipse fit, and the `measure()` orchestrator are real, working code (22/23 gtest cases green — everything except the G0 gate itself). G0 fails at the accuracy threshold with a specific, diagnosed root cause; see decision log below before touching `src/limb/limb.cpp` again.

## Gates

| Gate | Status | Evidence |
|---|---|---|
| G0 synthetic | **failing** | `ctest -R G0`: mean\|err\| 0.165% (need ≤0.05%), p99 1.30% (need ≤0.2%), 0 refused. See decision log for root cause + what was tried. |
| G1 real ≤1% | pending | blocked on G0 |
| G2 stretch ≤0.1% | pending | — |
| G3 novelty | pending | SEARCH_LOG.md seeded, audit not started |
| G4 holdout | pending | — |
| G5 nav preview | pending | — |

## Best so far
Not applicable yet — the autoresearch loop (P3) hasn't started; experiment 0000 (naive baseline) isn't run. Current best G0 config (hand-built, not loop-optimized): mean 0.165%, p99 1.30%, worst 2.11% — see "G0 status detail" below.

## Last experiments
none (P3 not started).

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

## G0 status detail (2026-07-20)

Best validated configuration (what's actually in `src/limb/limb.cpp` right now):
- **Uniform ellipses / true steps** (no Lambert shading): mean ≈ 0.02–0.03%, worst ≈ 0.23% — comfortably inside gate on their own.
- **Lambert-sphere crescents** (`lambert_r*` cases, phase α ∈ {0°, 20°, 60°, 90°, 120°}): α=0° near-perfect; α=20° small residual (≤0.5%); **α≥60° carries a systematic 0.3–2.0% bias that grows with α**, and this family alone drags the aggregate mean to 0.165% / p99 to 1.30% / worst to 2.11% (the `lambert_r150_a120_az130_s1.0_snr20` case).
- Pipeline: pass 1/2 coarse-seeded rays → RANSAC ellipse → pass 3 "joint" consensus refinement (pooled profile fit for σ, per-ray Huber-weighted LS on an isotropic-radius + 2-center ellipse-perturbation basis, `src/limb/limb.cpp::joint_refine`).

**Root cause (verified numerically, not guessed):** the Lambert limb's pre-blur intensity profile is exact and known —
`μ(d,θ) = sinα·cospsi(θ)·(1−d/R) + cosα·√(2d/R−(d/R)²)`, d = depth inside the limb, cospsi(θ) = cos(angle to sun). This was checked point-for-point against the renderer's own unblurred physics (`diag_unblurred.cpp`) and matches to double-precision. The problem is downstream: a **pure 1D-radial Gaussian blur of that exact profile does NOT match the rendered (blurred) image** — off by several DN (~1–4% of contrast) right where it matters, verified ray-by-ray against the renderer (`diag_physics.cpp`). Two contributing, both-confirmed mechanisms:
1. **Pixel-aperture pre-averaging.** The renderer (correctly, physically) box-averages each pixel's footprint before applying the Gaussian PSF — real detectors do the same. The Lambert profile's √d term has an **infinite slope at d=0** (a genuine kink, not a numerical artifact); box-averaging a slope-singular function is *not* equivalent to widening a Gaussian blur applied to the un-averaged function. Including the box-average in the reference model (`diag_box.cpp`) closed roughly half the gap.
2. **2D tangential mixing** near the curved limb — a full 2D Gaussian convolution of the exact `μ(x,y)` (`diag_physics2d.cpp`) did not, by itself, close the remaining gap either; it isn't a single missing term, it's the combination.

**What was tried and reverted** (in order, each one measured against the full 120-case G0 sweep, not eyeballed):
1. A "physically exact" composite basis (`composite_col` in `limb.cpp`) built from the verified `μ(d,θ)` formula, with a single free per-ray gain — theoretically clean, but inherits the pixel-aperture/2D-mixing gap above: **made the sweep worse** (mean 0.1645%→0.2377%).
2. An isotropic-radius reparametrization of the ellipse-perturbation LS (`da`,`db` collapsed to one column when a≈b) — genuinely fixes a real collinearity problem (near-circular geometry makes ∂r/∂a, ∂r/∂b nearly parallel, especially on a short crescent arc) but is **numerically neutral** on the aggregate score (0.1645%→0.1652%) once isolated from (1). Kept — it's free correctness, just not sufficient alone.
3. Removing the "common-mode override" (forcing the per-ray Huber fit's mean offset to match a separately pooled fit) — this override, inherited from the original free-shape estimator, was found to **actively cancel** the correct isotropic-radius correction whenever the pooled fit reported near-zero (which it usually does, since it answers a different question). Fix: **kept the override, but gated it off specifically when the (currently-disabled) physical composite mode is active** — it's still load-bearing for the free-shape path.
4. A hybrid: physical composite (dominant, correctly θ-weighted shape) + one small **free** empirical linear-in-d correction to absorb the aperture/2D residual — closed some of the gap in a single hand-picked case (0.44%→0.34% on `lambert_r150_a60_az0_s1.0_snr20`) but made the 2-column per-ray LS **ill-conditioned across large parts of the (s0,σ) search space**, causing the pooled fit to fail outright on other cases. Reverted as too fragile to ship.

**Net result:** physical composite mode is implemented and compiles (`composite_col`, `bilinear_sse_physical`, `pooled_physical_fit` in `limb.cpp`) but is **force-disabled** (`const bool physical = false && ...` in `joint_refine`) with a comment pointing here. The free-shape (H, Cs, M all ≥0) fallback — the ORIGINAL, less "correct" model — is what's actually live, at mean 0.165%/p99 1.30%/worst 2.11%, still short of gate but with **no refusals and no worse-than-2.2% single-case error**.

**For whoever picks this up next** (algorithm-architect task, needs a fresh brief, not a re-read of this whole log): the fix almost certainly requires modeling the pixel-aperture + tangential-mixing correction analytically or via a per-image-calibrated (not per-ray-free) empirical term — the per-ray-free version (attempt 4) is the wrong axis of freedom (too many independent unstable parameters); a **single shared aperture-width or shape-correction parameter per pooled fit**, estimated once and applied identically to every ray, is the untried next step and should sidestep the ill-conditioning that killed attempt 4. The four `diag_*.cpp` scratch tools referenced above (in the session's scratchpad, not committed) are cheap to reproduce from the descriptions here if needed.

## Token-spend notes
- 2026-07-17: P0 done by main session (Fable 5, owner-selected). No subagents spawned — scaffold is mechanical; roster defined in `.claude/agents/` for later phases.
- 2026-07-20: P1 estimator session, main model Fable 5 max-effort, no subagents. Heavy iterative numerical debugging (physics derivation + 6 scratch diagnostic programs) to isolate the G0 root cause — appropriate use of the architect-tier model per §13, this is exactly the "accuracy plateau / synthetic divergence" case the roster reserves for it.

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
- 2026-07-20 **G0 not passing at P1 code-complete**: see "G0 status detail" above for the full root-cause trail (Lambert crescent sub-pixel bias at α≥60°, pixel-aperture smoothing of a slope-singular profile). Committing the estimator machinery anyway — it's real, tested, working code for every case except moderate/high-phase crescents, and the git history should show the honest attempt-and-revert trail (§11.1 "failed experiments are data"), not a clean-looking commit that hides how hard this sub-problem was.
- 2026-07-20 **Real-image smoke test found a pre-existing gap, not a regression**: `measure_limb` on a real 1024×1024 Rhea frame (no `--sun-dir`, no detection gate) returned `a_px=2193` — nonsensical for a 1024px image. Root cause: `coarse_disk` has no ROI seed (detection gate unwired) and no Sun hint (sidecar → CLI plumbing unwired) on real frames, so it has nothing to anchor to and the ray classifier runs with zero terminator discrimination. Both are already-scoped P1/P2 next_actions, not new work discovered by this test — logged so nobody re-diagnoses it from scratch.
