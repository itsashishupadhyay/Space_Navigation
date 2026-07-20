# SESSION_LOG — append-only session handoffs (§14.1)

## Session 2026-07-17T08:22Z  (main model: fable-5, max effort — owner-selected for P0 architecture)
did:     (in progress) surveyed repo + submodule assets; toolchain verified (cmake 4.3.2, ninja,
         Homebrew OpenCV 4.12, python3.12 + ICES venv has spiceypy 8.1.0).
         KEY FINDS: submodule manifest local_path points to /Users/upadhyay/dev/ICES/object_detection_opencv_cpp/
         which still holds data/ (421 MB, 2479 PNGs) and spice_cache/ (3.4 GB, all kernels) — no downloads needed
         for P0. spacecraft_state.csv: 1582 rows; primary bodies with state: enceladus 58, iapetus 41, rhea 38,
         dione 33, tethys 31, mimas 27 (~228 primaries).
did(2): P0 COMPLETE. Submodules -> third_party/ (synced). CMake scaffold: limbnav static lib
         (io/geometry/preprocess real + tested; limb/fit honest REFUSED stubs), measure_limb +
         batch_eval apps, gtest 17/17 green (Release, zero warnings). Scripts: make_ground_truth.py
         (13 bodies from pck00011), make_geometry_sidecar.py (limbpt expected-limb, kernel-hash
         verification, Fitzgibbon fit), check_disk_budget.sh. data/spice -> ICES cache symlink.
         154 sidecars QA'd: fit rms <= 6e-3 px, Jupiter b/a vs oblate model 0.006%, Moon +0.012%
         (aberration). Agents roster in .claude/agents/. SEARCH_LOG seeded. README rewritten.
         DISCOVERY: CKs end early-2004 -> zero tour-moon attitude; Jupiter-flyby moons are <= 47 px
         (below 150 px floor) -> G1 needs targeted tour-CK downloads in P2 (decision log).
stopped: P0 exit criteria all met (build+ctest green; 154 >= 50 sidecars; Deck live)
next:    P1 per STATE.json next_actions — start with the synthetic renderer + G0 harness
first command on resume: ctest --test-dir build  # then start P1 renderer in tests/synthetic

## Session 2026-07-17T10:05Z  (main model: fable-5, max effort)
did:     resume protocol clean (ctest 17/17, 154 sidecars, budget OK, tree clean)
plan:    P1 synthetic estimator -> G0. Order: (1) tests/synthetic renderer (AA ellipse,
         Lambert crescent, PSF, seeded noise), (2) localizers parabola/erf(+ramp model)/zernike
         with per-localizer bias unit tests, (3) coarse disk + sun-semicircle classify +
         RANSAC/fits, (4) measure() orchestrator (2-pass), (5) G0 sweep gtest, (6) wire
         measure_limb --sun-dir, (7) gate/G0-pass tag if green.
notes:   detection gate (ONNX) + distortion correction are real-data stages -> P2, logged.
         KEY RISK identified up front: Lambert low-phase limb is a sqrt ramp, not a step —
         pure-erf localizer will bias inward ~0.5-1 px; erf model gets an interior-slope
         term (ramp-aware) + unit test asserting |bias| < 0.05 px on both step and Lambert
         profiles across sigma 0.5-3.
progress: renderer+localizers+coarse+classify+RANSAC+measure() built; localizer unit suite
         green after 3 estimator fixes (exact conv model -> substitution quadrature -> M>=0
         constraint + BIC family selection; noisy-step mean bias 0.176->0.038 px).
         G0 sweep: ellipse/step family AT GATE (mean 0.021%, worst 0.167%); lambert family
         mean 0.50% (was: crossing-init fix took alpha0 per-ray scatter -23+/-36 -> -1.9+/-1.9 px;
         sweep now mean 0.234%, p99 1.43%, worst 2.03%). Residual: per-ray family ambiguity
         at low local SNR (inward on alpha<=20, outward on alpha=120 crescents).
in-flight: implementing pass-3 consensus profile refinement (pooled family/sigma/shape fit
         over all rays, then 1-DOF per-ray relocalization, then ellipse refit) in
         src/limb + measure(); expect this to close both bias families.
stopped: (in progress)
first command on resume: cmake --build build && ./build/tests/limbnav_tests --gtest_filter='G0.*'

## Session 2026-07-20T02:05Z  (main model: fable-5, max effort — continuation, no subagents)
did:     resumed mid pass-3 "joint refinement" (Huber LS on an ellipse-perturbation basis +
         pooled profile fit). Reverted a wrong "horn cut" (bad phase-angle physics, was causing
         5 REFUSED cases) -> back to a clean 0/120 refused baseline at mean 0.1645%.
         Then spent the bulk of the session chasing the residual Lambert-crescent bias at
         alpha>=60 deg down to ROOT CAUSE (verified numerically, not guessed): a pure 1D-radial
         Gaussian blur of the exact Lambert mu(d,theta) profile does not match the renderer's
         actual (pixel-aperture-averaged) output near the sqrt-profile's slope singularity at
         d=0. Built and tried a "physically exact" composite-shape estimator
         (composite_col/bilinear_sse_physical/pooled_physical_fit in limb.cpp) plus an isotropic-
         radius reparametrization of the perturbation LS plus an empirical-correction hybrid --
         full trail (what improved, what regressed, exact numbers) is in DECK.md "G0 status
         detail", not duplicated here. NET: isotropic-radius kept (free, neutral-to-positive);
         physical composite mode implemented but FORCE-DISABLED (regressed the sweep); empirical
         hybrid reverted (made the estimator ill-conditioned). Landed back on the free-shape
         (H,Cs,M>=0) model, mean 0.1645% -> currently 0.165% (isotropic reparametrization noise).
         Full suite: 22/23 gtest green, G0 fails cleanly at the accuracy threshold (not a crash,
         not a refusal spike) -- exactly the honest failure mode the harness is designed to show.
         Real-image smoke test on a real Rhea frame found a pre-existing (not new) gap: no
         detection-gate ROI seed + no Sun-hint wiring for real frames yet -> nonsensical fit.
         Logged as already-scoped P1/P2 work, not a new bug.
stopped: G0 still failing; committing the P1 estimator machinery as real, working, honestly-
         tested code per the git-cadence discipline (§11.1) -- this is NOT a "kept experiment"
         in the autoresearch-loop sense (P3 hasn't started), it's a phase checkpoint.
next:    algorithm-architect task (fresh brief from DECK.md "G0 status detail", not a re-read of
         this log): design a pixel-aperture-and/or-2D-mixing-aware limb profile correction. The
         untried next step flagged in the Deck: a SINGLE shared aperture/shape-correction
         parameter per pooled fit (not free per-ray) to avoid the ill-conditioning that killed
         the last attempt.
first command on resume: cmake --build build && ./build/tests/limbnav_tests --gtest_filter='G0.*'
  # then read research/DECK.md "G0 status detail" before touching src/limb/limb.cpp
