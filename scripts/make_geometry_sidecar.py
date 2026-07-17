#!/usr/bin/env python3
"""Per-image SPICE geometry sidecars (§6.2) — extends the submodule's
compute_spacecraft_state.py.

For each manifest image (restricted to rows with verified SPICE state), emits
data/sidecar/<image_id>.json containing everything the eval harness compares
against — and nothing the flight binary is allowed to see (§4.3):

  et, range_km, sub_observer_lat_deg (planetocentric B),
  north_pole_position_angle_deg, sun_dir_cam, phase_angle_deg,
  predicted_center_px, and expected_limb: CSPICE limbpt() on the PCK
  tri-axial ellipsoid, projected to pixels, ellipse-fitted →
  (a_px, b_px, phi_deg, center) + the corresponding km values.

Pixel convention (P0, flagged PIXEL_CONVENTION_UNVALIDATED in every sidecar
until the P1 real-image cross-check):
  - camera frame CASSINI_ISS_NAC, boresight read from the IK at runtime;
  - focal length & pitch from the submodule's instruments.csv (§3 — never
    hardcoded here);
  - principal point at ((nx-1)/2, (ny-1)/2) with pixel centers at integers;
  - px = cx + f_px * (v·X̂)/(v·B̂),  py = cy + f_px * (v·Ŷ)/(v·B̂)
    where B̂ is the IK boresight, X̂/Ŷ the camera frame axes;
  - north-pole position angle: atan2(d·X̂, -(d·Ŷ)) in degrees — 0° is image
    "up" (-y), 90° is image right (+x) — for the pole direction d projected
    on the image plane.

Kernel integrity: every loaded kernel's sha256 must match the 16-hex prefix
recorded in the submodule's spacecraft_state.csv (same kernel set); any
mismatch aborts. Hashes are cached in data/spice_hashes.json keyed by
(size, mtime).

Usage:
  .venv/bin/python scripts/make_geometry_sidecar.py \
      [--bodies rhea,dione,tethys,enceladus,mimas,iapetus] \
      [--limit N] [--ncuts 360] [--out data/sidecar]
"""

import argparse
import csv
import glob
import hashlib
import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone

import numpy as np
import spiceypy as spice

SCHEMA_VERSION = 1
OBSERVER = "CASSINI"
CAMERA_FRAME = "CASSINI_ISS_NAC"
NAC_INST_ID = -82360
ABCORR = "LT+S"
POLAR_DEGENERATE_DEG = 60.0  # §5.2

TEXT_KERNELS = ["naif0012.tls", "cas00172.tsc", "cas_v43.tf",
                "cas_iss_v10.ti", "pck00011.tpc"]

DEFAULT_BODIES = "rhea,dione,tethys,enceladus,mimas,iapetus"

# manifest body label -> SPICE body name (subset we generate sidecars for;
# same mapping the submodule uses, § "assets in hand")
BODY_MAP = {
    "rhea": "RHEA", "dione": "DIONE", "tethys": "TETHYS",
    "enceladus": "ENCELADUS", "mimas": "MIMAS", "iapetus": "IAPETUS",
    "titan": "TITAN", "saturn": "SATURN", "jupiter": "JUPITER",
    "europa": "EUROPA", "ganymede": "GANYMEDE", "callisto": "CALLISTO",
    "io": "IO", "hyperion": "HYPERION", "phoebe": "PHOEBE",
    "janus": "JANUS", "epimetheus": "EPIMETHEUS", "moon": "MOON",
}


def repo_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def git_sha(cwd: str) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short=12", "HEAD"], cwd=cwd, text=True
        ).strip()
    except Exception:
        return "unknown"


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_kernels(kdir: str):
    """Same order as the submodule's compute_spacecraft_state.py."""
    loaded = []
    for k in TEXT_KERNELS:
        p = os.path.join(kdir, k)
        if not os.path.exists(p):
            sys.exit(f"ERROR: required text kernel missing: {p}")
        spice.furnsh(p)
        loaded.append(k)
    for p in sorted(glob.glob(os.path.join(kdir, "*.bsp"))):
        spice.furnsh(p)
        loaded.append(os.path.basename(p))
    for p in sorted(glob.glob(os.path.join(kdir, "*rc.bc"))):
        spice.furnsh(p)
        loaded.append(os.path.basename(p))
    return loaded


def hash_kernels(kdir: str, names, cache_path: str):
    """sha256 per kernel, cached by (size, mtime) — 3.4 GB rehash is slow."""
    cache = {}
    if os.path.exists(cache_path):
        try:
            with open(cache_path) as f:
                cache = json.load(f)
        except Exception:
            cache = {}
    out, dirty = {}, False
    for name in names:
        p = os.path.join(kdir, name)
        st = os.stat(p)
        key = f"{st.st_size}:{int(st.st_mtime)}"
        ent = cache.get(name)
        if ent and ent.get("key") == key:
            out[name] = ent["sha256"]
        else:
            out[name] = sha256_file(p)
            cache[name] = {"key": key, "sha256": out[name]}
            dirty = True
    if dirty:
        tmp = cache_path + ".tmp"
        os.makedirs(os.path.dirname(cache_path), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump(cache, f, indent=1)
        os.replace(tmp, cache_path)
    return out


def verify_against_state_csv(state_csv: str, kernel_hashes: dict):
    """The recorded 16-hex prefixes in spacecraft_state.csv must match."""
    with open(state_csv) as f:
        row = next(csv.DictReader(f))
    recorded = {}
    for ent in row["kernel_sha256"].split("; "):
        name, pref = ent.rsplit(":", 1)
        recorded[name.strip()] = pref.strip()
    mismatches = [
        n for n, pref in recorded.items()
        if n in kernel_hashes and not kernel_hashes[n].startswith(pref)
    ]
    if mismatches:
        sys.exit(f"ERROR: kernel hash mismatch vs spacecraft_state.csv: "
                 f"{mismatches} — refusing to generate sidecars from a "
                 f"different kernel set")
    missing = [n for n in recorded if n not in kernel_hashes]
    return missing


def load_instrument(instr_csv: str):
    with open(instr_csv) as f:
        for row in csv.DictReader(f):
            if row["mission"] == "cassini" and row["instrument"] == "issna":
                return {
                    "nx": int(row["array_px_x"]),
                    "ny": int(row["array_px_y"]),
                    "pitch_um": float(row["pixel_pitch_um"]),
                    "focal_mm": float(row["focal_length_mm"]),
                    "ifov_arcsec": float(row["ifov_arcsec_per_px"]),
                }
    sys.exit(f"ERROR: cassini/issna row not found in {instr_csv}")


def fit_ellipse_direct(x: np.ndarray, y: np.ndarray):
    """Fitzgibbon/Pilu/Fisher direct LS ellipse fit (the same family the C++
    side will use). Returns (a, b, phi_rad, cx, cy) with a >= b."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    # center/scale for conditioning
    mx, my = x.mean(), y.mean()
    sx = max(x.std(), 1e-9)
    sy = max(y.std(), 1e-9)
    s = (sx + sy) / 2.0
    xn, yn = (x - mx) / s, (y - my) / s

    D1 = np.column_stack([xn * xn, xn * yn, yn * yn])
    D2 = np.column_stack([xn, yn, np.ones_like(xn)])
    S1 = D1.T @ D1
    S2 = D1.T @ D2
    S3 = D2.T @ D2
    T = -np.linalg.solve(S3, S2.T)
    M = S1 + S2 @ T
    C = np.array([[0.0, 0.0, 2.0], [0.0, -1.0, 0.0], [2.0, 0.0, 0.0]])
    evals, evecs = np.linalg.eig(np.linalg.solve(C, M))
    cond = 4.0 * evecs[0] * evecs[2] - evecs[1] ** 2
    a1 = evecs[:, np.real(cond) > 0][:, 0].real
    coef = np.concatenate([a1, T @ a1])  # A,B,C,D,E,F in normalized coords

    A, B, Cc, D, E, F = coef
    # de-normalize: x = (X-mx)/s etc.
    A2 = A / (s * s)
    B2 = B / (s * s)
    C2 = Cc / (s * s)
    D2c = -2 * A * mx / (s * s) - B * my / (s * s) + D / s
    E2 = -2 * Cc * my / (s * s) - B * mx / (s * s) + E / s
    F2 = (A * mx * mx / (s * s) + B * mx * my / (s * s) + Cc * my * my / (s * s)
          - D * mx / s - E * my / s + F)
    A, B, Cc, D, E, F = A2, B2, C2, D2c, E2, F2

    # conic -> geometric parameters
    den = B * B - 4 * A * Cc
    if den >= 0:
        raise ValueError("degenerate conic (not an ellipse)")
    cx = (2 * Cc * D - B * E) / den
    cy = (2 * A * E - B * D) / den
    # evaluate constant at center
    Fc = A * cx * cx + B * cx * cy + Cc * cy * cy + D * cx + E * cy + F
    M0 = np.array([[A, B / 2], [B / 2, Cc]])
    evals2, evecs2 = np.linalg.eigh(M0)
    # semi-axes: sqrt(-Fc / eigenvalue)
    with np.errstate(invalid="raise"):
        axes = np.sqrt(-Fc / evals2)
    order = np.argsort(axes)[::-1]  # major first
    a_ax, b_ax = float(axes[order[0]]), float(axes[order[1]])
    vmaj = evecs2[:, order[0]]
    phi = math.atan2(vmaj[1], vmaj[0])
    return a_ax, b_ax, phi, float(cx), float(cy), (A, B, Cc, D, E, F)


def sampson_rms(coef, x, y):
    A, B, C, D, E, F = coef
    val = A * x * x + B * x * y + C * y * y + D * x + E * y + F
    gx = 2 * A * x + B * y + D
    gy = 2 * C * y + B * x + E
    g = np.sqrt(gx * gx + gy * gy)
    g[g < 1e-12] = 1e-12
    return float(np.sqrt(np.mean((val / g) ** 2)))


class Camera:
    def __init__(self, instr):
        self.f_px = instr["focal_mm"] * 1000.0 / instr["pitch_um"]
        self.ifov_rad = 1.0 / self.f_px
        self.cx = (instr["nx"] - 1) / 2.0
        self.cy = (instr["ny"] - 1) / 2.0
        self.nx, self.ny = instr["nx"], instr["ny"]
        bs = np.array(spice.gdpool(f"INS{NAC_INST_ID}_BORESIGHT", 0, 3))
        self.boresight = bs / np.linalg.norm(bs)
        # camera-frame basis (unit axes in the frame itself)
        self.xhat = np.array([1.0, 0.0, 0.0])
        self.yhat = np.array([0.0, 1.0, 0.0])
        # sanity: IFOV vs the recorded arcsec value
        rec = instr["ifov_arcsec"] / (180.0 * 3600.0 / math.pi)
        if abs(self.ifov_rad - rec) / rec > 1e-3:
            sys.exit("ERROR: IFOV cross-check failed (instruments.csv vs pitch/f)")

    def project(self, v_cam: np.ndarray):
        """Unit-agnostic pinhole projection. None if behind camera."""
        z = float(np.dot(v_cam, self.boresight))
        if z <= 0:
            return None
        return (self.cx + self.f_px * float(np.dot(v_cam, self.xhat)) / z,
                self.cy + self.f_px * float(np.dot(v_cam, self.yhat)) / z)


def process_image(img, cam, ncuts):
    """Compute the sidecar dict for one manifest row, or (None, reason)."""
    target = BODY_MAP[img["body"]]
    fixref = "IAU_" + target
    flags = ["PIXEL_CONVENTION_UNVALIDATED"]

    et = spice.utc2et(img["timestamp_utc"])

    # apparent position of target center as seen from Cassini
    v_j2000, lt = spice.spkpos(target, et, "J2000", ABCORR, OBSERVER)
    v_j2000 = np.array(v_j2000)
    range_km = float(np.linalg.norm(v_j2000))

    try:
        r_j2c = np.array(spice.pxform("J2000", CAMERA_FRAME, et))
    except Exception:
        return None, "NO_CK_COVERAGE"

    center_px = cam.project(r_j2c @ (v_j2000 / range_km))
    if center_px is None:
        return None, "TARGET_BEHIND_CAMERA"
    if not (-0.25 * cam.nx <= center_px[0] <= 1.25 * cam.nx and
            -0.25 * cam.ny <= center_px[1] <= 1.25 * cam.ny):
        flags.append("TARGET_FAR_OFF_BORESIGHT")

    # sub-observer planetocentric latitude B
    spoint, _, _ = spice.subpnt("NEAR POINT/ELLIPSOID", target, et, fixref,
                                ABCORR, OBSERVER)
    _, _, B_rad = spice.reclat(np.array(spoint))
    if abs(math.degrees(B_rad)) > POLAR_DEGENERATE_DEG:
        flags.append("POLAR_DEGENERATE")

    # phase angle
    phase_rad = spice.phaseq(et, target, "SUN", OBSERVER, ABCORR)

    # illumination direction (target -> Sun), in camera frame
    s_j2000, _ = spice.spkpos("SUN", et, "J2000", ABCORR, target)
    s_j2000 = np.array(s_j2000)
    sun_dir_cam = r_j2c @ (s_j2000 / np.linalg.norm(s_j2000))

    # north-pole position angle in the image
    r_bf2j = np.array(spice.pxfrm2(fixref, "J2000", et - lt, et))
    pole_cam = r_j2c @ (r_bf2j @ np.array([0.0, 0.0, 1.0]))
    npa_deg = math.degrees(math.atan2(
        float(np.dot(pole_cam, cam.xhat)), -float(np.dot(pole_cam, cam.yhat))))

    # expected limb via limbpt on the PCK tri-axial ellipsoid
    rolstp = 2.0 * math.pi / ncuts
    pts_px = []
    for refvec in ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]):
        try:
            npts, points, epochs, tangts = spice.limbpt(
                "TANGENT/ELLIPSOID", target, et, fixref, ABCORR,
                "ELLIPSOID LIMB", OBSERVER, refvec, rolstp, ncuts,
                1.0e-4, 1.0e-7, ncuts)
            break
        except Exception as e:  # refvec parallel to limb axis, retry
            err = e
    else:
        return None, f"LIMBPT_FAILED: {err}"

    idx = 0
    for i in range(ncuts):
        for _ in range(npts[i]):
            tang = np.array(tangts[idx])
            epoch = epochs[idx]
            idx += 1
            xf = np.array(spice.pxfrm2(fixref, "J2000", epoch, et))
            u = xf @ tang
            u = r_j2c @ (u / np.linalg.norm(u))
            p = cam.project(u)
            if p is not None:
                pts_px.append(p)
    if len(pts_px) < 16:
        return None, f"LIMBPT_TOO_FEW_POINTS: {len(pts_px)}"

    pts = np.array(pts_px)
    try:
        a_px, b_px, phi, ecx, ecy, coef = fit_ellipse_direct(pts[:, 0], pts[:, 1])
    except Exception as e:
        return None, f"ELLIPSE_FIT_FAILED: {e}"
    rms_px = sampson_rms(coef, pts[:, 0], pts[:, 1])

    # QA cross-check for near-spherical bodies: circle radius from mean radius
    _, radii = spice.bodvrd(target, "RADII", 3)
    r_mean = float(sum(radii)) / 3.0
    sphere_check_px = (math.asin(min(1.0, r_mean / range_km)) / cam.ifov_rad
                       if range_km > r_mean else float("nan"))

    sidecar = {
        "schema_version": SCHEMA_VERSION,
        "image_id": img["image_id"],
        "body": img["body"],
        "spice_target": target,
        "filter": img.get("filter", ""),
        "timestamp_utc": img["timestamp_utc"],
        "et": float(et),
        "range_km": range_km,
        "light_time_s": float(lt),
        "sub_observer_lat_deg": math.degrees(B_rad),
        "north_pole_position_angle_deg": npa_deg,
        "sun_dir_cam": [float(x) for x in sun_dir_cam],
        "phase_angle_deg": math.degrees(phase_rad),
        "predicted_center_px": [center_px[0], center_px[1]],
        "expected_limb": {
            "a_px": a_px,
            "b_px": b_px,
            "phi_deg": math.degrees(phi),
            "center_px": [ecx, ecy],
            "a_km": range_km * math.sin(a_px * cam.ifov_rad),
            "b_km": range_km * math.sin(b_px * cam.ifov_rad),
            "n_pts": int(len(pts_px)),
            "fit_rms_px": rms_px,
            "sphere_check_px": sphere_check_px,
        },
        "body_radii_km": [float(radii[0]), float(radii[1]), float(radii[2])],
        "flags": flags,
    }
    return sidecar, None


def main() -> int:
    root = repo_root()
    sub = os.path.join(root, "third_party/object_detection_opencv_cpp")
    ap = argparse.ArgumentParser()
    ap.add_argument("--kernels", default=os.path.join(root, "data/spice"))
    ap.add_argument("--manifest",
                    default=os.path.join(sub, "artifacts/cassini_issna/image_manifest.csv"))
    ap.add_argument("--state-csv",
                    default=os.path.join(sub, "artifacts/spacecraft_state.csv"))
    ap.add_argument("--instruments",
                    default=os.path.join(sub, "artifacts/instruments.csv"))
    ap.add_argument("--out", default=os.path.join(root, "data/sidecar"))
    ap.add_argument("--bodies", default=DEFAULT_BODIES,
                    help="comma-separated manifest body labels")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--ncuts", type=int, default=360)
    args = ap.parse_args()

    bodies = {b.strip().lower() for b in args.bodies.split(",") if b.strip()}
    unknown = bodies - set(BODY_MAP)
    if unknown:
        sys.exit(f"ERROR: no SPICE mapping for bodies: {sorted(unknown)}")

    print("Loading kernels...")
    names = load_kernels(args.kernels)
    print(f"  {len(names)} kernels loaded from {args.kernels}")

    print("Verifying kernel hashes against spacecraft_state.csv...")
    hashes = hash_kernels(args.kernels, names,
                          os.path.join(root, "data/spice_hashes.json"))
    missing = verify_against_state_csv(args.state_csv, hashes)
    if missing:
        print(f"  note: kernels recorded but not on disk: {missing}")
    print("  hashes OK")

    # images that have verified state WITH attitude
    with_state = {}
    with open(args.state_csv) as f:
        for row in csv.DictReader(f):
            if row["attitude_q_w"].strip():
                with_state[row["image_id"]] = row

    cam = Camera(load_instrument(args.instruments))

    todo = []
    with open(args.manifest) as f:
        for row in csv.DictReader(f):
            if row["body"].strip().lower() in bodies and row["image_id"] in with_state:
                todo.append(row)
    if args.limit:
        todo = todo[: args.limit]
    print(f"{len(todo)} images to process (bodies: {sorted(bodies)})")

    os.makedirs(args.out, exist_ok=True)
    gen_sha = git_sha(root)
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    combined = hashlib.sha256(
        "".join(f"{k}:{v}" for k, v in sorted(hashes.items())).encode()
    ).hexdigest()[:16]

    ok, skipped, reasons = 0, 0, {}
    for i, img in enumerate(todo):
        img = dict(img)
        img["body"] = img["body"].strip().lower()
        try:
            sidecar, reason = process_image(img, cam, args.ncuts)
        except Exception as e:
            # e.g. SPKINSUFFDATA: kernel set lacks the body's ephemeris at
            # this epoch (2001 Saturn-approach frames). Skip, never fake.
            short = str(e).split("\n")[0][:80]
            sidecar, reason = None, f"SPICE_ERROR ({short})"
        if sidecar is None:
            skipped += 1
            key = reason.split(":")[0]
            reasons[key] = reasons.get(key, 0) + 1
            continue

        # consistency: |range| vs the recorded state row's position magnitude
        srow = with_state[img["image_id"]]
        rec_range = math.sqrt(sum(float(srow[f"position_km_{a}"]) ** 2
                                  for a in "xyz"))
        if abs(rec_range - sidecar["range_km"]) / rec_range > 1e-3:
            sidecar["flags"].append("RANGE_MISMATCH_STATECSV")

        sidecar["provenance"] = {
            "generated_utc": now,
            "generator_git_sha": gen_sha,
            "kernel_set_sha16": combined,
            "abcorr": ABCORR,
            "observer": OBSERVER,
            "camera_frame": CAMERA_FRAME,
            "ncuts": args.ncuts,
        }
        out_path = os.path.join(args.out, f"{img['image_id']}.json")
        tmp = out_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(sidecar, f, indent=1)
        os.replace(tmp, out_path)
        ok += 1
        if (i + 1) % 50 == 0:
            print(f"  {i + 1}/{len(todo)} done...")

    spice.kclear()
    print(f"\nwrote {ok} sidecars to {args.out}; skipped {skipped}")
    if reasons:
        for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"  skip reason {k}: {v}")
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
