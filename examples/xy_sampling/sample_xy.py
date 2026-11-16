"""Standalone walkthrough of the XY sampling + look-at step.

This file re-implements, in one place and with comments, the part of the
pipeline that turns "a rectangle in XY + a list of heights" into camera poses:

    sample X,Y  ->  add the height + jitter  ->  look_at  ->  geometry gates

It deliberately imports nothing from ``viewpoint_sim.world``, so it needs no
MuJoCo, no GPU and no display: sampling is pure geometry. Every gate below is
the same arithmetic used by ``viewpoint_sim.planner.geometry_check``; the
production code interleaves them with IK / collision / path planning, which is
exactly what this demo leaves out so the sampling itself is visible.

Run:
    .venv/bin/python examples/xy_sampling/sample_xy.py
    .venv/bin/python examples/xy_sampling/sample_xy.py --seed 1 --no-gif
    .venv/bin/python examples/xy_sampling/sample_xy.py --verify outputs/demo
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# The demo lives outside the package, so make the repository root importable
# before touching viewpoint_sim. This keeps `python examples/.../sample_xy.py`
# working from any directory without an editable install.
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from viewpoint_sim.config import load_config
from viewpoint_sim.geometry import (
    angle_deg,
    box_vertices,
    image_contains,
    look_at,
    project,
    tray_ratio,
)
from viewpoint_sim.pose_generation import (
    camera_position as _camera_position,
    look_at_pose as _look_at_pose,
)


# ---------------------------------------------------------------------------
# 1-2. The two steps themselves live in the package:
#      viewpoint_sim/pose_generation.py
#      This demo imports them instead of copying, so there is one definition.
# ---------------------------------------------------------------------------
def sample_eye(cfg, height_index, rng, jitter_rng):
    """步骤 1：plane position + height + Z jitter -> optical centre.

    Thin wrapper over ``pose_generation.camera_position`` so the demo and the
    production pipeline cannot drift apart.
    """
    return _camera_position(cfg, height_index, rng, jitter_rng)


def pose_from_eye(cfg, eye):
    """步骤 2：Look-at the Food ROI centre -> full 4x4 camera pose."""
    return _look_at_pose(cfg, eye)


# ---------------------------------------------------------------------------
# 3. The geometry gates, in the same order and with the same names as production
# ---------------------------------------------------------------------------
def check_geometry(cfg, T_BC, height_index):
    """Return (metrics, None) if the pose passes, else (partial_metrics, reason)."""
    roi, cam, sampling = cfg["roi"], cfg["camera"], cfg["sampling"]

    # The ROI is grown by roi.margin_m before the check so a pose that only just
    # fits is not accepted - the margin is the "do not sit on the border" budget.
    pad = roi["margin_m"]
    food = box_vertices(np.array(roi["food_min_B"]) - pad,
                        np.array(roi["food_max_B"]) + pad)

    # -- Gate 1: does the whole 3D box land inside the RGB frame? -------------
    # 8 corners, pinhole projection, then a rectangular containment test.
    uv, z = project(food, T_BC, cam["rgb"])
    margin = np.array(sampling["image_margin_fraction"]) * [cam["rgb"]["width"], cam["rgb"]["height"]]
    if not image_contains(uv, z, cam["rgb"], margin):
        return {"food_uv": uv.tolist()}, "food_out_of_rgb_fov"

    # -- Gate 2: same test against the depth camera ---------------------------
    # T_CD is the third camera in the chain; here it is identity because the
    # depth and colour sensors are modelled as perfectly aligned.
    duv, dz = project(food, T_BC @ np.array(cam["T_CD"]), cam["depth"])
    if not image_contains(duv, dz, cam["depth"], margin):
        return {"food_uv": uv.tolist(), "food_depth_uv": duv.tolist()}, "food_out_of_depth_fov"

    # -- Gate 3: is the whole box inside the sensor's working range? ----------
    lo, hi = cam["depth_range_m"]
    if dz.min() < lo or dz.max() > hi:
        return {"food_uv": uv.tolist(), "roi_depth_min_m": float(dz.min()),
                "roi_depth_max_m": float(dz.max())}, "depth_range"

    # -- Gate 4: tilt of the optical axis away from straight down -------------
    tilt = angle_deg(T_BC[:3, 2], [0, 0, -1])
    if tilt > sampling["max_tilt_deg"]:
        return {"food_uv": uv.tolist(), "tilt_deg": tilt}, "tilt"

    # -- Gate 5: the tray must be in front of the camera, not behind it -------
    tuv, tz = project(np.array(roi["tray_corners_B"]), T_BC, cam["rgb"])
    if np.any(tz <= 0):
        return {"food_uv": uv.tolist(), "tilt_deg": tilt}, "tray_behind_camera"

    # -- Gate 6: how much of the frame does the tray cover? -------------------
    ratio = tray_ratio(tuv, cam["rgb"])
    thresholds = sampling["tray_min_ratio_by_height"]
    if len(thresholds) != len(sampling["heights_above_tray_m"]):
        raise ValueError("tray_min_ratio_by_height must have one entry per configured height")
    metrics = {
        "food_uv": uv.tolist(),
        "food_z": z.tolist(),
        "tray_uv": tuv.tolist(),
        "tray_ratio_raw": ratio,
        "roi_depth_min_m": float(dz.min()),
        "roi_depth_max_m": float(dz.max()),
        "tilt_deg": tilt,
        "margin_px": margin.tolist(),
    }
    if ratio < thresholds[height_index]:
        return metrics, "tray_ratio"
    return metrics, None


# ---------------------------------------------------------------------------
# 4. Drawing helpers (Pillow only - no matplotlib in this project)
# ---------------------------------------------------------------------------
BOX_COLOR = (140, 200, 255)
TRAY_COLOR = (255, 200, 60)
ROI_COLOR = (60, 235, 170)
SAMPLE_COLOR = (150, 165, 185)
ACCEPT_COLOR = (90, 235, 120)
REJECT_COLOR = (205, 110, 110)
BG = (14, 24, 37)


def _map_xy(x, y, extent, size, pad):
    """Base-frame XY -> pixels, with +X pointing up the page."""
    x0, x1, y0, y1 = extent
    span = max(x1 - x0, y1 - y0)
    scale = (size - 2 * pad) / span
    cx, cy = size / 2, size / 2
    return cx + (y - (y0 + y1) / 2) * scale, cy - (x - (x0 + x1) / 2) * scale


def _extent(cfg):
    """Bounding XY extent worth showing: sampling rectangle + tray + ROI."""
    s, roi = cfg["sampling"], cfg["roi"]
    poly = np.array(roi["tray_corners_B"])
    xs = [*s["x_B_m"], poly[:, 0].min(), poly[:, 0].max(),
          roi["food_min_B"][0], roi["food_max_B"][0]]
    ys = [*s["y_B_m"], poly[:, 1].min(), poly[:, 1].max(),
          roi["food_min_B"][1], roi["food_max_B"][1]]
    return min(xs) - .02, max(xs) + .02, min(ys) - .02, max(ys) + .02


def _rect(a, b):
    """Axis-aligned rectangle from two opposite pixel corners, order-normalised."""
    return [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]


def _convex_hull(points):
    """Andrew's monotone chain; returns the hull in counter-clockwise order.

    A projected box is the convex hull of its 8 projected corners, but the
    corners themselves come back from ``box_vertices`` in index order (a binary
    count), which zig-zags across the silhouette. Drawing them in that order
    produces a star, so the hull has to be taken first.
    """
    pts = sorted({(float(x), float(y)) for x, y in points})
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def draw_topview(cfg, records, upto=None, size=960):
    """Top-down map: sampling rectangle, tray, ROI footprint, candidate points."""
    extent = _extent(cfg)
    img = Image.new("RGB", (size, size), BG)
    d = ImageDraw.Draw(img)

    def P(x, y):
        return _map_xy(x, y, extent, size, 60)

    roi = cfg["roi"]
    # Tray footprint
    tray = np.array(roi["tray_corners_B"])
    d.polygon([P(p[0], p[1]) for p in tray], outline=TRAY_COLOR, width=2)
    # Food ROI footprint
    lo, hi = roi["food_min_B"], roi["food_max_B"]
    d.rectangle(_rect(P(lo[0], hi[1]), P(hi[0], lo[1])), outline=ROI_COLOR, width=2)
    # Sampling rectangle, dashed
    (x0, x1), (y0, y1) = cfg["sampling"]["x_B_m"], cfg["sampling"]["y_B_m"]
    for a, b in [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                 ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]:
        _dashed(d, P(*a), P(*b), SAMPLE_COLOR, 2, 10)

    # Every candidate sampled so far
    shown = records if upto is None else records[:upto]
    for rec in shown:
        x, y = rec["eye"][0], rec["eye"][1]
        px, py = P(x, y)
        color = ACCEPT_COLOR if rec["accepted"] else REJECT_COLOR
        r = 5 if rec["accepted"] else 4
        d.ellipse([px - r, py - r, px + r, py + r], fill=color)
        if rec["accepted"]:
            # Small arrow showing which way the camera looks, flattened to XY.
            d.line([px, py, *P(rec["target_xy"][0], rec["target_xy"][1])], fill=color, width=1)

    # Legend
    d.text((20, 18), "XY sampling - top view (+X = away from robot, up)", fill=(220, 230, 240))
    d.text((20, 40), f"samples: {len(shown)}   accepted: {sum(r['accepted'] for r in shown)}",
           fill=(180, 195, 210))
    d.text((20, size - 34), "grey = rejected   green = accepted   dashed = sampling rectangle",
           fill=(140, 155, 170))
    return img


def _dashed(draw, a, b, color, width, dash):
    a, b = np.array(a, float), np.array(b, float)
    length = np.linalg.norm(b - a)
    if length < 1e-9:
        return
    n = max(1, int(length // (2 * dash)))
    for i in range(n + 1):
        t0 = (2 * i * dash) / length
        t1 = min(1.0, (2 * i * dash + dash) / length)
        draw.line([*a + (b - a) * t0, *a + (b - a) * t1], fill=color, width=width)


def draw_frustum_3d(cfg, rec, size=(920, 720)):
    """Oblique 3D view: camera at the sampled pose, rays to the ROI corners."""
    w, h = size
    view = look_at([-0.72, -0.60, 0.66], [0.31, 0.0, 0.20], [0, 0, 1])
    cam = {"width": w, "height": h, "fx": 560.0, "fy": 560.0,
           "cx": w / 2 - .5, "cy": h / 2 - .5}

    img = Image.new("RGB", size, BG)
    d = ImageDraw.Draw(img)

    def P(points_B):
        uv, z = project(np.atleast_2d(points_B), view, cam)
        return uv, z

    def segment(a, b, color, width=1):
        uv, z = P([a, b])
        if not np.all(z > 0.01):
            return
        p0 = (float(uv[0][0]), float(uv[0][1]))
        p1 = (float(uv[1][0]), float(uv[1][1]))
        # Skip segments whose endpoints are far outside the canvas; PIL handling
        # of huge coordinates is unreliable.
        if not all(-2000 <= v <= 4000 for v in (*p0, *p1)):
            return
        d.line([p0, p1], fill=color, width=width)

    def wire(lo, hi, color, width=1):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        pts = box_vertices(lo, hi)
        for i in range(8):
            for bit in (1, 2, 4):
                j = i ^ bit
                if i < j:
                    segment(pts[i], pts[j], color, width)

    scene = cfg["scene"]
    table = scene["table"]
    half = np.array(table["size_m"]) / 2
    wire(np.array(table["position_B_m"]) - half, np.array(table["position_B_m"]) + half, (58, 70, 86))

    tray = scene["tray"]
    thalf = np.array(tray["size_m"]) / 2
    wire(np.array(tray["position_B_m"]) - thalf, np.array(tray["position_B_m"]) + thalf, TRAY_COLOR)

    roi = cfg["roi"]
    wire(roi["food_min_B"], roi["food_max_B"], ROI_COLOR, 2)

    for food in scene["foods"]:
        p = np.array(food["position_B_m"])
        if food["shape"] == "box":
            fh = np.array(food["size_m"]) / 2
        else:
            fh = np.repeat(food["radius_m"], 3)
        wire(p - fh, p + fh, tuple(int(c * 255) for c in food["rgba"][:3]))

    # Camera: rays to the (padded) ROI corners plus the optical axis.
    eye = np.array(rec["eye"])
    T = np.array(rec["T_BC"])
    pad = roi["margin_m"]
    corners = box_vertices(np.array(roi["food_min_B"]) - pad, np.array(roi["food_max_B"]) + pad)
    for c in corners:
        segment(eye, c, (70, 110, 140))
    axis_end = eye + T[:3, 2] * 0.30
    segment(eye, axis_end, ACCEPT_COLOR, 2)

    px, py = P(eye)[0][0]
    d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=ACCEPT_COLOR)

    d.text((18, 16), f"{rec['id']}   h = {rec['height'] * 1000:.0f} mm   "
                     f"tilt = {rec['metrics']['tilt_deg']:.1f} deg", fill=(225, 235, 245))
    d.text((18, 38), f"tray ratio = {rec['metrics']['tray_ratio_raw']:.3f}   "
                     f"depth = {rec['metrics']['roi_depth_min_m'] * 1000:.0f}"
                     f"-{rec['metrics']['roi_depth_max_m'] * 1000:.0f} mm",
           fill=(170, 190, 205))
    d.text((18, h - 30), "green = optical axis   cyan box = Food ROI   yellow box = tray",
           fill=(130, 150, 165))
    return img


def draw_fov(cfg, rec, scale=1):
    """What the camera would see: ROI polygon, tray polygon, safety-margin box."""
    cam = cfg["camera"]["rgb"]
    w, h = cam["width"] * scale, cam["height"] * scale
    pad = 60
    img = Image.new("RGB", (w + 2 * pad, h + 2 * pad + 40), BG)
    d = ImageDraw.Draw(img)

    def P(u, v):
        return pad + u * scale, 40 + pad + v * scale

    # Frame + safety margin rectangle
    d.rectangle([P(0, 0), P(cam["width"] - 1, cam["height"] - 1)], outline=(70, 85, 100))
    margin = np.array(cfg["sampling"]["image_margin_fraction"]) * [cam["width"], cam["height"]]
    d.rectangle([P(margin[0], margin[1]),
                 P(cam["width"] - 1 - margin[0], cam["height"] - 1 - margin[1])],
                outline=(110, 130, 150))

    tray = [P(*p) for p in rec["metrics"]["tray_uv"]]
    d.polygon(tray, outline=TRAY_COLOR, width=2)

    # The ROI silhouette is the convex hull of its 8 projected corners: a box
    # seen from a generic direction shows 6 of its 12 edges, so the outline is
    # a hexagon, not a quadrilateral. A quad only appears in a face-on view.
    corners = rec["metrics"]["food_uv"]
    depth = rec["metrics"].get("roi_depth_min_m")
    if all(z > 0 for z in rec.get("food_z", [1.0] * len(corners))):
        hull = [P(*p) for p in _convex_hull(corners)]
        d.polygon(hull, outline=ROI_COLOR, width=2)
    else:
        # Some corner is behind the camera; the hull would be meaningless.
        d.text((pad, 40 + pad + h + 32), "ROI partly behind the camera",
               fill=REJECT_COLOR)

    d.text((pad, 14), f"{rec['id']} - camera image, tray ratio {rec['metrics']['tray_ratio_raw']:.3f}",
           fill=(225, 235, 245))
    d.text((pad, 40 + pad + h + 12),
           f"green = padded Food ROI silhouette ({len(_convex_hull(corners))}-gon)   "
           "yellow = tray   grey box = 10% safe margin",
           fill=(130, 150, 165))
    return img


# ---------------------------------------------------------------------------
# 5. Main loop: sample, gate, dedupe - nothing else
# ---------------------------------------------------------------------------
def run(cfg, seed=None, quiet=False):
    cfg = json.loads(json.dumps(cfg))
    if seed is not None:
        cfg["seed"] = seed
    s = cfg["sampling"]

    # Same spawn(4) as production so the streams line up index for index.
    sample_rng, _, _, jitter_rng = [np.random.default_rng(k)
                                    for k in np.random.SeedSequence(cfg["seed"]).spawn(4)]

    records, summary = [], []
    accepted_axes, all_axes = [], []
    target_xy = (np.array(cfg["roi"]["food_min_B"]) + np.array(cfg["roi"]["food_max_B"])) / 2

    for height_index, h in enumerate(s["heights_above_tray_m"]):
        accepted, attempts, reasons = 0, 0, {}
        axes = []
        while accepted < s["target_per_height"] and attempts < s["max_attempts_per_height"]:
            attempts += 1
            candidate_id = f"h{height_index:02d}_c{attempts:04d}"

            eye, base_h, jitter = sample_eye(cfg, height_index, sample_rng, jitter_rng)
            T_BC = pose_from_eye(cfg, eye)
            metrics, reason = check_geometry(cfg, T_BC, height_index)

            # Dedupe comes before the (expensive) robot checks in production;
            # here it is the last gate because everything else is already cheap.
            if reason is None:
                compare = all_axes if s["deduplicate_across_heights"] else axes
                if any(angle_deg(T_BC[:3, 2], d) < s["min_axis_angle_deg"] for d in compare):
                    reason = "duplicate_axis"

            ok = reason is None
            if ok:
                accepted += 1
                axes.append(T_BC[:3, 2].copy())
                all_axes.append(T_BC[:3, 2].copy())
            else:
                reasons[reason] = reasons.get(reason, 0) + 1

            records.append({
                "id": candidate_id,
                "height_index": height_index,
                "height": h + jitter,
                "base_height": base_h,
                "jitter": jitter,
                "eye": eye,
                "target_xy": target_xy,
                "T_BC": T_BC,
                "accepted": ok,
                "reason": reason,
                "metrics": metrics,
            })
            if not quiet:
                mark = "OK  " if ok else "rej "
                print(f"  {mark}{candidate_id}  eye=({eye[0]:.4f}, {eye[1]:.4f}, {eye[2]:.4f})  "
                      f"{reason or 'accepted'}")

        summary.append({
            "height_above_tray_m": h,
            "accepted": accepted,
            "target": s["target_per_height"],
            "attempts": attempts,
            "rejections": reasons,
            "acceptance_rate": accepted / max(1, attempts),
        })
        if not quiet:
            print(f"height {h:.3f} m: {accepted}/{s['target_per_height']} from {attempts} attempts")
    return records, summary


def write_outputs(cfg, records, summary, output, gif=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    accepted = [(i, r) for i, r in enumerate(records) if r["accepted"]]

    # CSV: one row per attempt, easy to open in any spreadsheet.
    with (output / "candidates.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["id", "height_index", "height_m", "jitter_m",
                    "eye_x", "eye_y", "eye_z", "accepted", "reason", "tray_ratio"])
        for r in records:
            ratio = r["metrics"].get("tray_ratio_raw") if r["metrics"] else None
            w.writerow([r["id"], r["height_index"], f"{r['height']:.6f}", f"{r['jitter']:.6f}",
                        *(f"{v:.6f}" for v in r["eye"]), r["accepted"], r["reason"] or "accepted",
                        "" if ratio is None else f"{ratio:.6f}"])

    # summary.txt mirrors the fields run() writes into summary.json.
    lines = [f"seed: {cfg['seed']}",
             f"x range: {cfg['sampling']['x_B_m']} m",
             f"y range: {cfg['sampling']['y_B_m']} m",
             f"target per height: {cfg['sampling']['target_per_height']}",
             f"z jitter: {cfg['sampling']['z_jitter_m']} m",
             ""]
    for row in summary:
        lines.append(f"height {row['height_above_tray_m']:.3f} m: "
                     f"{row['accepted']}/{row['target']} from {row['attempts']} attempts "
                     f"(rate {row['acceptance_rate']:.2f})")
        for reason, count in sorted(row["rejections"].items()):
            lines.append(f"    {reason}: {count}")
    (output / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # Static top view with everything on it.
    draw_topview(cfg, records).save(output / "topview.png")

    # Per-accepted-view images: the 3D frustum, and what the sensor would see.
    for _, rec in accepted:
        draw_frustum_3d(cfg, rec).save(output / f"frustum_{rec['id']}.png")
        draw_fov(cfg, rec).save(output / f"fov_{rec['id']}.png")

    # Animated build-up: one frame per accepted view (as requested).
    if gif and accepted:
        frames = []
        for n, (index, rec) in enumerate(accepted):
            frame = draw_topview(cfg, records, upto=index + 1, size=720)
            d = ImageDraw.Draw(frame)
            d.rectangle([0, 0, 719, 719], outline=ACCEPT_COLOR, width=3)
            d.text((20, 66), f"accepted {n + 1}/{len(accepted)}   {rec['id']}",
                   fill=ACCEPT_COLOR)
            frames.append(frame)
        frames.append(draw_topview(cfg, records, size=720))
        frames[0].save(output / "sampling.gif", save_all=True, append_images=frames[1:],
                       duration=700, loop=0)
    return output


def verify(cfg, records, result_dir):
    """Compare the sampled eye points with the poses recorded by a real run."""
    views = json.loads((Path(result_dir) / "views.json").read_text(encoding="utf-8"))
    recorded = {v["id"]: np.array(v["T_BC_target"])[:3, 3] for v in views}
    matched, mismatched, missing = 0, [], []
    for rec in records:
        if not rec["accepted"]:
            continue
        if rec["id"] not in recorded:
            missing.append(rec["id"])
            continue
        if np.allclose(rec["eye"], recorded[rec["id"]], atol=1e-9):
            matched += 1
        else:
            mismatched.append((rec["id"], rec["eye"], recorded[rec["id"]]))
    print(f"compared {matched + len(mismatched)} accepted view(s) against {result_dir}/views.json")
    for cid, got, want in mismatched:
        print(f"  MISMATCH {cid}: demo={np.round(got, 6)} recorded={np.round(want, 6)}")
    if missing:
        print(f"  absent from views.json: {missing}")
    print(f"{matched} matched, {len(mismatched)} mismatched")
    return not mismatched and not missing


def main(argv=None):
    p = argparse.ArgumentParser(description="Standalone XY sampling walkthrough")
    p.add_argument("--config", default=None)
    p.add_argument("--output", default="outputs/my_xy_sampling")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--no-gif", action="store_true")
    p.add_argument("--verify", metavar="RESULT_DIR", default=None,
                   help="Compare sampled eye points with RESULT_DIR/views.json")
    p.add_argument("--quiet", action="store_true")
    a = p.parse_args(argv)

    cfg = load_config(a.config)
    # Apply the seed override before sampling so summary.txt reports the seed
    # that was actually used, not the one in the file.
    if a.seed is not None:
        cfg["seed"] = a.seed
    records, summary = run(cfg, quiet=a.quiet)
    out = write_outputs(cfg, records, summary, a.output, gif=not a.no_gif)

    accepted = sum(r["accepted"] for r in records)
    print(f"\n{len(records)} attempts, {accepted} accepted, {len(records) - accepted} rejected")
    print(f"wrote {out.resolve()}")

    if a.verify:
        print()
        return 0 if verify(cfg, records, a.verify) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
