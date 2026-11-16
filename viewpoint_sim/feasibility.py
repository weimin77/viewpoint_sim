"""Geometric feasibility gates for a candidate camera pose.

Six pure checks decide whether a pose is worth handing to the robot at all:

    1  the padded Food ROI projects inside the RGB frame (minus the safe margin)
    2  the same for the depth sensor (via T_CD)
    3  every ROI corner sits inside the configured depth range
    4  the optical axis is not tilted further than max_tilt_deg
    5  the tray corners are in front of the camera, not behind it
    6  the tray covers at least the configured fraction of the frame

They are ordered cheapest-first and short-circuit: a pose that fails gate 1 never
pays for the polygon clipping in gate 6. That ordering is also what makes the
rejection reasons meaningful - the first failure is the one reported, and the
statistics in ``summary.json`` show which limit actually binds at each height.

Nothing here touches MuJoCo, rendering or the random stream: given a config and a
4x4 pose the verdict is deterministic.

Two entry points share one implementation:

    check(cfg, T_BC, height_index)            -> (metrics | None, reason)
    check_detailed(cfg, T_BC, height_index)   -> (partial_metrics, reason)

``check`` is the production contract (failures carry no metrics). ``check_detailed``
keeps whatever was computed before the failing gate so a diagnostic tool can draw
the offending projection.
"""
import numpy as np

from .geometry import box_vertices, image_contains, project, tray_ratio, angle_deg


def padded_food_corners(cfg):
    """The 8 corners of the Food ROI grown by ``roi.margin_m``.

    The margin is not cosmetic: it is the border budget that keeps a pose which
    "just fits" from being accepted and then clipped after a small calibration
    error.
    """
    roi = cfg["roi"]
    pad = roi["margin_m"]
    return box_vertices(np.asarray(roi["food_min_B"], float) - pad,
                        np.asarray(roi["food_max_B"], float) + pad)


def image_margin_px(cfg):
    """Safe-margin in pixels, as (horizontal, vertical)."""
    cam = cfg["camera"]["rgb"]
    fraction = np.asarray(cfg["sampling"]["image_margin_fraction"], float)
    return fraction * [cam["width"], cam["height"]]


def tray_threshold(cfg, height_index):
    """Configured minimum tray ratio for this height layer.

    Raises if the threshold list and the height list disagree, because a silent
    default here would quietly disable the gate.
    """
    sampling = cfg["sampling"]
    thresholds = sampling["tray_min_ratio_by_height"]
    if len(thresholds) != len(sampling["heights_above_tray_m"]):
        raise ValueError("tray_min_ratio_by_height must have one entry per configured height")
    return thresholds[height_index]


def check_detailed(cfg, T_BC, height_index):
    """Run the gates, keeping partial metrics on failure.

    Returns ``(metrics, reason)``. ``reason`` is ``None`` when the pose passes,
    otherwise one of the strings below; ``metrics`` is always a dict, holding
    only the quantities computed before the failing gate.
    """
    roi, cam, sampling = cfg["roi"], cfg["camera"], cfg["sampling"]
    T_BC = np.asarray(T_BC, float)

    food = padded_food_corners(cfg)
    margin = image_margin_px(cfg)

    # -- Gate 1: the whole 3D box lands inside the RGB frame -------------------
    uv, z = project(food, T_BC, cam["rgb"])
    if not image_contains(uv, z, cam["rgb"], margin):
        return {"food_uv": uv.tolist(), "food_z": z.tolist()}, "food_out_of_rgb_fov"

    # -- Gate 2: same test against the depth camera ---------------------------
    # T_CD is identity for an aligned RGB-D sensor, so this gate degenerates to
    # gate 1 there; it only becomes informative with separate sensors.
    duv, dz = project(food, T_BC @ np.asarray(cam["T_CD"], float), cam["depth"])
    if not image_contains(duv, dz, cam["depth"], margin):
        return ({"food_uv": uv.tolist(), "food_z": z.tolist(),
                 "food_depth_uv": duv.tolist()}, "food_out_of_depth_fov")

    # -- Gate 3: is the whole box inside the sensor's working range? ----------
    lo, hi = cam["depth_range_m"]
    if dz.min() < lo or dz.max() > hi:
        return ({"food_uv": uv.tolist(), "food_z": z.tolist(),
                 "roi_depth_min_m": float(dz.min()),
                 "roi_depth_max_m": float(dz.max())}, "depth_range")

    # -- Gate 4: tilt of the optical axis away from straight down -------------
    tilt = angle_deg(T_BC[:3, 2], [0, 0, -1])
    if tilt > sampling["max_tilt_deg"]:
        return ({"food_uv": uv.tolist(), "food_z": z.tolist(),
                 "tilt_deg": tilt}, "tilt")

    # -- Gate 5: the tray must be in front of the camera ----------------------
    tuv, tz = project(np.asarray(roi["tray_corners_B"], float), T_BC, cam["rgb"])
    if np.any(tz <= 0):
        return ({"food_uv": uv.tolist(), "food_z": z.tolist(),
                 "tilt_deg": tilt}, "tray_behind_camera")

    # -- Gate 6: how much of the frame does the tray cover? -------------------
    ratio = tray_ratio(tuv, cam["rgb"])
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
    if ratio < tray_threshold(cfg, height_index):
        return metrics, "tray_ratio"
    return metrics, None


def check(cfg, T_BC, height_index):
    """Production contract: ``(metrics, None)`` when accepted, ``(None, reason)`` otherwise.

    ``food_z`` is dropped on the way out: it exists for diagnostic drawings, and
    the accepted metrics dict is persisted verbatim into metadata.json, so adding
    a key here would silently change the on-disk schema.
    """
    metrics, reason = check_detailed(cfg, T_BC, height_index)
    if reason is not None:
        return None, reason
    metrics.pop("food_z", None)
    return metrics, None
