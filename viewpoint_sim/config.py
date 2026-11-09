import copy
import json
from pathlib import Path
import numpy as np
from .geometry import transform

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / "assets" / "piper").is_dir():
    ROOT = Path(__file__).resolve().parent


def load_config(path=None):
    path = Path(path) if path else ROOT / "configs/default.json"
    cfg = json.loads(path.read_text(encoding="utf-8"))
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    if cfg["units"] != "m_rad":
        raise ValueError("All lengths must use meters, angles radians (units=m_rad)")
    for name in ("rgb", "depth"):
        c = cfg["camera"][name]
        if min(c["width"], c["height"], c["fx"], c["fy"]) <= 0:
            raise ValueError("Invalid camera size/focal length")
        if not (0 <= c["cx"] < c["width"] and 0 <= c["cy"] < c["height"]):
            raise ValueError("Principal point must be inside the image")
    if cfg["camera"]["rgb"] != cfg["camera"]["depth"] or not np.allclose(cfg["camera"]["T_CD"], np.eye(4)):
        raise ValueError("v0.1 requires pre-aligned RGB/Depth with identical intrinsics and T_CD=I")
    a = np.asarray(cfg["roi"]["food_min_B"])
    b = np.asarray(cfg["roi"]["food_max_B"])
    if not np.all(a < b):
        raise ValueError("Food ROI must have positive dimensions")
    if cfg["camera"]["depth_range_m"][0] >= cfg["camera"]["depth_range_m"][1]:
        raise ValueError("Invalid depth range")
    s = cfg["sampling"]
    if s["target_per_height"] < 1 or s["max_attempts_per_height"] < 1 or not s["heights_above_tray_m"]:
        raise ValueError("Counts and height list must be nonempty/positive")
    if not 0 <= s["min_axis_angle_deg"] <= 180:
        raise ValueError("Invalid minimum axis angle")
    if any(h <= 0 for h in s["heights_above_tray_m"]):
        raise ValueError("Camera height must be positive")
    for k in ("x_B_m", "y_B_m"):
        if s[k][0] >= s[k][1]:
            raise ValueError("Sampling interval must have positive width")
    if cfg["motion"]["edge_resolution_rad"] <= 0 or cfg["motion"]["collision_margin_m"] < 0:
        raise ValueError("Invalid collision parameters")
    for f in cfg["scene"]["foods"]:
        half = np.asarray(f["size_m"]) / 2 if f["shape"] == "box" else np.repeat(f["radius_m"], 3)
        p = np.asarray(f["position_B_m"])
        if np.any(p-half < a-1e-6) or np.any(p+half > b+1e-6):
            raise ValueError(f"Food {f['name']} exceeds the unpadded Food ROI")
    return cfg


def mount_transform(cfg):
    m = cfg["camera"]["mount"]
    return transform(m["translation_F_m"], m["rpy_FC_rad"])


def clone(cfg):
    return copy.deepcopy(cfg)
