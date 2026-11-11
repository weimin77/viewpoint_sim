import copy
import json
from pathlib import Path
import numpy as np
from .geometry import transform

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / "assets").is_dir():
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
    # Optional Z perturbation: delta ~ U(-amplitude, +amplitude) added to the
    # configured base height of each candidate. The optical centre must stay
    # above the tray plane, so the amplitude cannot swallow the smallest height.
    amplitude = s.get("z_perturbation_m", 0.)
    if isinstance(amplitude, bool) or not isinstance(amplitude, (int, float)):
        raise ValueError("sampling.z_perturbation_m must be a number")
    if amplitude < 0:
        raise ValueError("sampling.z_perturbation_m must be non-negative")
    if amplitude >= min(s["heights_above_tray_m"]):
        raise ValueError("sampling.z_perturbation_m must stay below every configured height")
    for k in ("x_B_m", "y_B_m"):
        if s[k][0] >= s[k][1]:
            raise ValueError("Sampling interval must have positive width")
    if cfg["motion"]["edge_resolution_rad"] <= 0 or cfg["motion"]["collision_margin_m"] < 0:
        raise ValueError("Invalid collision parameters")
    if cfg["motion"].get("replay_speed_rad_per_s", 1.) <= 0 or cfg["motion"].get("replay_pause_seconds", 0.) < 0:
        raise ValueError("Invalid replay timing parameters")
    crop = cfg["quality"].get("crop")
    if crop is not None:
        if not isinstance(crop.get("enabled", False), bool):
            raise ValueError("quality.crop.enabled must be a boolean")
        padding = crop.get("padding_px", cfg["quality"].get("crop_padding_px", 10))
        if padding < 0 or padding >= min(cfg["camera"]["rgb"]["width"], cfg["camera"]["rgb"]["height"])/2:
            raise ValueError("Invalid quality.crop.padding_px")
    scene = cfg["scene"]
    tray = scene["tray"]
    tray_lo = [tray["position_B_m"][i]-tray["size_m"][i]/2 for i in range(3)]
    tray_hi = [tray["position_B_m"][i]+tray["size_m"][i]/2 for i in range(3)]
    if abs(tray_hi[2]-cfg["roi"]["tray_z_B_m"]) > 1e-9:
        raise ValueError("scene.tray top face must equal roi.tray_z_B_m")
    oven = scene.get("oven")
    if oven:
        x0, x1 = (float(v) for v in oven["x_m"])
        y0, y1 = (float(v) for v in oven["y_m"])
        t, ft = float(oven["wall_thickness_m"]), float(oven["floor_thickness_m"])
        if x0 >= x1 or y0 >= y1:
            raise ValueError("Oven footprint must have positive extent")
        if t <= 0 or ft <= 0 or float(oven["height_m"]) <= 0:
            raise ValueError("Oven wall thickness, floor thickness and height must be positive")
        if 2*t >= min(x1-x0, y1-y0):
            raise ValueError("Oven walls leave no cavity")
        if float(oven["floor_top_z_m"]) >= cfg["roi"]["tray_z_B_m"]:
            raise ValueError("Oven floor must sit below the tray")
        # Only the -X face is open, so the tray has to clear the three walls.
        if (tray_lo[0] < x0-1e-9 or tray_hi[0] > x1-t+1e-9
                or tray_lo[1] < y0+t-1e-9 or tray_hi[1] > y1-t+1e-9):
            raise ValueError("scene.tray must fit inside the oven cavity")
        rack = scene.get("rack")
        if rack:
            wire = float(rack["wire_size_m"])
            if float(rack["margin_m"]) < 0 or wire <= 0:
                raise ValueError("Invalid rack dimensions")
            # The shelf hangs on the side walls at the tray bottom, so there must
            # be clear air between the oven floor and the underside of the shelf.
            if tray_lo[2]-float(oven["floor_top_z_m"]) <= 2*wire:
                raise ValueError("Oven floor is too close to the tray for a shelf")
            if int(rack["longitudinal_rails"]) < 2:
                raise ValueError("scene.rack.longitudinal_rails must be at least 2 "
                                 "(the outer two rails are what carry the shelf)")
            if int(rack["cross_slats"]) < 1:
                raise ValueError("scene.rack.cross_slats must be at least 1")
            # The shelf carries the tray, so the tray may not overhang it. The
            # shelf's X extent comes from the tray plus `margin_m` on each side,
            # so only the Y direction (the cavity width) can ever fail here.
            if (tray_lo[1] < y0+t-1e-9 or tray_hi[1] > y1-t+1e-9):
                raise ValueError("scene.tray overhangs the wall-mounted shelf "
                                 "(tray must stay inside the cavity width)")
    for f in scene["foods"]:
        half = np.asarray(f["size_m"]) / 2 if f["shape"] == "box" else np.repeat(f["radius_m"], 3)
        p = np.asarray(f["position_B_m"])
        if np.any(p-half < a-1e-6) or np.any(p+half > b+1e-6):
            raise ValueError(f"Food {f['name']} exceeds the unpadded Food ROI")
    return cfg


def mount_transform(cfg):
    m = cfg["camera"]["mount"]
    return transform(m["translation_F_m"], m["rpy_FC_rad"])


def crop_settings(cfg):
    """Return (enabled, padding_px) for the optional joint Food+Tray crop.

    `quality.crop_padding_px` is the pre-0.2 spelling and is still honoured as
    the fallback padding so older result configs keep loading.
    """
    quality = cfg.get("quality", {})
    crop = quality.get("crop") or {}
    enabled = bool(crop.get("enabled", False))
    padding = crop.get("padding_px", quality.get("crop_padding_px", 10))
    return enabled, float(padding)


def clone(cfg):
    return copy.deepcopy(cfg)
