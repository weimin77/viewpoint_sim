"""Camera candidate pose generation: planar sampling + Look-at orientation.

Two independent steps, both pure geometry (numpy only - no MuJoCo, no rendering):

    步骤 1  sample_plane_position / sample_z_perturbation / camera_position
            In the configured XY rectangle, draw a uniform position, then place
            it at ``tray_z + h + delta`` so each configured height produces its
            own layer of viewpoints.

    步骤 2  look_at_pose
            Turn "where the camera is" into "where the camera points and how it
            is rolled" by aiming the optical axis at the Food ROI centre.

Everything here is a pure function of (config, random stream), so a seed fully
determines the sequence. The two random draws used by step 1 are kept in the
same order as the historical implementation (jitter first and only when
``z_jitter_m`` is non-zero, then X, then Y) so existing seeds keep reproducing
bit for bit.

Optical convention throughout: +X right, +Y down, +Z forward (OpenCV style).
``T_BC`` therefore maps camera-frame points into the Base frame.
"""
import numpy as np

from .geometry import look_at


def roi_center(cfg):
    """Centre of the Food ROI box, in the Base frame.

    This is the Look-at target: the point the optical axis is aimed at, not the
    tray centre and not the food centroid.
    """
    roi = cfg["roi"]
    return (np.asarray(roi["food_min_B"], float) + np.asarray(roi["food_max_B"], float)) / 2


# ---------------------------------------------------------------------------
# 步骤 1：平面位置采样 + 高度 + Z 扰动  ->  相机位置
# ---------------------------------------------------------------------------
def sample_plane_position(cfg, rng):
    """Uniform draw inside the configured XY rectangle.

    ``sampling.x_B_m`` / ``sampling.y_B_m`` are the only bounds; nothing here
    knows about reachability, occlusion or field of view. Rejecting invalid
    positions is the job of the caller, so the draw stays cheap and the
    acceptance statistics stay meaningful.
    """
    s = cfg["sampling"]
    x = float(rng.uniform(*s["x_B_m"]))
    y = float(rng.uniform(*s["y_B_m"]))
    return x, y


def sample_z_perturbation(cfg, rng):
    """Draw delta ~ U(-Delta, +Delta), or return exactly 0 when Delta is zero.

    The early return is deliberate: with Delta = 0 the random stream is not
    consumed at all, so turning the perturbation off leaves the historical
    sample sequence untouched.
    """
    amplitude = float(cfg["sampling"]["z_jitter_m"])
    if amplitude <= 0.0:
        return 0.0
    return float(rng.uniform(-amplitude, amplitude))


def camera_position(cfg, height_index, plane_rng, jitter_rng):
    """Optical centre for one candidate on the given height layer.

    ``height_index`` indexes ``sampling.heights_above_tray_m``; the height is
    measured from the tray surface, so

        z = roi.tray_z_B_m + heights[height_index] + delta

    Note this is the *optical centre* height. The flange sits somewhere else
    once the wrist orientation is applied, which is why the flange pose can
    only be computed after IK.
    """
    s = cfg["sampling"]
    heights = s["heights_above_tray_m"]
    if not 0 <= height_index < len(heights):
        raise IndexError(f"height_index {height_index} is outside 0..{len(heights) - 1}")

    # Order matters: jitter is drawn first, and only when enabled.
    delta = sample_z_perturbation(cfg, jitter_rng)
    x, y = sample_plane_position(cfg, plane_rng)
    z = float(cfg["roi"]["tray_z_B_m"]) + float(heights[height_index]) + delta
    return np.array([x, y, z]), float(heights[height_index]), delta


# ---------------------------------------------------------------------------
# 步骤 2：Look-at 自动朝向  ->  完整相机位姿
# ---------------------------------------------------------------------------
def look_at_pose(cfg, eye_B, target_B=None):
    """Camera pose T_BC that aims the optical +Z axis at the ROI centre.

    Returns a 4x4 matrix. The rotation is not free: Look-at only removes two
    degrees of freedom (where to look), leaving the roll to the reference up
    vector. That is what makes the result reproducible instead of "some valid
    orientation".
    """
    target = roi_center(cfg) if target_B is None else np.asarray(target_B, float)
    return look_at(np.asarray(eye_B, float), target, cfg["sampling"]["reference_up_B"])


def candidate_pose(cfg, height_index, plane_rng, jitter_rng):
    """Convenience wrapper: one candidate = position (step 1) + orientation (step 2).

    Returns ``(eye_B, T_BC, base_height_m, delta_m)``. Kept separate from the
    two steps so callers can stop after step 1 (e.g. to plot raw positions) or
    feed their own position into step 2.
    """
    eye, base_height, delta = camera_position(cfg, height_index, plane_rng, jitter_rng)
    return eye, look_at_pose(cfg, eye), base_height, delta
