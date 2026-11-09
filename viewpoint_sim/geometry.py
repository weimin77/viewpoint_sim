"""Meters, radians, column vectors. T_AB maps coordinates from B into A.

Optical axes: +X right, +Y down, +Z forward. Pinhole images are undistorted.
"""
from itertools import product
import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def inv(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def apply(T, points):
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def box_vertices(lo, hi):
    return np.array(list(product(*zip(lo, hi))), dtype=float)


def look_at(eye, target, up=(0, 1, 0)):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    d = target - eye
    if np.linalg.norm(d) < 1e-9:
        raise ValueError("Look-at eye and target must differ")
    d /= np.linalg.norm(d)
    u = np.asarray(up, float)
    if np.linalg.norm(u) < 1e-9:
        raise ValueError("Reference up must be nonzero")
    u /= np.linalg.norm(u)
    if abs(d @ u) > .98:
        u = np.eye(3)[np.argmin(np.abs(d))]
    right = np.cross(d, u)
    right /= np.linalg.norm(right)
    T = np.eye(4)
    T[:3, :3] = np.column_stack((right, np.cross(d, right), d))
    T[:3, 3] = eye
    return T


def project(points_B, T_BC, camera):
    pc = apply(inv(T_BC), points_B)
    z = pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = pc[:, :2] / z[:, None]
    uv = uv * [camera["fx"], camera["fy"]] + [camera["cx"], camera["cy"]]
    return uv, z


def image_contains(uv, z, camera, margin_px):
    m = np.broadcast_to(margin_px, (2,))
    return bool(np.all(z > 0) and np.isfinite(uv).all()
                and np.all(uv >= m)
                and np.all(uv <= np.array([camera["width"] - 1, camera["height"] - 1]) - m))


def polygon_area(poly):
    p = np.asarray(poly, float)
    if len(p) < 3:
        return 0.0
    return abs(float(p[:, 0] @ np.roll(p[:, 1], 1) - p[:, 1] @ np.roll(p[:, 0], 1))) * .5


def clip_polygon(poly, rect):
    """Sutherland-Hodgman clipping; rect is continuous [x0,y0,x1,y1]."""
    out = [np.asarray(p, float) for p in poly]
    for axis, bound, sign in [(0, rect[0], 1), (0, rect[2], -1),
                               (1, rect[1], 1), (1, rect[3], -1)]:
        src, out = out, []
        if not src:
            break
        a = src[-1]
        for b in src:
            inside_a = sign * (a[axis] - bound) >= 0
            inside_b = sign * (b[axis] - bound) >= 0
            if inside_a != inside_b:
                t = (bound - a[axis]) / (b[axis] - a[axis])
                out.append(a + t * (b - a))
            if inside_b:
                out.append(b)
            a = b
    return np.asarray(out).reshape(-1, 2)


def tray_ratio(uv, camera, rect=None):
    if rect is None:
        rect = [0, 0, camera["width"], camera["height"]]
    return polygon_area(clip_polygon(uv, rect)) / ((rect[2] - rect[0]) * (rect[3] - rect[1]))


def angle_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1))))


def crop_box(food_uv, tray_uv, camera, padding):
    p = np.vstack((food_uv, tray_uv))
    lo = np.maximum(np.floor(p.min(0) - padding), [0, 0]).astype(int)
    hi = np.minimum(np.ceil(p.max(0) + padding) + 1,
                    [camera["width"], camera["height"]]).astype(int)
    return [int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1])]


def cropped_intrinsics(camera, rect):
    out = dict(camera)
    x0, y0, x1, y1 = rect
    out.update(width=x1-x0, height=y1-y0, cx=camera["cx"]-x0, cy=camera["cy"]-y0)
    return out


def projection_matrix(camera, near=.01, far=2.):
    """OpenGL projection for OpenCV pixels, including an off-center principal point.

    Pixel centers are integers in OpenCV; the half-pixel shift maps to raster centers.
    Returns a column-major list for PyBullet.
    """
    w, h = camera["width"], camera["height"]
    fx, fy, cx, cy = (camera[k] for k in ("fx", "fy", "cx", "cy"))
    P = np.array([[2*fx/w, 0, 1-2*(cx+.5)/w, 0],
                  [0, 2*fy/h, 2*(cy+.5)/h-1, 0],
                  [0, 0, -(far+near)/(far-near), -2*far*near/(far-near)],
                  [0, 0, -1, 0]])
    return P.flatten(order="F").tolist()
