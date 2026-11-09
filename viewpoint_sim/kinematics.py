"""URDF FK + analytic geometric Jacobian, bounded multi-start SE(3) IK.

FK uses URDF link frames, never the inertial/center-of-mass frame.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .geometry import transform


class Kinematics:
    def __init__(self, urdf, flange="link6", base_position=(0, 0, 0)):
        root = ET.parse(Path(urdf)).getroot()
        by_child = {e.find("child").get("link"): e for e in root.findall("joint")}
        chain, name = [], flange
        while name in by_child:
            j = by_child[name]
            chain.append(j)
            name = j.find("parent").get("link")
        self.flange_link = flange
        self.flange_parent = chain[0].find("parent").get("link") if chain else None
        self.chain, self.names, lower, upper = [], [], [], []
        for j in reversed(chain):
            o = j.find("origin")
            xyz = list(map(float, o.get("xyz", "0 0 0").split())) if o is not None else [0]*3
            rpy = list(map(float, o.get("rpy", "0 0 0").split())) if o is not None else [0]*3
            moving = j.get("type") != "fixed"
            axis = np.array(list(map(float, j.find("axis").get("xyz").split()))) if moving else None
            self.chain.append((transform(xyz, rpy), axis))
            if moving:
                self.names.append(j.get("name"))
                lower.append(float(j.find("limit").get("lower")))
                upper.append(float(j.find("limit").get("upper")))
        self.lower, self.upper = np.array(lower), np.array(upper)
        self.base = transform(base_position)

    def fk(self, q, jacobian=False):
        T, k, axes, origins = self.base.copy(), 0, [], []
        for origin, axis in self.chain:
            T = T @ origin
            if axis is not None:
                axes.append(T[:3, :3] @ axis)
                origins.append(T[:3, 3].copy())
                r = np.eye(4)
                r[:3, :3] = Rotation.from_rotvec(axis * q[k]).as_matrix()
                T = T @ r
                k += 1
        if not jacobian:
            return T
        J = np.zeros((6, len(q)))
        for i, (axis, p) in enumerate(zip(axes, origins)):
            J[:3, i] = np.cross(axis, T[:3, 3] - p)
            J[3:, i] = axis
        return T, J

    def solve(self, target, seed, rng, config):
        margin = config["joint_limit_margin_rad"]
        # Characteristic length makes rotational and translation rows comparable.
        scale = config.get("ik_characteristic_length_m", .25)
        low, high = self.lower + margin, self.upper - margin
        starts = [np.clip(seed, low, high), (low+high)/2]
        starts += [rng.uniform(low, high) for _ in range(max(0, config["ik_starts"]-2))]
        solutions = []
        for start in starts:
            def residual(q):
                actual = self.fk(q)
                rot = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
                return np.r_[(actual[:3, 3]-target[:3, 3]), rot * scale]
            result = least_squares(residual, start, bounds=(low, high),
                                   max_nfev=70, ftol=1e-8, xtol=1e-9, gtol=1e-9)
            q = result.x
            actual, J = self.fk(q, True)
            pe = np.linalg.norm(actual[:3, 3]-target[:3, 3])
            re = np.degrees(Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude())
            if pe > config["ik_position_tolerance_m"] or re > config["ik_rotation_tolerance_deg"]:
                continue
            # Characteristic length 0.25 m makes rotational and translation rows comparable.
            sigma = float(np.linalg.svd(np.vstack((J[:3], J[3:]*scale)), compute_uv=False)[-1])
            if sigma < config["min_scaled_jacobian_singular_value"]:
                continue
            if not any(np.linalg.norm(q-s[0]) < 1e-3 for s in solutions):
                solutions.append((q, {"position_error_m":float(pe), "rotation_error_deg":float(re),
                                      "scaled_jacobian_sigma_min":sigma}))
            # A nearby valid branch is sufficient; still try alternatives if it collides later.
            if len(solutions) >= 3:
                break
        solutions.sort(key=lambda s: np.linalg.norm(s[0]-seed))
        return solutions
