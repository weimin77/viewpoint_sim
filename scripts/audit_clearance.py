"""Safety audit: how much collision margin can this scene actually enforce?

Not part of the package. The pipeline only ever generates straight lines in
joint space, so this script samples those lines and asks the production
``Checker`` one question per candidate margin: does anything violate it?

Doing the sweep this way keeps the answer in the same currency the code uses -
if the sweep says "clean up to 34 mm", then ``motion.collision_margin_m`` can
be set to 34 mm and every transit the pipeline can produce is provably clear.
Raw ``mj_geomDistance`` is deliberately not used for the headline number: it
returns the saturated ``distmax`` for far pairs, so reading it as a clearance
needs care and gives no statement about what the checker enforces.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from viewpoint_sim.config import load_config
from viewpoint_sim.geometry import inv, look_at
from viewpoint_sim.world import World

MARGINS_MM = (10, 12, 15, 20, 25, 30, 35, 40, 45, 50, 60)
STRUCTURAL = {frozenset(("col_link5", "col_camera_bracket"))}
# The camera bracket is bolted to link5, so this pair's distance is a fixed
# 10.5 mm property of the hardware, not a motion clearance. Excluding it is what
# lets the bisection below report the clearance the arm actually has left.


def collect_poses(world, cfg, rng, per_height=20):
    """Reachable, collision-free viewpoints spread over the sampling region."""
    s = cfg["sampling"]
    centre = (np.array(cfg["roi"]["food_min_B"])
              + np.array(cfg["roi"]["food_max_B"])) / 2
    seed = np.array(cfg["robot"]["home_q_rad"])
    poses = []
    for h in s["heights_above_tray_m"]:
        got = tries = 0
        while got < per_height and tries < 600:
            tries += 1
            eye = np.array([rng.uniform(*s["x_B_m"]), rng.uniform(*s["y_B_m"]),
                            cfg["roi"]["tray_z_B_m"] + h])
            target = look_at(eye, centre, s["reference_up_B"])
            for q, _ in world.kin.solve(target @ inv(world.mount), seed, rng,
                                        cfg["robot"]):
                if world.collision(q) is None:
                    poses.append(q)
                    got += 1
                    break
    return poses


def segment_points(qa, qb, step=0.03):
    """Joint-space samples of the straight segment qa -> qb."""
    n = max(2, int(np.ceil(np.max(np.abs(qb - qa)) / step)) + 1)
    return [qa + (qb - qa) * u for u in np.linspace(0.0, 1.0, n)]


def without_structural(pairs, model):
    return [p for p in pairs
            if frozenset((model.geom(p[0]).name, model.geom(p[1]).name))
            not in STRUCTURAL]


def bisect_margin(world, points, lo=0.0005, hi=0.40, rounds=24):
    """Largest margin at which every sample point is still violation-free."""
    for _ in range(rounds):
        mid = (lo + hi) / 2
        world.checker.margin_m = mid
        if any(_violates(world, q) for q in points):
            hi = mid
        else:
            lo = mid
    return lo


def _violates(world, q):
    world.set_q(q)
    return world.checker.violation()


def main(config="configs/default.json", seed=2026, trials=60):
    cfg = load_config(config)
    rng = np.random.default_rng(seed)
    margins = [m / 1000 for m in MARGINS_MM]

    with World(cfg) as world:
        home = np.array(cfg["robot"]["home_q_rad"])
        print(f"config {config}")
        print(f"configured margin {cfg['motion']['collision_margin_m'] * 1000:.1f} mm")
        print(f"home {np.round(np.degrees(home), 1).tolist()} deg")

        poses = collect_poses(world, cfg, rng)
        print(f"feasible poses sampled: {len(poses)}")

        groups = {
            "posture": [(q, "posture") for q in poses],
            "home -> pose": [(p, "home") for q in poses
                             for p in segment_points(home, q)],
            "pose -> pose": [(p, "pair") for _ in range(trials)
                             for i, j in [rng.choice(len(poses), 2, replace=False)]
                             for p in segment_points(poses[i], poses[j])],
        }

        print(f"\n{'margin':>8}" + "".join(f"{k:>16}" for k in groups))
        for margin in margins:
            world.checker.margin_m = margin
            cells = []
            for name, points in groups.items():
                hits, example = 0, ""
                for q, tag in points:
                    world.set_q(q)
                    bad = world.checker.violation()
                    if bad:
                        hits += 1
                        example = example or bad
                cells.append(f"{hits}/{len(points)}")
            print(f"{margin * 1000:7.0f}mm" + "".join(f"{c:>16}" for c in cells))
        world.checker.margin_m = cfg["motion"]["collision_margin_m"]

        print("\nfirst offender per group at the largest margin (60 mm):")
        world.checker.margin_m = margins[-1]
        for name, points in groups.items():
            for q, tag in points:
                world.set_q(q)
                bad = world.checker.violation()
                if bad:
                    print(f"  {name:14} {bad}")
                    break

        # The 10.5 mm ceiling comes from a pair that is bolted together. Drop it
        # and bisect again, then split self vs environment: only the environment
        # number says anything about hitting the oven, table or tray.
        base_pairs = list(world.checker.pairs)

        def use(pairs):
            world.checker.pairs = pairs
            world.checker._first = np.array([p[0] for p in pairs], dtype=int)
            world.checker._second = np.array([p[1] for p in pairs], dtype=int)

        free = without_structural(base_pairs, world.model)
        dropped = len(base_pairs) - len(free)
        for label, kind in (("all pairs", None),
                            ("self only", "self_collision"),
                            ("environment only", "environment_collision")):
            use([p for p in free if kind is None or p[2] == kind])
            print(f"\n{label} ({dropped} bolted pair dropped):")
            for name, points in groups.items():
                qs = [q for q, _tag in points]
                best = bisect_margin(world, qs)
                world.checker.margin_m = best * 1.0005
                offender = next((bad for bad in (_violates(world, q) for q in qs)
                                 if bad), "")
                print(f"  {name:14} clean up to {best * 1000:7.2f} mm   "
                      f"(binding: {offender})")
        use(base_pairs)

    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
