"""Scene geometry: the oven carcass, the wire rack and the stacking order.

These checks read the same builders the simulator uses, so a config edit that
reopens the rear corner slot, drops the rack below the floor, or leaves food
floating above the tray fails here instead of silently changing every capture.
"""
import unittest
import numpy as np
from viewpoint_sim.config import ROOT, load_config
from viewpoint_sim.world import World

CONFIGS = ('configs/default.json',)


def slabs(parts):
    """AABB (lo, hi) per part, in the Base frame."""
    out = []
    for centre, size in parts:
        centre, size = np.asarray(centre, float), np.asarray(size, float)
        out.append((centre-size/2, centre+size/2))
    return out


def blocked(origin, direction, parts, max_distance=2.0):
    """Slab ray test: does the ray hit any panel before max_distance?"""
    origin = np.asarray(origin, float)
    direction = np.asarray(direction, float)
    for lo, hi in slabs(parts):
        tmin, tmax = 0.0, max_distance
        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                if origin[axis] < lo[axis] or origin[axis] > hi[axis]:
                    break
                continue
            a = (lo[axis]-origin[axis])/direction[axis]
            b = (hi[axis]-origin[axis])/direction[axis]
            tmin, tmax = max(tmin, min(a, b)), min(tmax, max(a, b))
            if tmin > tmax:
                break
        else:
            if tmin <= tmax:
                return True
    return False


class SceneTests(unittest.TestCase):
    def configs(self):
        for path in CONFIGS:
            yield path, load_config(ROOT/path)

    def test_oven_panels_leave_no_gap_behind_the_tray(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                oven = cfg["scene"]["oven"]
                parts = World._oven_boxes(oven)
                self.assertEqual(len(parts), 4)
                t = float(oven["wall_thickness_m"])
                x0, x1 = (float(v) for v in oven["x_m"])
                y0, y1 = (float(v) for v in oven["y_m"])
                z = float(oven["floor_top_z_m"]) + 0.03
                # Sweep the whole cavity, not just its centre: a slot at the rear
                # corner only shows up for origins close to a side wall.
                xs = np.linspace(x0+0.01, x1-t-0.002, 7)
                ys = np.linspace(y0+t+0.001, y1-t-0.001, 9)
                leaks = []
                for x in xs:
                    for y in ys:
                        for direction in ([1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1]):
                            if not blocked([x, y, z], direction, parts):
                                leaks.append((round(float(x), 4), round(float(y), 4),
                                              direction))
                self.assertEqual(leaks, [], f"{path}: the carcass leaks at {leaks[:4]}")
                # The -X face is the door: it must stay open for the arm.
                tray = cfg["scene"]["tray"]
                self.assertFalse(blocked([tray["position_B_m"][0], tray["position_B_m"][1], z],
                                         [-1, 0, 0], parts))

    def test_front_panel_is_flush_with_the_side_walls(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                oven = cfg["scene"]["oven"]
                y0, y1 = (float(v) for v in oven["y_m"])
                t = float(oven["wall_thickness_m"])
                x1 = float(oven["x_m"][1])
                panel, walls = slabs(World._oven_boxes(oven))[3], slabs(World._oven_boxes(oven))[1:3]
                # Full outer width: no inset that would leave a slot at either corner.
                self.assertAlmostEqual(panel[0][1], y0, places=12)
                self.assertAlmostEqual(panel[1][1], y1, places=12)
                # Reaches the far face and overlaps the full length of the walls.
                self.assertAlmostEqual(panel[1][0], x1, places=12)
                self.assertAlmostEqual(panel[0][0], x1-t, places=12)
                for lo, hi in walls:
                    self.assertLess(panel[0][0], hi[0])
                    self.assertGreaterEqual(panel[1][0], hi[0])

    def test_rack_hangs_on_the_side_walls_instead_of_standing_on_the_floor(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                scene = cfg["scene"]
                parts = World._rack_boxes(scene)
                rack, oven, tray = scene["rack"], scene["oven"], scene["tray"]
                count = int(rack["longitudinal_rails"]) + int(rack["cross_slats"])
                self.assertEqual(len(parts), count)
                boxes = slabs(parts)
                t = float(oven["wall_thickness_m"])
                inner_y0, inner_y1 = float(oven["y_m"][0])+t, float(oven["y_m"][1])-t
                # Cross bars reach both wall faces; that contact is what carries
                # the shelf, so nothing may be inset from the walls.
                self.assertAlmostEqual(min(lo[1] for lo, _ in boxes), inner_y0, places=12)
                self.assertAlmostEqual(max(hi[1] for _, hi in boxes), inner_y1, places=12)
                # No legs: every bar lives in one thin horizontal band and the
                # whole shelf clears the oven floor by a wide margin.
                low = min(lo[2] for lo, _ in boxes)
                high = max(hi[2] for _, hi in boxes)
                self.assertAlmostEqual(high-low, float(rack["wire_size_m"]), places=12)
                gap = low-float(oven["floor_top_z_m"])
                self.assertGreater(gap, 0.02, f"{path}: shelf only {gap*1000:.1f} mm above the floor")
                # The shelf still carries the tray: its top face is the tray bottom.
                tray_bottom = tray["position_B_m"][2]-tray["size_m"][2]/2
                self.assertAlmostEqual(high, tray_bottom, places=12)
                # And it stays inside the cavity depth.
                self.assertGreaterEqual(min(lo[0] for lo, _ in boxes), float(oven["x_m"][0]))
                self.assertLessEqual(max(hi[0] for _, hi in boxes), float(oven["x_m"][1])-t+1e-12)

    def test_food_rests_on_the_tray_and_tray_on_the_rack(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                tray = cfg["scene"]["tray"]
                tray_top = tray["position_B_m"][2]+tray["size_m"][2]/2
                tray_bottom = tray["position_B_m"][2]-tray["size_m"][2]/2
                self.assertAlmostEqual(tray_top, cfg["roi"]["tray_z_B_m"], places=12)
                rack_hi = max(hi[2] for _, hi in slabs(World._rack_boxes(cfg["scene"])))
                self.assertAlmostEqual(rack_hi, tray_bottom, places=12)
                for food in cfg["scene"]["foods"]:
                    drop = food["size_m"][2]/2 if food["shape"] == "box" else food["radius_m"]
                    self.assertGreaterEqual(food["position_B_m"][2]-drop, tray_top-1e-9,
                                            f"{path}: {food['name']} is not on the tray")

    def test_tray_stays_inside_the_cavity(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                oven, tray = cfg["scene"]["oven"], cfg["scene"]["tray"]
                t = float(oven["wall_thickness_m"])
                x0, x1 = (float(v) for v in oven["x_m"])
                y0, y1 = (float(v) for v in oven["y_m"])
                tx0 = tray["position_B_m"][0]-tray["size_m"][0]/2
                tx1 = tray["position_B_m"][0]+tray["size_m"][0]/2
                ty0 = tray["position_B_m"][1]-tray["size_m"][1]/2
                ty1 = tray["position_B_m"][1]+tray["size_m"][1]/2
                self.assertGreaterEqual(tx0, x0)
                self.assertLessEqual(tx1, x1-t)
                self.assertGreaterEqual(ty0, y0+t)
                self.assertLessEqual(ty1, y1-t)

    def test_tray_matches_the_shelf_footprint(self):
        for path, cfg in self.configs():
            with self.subTest(config=path):
                scene = cfg["scene"]
                tray, rack = scene["tray"], scene["rack"]
                margin = float(rack["margin_m"])
                boxes = slabs(World._rack_boxes(scene))
                shelf_x = (min(lo[0] for lo, _ in boxes), max(hi[0] for _, hi in boxes))
                shelf_y = (min(lo[1] for lo, _ in boxes), max(hi[1] for _, hi in boxes))
                tx0 = tray["position_B_m"][0]-tray["size_m"][0]/2
                tx1 = tray["position_B_m"][0]+tray["size_m"][0]/2
                ty0 = tray["position_B_m"][1]-tray["size_m"][1]/2
                ty1 = tray["position_B_m"][1]+tray["size_m"][1]/2
                # Same footprint up to the shelf margin on every side: the tray
                # fills the shelf instead of sitting as a small pad in the middle.
                self.assertAlmostEqual(shelf_x[0], tx0-margin, places=9)
                self.assertAlmostEqual(shelf_x[1], tx1+margin, places=9)
                self.assertAlmostEqual(shelf_y[0], ty0-margin, places=9)
                self.assertAlmostEqual(shelf_y[1], ty1+margin, places=9)
                # The tray never overhangs the shelf it rests on.
                self.assertGreaterEqual(tx0, shelf_x[0]-1e-12)
                self.assertGreaterEqual(ty0, shelf_y[0]-1e-12)

    def test_validation_rejects_a_detached_front_panel_scene(self):
        import copy
        from viewpoint_sim.config import validate_config
        for path, cfg in self.configs():
            bad = copy.deepcopy(cfg)
            # Tray wider than the cavity: the config must be refused, not clipped.
            bad["scene"]["tray"]["size_m"][1] = (bad["scene"]["oven"]["y_m"][1]
                                                 - bad["scene"]["oven"]["y_m"][0])
            with self.assertRaises(ValueError):
                validate_config(bad)
            bad = copy.deepcopy(cfg)
            bad["scene"]["oven"]["floor_top_z_m"] = cfg["roi"]["tray_z_B_m"]
            with self.assertRaises(ValueError):
                validate_config(bad)


if __name__ == '__main__':
    unittest.main()
