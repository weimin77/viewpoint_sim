"""The shipped robot model must stay loadable and internally consistent.

These checks are deliberately cheap: they parse the URDF/SRDF and run the
project's own FK, without starting a PyBullet client. The heavier rendering and
collision behaviour is covered by test_world.py.
"""
import json
import unittest
import numpy as np
from viewpoint_sim.config import ROOT, load_config, mount_transform, validate_config
from viewpoint_sim.kinematics import Kinematics

CONFIG = 'configs/default.json'
FLANGE = 'flange'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class RobotModelTests(unittest.TestCase):
    def test_config_validates_and_matches_its_urdf(self):
        cfg = load_config(ROOT/CONFIG)
        validate_config(cfg)
        self.assertTrue((ROOT/cfg['robot']['urdf']).is_file())
        self.assertTrue((ROOT/cfg['robot']['srdf']).is_file())
        self.assertEqual(cfg['robot']['flange_link'], FLANGE)
        kin = Kinematics(ROOT/cfg['robot']['urdf'], cfg['robot']['flange_link'],
                         cfg['robot']['base_position_B_m'])
        # Six actuated joints with sane limits; the fixed base and flange joints
        # must not be counted as degrees of freedom.
        self.assertEqual(kin.names, JOINTS)
        self.assertTrue(np.all(kin.lower < kin.upper))
        self.assertTrue(np.all(np.abs(kin.lower) <= np.pi + 1e-3))
        home = np.asarray(cfg['robot']['home_q_rad'], float)
        self.assertEqual(home.shape, (6,))
        self.assertTrue(np.all(home >= kin.lower) and np.all(home <= kin.upper))

    def test_fk_reaches_the_configured_flange_and_mount_is_rigid(self):
        cfg = load_config(ROOT/CONFIG)
        kin = Kinematics(ROOT/cfg['robot']['urdf'], cfg['robot']['flange_link'],
                         cfg['robot']['base_position_B_m'])
        home = np.asarray(cfg['robot']['home_q_rad'], float)
        T = kin.fk(home)
        self.assertTrue(np.all(np.isfinite(T)))
        np.testing.assert_allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-9)
        np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=0)
        self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1., places=9)
        # The camera optical centre must sit above the tray plane at home, so a
        # naive home pose never starts below the work surface.
        camera = T@mount_transform(cfg)
        self.assertGreater(camera[2, 3], cfg['roi']['tray_z_B_m'])

    def test_reference_result_directory_agrees_with_the_config(self):
        path = ROOT/'outputs/demo'/'config.json'
        if not path.is_file():
            self.skipTest("outputs/demo has not been generated")
        saved = json.loads(path.read_text(encoding='utf-8'))['robot']
        current = load_config(ROOT/CONFIG)['robot']
        self.assertEqual(saved['urdf'], current['urdf'])
        self.assertEqual(saved['flange_link'], current['flange_link'])


if __name__ == '__main__':
    unittest.main()
