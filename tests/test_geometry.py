import unittest
import numpy as np
from viewpoint_sim.config import load_config, mount_transform
from viewpoint_sim.geometry import (look_at, transform, inv, apply, project, box_vertices,
    image_contains, clip_polygon, polygon_area, cropped_intrinsics, crop_box)
from viewpoint_sim.planner import geometry_check


class GeometryTests(unittest.TestCase):
    def test_look_at_and_vertical_degeneracy(self):
        for up in [(0,1,0),(0,0,1)]:
            T=look_at([.35,0,.35],[.35,0,.05],up)
            np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.)
            np.testing.assert_allclose(T[:3,2],[0,0,-1])

    def test_hand_eye_transform_has_rotation_and_translation(self):
        T_BC=look_at([.32,.06,.38],[.35,0,.055])
        T_FC=mount_transform(load_config())
        T_BF=T_BC@inv(T_FC)
        np.testing.assert_allclose(T_BF@T_FC,T_BC,atol=1e-12)
        self.assertGreater(np.linalg.norm(T_BC[:3,3]-T_BF[:3,3]),.05)
        p=np.array([[.25,-.04,.08]])
        np.testing.assert_allclose(apply(inv(T_BC),p),apply(inv(T_FC),apply(inv(T_BF),p)),atol=1e-12)

    def test_eight_vertices_bound_the_whole_pinhole_box(self):
        cfg=load_config();T=look_at([.35,0,.38],[.35,0,.055]);c=cfg['camera']['rgb']
        lo,hi=np.array(cfg['roi']['food_min_B']),np.array(cfg['roi']['food_max_B'])
        uv,z=project(box_vertices(lo,hi),T,c)
        self.assertTrue(image_contains(uv,z,c,[64,36]))
        pts=np.random.default_rng(0).uniform(lo,hi,(5000,3))
        puv,pz=project(pts,T,c)
        self.assertTrue(image_contains(puv,pz,c,[64,36]))
        behind=T.copy();behind[:3,3]=[.35,0,-.4]
        self.assertFalse(image_contains(*project(box_vertices(lo,hi),behind,c),c,[64,36]))

    def test_polygon_clipping_handles_partially_visible_tray(self):
        poly=[[-1,-1],[3,-1],[3,2],[-1,2]]
        self.assertAlmostEqual(polygon_area(clip_polygon(poly,[0,0,2,1])),2)

    def test_joint_crop_preserves_pixel_projection(self):
        cfg=load_config();c=cfg['camera']['rgb'];T=look_at([.32,.04,.36],[.35,0,.055])
        pts=box_vertices(cfg['roi']['food_min_B'],cfg['roi']['food_max_B'])
        uv,z=project(pts,T,c);tuv,_=project(cfg['roi']['tray_corners_B'],T,c)
        rect=crop_box(uv,tuv,c,10);cc=cropped_intrinsics(c,rect)
        cuv,_=project(pts,T,cc)
        np.testing.assert_allclose(cuv,uv-np.array(rect[:2]),atol=1e-12)

    def test_unusable_height_is_rejected(self):
        cfg=load_config()
        for h in (.10,.65):
            T=look_at([.35,0,cfg['roi']['tray_z_B_m']+h],[.35,0,.055])
            geo,reason=geometry_check(cfg,T,0)
            self.assertIsNone(geo)
            self.assertIn(reason,['food_out_of_rgb_fov','depth_range'])

    def test_relook_keeps_target_centered_on_height_change(self):
        cfg=load_config();c=cfg['camera']['rgb'];target=np.array([[.35,0,.055]])
        T=look_at([.26,.06,.32],target[0])
        shifted=T.copy();shifted[2,3]+=.1
        new=look_at(shifted[:3,3],target[0])
        a,_=project(target,shifted,c);b,_=project(target,new,c)
        np.testing.assert_allclose(b[0],[c['cx'],c['cy']],atol=1e-10)
        self.assertGreater(np.linalg.norm(a-b),5)


if __name__=='__main__':
    unittest.main()
