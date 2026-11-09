import unittest
import numpy as np
import copy
from viewpoint_sim.config import load_config, mount_transform, validate_config, crop_settings
from viewpoint_sim.geometry import (look_at, transform, inv, apply, project, box_vertices,
    image_contains, clip_polygon, polygon_area, cropped_intrinsics, crop_box)
from viewpoint_sim.planner import geometry_check

CFG = load_config()
CENTER = (np.array(CFG['roi']['food_min_B'])+np.array(CFG['roi']['food_max_B']))/2
MARGIN = np.array(CFG['sampling']['image_margin_fraction'])*[CFG['camera']['rgb']['width'], CFG['camera']['rgb']['height']]


class GeometryTests(unittest.TestCase):
    def camera(self, h=None, xy=None):
        h = CFG['sampling']['heights_above_tray_m'][-1] if h is None else h
        xy = CENTER[:2] if xy is None else xy
        return look_at([xy[0], xy[1], CFG['roi']['tray_z_B_m']+h], CENTER,
                       CFG['sampling']['reference_up_B'])

    def test_look_at_and_vertical_degeneracy(self):
        for up in [(0,1,0),(0,0,1)]:
            T=look_at([CENTER[0],0,.35],[CENTER[0],0,.05],up)
            np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.)
            np.testing.assert_allclose(T[:3,2],[0,0,-1])

    def test_hand_eye_transform_has_rotation_and_translation(self):
        T_BC=self.camera(xy=(CENTER[0]+.03,CENTER[1]+.02))
        T_FC=mount_transform(CFG)
        T_BF=T_BC@inv(T_FC)
        np.testing.assert_allclose(T_BF@T_FC,T_BC,atol=1e-12)
        self.assertGreater(np.linalg.norm(T_BC[:3,3]-T_BF[:3,3]),.02)
        self.assertGreater(np.linalg.norm(T_BC[:3,:3]-T_BF[:3,:3]),.5)
        p=np.array([[.20,-.04,.08]])
        np.testing.assert_allclose(apply(inv(T_BC),p),apply(inv(T_FC),apply(inv(T_BF),p)),atol=1e-12)

    def test_eight_vertices_bound_the_whole_pinhole_box(self):
        T=self.camera();c=CFG['camera']['rgb']
        lo,hi=np.array(CFG['roi']['food_min_B']),np.array(CFG['roi']['food_max_B'])
        uv,z=project(box_vertices(lo,hi),T,c)
        self.assertTrue(image_contains(uv,z,c,MARGIN))
        pts=np.random.default_rng(0).uniform(lo,hi,(5000,3))
        puv,pz=project(pts,T,c)
        self.assertTrue(image_contains(puv,pz,c,MARGIN))
        behind=T.copy();behind[:3,3]=[CENTER[0],0,-.4]
        self.assertFalse(image_contains(*project(box_vertices(lo,hi),behind,c),c,MARGIN))

    def test_polygon_clipping_handles_partially_visible_tray(self):
        poly=[[-1,-1],[3,-1],[3,2],[-1,2]]
        self.assertAlmostEqual(polygon_area(clip_polygon(poly,[0,0,2,1])),2)

    def test_joint_crop_preserves_pixel_projection(self):
        c=CFG['camera']['rgb'];T=self.camera(xy=(CENTER[0]+.02,CENTER[1]+.03))
        pts=box_vertices(CFG['roi']['food_min_B'],CFG['roi']['food_max_B'])
        uv,z=project(pts,T,c);tuv,_=project(CFG['roi']['tray_corners_B'],T,c)
        rect=crop_box(uv,tuv,c,10);cc=cropped_intrinsics(c,rect)
        cuv,_=project(pts,T,cc)
        np.testing.assert_allclose(cuv,uv-np.array(rect[:2]),atol=1e-12)

    def test_unusable_height_is_rejected(self):
        for h in (.10,.65):
            T=look_at([CENTER[0],0,CFG['roi']['tray_z_B_m']+h],CENTER,CFG['sampling']['reference_up_B'])
            geo,reason=geometry_check(CFG,T,0)
            self.assertIsNone(geo)
            self.assertIn(reason,['food_out_of_rgb_fov','depth_range'])

    def test_crop_switch_defaults_off_and_keeps_legacy_padding(self):
        cfg = load_config()
        self.assertEqual(crop_settings(cfg), (False, 10.))
        enabled = copy.deepcopy(cfg)
        enabled['quality']['crop'] = {'enabled': True, 'padding_px': 12}
        validate_config(enabled)
        self.assertEqual(crop_settings(enabled), (True, 12.))
        legacy = copy.deepcopy(cfg)
        legacy['quality'].pop('crop')
        legacy['quality']['crop_padding_px'] = 7
        validate_config(legacy)
        self.assertEqual(crop_settings(legacy), (False, 7.))
        bad = copy.deepcopy(cfg)
        bad['quality']['crop'] = {'enabled': True, 'padding_px': 400}
        with self.assertRaises(ValueError):
            validate_config(bad)

    def test_relook_keeps_target_centered_on_height_change(self):
        c=CFG['camera']['rgb'];target=CENTER[None,:]
        T=self.camera(h=CFG['sampling']['heights_above_tray_m'][0],xy=(CENTER[0]-.04,CENTER[1]+.03))
        shifted=T.copy();shifted[2,3]+=.06
        new=look_at(shifted[:3,3],CENTER,CFG['sampling']['reference_up_B'])
        a,_=project(target,shifted,c);b,_=project(target,new,c)
        np.testing.assert_allclose(b[0],[c['cx'],c['cy']],atol=1e-10)
        self.assertGreater(np.linalg.norm(a-b),5)


if __name__=='__main__':
    unittest.main()
