import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
from PIL import Image
from viewpoint_sim.config import load_config,ROOT,validate_config,mount_transform
from viewpoint_sim.geometry import look_at,inv,project,box_vertices,image_contains,angle_deg
from viewpoint_sim.world import World
from viewpoint_sim.motion import edge_free
from viewpoint_sim.acquisition import inspect_capture,save_capture
from viewpoint_sim.planner import geometry_check


class GeometryAndConfigTests(unittest.TestCase):
    def test_defaults_and_user_options(self):
        c=load_config();self.assertEqual(c['engine'],'mujoco');self.assertEqual(c['robot']['model'],'myCobot Pro 630')
        self.assertEqual(c['postprocess']['crop_mode'],'none');self.assertEqual(c['sampling']['z_jitter_m'],0)
        self.assertFalse(c['sampling']['deduplicate_across_heights'])
        c['sampling']['z_jitter_m']=-.01
        with self.assertRaises(ValueError):validate_config(c)

    def test_look_at_and_mount_transform(self):
        T=look_at([.35,0,.45],[.35,0,.13],[0,0,1]);m=mount_transform(load_config())
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
        np.testing.assert_allclose((T@inv(m))@m,T,atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1)

    def test_unit_conversion_matches_robot_scale(self):
        info=json.loads((ROOT/'assets/mycobot_pro_630/mesh_conversion.json').read_text())
        self.assertEqual(len(info),7)
        for entry in info:
            self.assertAlmostEqual(entry['unit_to_m'],.001)
            self.assertLess(max(entry['extent_m']),.40)
            self.assertGreater(max(entry['extent_m']),.05)

    def test_low_and_high_heights_are_rejected(self):
        c=load_config();center=(np.array(c['roi']['food_min_B'])+c['roi']['food_max_B'])/2
        for h in [.10,.65]:
            T=look_at([center[0],center[1],c['roi']['tray_z_B_m']+h],center)
            g,reason=geometry_check(c,T,0);self.assertIsNone(g);self.assertIsNotNone(reason)


class MuJoCoWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=load_config();cls.world=World(cls.cfg)
        cls.T=look_at([.35,.03,.455],[.35,0,.13])

    @classmethod
    def tearDownClass(cls):cls.world.close()

    def setUp(self):self.world.set_q(self.cfg['robot']['home_q_rad'])

    def test_mujoco_fk_matches_official_urdf(self):
        w=self.world;rng=np.random.default_rng(1)
        for q in [np.array(self.cfg['robot']['home_q_rad']),*rng.uniform(w.kin.lower+.02,w.kin.upper-.02,(4,6))]:
            w.set_q(q);np.testing.assert_allclose(w.flange_pose(),w.kin.fk(q),atol=1e-9)
            np.testing.assert_allclose(w.camera_pose(),w.flange_pose()@w.mount,atol=1e-9)

    def test_rgb_depth_and_segmentation_are_aligned(self):
        w=self.world;T=look_at([.35,0,.455],[.35,0,.13]);rgb,z,ids=w.render(T)
        self.assertEqual(rgb.shape,(360,640,3));self.assertEqual(z.shape,ids.shape)
        self.assertAlmostEqual(float(z[180,320]),.36,places=4)
        self.assertTrue(all((ids==body).sum()>100 for body in w.food_ids))

    def test_off_center_camera_intrinsics(self):
        w=self.world;c=copy.deepcopy(self.cfg['camera']['rgb']);c.update(cx=280.,cy=155.)
        T=look_at([.35,0,.455],[.35,0,.13]);_,_,ids=w.render(T,c)
        f=self.cfg['scene']['foods'][2];half=np.array(f['size_m'])/2;center=np.array(f['position_B_m'])
        uv,_=project(box_vertices(center-half,center+half),T,c)
        ys,xs=np.where(ids==w.food_ids[2]);actual=np.array([xs.min(),ys.min(),xs.max(),ys.max()])
        expected=np.r_[uv.min(0),uv.max(0)]
        np.testing.assert_allclose(actual,expected,atol=1.2)

    def test_isolated_food_silhouettes(self):
        w=self.world;_,_,ids=w.render(self.T);reference=w.isolated_food_masks(self.T)
        for body,mask in zip(w.food_ids,reference):
            self.assertGreater(mask.sum(),100)
            self.assertLessEqual((ids==body).sum(),mask.sum())
        self.assertTrue(all(w.model.geom_group[g]==2 for g in w.food_geoms))

    def test_default_crop_disabled_and_optional_crop_exact(self):
        w=self.world;cap=inspect_capture(w,self.T);g,err=geometry_check(self.cfg,self.T,1);self.assertIsNone(err)
        with tempfile.TemporaryDirectory() as tmp:
            none=Path(tmp)/'none';meta=save_capture(none,cap,{},g,self.cfg)
            self.assertFalse((none/'crop_rgb.png').exists());self.assertIsNone(meta['crop_bbox_xyxy'])
            cfg=copy.deepcopy(self.cfg);cfg['postprocess']['crop_mode']='tray_roi'
            derived=Path(tmp)/'derived';meta=save_capture(derived,cap,{},g,cfg);x0,y0,x1,y1=meta['crop_bbox_xyxy']
            for name in ['rgb','depth_mm','instance','food_mask']:
                original=np.array(Image.open(derived/(name+'.png')));crop=np.array(Image.open(derived/('crop_'+name+'.png')))
                np.testing.assert_array_equal(crop,original[y0:y1,x0:x1])
            self.assertAlmostEqual(meta['K_crop']['cx'],cfg['camera']['rgb']['cx']-x0)

    def test_position_servos_settle_using_physics_steps(self):
        w=self.world;a=w.q;b=a.copy();b[0]+=.03
        self.assertTrue(edge_free(w,a,b));w.set_q(a)
        result=w.execute([a.tolist(),b.tolist()])
        self.assertGreater(result['physics_steps'],0)
        self.assertLess(result['arrived_joint_error_rad'],self.cfg['motion']['settle_joint_error_rad'])
        self.assertLess(result['arrived_joint_velocity_rad_s'],self.cfg['motion']['settle_velocity_rad_s'])
        np.testing.assert_allclose(w.camera_pose(),w.kin.fk(w.q)@w.mount,atol=1e-9)

    def test_fixed_obstacle_changes_feasibility(self):
        q=np.array([-.132839,1.14791,-1.14529,1.55486,-1.66197,.13358])
        # Recompute the accurate official-URDF target solution before testing.
        ss=self.world.kin.solve(self.T@inv(self.world.mount),q,np.random.default_rng(0),self.cfg['robot'])
        q=next(q for q,_ in ss if self.world.collision(q) is None)
        self.world.set_q(q)
        location=self.world.data.xpos[self.world.model.body('camera_housing').id].copy().tolist()
        c=copy.deepcopy(self.cfg)
        c['scene']['obstacles'].append(dict(name='fixture_obstacle',position_B_m=location,size_m=[.06,.06,.06]))
        with World(c) as obstructed:self.assertIn('fixture_obstacle',obstructed.collision(q))

    def test_oven_top_is_open(self):
        self.assertFalse(any('roof' in (self.world.model.geom(i).name or '') for i in range(self.world.model.ngeom)))


class PathTests(unittest.TestCase):
    def test_safe_endpoints_do_not_prove_the_edge_safe(self):
        c=load_config();c['scene']['oven']['enabled']=False;c['scene']['foods']=[]
        with World(c) as w:
            a=w.q;b=a.copy();b[0]+=.8;w.set_q((a+b)/2)
            location=w.data.xpos[w.model.body('camera_housing').id].copy().tolist()
        c['scene']['obstacles'].append(dict(name='mid_edge_obstacle',position_B_m=location,size_m=[.035,.035,.035]))
        with World(c) as w:
            self.assertIsNone(w.collision(a));self.assertIsNone(w.collision(b));self.assertFalse(edge_free(w,a,b))


if __name__=='__main__':unittest.main()
