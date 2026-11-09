import unittest
import numpy as np
import pybullet as p
from viewpoint_sim.config import load_config
from viewpoint_sim.world import World
from viewpoint_sim.geometry import look_at, inv, project
from viewpoint_sim.acquisition import inspect_capture
from viewpoint_sim.motion import edge_free, plan, path_free

CFG = load_config()
CENTER = (np.array(CFG['roi']['food_min_B'])+np.array(CFG['roi']['food_max_B']))/2


class WorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=CFG
        cls.world=World(cls.cfg)
        cls.eye=look_at([CENTER[0]-.02,CENTER[1]+.04,
                         cls.cfg['roi']['tray_z_B_m']+cls.cfg['sampling']['heights_above_tray_m'][1]],
                        CENTER, cls.cfg['sampling']['reference_up_B'])
        solutions=cls.world.kin.solve(cls.eye@inv(cls.world.mount),
            np.array(cls.cfg['robot']['home_q_rad']),np.random.default_rng(0),cls.cfg['robot'])
        cls.sol=next(q for q,_ in solutions if cls.world.collision(q) is None)

    @classmethod
    def tearDownClass(cls):
        cls.world.close()

    def test_fk_matches_bullet_link_frame(self):
        w=self.world
        for q in [self.cfg['robot']['home_q_rad'],self.sol]:
            w.set_q(q)
            np.testing.assert_allclose(w.flange_pose(),w.kin.fk(q),atol=1e-6)

    def test_actual_camera_pose_and_collision_free_path(self):
        w=self.world
        home=np.array(self.cfg['robot']['home_q_rad'])
        self.assertIsNone(w.collision(home));self.assertIsNone(w.collision(self.sol))
        path=plan(w,home,self.sol,np.random.default_rng(0))
        self.assertIsNotNone(path);self.assertTrue(path_free(w,path))
        w.set_q(self.sol)
        np.testing.assert_allclose(w.camera_pose(),self.eye,atol=1e-5)

    def test_visible_masks_and_depth(self):
        w=self.world;w.set_q(self.sol)
        cap=inspect_capture(w,w.camera_pose())
        self.assertEqual(cap['failures'],[])
        self.assertEqual(set(np.unique(cap['labels'])),{0,1,2,3})
        for metric in cap['metrics']:
            self.assertAlmostEqual(metric['visible_fraction'],1.)
            self.assertAlmostEqual(metric['valid_depth_fraction'],1.)
        self.assertTrue((cap['depth_mm'][cap['labels']>0]>0).all())

    def test_known_overhead_obstacle_is_detected_as_occlusion(self):
        w=self.world
        # A wide opaque horizontal plate between camera and food, inside the frustum.
        z=w.cfg['roi']['tray_z_B_m']+.14
        obstacle=w.box('test_plate',[CENTER[0],CENTER[1],z],[.22,.17,.008],[.4,.4,.4,1])
        # World visual list must include this new shape for the isolated reference pass.
        shapes=[(obstacle,v[1],v[7]) for v in p.getVisualShapeData(obstacle,physicsClientId=w.cid)]
        w.visuals.extend(shapes)
        try:
            w.set_q(self.sol);cap=inspect_capture(w,w.camera_pose())
            self.assertIn('occlusion',cap['failures'])
            self.assertTrue(all(q['visible_fraction']<.1 for q in cap['metrics']))
        finally:
            p.removeBody(obstacle,physicsClientId=w.cid)
            w.environment.remove(obstacle)
            w.visuals=[v for v in w.visuals if v[0]!=obstacle]

    def test_collision_check_rejects_robot_through_obstacle(self):
        w=self.world;w.set_q(self.sol)
        position=p.getLinkState(w.robot,w.link_ids['joint4'],computeForwardKinematics=True,physicsClientId=w.cid)[4]
        obstacle=w.box('test_collision',position,[.06,.06,.06],[.7,.1,.1,1])
        try:
            self.assertTrue(w.collision(self.sol).startswith('environment_collision'))
            self.assertFalse(edge_free(w,np.array(self.cfg['robot']['home_q_rad']),self.sol))
        finally:
            p.removeBody(obstacle,physicsClientId=w.cid)
            w.environment.remove(obstacle)

    def test_capture_exports_follow_crop_switch(self):
        import copy, tempfile
        from pathlib import Path
        from PIL import Image
        from viewpoint_sim.acquisition import save_capture
        from viewpoint_sim.planner import geometry_check
        w=self.world;w.set_q(self.sol);cap=inspect_capture(w,w.camera_pose())
        geo,reason=geometry_check(w.cfg,w.camera_pose(),1);self.assertIsNone(reason)
        cam=(w.cfg['camera']['rgb']['width'],w.cfg['camera']['rgb']['height'])
        for enabled in (False,True):
            cfg=copy.deepcopy(w.cfg);cfg['quality']['crop']={'enabled':enabled,'padding_px':10}
            with tempfile.TemporaryDirectory() as d:
                record=save_capture(Path(d)/'v',dict(cap),{'id':'v','height_index':1},geo,cfg)
                folder=Path(d)/'v';files=sorted(p.name for p in folder.iterdir())
                self.assertEqual(any(n.startswith('crop_') for n in files),enabled)
                for name in ('rgb','depth_mm','instance','food_mask'):
                    self.assertEqual(Image.open(folder/(name+'.png')).size,cam)
                self.assertEqual(record['image_size_xy'],list(cam))
                self.assertIn('roi_bbox_xyxy',record)
                self.assertEqual('crop_bbox_xyxy' in record,enabled)

    def test_nonadjacent_arm_self_collision_is_rejected(self):
        q=np.array([1.908605,-.440094,.257679,-2.454708,1.427248,.23887])
        self.assertTrue(self.world.collision(q).startswith('self_collision'))


class RendererTests(unittest.TestCase):
    def test_free_endpoints_can_still_have_an_unsafe_direct_edge(self):
        cfg=load_config();cfg['scene']['foods']=[];cfg['scene']['obstacles']=[]
        with World(cfg) as w:
            a=np.array(cfg['robot']['home_q_rad']);b=a.copy();b[0]=1.3
            middle=(a+b)/2;w.set_q(middle)
            center=p.getLinkState(w.robot,w.link_ids['camera_housing'],computeForwardKinematics=True,physicsClientId=w.cid)[4]
            w.box('mid_edge_obstacle',center,[.04,.04,.04],[.8,.1,.1,1])
            self.assertIsNone(w.collision(a));self.assertIsNone(w.collision(b))
            self.assertFalse(edge_free(w,a,b))
            alternative=plan(w,a,b,np.random.default_rng(4))
            self.assertIsNotNone(alternative)
            self.assertTrue(path_free(w,alternative))

    def test_off_center_intrinsics_and_optical_depth(self):
        cfg=load_config();cfg['scene']['foods']=[];cfg['scene']['obstacles']=[]
        with World(cfg) as w:
            c=dict(cfg['camera']['rgb']);c['cx']=280.;c['cy']=155.
            T=look_at([.2,0,.35],[.2,0,.02])
            rgb,z,ids=w.render(T,c)
            self.assertAlmostEqual(float(z[155,280]),.33,places=4)
            marker=w.box('marker',[.235,.025,.08],[.006,.006,.006],[1,0,0,1])
            _,_,ids=w.render(T,c)
            yy,xx=np.where(ids==marker)
            expected,_=project([[.235,.025,.083]],T,c)
            actual=np.array([(xx.min()+xx.max())/2,(yy.min()+yy.max())/2])
            np.testing.assert_allclose(actual,expected[0],atol=1.5)


if __name__=='__main__':
    unittest.main()
