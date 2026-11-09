import unittest
import numpy as np
import pybullet as p
from viewpoint_sim.config import load_config
from viewpoint_sim.world import World
from viewpoint_sim.geometry import look_at, inv, project
from viewpoint_sim.acquisition import inspect_capture
from viewpoint_sim.motion import edge_free, plan, path_free


class WorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=load_config()
        cls.world=World(cls.cfg)
        cls.eye=look_at([.32,.04,.36],[.35,0,.055])
        cls.sol=cls.world.kin.solve(cls.eye@inv(cls.world.mount),np.array(cls.cfg['robot']['home_q_rad']),np.random.default_rng(0),cls.cfg['robot'])[0][0]

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
        obstacle=w.box('test_plate',[.35,0,.20],[.29,.21,.01],[.4,.4,.4,1])
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
        position=p.getLinkState(w.robot,w.link_ids['link3'],computeForwardKinematics=True,physicsClientId=w.cid)[4]
        obstacle=w.box('test_collision',position,[.08,.08,.08],[.7,.1,.1,1])
        try:
            self.assertTrue(w.collision(self.sol).startswith('environment_collision'))
            self.assertFalse(edge_free(w,np.array(self.cfg['robot']['home_q_rad']),self.sol))
        finally:
            p.removeBody(obstacle,physicsClientId=w.cid)
            w.environment.remove(obstacle)

    def test_nonadjacent_arm_self_collision_is_rejected(self):
        q=np.array([-2.1214502258522323,.6709876155363461,-.04433450426566177,
                    .78116500533591,.8832914295100773,-1.8691016742991902])
        self.assertTrue(self.world.collision(q).startswith('self_collision'))


class RendererTests(unittest.TestCase):
    def test_free_endpoints_can_still_have_an_unsafe_direct_edge(self):
        cfg=load_config();cfg['scene']['foods']=[];cfg['scene']['obstacles']=[]
        with World(cfg) as w:
            a=np.array(cfg['robot']['home_q_rad']);b=a.copy();b[0]=1.3
            middle=(a+b)/2;w.set_q(middle)
            center=p.getLinkState(w.robot,w.link_ids['camera_housing'],computeForwardKinematics=True,physicsClientId=w.cid)[4]
            w.box('mid_edge_obstacle',center,[.045,.045,.045],[.8,.1,.1,1])
            self.assertIsNone(w.collision(a));self.assertIsNone(w.collision(b))
            self.assertFalse(edge_free(w,a,b))
            alternative=plan(w,a,b,np.random.default_rng(4))
            self.assertIsNotNone(alternative)
            self.assertTrue(path_free(w,alternative))

    def test_off_center_intrinsics_and_optical_depth(self):
        cfg=load_config();cfg['scene']['foods']=[];cfg['scene']['obstacles']=[]
        with World(cfg) as w:
            c=dict(cfg['camera']['rgb']);c['cx']=280.;c['cy']=155.
            T=look_at([.35,0,.35],[.35,0,.02])
            rgb,z,ids=w.render(T,c)
            self.assertAlmostEqual(float(z[155,280]),.33,places=4)
            marker=w.box('marker',[.385,.025,.08],[.006,.006,.006],[1,0,0,1])
            _,_,ids=w.render(T,c)
            yy,xx=np.where(ids==marker)
            expected,_=project([[.385,.025,.083]],T,c)
            actual=np.array([(xx.min()+xx.max())/2,(yy.min()+yy.max())/2])
            np.testing.assert_allclose(actual,expected[0],atol=1.5)


if __name__=='__main__':
    unittest.main()
