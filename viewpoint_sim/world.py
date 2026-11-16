"""MuJoCo workcell: model instantiation, metric rendering, mask isolation.

Collision checking lives in ``collision`` and the execution loops in
``execution``; this class wires them to one live ``MjModel``/``MjData`` pair and
owns the renderer cache. Anything that needs to *move* the arm ends up here
because only here is there a model to move.
"""
import os,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
from .config import ROOT,mount_transform
from .kinematics import Kinematics
from .model_builder import build_model_xml
from .geometry import look_at
from . import collision, execution


class World:
    def __init__(self,cfg,gui=False):
        self.cfg,self.gui=cfg,gui
        self.mount=mount_transform(cfg)
        self.kin=Kinematics(ROOT/cfg['robot']['urdf'],cfg['robot']['flange_link'],cfg['robot']['base_position_B_m'])
        self.xml=build_model_xml(cfg)
        self.model=mujoco.MjModel.from_xml_string(self.xml)
        self.data=mujoco.MjData(self.model)
        self.joint_ids=np.array([self.model.joint(name).id for name in self.kin.names])
        self.qadr=self.model.jnt_qposadr[self.joint_ids]
        self.dadr=self.model.jnt_dofadr[self.joint_ids]
        self.flange=self.model.body(cfg['robot']['flange_link']).id
        self.optical_site=self.model.site('camera_optical').id
        self.camera_id=self.model.camera('render_camera').id
        self.mocap_id=self.model.body_mocapid[self.model.body('render_pose').id]
        self.food_ids=[self.model.body(f['name']).id for f in cfg['scene']['foods']]
        self.names={i:mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(self.model.nbody)}
        self.robot_geoms=[i for i in range(self.model.ngeom) if (self.model.geom(i).name or '').startswith('col_')]
        self.env_geoms=[i for i in range(self.model.ngeom) if (self.model.geom(i).name or '').startswith(('env_','food_'))]
        self.food_geoms=[i for i in self.env_geoms if int(self.model.geom_bodyid[i]) in self.food_ids]
        # The pair list and the exclusion matrix both come from the generated
        # XML, so an added obstacle joins the test set by being built.
        self.checker=collision.Checker(self.model,self.data,
            collision.geom_pairs(self.robot_geoms,self.env_geoms,
                                 lambda geom:int(self.model.geom_bodyid[geom]),self.names,
                                 collision.excluded_body_pairs(self.xml,lambda name:self.model.body(name).id)),
            cfg['motion']['collision_margin_m'])
        self.renderers={}
        self.viewer=None
        self.set_q(cfg['robot']['home_q_rad'])
        if gui:
            from mujoco import viewer as mjviewer
            self.viewer=mjviewer.launch_passive(self.model,self.data,show_left_ui=False,show_right_ui=False)
            self.viewer.cam.lookat[:]=[.28,0,.28]
            self.viewer.cam.distance=1.35
            self.viewer.cam.azimuth=145
            self.viewer.cam.elevation=-16
            self.viewer.opt.geomgroup[3]=0
            self.viewer.sync()

    @property
    def q(self):return self.data.qpos[self.qadr].copy()

    def set_q(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(len(self.joint_ids),):raise ValueError('Expected six arm joints')
        self.data.qpos[self.qadr]=q
        self.data.qvel[:]=0
        self.data.ctrl[:]=q
        self.data.qacc_warmstart[:]=0
        mujoco.mj_forward(self.model,self.data)

    def flange_pose(self):
        T=np.eye(4);T[:3,:3]=self.data.xmat[self.flange].reshape(3,3);T[:3,3]=self.data.xpos[self.flange]
        return T

    def camera_pose(self):
        T=np.eye(4);T[:3,:3]=self.data.site_xmat[self.optical_site].reshape(3,3);T[:3,3]=self.data.site_xpos[self.optical_site]
        return T

    def collision_current(self):
        reason=collision.joint_limit_violation(self.q,self.kin.lower,self.kin.upper,
                                               self.cfg['robot']['joint_limit_margin_rad'])
        return reason or self.checker.violation()

    def collision(self,q):
        self.set_q(q)
        return self.collision_current()

    def _camera(self,T_BC,camera):
        # OpenCV optical: x right, y down, z forward. MuJoCo camera: x right,
        # y up, negative z forward. The 180-degree x rotation is essential.
        R=T_BC[:3,:3]@np.diag([1,-1,-1]);xyzw=Rotation.from_matrix(R).as_quat()
        self.data.mocap_pos[self.mocap_id]=T_BC[:3,3]
        self.data.mocap_quat[self.mocap_id]=xyzw[[3,0,1,2]]
        w,h=camera['width'],camera['height']
        self.model.cam_sensorsize[self.camera_id]=[w,h]
        self.model.cam_resolution[self.camera_id]=[w,h]
        self.model.cam_intrinsic[self.camera_id]=[camera['fx'],camera['fy'],
                                                 w/2-(camera['cx']+.5),camera['cy']+.5-h/2]
        mujoco.mj_kinematics(self.model,self.data)
        mujoco.mj_camlight(self.model,self.data)

    def _renderer(self,camera):
        key=(camera['width'],camera['height'])
        if key not in self.renderers:
            self.renderers[key]=mujoco.Renderer(self.model,height=key[1],width=key[0])
        return self.renderers[key]

    def render(self,T_BC,camera=None,segmentation_only=False,food_only=False):
        camera=camera or self.cfg['camera']['rgb']
        self._camera(T_BC,camera)
        renderer=self._renderer(camera)
        option=mujoco.MjvOption();mujoco.mjv_defaultOption(option)
        option.geomgroup[3:]=0
        if food_only:option.geomgroup[:]=0;option.geomgroup[2]=1
        renderer.disable_depth_rendering();renderer.disable_segmentation_rendering()
        renderer.update_scene(self.data,camera='render_camera',scene_option=option)
        rgb=None if segmentation_only else renderer.render().copy()
        depth=None
        if not segmentation_only:
            renderer.enable_depth_rendering();depth=renderer.render().copy();renderer.disable_depth_rendering()
        renderer.enable_segmentation_rendering();seg=renderer.render().copy();renderer.disable_segmentation_rendering()
        ids=seg[:,:,0];valid=(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))&(ids>=0)
        body=np.full(ids.shape,-1,dtype=np.int32)
        body[valid]=self.model.geom_bodyid[ids[valid]]
        return rgb,depth,body

    def isolated_food_masks(self,T_BC):
        groups=self.model.geom_group.copy();masks=[]
        try:
            for body in self.food_ids:
                self.model.geom_group[self.food_geoms]=5
                own=[g for g in self.food_geoms if self.model.geom_bodyid[g]==body]
                self.model.geom_group[own]=2
                _,_,ids=self.render(T_BC,segmentation_only=True,food_only=True)
                masks.append(ids==body)
        finally:self.model.geom_group[:]=groups
        return masks

    def overview(self):
        c={'width':960,'height':640,'fx':800.,'fy':800.,'cx':479.5,'cy':319.5}
        return self.render(look_at([-.80,-.70,.65],[.28,0,.28],[0,0,1]),c)[0]

    def draw_view(self,T_BC):
        if self.viewer is not None:self.viewer.sync()

    def execute(self,path):
        motion=self.cfg['motion'];kinematic=motion['execution_mode']=='kinematic'
        settle_frame=self.model.opt.timestep*12
        on_frame=None
        if self.viewer is not None:
            # Pacing lives here, not in the loop: only a live viewer needs wall time.
            def on_frame():
                self.viewer.sync();time.sleep(.015 if kinematic else settle_frame)
        return execution.execute(self.model,self.data,motion,path,set_q=self.set_q,
                                 current_q=lambda:self.q,dof_addresses=self.dadr,
                                 collide=self.collision_current,on_frame=on_frame)

    def export_scene(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        relative=os.path.relpath(ROOT/'assets/mycobot_pro_630',path.parent)
        path.write_text(build_model_xml(self.cfg,relative),encoding='utf-8')

    def close(self):
        for renderer in self.renderers.values():renderer.close()
        self.renderers.clear()
        if self.viewer is not None:self.viewer.close()

    def __enter__(self):return self
    def __exit__(self,*_):self.close()
