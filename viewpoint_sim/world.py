"""PyBullet workcell. Motion is kinematic, with explicit configuration/path checks."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from .config import ROOT, mount_transform
from .geometry import transform, projection_matrix, apply, inv, box_vertices
from .kinematics import Kinematics


class World:
    def __init__(self, cfg, gui=False):
        self.cfg, self.gui = cfg, gui
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        if self.cid < 0:
            raise RuntimeError("Cannot connect to PyBullet; try --headless")
        p.setGravity(0, 0, 0, physicsClientId=self.cid)
        self.tmp = tempfile.TemporaryDirectory(prefix="viewpoint_sim_")
        self.mount = mount_transform(cfg)
        robot = cfg["robot"]
        source = ROOT / robot["urdf"]
        self.kin = Kinematics(source, robot["flange_link"], robot["base_position_B_m"])
        urdf = self._camera_urdf(source)
        self.robot = p.loadURDF(str(urdf), robot["base_position_B_m"], useFixedBase=True,
                               flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.cid)
        self.link_ids = {"dummy_link": -1}
        self.joints = []
        for j in range(p.getNumJoints(self.robot, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot, j, physicsClientId=self.cid)
            self.link_ids[info[12].decode()] = j
            if info[1].decode() in self.kin.names:
                self.joints.append(j)
        self.flange = self.link_ids[robot["flange_link"]]
        srdf = ET.parse(ROOT / robot["srdf"])
        self.allowed = {frozenset((self.link_ids[e.get("link1")], self.link_ids[e.get("link2")]))
                        for e in srdf.findall("disable_collisions")
                        if e.get("link1") in self.link_ids and e.get("link2") in self.link_ids}
        # Links joined by a joint cannot meaningfully collide (MoveIt "Adjacent").
        tree = ET.parse(source).getroot()
        links = [e.get("name") for e in tree.findall("link")]
        children = {e.find("child").get("link") for e in tree.findall("joint")}
        for root_link in (n for n in links if n not in children):
            self.link_ids.setdefault(root_link, -1)
        for j in tree.findall("joint"):
            a, b = j.find("parent").get("link"), j.find("child").get("link")
            if a in self.link_ids and b in self.link_ids:
                self.allowed.add(frozenset((self.link_ids[a], self.link_ids[b])))
        # Only explicit attachment contacts are exempted for the fixture.
        self.camera_links = ("camera_bracket", "camera_housing")
        for a, b in [(robot["flange_link"], self.camera_links[0]),
                     (robot["flange_link"], self.camera_links[1]),
                     (self.camera_links[0], self.camera_links[1])]:
            self.allowed.add(frozenset((self.link_ids[a], self.link_ids[b])))
        self.environment, self.food_ids, self.names = [], [], {self.robot: "robot"}
        scene = cfg["scene"]
        self.table = self.box("table", scene["table"]["position_B_m"], scene["table"]["size_m"], [.24,.29,.35,1])
        self.tray = self.box("tray", scene["tray"]["position_B_m"], scene["tray"]["size_m"], [.28,.32,.36,1])
        for ob in scene["obstacles"]:
            self.box(ob["name"], ob["position_B_m"], ob["size_m"], [.55,.60,.65,1])
        for food in scene["foods"]:
            if food["shape"] == "box":
                bid = self.box(food["name"], food["position_B_m"], food["size_m"], food["rgba"], food=True)
            elif food["shape"] == "sphere":
                col = p.createCollisionShape(p.GEOM_SPHERE, radius=food["radius_m"], physicsClientId=self.cid)
                vis = p.createVisualShape(p.GEOM_SPHERE, radius=food["radius_m"], rgbaColor=food["rgba"], physicsClientId=self.cid)
                bid = p.createMultiBody(0, col, vis, food["position_B_m"], physicsClientId=self.cid)
                self.names[bid] = food["name"]
            else:
                raise ValueError("Only box/sphere food shapes are supported")
            self.food_ids.append(bid)
        self.visuals = [(b, v[1], v[7]) for b in [self.robot]+self.environment+self.food_ids
                        for v in p.getVisualShapeData(b, physicsClientId=self.cid)]
        self.set_q(robot["home_q_rad"])
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.cid)
            p.resetDebugVisualizerCamera(.62, 45, -28, [.16,0,.10], physicsClientId=self.cid)
            self.draw_scene()

    def draw_scene(self):
        roi=self.cfg['roi'];s=self.cfg['sampling']
        pad=roi['margin_m']
        points=box_vertices(np.array(roi['food_min_B'])-pad,np.array(roi['food_max_B'])+pad)
        for i in range(8):
            for bit in (1,2,4):
                j=i^bit
                if i<j:
                    p.addUserDebugLine(points[i],points[j],[.2,.95,.65],2,physicsClientId=self.cid)
        tray=np.array(roi['tray_corners_B'])
        for a,b in zip(tray,np.roll(tray,-1,axis=0)):
            p.addUserDebugLine(a,b,[1,.75,.2],2,physicsClientId=self.cid)
        for i,h in enumerate(s['heights_above_tray_m']):
            z=roi['tray_z_B_m']+h;x0,x1=s['x_B_m'];y0,y1=s['y_B_m']
            corners=np.array([[x0,y0,z],[x1,y0,z],[x1,y1,z],[x0,y1,z]])
            for a,b in zip(corners,np.roll(corners,-1,axis=0)):
                p.addUserDebugLine(a,b,[.3,.6,.95],1,physicsClientId=self.cid)
            p.addUserDebugText(f'h={h*1000:.0f} mm',corners[0],textColorRGB=[.3,.6,.95],textSize=1,physicsClientId=self.cid)

    def draw_view(self,T_BC):
        if not self.gui:
            return
        c=self.cfg['camera']['rgb'];depth=.10
        uv=np.array([[0,0],[c['width']-1,0],[c['width']-1,c['height']-1],[0,c['height']-1]])
        corners=np.c_[(uv-[c['cx'],c['cy']])/[c['fx'],c['fy']]*depth,np.full(4,depth)]
        corners=apply(T_BC,corners)
        for a,b in zip(corners,np.roll(corners,-1,axis=0)):
            p.addUserDebugLine(a,b,[.2,.9,.7],1,physicsClientId=self.cid)
        for corner in corners:
            p.addUserDebugLine(T_BC[:3,3],corner,[.2,.9,.7],1,physicsClientId=self.cid)

    def _camera_urdf(self, source):
        tree = ET.parse(source)
        root = tree.getroot()
        for mesh in root.findall(".//mesh"):
            mesh.set("filename", str((source.parent/mesh.get("filename")).resolve()))
        def add_box(name, xyz, rpy, size, rgba):
            link = ET.SubElement(root, "link", name=name)
            inert = ET.SubElement(link, "inertial")
            ET.SubElement(inert, "mass", value="0.06")
            ET.SubElement(inert, "inertia", ixx="0.00001", ixy="0", ixz="0", iyy="0.00001", iyz="0", izz="0.00001")
            for tag in ["visual", "collision"]:
                el = ET.SubElement(link, tag)
                g = ET.SubElement(el, "geometry")
                ET.SubElement(g, "box", size=" ".join(map(str, size)))
                if tag == "visual":
                    mat = ET.SubElement(el, "material", name=name)
                    ET.SubElement(mat, "color", rgba=" ".join(map(str, rgba)))
            joint = ET.SubElement(root, "joint", name=name+"_fixed", type="fixed")
            ET.SubElement(joint, "parent", link=self.cfg["robot"]["flange_link"])
            ET.SubElement(joint, "child", link=name)
            ET.SubElement(joint, "origin", xyz=" ".join(map(str, xyz)), rpy=" ".join(map(str, rpy)))
        cam = self.cfg["camera"]
        size = np.array(cam["housing_size_m"])
        center = self.mount[:3, 3] - self.mount[:3, :3] @ [0,0,size[2]/2]
        add_box("camera_housing", center, cam["mount"]["rpy_FC_rad"], size, [.10,.15,.20,1])
        bs = cam["bracket_size_m"]
        add_box("camera_bracket", [0,0,bs[2]/2], [0,0,0], bs, [.18,.65,.70,1])
        path = Path(self.tmp.name)/"robot_camera.urdf"
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return path

    def box(self, name, xyz, size, rgba, food=False):
        half = (np.asarray(size)/2).tolist()
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.cid)
        bid = p.createMultiBody(0, col, vis, xyz, physicsClientId=self.cid)
        self.names[bid] = name
        if not food:
            self.environment.append(bid)
        return bid

    def set_q(self, q):
        for j, v in zip(self.joints, q):
            p.resetJointState(self.robot, j, float(v), targetVelocity=0, physicsClientId=self.cid)
        self.q = np.asarray(q).copy()

    def flange_pose(self):
        state = p.getLinkState(self.robot, self.flange, computeForwardKinematics=True, physicsClientId=self.cid)
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(state[5]).as_matrix()
        T[:3, 3] = state[4]
        return T

    def camera_pose(self):
        return self.flange_pose() @ self.mount

    def collision(self, q):
        rc = self.cfg["robot"]
        margin = rc["joint_limit_margin_rad"]
        if np.any(q < self.kin.lower+margin-1e-8) or np.any(q > self.kin.upper-margin+1e-8):
            return "joint_limit"
        self.set_q(q)
        distance = self.cfg["motion"]["collision_margin_m"]
        for c in p.getClosestPoints(self.robot, self.robot, distance, physicsClientId=self.cid):
            a, b = c[3], c[4]
            if a >= b or frozenset((a,b)) in self.allowed:
                continue
            # The wrist mesh ends on the flange plane. Its contact with the
            # bracket's mounting face is intentional; exempt only that small
            # nonpenetrating interface, not the entire link5/bracket pair.
            if {a,b} == {self.link_ids[self.kin.flange_parent],self.link_ids["camera_bracket"]} and c[8] >= -1e-5:
                points_F = apply(inv(self.flange_pose()),[c[5],c[6]])
                if np.max(np.abs(points_F[:,2])) < .003 and np.max(np.linalg.norm(points_F[:,:2],axis=1)) < .012:
                    continue
            return f"self_collision:{a}:{b}"
        for body in self.environment+self.food_ids:
            for c in p.getClosestPoints(self.robot, body, distance, physicsClientId=self.cid):
                # The fixed robot base may touch its mounting table.
                if body == self.table and c[3] == -1:
                    continue
                return f"environment_collision:{self.names[body]}"
        return None

    def render(self, T_BC, camera=None):
        camera = camera or self.cfg["camera"]["rgb"]
        R, t = T_BC[:3, :3], T_BC[:3, 3]
        view = p.computeViewMatrix(t, t+R[:,2], -R[:,1])
        near, far = .01, 2.
        w, h = camera["width"], camera["height"]
        result = p.getCameraImage(w, h, view, projection_matrix(camera, near, far),
                                 renderer=p.ER_TINY_RENDERER,
                                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                                 shadow=0, lightDirection=[-.5,-.5,1],
                                 physicsClientId=self.cid)
        rgb = np.asarray(result[2], dtype=np.uint8).reshape(h,w,4)[...,:3].copy()
        buf = np.asarray(result[3]).reshape(h,w)
        depth = far*near/(far-(far-near)*buf)
        seg = np.asarray(result[4], dtype=np.int32).reshape(h,w)
        body = np.where(seg < 0, -1, seg & ((1 << 24)-1))
        return rgb, depth, body

    def isolated_food_masks(self, T_BC):
        """Reference visible silhouette of each food with every occluder removed.

        Each food keeps its own self-occlusion. Other foods are removed too, so
        mutual food occlusion is measured rather than silently ignored.
        """
        masks = []
        try:
            for body in self.food_ids:
                for b, link, color in self.visuals:
                    rgba = color if b == body else (0,0,0,0)
                    p.changeVisualShape(b, link, rgbaColor=rgba, physicsClientId=self.cid)
                _, _, ids = self.render(T_BC)
                masks.append(ids == body)
        finally:
            for b, link, color in self.visuals:
                p.changeVisualShape(b, link, rgbaColor=color, physicsClientId=self.cid)
        return masks

    def overview(self):
        # The perspective is only for a third-person preview, not capture geometry.
        eye = [.62,-.58,.50]
        target = [.16,0,.14]
        from .geometry import look_at
        c = {"width":960,"height":640,"fx":560.,"fy":560.,"cx":479.5,"cy":319.5}
        return self.render(look_at(eye, target, [0,0,1]), c)[0]

    def close(self):
        if p.isConnected(self.cid):
            p.disconnect(physicsClientId=self.cid)
        self.tmp.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
