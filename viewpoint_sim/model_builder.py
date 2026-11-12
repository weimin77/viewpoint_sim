"""Build MJCF from the pinned official Pro 630 URDF, plus a camera workcell.

URDF joint origins/axes/limits and visual transforms are copied unchanged.
Dynamics inertias and servo gains are explicitly example parameters.
"""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from .config import ROOT, mount_transform


def numbers(values):
    return ' '.join(format(float(x),'.12g') for x in values)


def quat(R):
    x,y,z,w=Rotation.from_matrix(R).as_quat()
    return numbers([w,x,y,z])


def origin(element):
    o=element.find('origin')
    if o is None:return np.zeros(3),np.eye(3)
    xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
    rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
    return xyz,Rotation.from_euler('xyz',rpy).as_matrix()


def build_model_xml(cfg, meshdir=None):
    robot_dir=ROOT/'assets/mycobot_pro_630'
    urdf=E.parse(ROOT/cfg['robot']['urdf']).getroot()
    root=E.Element('mujoco',model='myCobot_Pro_630_viewpoint_workcell')
    E.SubElement(root,'compiler',angle='radian',meshdir=str(meshdir or robot_dir),autolimits='true')
    option=E.SubElement(root,'option',timestep=str(cfg['motion']['timestep_s']),integrator='implicitfast',gravity='0 0 -9.81')
    E.SubElement(option,'flag',nativeccd='enable')
    E.SubElement(root,'size',njmax='3000',nconmax='1000')
    visual=E.SubElement(root,'visual')
    E.SubElement(visual,'global',offwidth='1000',offheight='700',azimuth='135',elevation='-20')
    E.SubElement(visual,'quality',shadowsize='2048',offsamples='4')
    E.SubElement(visual,'headlight',ambient='.45 .45 .45',diffuse='.5 .5 .5',specular='.1 .1 .1')
    E.SubElement(visual,'map',znear='.002',zfar='20')
    default=E.SubElement(root,'default')
    E.SubElement(default,'geom',friction='.7 .01 .001',solref='.02 1')
    E.SubElement(default,'joint',damping='4',armature='.02')
    asset=E.SubElement(root,'asset')
    E.SubElement(asset,'texture',name='floor_tex',type='2d',builtin='checker',rgb1='.20 .24 .29',rgb2='.24 .28 .33',width='256',height='256')
    E.SubElement(asset,'material',name='floor_mat',texture='floor_tex',texrepeat='5 4',reflectance='.05')
    for name in ['base']+[f'link{i}' for i in range(1,7)]:
        E.SubElement(asset,'mesh',name=name+'_mesh',file=name+'.obj')
    world=E.SubElement(root,'worldbody')
    E.SubElement(world,'light',pos='-.1 -.5 1.7',dir='.2 .3 -1',diffuse='.85 .85 .85',castshadow='true')
    E.SubElement(world,'light',pos='.6 .25 .45',dir='-.3 -.2 -.8',diffuse='.35 .35 .30',castshadow='false')
    table=cfg['scene']['table']
    body=E.SubElement(world,'body',name='table',pos=numbers(table['position_B_m']))
    E.SubElement(body,'geom',name='env_table',type='box',size=numbers(np.array(table['size_m'])/2),material='floor_mat',group='0')

    def box(name,pos,size,color,food=False):
        body=E.SubElement(world,'body',name=name,pos=numbers(pos))
        E.SubElement(body,'geom',name=('food_' if food else 'env_')+name,type='box',
                     size=numbers(np.array(size)/2),rgba=numbers(color),group='2' if food else '0')
        return body

    oven=cfg['scene']['oven']
    if oven['enabled']:
        x0,x1=oven['x_B_m'];y0,y1=oven['y_B_m'];z0,z1=oven['z_B_m'];t=oven['wall_thickness_m']
        metal=[.40,.43,.47,1]
        box('oven_floor',[(x0+x1)/2,(y0+y1)/2,z0-t/2],[x1-x0+2*t,y1-y0+2*t,t],metal)
        box('oven_back',[x1+t/2,(y0+y1)/2,(z0+z1)/2],[t,y1-y0+2*t,z1-z0],metal)
        box('oven_left',[(x0+x1)/2,y0-t/2,(z0+z1)/2],[x1-x0,t,z1-z0],metal)
        box('oven_right',[(x0+x1)/2,y1+t/2,(z0+z1)/2],[x1-x0,t,z1-z0],metal)
        # The top stays open. Only side walls, back wall, rails and wire rack.
        rack=E.SubElement(world,'body',name='oven_rack')
        z=oven['rack_z_B_m'];r=oven['rack_rod_radius_m']
        def rod(name,a,b):
            E.SubElement(rack,'geom',name='env_'+name,type='capsule',fromto=numbers([*a,*b]),size=str(r),
                         rgba='.68 .71 .73 1',group='0')
        for j,y in enumerate(np.linspace(y0+.02,y1-.02,13)):
            rod('rack_bar_'+str(j),[x0+.015,y,z],[x1-.015,y,z])
        for j,x in enumerate([x0+.015,x1-.015]):
            rod('rack_cross_'+str(j),[x,y0+.02,z],[x,y1-.02,z])
        for k,height in enumerate(oven['rail_heights_B_m']):
            for side,y in enumerate([y0+.008,y1-.008]):
                rod(f'rail_{k}_{side}',[x0+.01,y,height],[x1-.01,y,height])
    tray=cfg['scene']['tray']
    box('tray',tray['position_B_m'],tray['size_m'],[.25,.28,.31,1])
    # Tray lip remains below the Food ROI bottom; it is part of physical geometry.
    if tray.get('lip_height_m',0)>0:
        tx,ty,tz=tray['position_B_m'];sx,sy,sz=tray['size_m'];lh=tray['lip_height_m'];th=.003
        for name,position,size in [
            ('tray_left',[tx,ty-sy/2+th/2,tz+sz/2+lh/2],[sx,th,lh]),
            ('tray_right',[tx,ty+sy/2-th/2,tz+sz/2+lh/2],[sx,th,lh]),
            ('tray_front',[tx-sx/2+th/2,ty,tz+sz/2+lh/2],[th,sy,lh]),
            ('tray_back',[tx+sx/2-th/2,ty,tz+sz/2+lh/2],[th,sy,lh])]:box(name,position,size,[.35,.38,.40,1])
    for ob in cfg['scene'].get('obstacles',[]):box(ob['name'],ob['position_B_m'],ob['size_m'],ob.get('rgba',[.5,.5,.5,1]))
    for f in cfg['scene']['foods']:
        if f['shape']=='box':box(f['name'],f['position_B_m'],f['size_m'],f['rgba'],True)
        else:
            b=E.SubElement(world,'body',name=f['name'],pos=numbers(f['position_B_m']))
            E.SubElement(b,'geom',name='food_'+f['name'],type='sphere',size=str(f['radius_m']),rgba=numbers(f['rgba']),group='2')
    base=E.SubElement(world,'body',name='base',pos=numbers(cfg['robot']['base_position_B_m']))
    links={e.get('name'):e for e in urdf.findall('link')}
    by_parent={j.find('parent').get('link'):j for j in urdf.findall('joint')}
    colors={0:[.25,.28,.32,1],1:[.70,.73,.76,1],2:[.38,.41,.45,1],3:[.38,.41,.45,1],4:[.69,.71,.73,1],5:[.67,.69,.72,1],6:[.32,.35,.39,1]}
    mass=[.9,1.4,1.2,.45,.35,.15]
    for index,name in enumerate(['base']+[f'link{i}' for i in range(1,7)]):
        if index:
            joint=by_parent['base' if index==1 else f'link{index-1}']
            pos,R=origin(joint)
            base=E.SubElement(base,'body',name=name,pos=numbers(pos),quat=quat(R),gravcomp='1')
            lim=joint.find('limit');axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            E.SubElement(base,'joint',name=joint.get('name'),type='hinge',axis=numbers(axis),range=lim.get('lower')+' '+lim.get('upper'))
            E.SubElement(base,'inertial',pos='0 0 0',mass=str(mass[index-1]),diaginertia='.008 .008 .008')
        pos,R=origin(links[name].find('visual'))
        E.SubElement(base,'geom',name='vis_'+name,type='mesh',mesh=name+'_mesh',pos=numbers(pos),quat=quat(R),
                     rgba=numbers(colors[index]),group='1',contype='0',conaffinity='0',mass='0')
        # Same transformed mesh, convex for collision in MuJoCo; visual and contact
        # meshes are kept separate so segmentation/rendering do not show hulls.
        E.SubElement(base,'geom',name='col_'+name,type='mesh',mesh=name+'_mesh',pos=numbers(pos),quat=quat(R),
                     rgba='.8 .2 .2 .15',group='3',mass='0')
    mount=mount_transform(cfg);cam=cfg['camera']
    half=np.array(cam['housing_size_m'])/2
    housing=E.SubElement(base,'body',name='camera_housing',pos=numbers(mount[:3,3]-mount[:3,:3]@[0,0,half[2]]),quat=quat(mount[:3,:3]))
    E.SubElement(housing,'geom',name='col_camera_housing',type='box',size=numbers(half),rgba='.08 .14 .20 1',group='1')
    bracket=E.SubElement(base,'body',name='camera_bracket',pos=numbers(cam['bracket_center_F_m']))
    E.SubElement(bracket,'geom',name='col_camera_bracket',type='box',size=numbers(np.array(cam['bracket_size_m'])/2),rgba='.10 .65 .68 1',group='1')
    E.SubElement(base,'site',name='camera_optical',pos=numbers(mount[:3,3]),quat=quat(mount[:3,:3]),size='.003',rgba='0 0 0 0')
    optical_to_gl=np.diag([1,-1,-1])
    camera_attrs=dict(sensorsize='640 360',resolution='640 360',focal='337.2 324.7',principal='0 0',ipd='0')
    E.SubElement(base,'camera',name='wrist_camera',pos=numbers(mount[:3,3]),quat=quat(mount[:3,:3]@optical_to_gl),**camera_attrs)
    pose=E.SubElement(world,'body',name='render_pose',mocap='true',pos='0 0 1')
    E.SubElement(pose,'camera',name='render_camera',**camera_attrs)
    contact=E.SubElement(root,'contact')
    srdf=E.parse(ROOT/cfg['robot']['srdf']).getroot()
    allowed=set()
    for e in srdf.findall('disable_collisions'):
        a,b=e.get('link1'),e.get('link2')
        if a in links and b in links:allowed.add(tuple(sorted((a,b))))
    allowed.update(tuple(sorted(pair)) for pair in [('link6','camera_housing'),('link6','camera_bracket'),('camera_housing','camera_bracket')])
    for a,b in sorted(allowed):E.SubElement(contact,'exclude',body1=a,body2=b)
    actuator=E.SubElement(root,'actuator')
    for j in urdf.findall('joint'):
        E.SubElement(actuator,'position',name='servo_'+j.get('name'),joint=j.get('name'),kp='1000',kv='65',ctrlrange=j.find('limit').get('lower')+' '+j.find('limit').get('upper'))
    E.indent(root)
    return E.tostring(root,encoding='unicode')
