import copy,json
from pathlib import Path
import numpy as np
from .geometry import transform

ROOT=Path(__file__).resolve().parents[1]
if not (ROOT/'assets/mycobot_pro_630').is_dir():ROOT=Path(__file__).resolve().parent


def load_config(path=None):
    cfg=json.loads((Path(path) if path else ROOT/'configs/default.json').read_text(encoding='utf-8'))
    return validate_config(cfg)


def validate_config(cfg):
    if cfg['units']!='m_rad' or cfg['engine']!='mujoco' or cfg['robot']['model']!='myCobot Pro 630':
        raise ValueError('Expected MuJoCo / myCobot Pro 630, lengths in meters and angles in radians')
    cam=cfg['camera']
    if cam['rgb']!=cam['depth'] or not np.allclose(cam['T_CD'],np.eye(4)):
        raise ValueError('This version requires aligned RGB/Depth with equal intrinsics and T_CD=I')
    for c in [cam['rgb'],cam['depth']]:
        if min(c['width'],c['height'],c['fx'],c['fy'])<=0:raise ValueError('Invalid camera dimensions/intrinsics')
        if not (0<=c['cx']<c['width'] and 0<=c['cy']<c['height']):raise ValueError('Invalid principal point')
    lo,hi=np.array(cfg['roi']['food_min_B']),np.array(cfg['roi']['food_max_B'])
    if not np.all(lo<hi):raise ValueError('Invalid Food ROI')
    if not 0<cam['depth_range_m'][0]<cam['depth_range_m'][1]:raise ValueError('Invalid depth range')
    s=cfg['sampling']
    if len(s['tray_min_ratio_by_height'])!=len(s['heights_above_tray_m']):raise ValueError('One tray threshold is required per height')
    if s['target_per_height']<1 or s['max_attempts_per_height']<1 or not s['heights_above_tray_m']:raise ValueError('Invalid sampling counts')
    if s['z_jitter_m']<0 or any(h<=s['z_jitter_m'] for h in s['heights_above_tray_m']):raise ValueError('Invalid height/jitter')
    if any(s[k][0]>=s[k][1] for k in ['x_B_m','y_B_m']):raise ValueError('Invalid XY range')
    if not 0<=s['min_axis_angle_deg']<=180:raise ValueError('Invalid axis angle')
    if cfg['postprocess']['crop_mode'] not in ['none','tray_roi']:raise ValueError('crop_mode must be none or tray_roi')
    if cfg['motion']['execution_mode'] not in ['kinematic','position_control']:raise ValueError('Invalid execution mode')
    if cfg['motion']['edge_resolution_rad']<=0 or cfg['motion']['collision_margin_m']<0:raise ValueError('Invalid motion settings')
    for f in cfg['scene']['foods']:
        half=np.array(f['size_m'])/2 if f['shape']=='box' else np.repeat(f['radius_m'],3)
        p=np.array(f['position_B_m'])
        if np.any(p-half<lo-1e-6) or np.any(p+half>hi+1e-6):raise ValueError(f"Food {f['name']} is outside the ROI")
    return cfg


def mount_transform(cfg):
    m=cfg['camera']['mount'];return transform(m['translation_F_m'],m['rpy_FC_rad'])


def clone(cfg):return copy.deepcopy(cfg)
