"""Controlled same-XY height experiment: translation versus recomputed Look-at.

This experiment reports geometry only, and does not label comparison poses as
robot-executable without running the full planner on them.
"""
from pathlib import Path
import json
import numpy as np
from .geometry import look_at, inv, project
from .planner import geometry_check


def compare_heights(cfg, views, output):
    if not views:
        return []
    first=views[0];initial=np.array(first['T_BC_actual'])
    center=(np.array(cfg['roi']['food_min_B'])+cfg['roi']['food_max_B'])/2
    mount=np.array(first['T_FC']);rows=[]
    for i,h in enumerate(cfg['sampling']['heights_above_tray_m']):
        translated=initial.copy();translated[2,3]=cfg['roi']['tray_z_B_m']+h
        for mode,T in [('translate',translated),('relook',look_at(translated[:3,3],center,cfg['sampling']['reference_up_B']))]:
            g,reason=geometry_check(cfg,T,i)
            uv,z=project(center[None,:],T,cfg['camera']['rgb'])
            displacement=uv[0]-[cfg['camera']['rgb']['cx'],cfg['camera']['rgb']['cy']]
            flange=T@inv(mount)
            rows.append(dict(height_above_tray_m=h,mode=mode,T_BC=T.tolist(),T_BF=flange.tolist(),
                             target_center_offset_px=float(np.linalg.norm(displacement)),
                             geometry_pass=reason is None,reason=reason,geometry=g,
                             robot_checks_performed=False))
    (Path(output)/'height_comparison.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    return rows
