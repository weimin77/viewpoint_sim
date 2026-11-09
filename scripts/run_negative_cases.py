"""Save bounded failing experiments and verify hard constraints never get relaxed."""
from pathlib import Path
import copy
import json
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from viewpoint_sim.config import load_config
from viewpoint_sim.world import World
from viewpoint_sim.planner import run


def main():
    root=Path(sys.argv[1] if len(sys.argv)>1 else 'outputs/negative_cases')
    base=load_config();cases=[]
    roi=base['roi'];center=(np.array(roi['food_min_B'])+np.array(roi['food_max_B']))/2
    height=base['sampling']['heights_above_tray_m'][len(base['sampling']['heights_above_tray_m'])//2]
    for name in ['too_low','beyond_depth','impossible_tray_ratio','opaque_roof','unreachable']:
        cfg=copy.deepcopy(base);s=cfg['sampling']
        s.update(heights_above_tray_m=[height],target_per_height=1,max_attempts_per_height=8,tray_min_ratio_by_height=[0])
        if name=='too_low':s['heights_above_tray_m']=[.08]
        if name=='beyond_depth':s['heights_above_tray_m']=[.65]
        if name=='impossible_tray_ratio':s['tray_min_ratio_by_height']=[.95]
        if name=='opaque_roof':
            span=np.array(roi['food_max_B'])-np.array(roi['food_min_B'])
            cfg['scene']['obstacles'].append(dict(name='opaque_roof',
                position_B_m=[float(center[0]),float(center[1]),roi['tray_z_B_m']+.16],
                size_m=[float(span[0]+.04),float(span[1]+.04),.006]))
        if name=='unreachable':cfg['robot']['base_position_B_m']=[-2,0,0]
        with World(cfg) as world:
            summary=run(world,root/name)
        assert summary['accepted_total']==0 and not summary['complete'],name
        cases.append(dict(case=name,passed=True,rejections=summary['heights'][0]['rejections']))
    (root/'verification.json').write_text(json.dumps(cases,indent=2),encoding='utf-8')
    print(json.dumps(cases,indent=2))


if __name__=='__main__':
    main()
