"""Save bounded failing experiments and verify hard constraints never get relaxed."""
from pathlib import Path
import copy
import json
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from viewpoint_sim.config import load_config
from viewpoint_sim.world import World
from viewpoint_sim.planner import run


def main():
    root=Path(sys.argv[1] if len(sys.argv)>1 else 'outputs/negative_cases')
    base=load_config();cases=[]
    for name in ['too_low','beyond_depth','impossible_tray_ratio','opaque_roof','unreachable']:
        cfg=copy.deepcopy(base);s=cfg['sampling']
        s.update(heights_above_tray_m=[.30],target_per_height=1,max_attempts_per_height=8,tray_min_ratio_by_height=[0])
        if name=='too_low':s['heights_above_tray_m']=[.10]
        if name=='beyond_depth':s['heights_above_tray_m']=[.65]
        if name=='impossible_tray_ratio':s['tray_min_ratio_by_height']=[.95]
        if name=='opaque_roof':
            cfg['scene']['obstacles'].append(dict(name='opaque_roof',position_B_m=[.35,0,.20],size_m=[.32,.24,.01]))
        if name=='unreachable':cfg['robot']['base_position_B_m']=[-2,0,0]
        with World(cfg) as world:
            summary=run(world,root/name)
        assert summary['accepted_total']==0 and not summary['complete'],name
        cases.append(dict(case=name,passed=True,rejections=summary['heights'][0]['rejections']))
    (root/'verification.json').write_text(json.dumps(cases,indent=2),encoding='utf-8')
    print(json.dumps(cases,indent=2))


if __name__=='__main__':
    main()
