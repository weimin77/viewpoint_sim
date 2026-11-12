"""Read exports back and revalidate camera geometry, joint paths, and labels."""
from pathlib import Path
import json,sys
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from viewpoint_sim.config import load_config
from viewpoint_sim.world import World
from viewpoint_sim.planner import geometry_check
from viewpoint_sim.motion import path_free
from viewpoint_sim.geometry import angle_deg


def verify(root):
    root=Path(root);cfg=load_config(root/'config.json');views=json.loads((root/'views.json').read_text());s=json.loads((root/'summary.json').read_text())
    assert s['engine']=='mujoco' and s['robot_model']=='myCobot Pro 630'
    assert len(views)==s['accepted_total'];axes=[]
    with World(cfg) as w:
        for v in views:
            assert path_free(w,v['path_q_rad']),v['id'];w.set_q(v['q_actual_rad'])
            T=np.array(v['T_BC_actual']);np.testing.assert_allclose(T,w.camera_pose(),atol=1e-9)
            np.testing.assert_allclose(np.array(v['T_BF_actual'])@v['T_FC'],T,atol=1e-9)
            target=np.array(v['T_BC_target'])
            assert np.linalg.norm(T[:3,3]-target[:3,3])<cfg['robot']['ik_position_tolerance_m']
            assert abs(v['z_jitter_m'])<=cfg['sampling']['z_jitter_m']+1e-12
            assert abs(target[2,3]-cfg['roi']['tray_z_B_m']-v['base_height_above_tray_m']-v['z_jitter_m'])<1e-10
            assert geometry_check(cfg,T,v['height_index'])[1] is None
            for i,a in axes:
                if cfg['sampling']['deduplicate_across_heights'] or i==v['height_index']:
                    assert angle_deg(T[:3,2],a)>=cfg['sampling']['min_axis_angle_deg']-1e-6
            axes.append((v['height_index'],T[:3,2]))
            f=root/v['id'];mask=np.array(Image.open(f/'instance.png'));rgb=np.array(Image.open(f/'rgb.png'));depth=np.array(Image.open(f/'depth_mm.png'))
            assert mask.shape==rgb.shape[:2]==depth.shape
            assert np.array_equal(mask>0,np.array(Image.open(f/'food_mask.png'))>0)
            if cfg['postprocess']['crop_mode']=='none':
                assert not list(f.glob('crop_*.png')) and v['crop_bbox_xyxy'] is None
            else:
                x0,y0,x1,y1=v['crop_bbox_xyxy']
                for name in ['rgb','depth_mm','instance','food_mask']:
                    np.testing.assert_array_equal(np.array(Image.open(f/('crop_'+name+'.png'))),np.array(Image.open(f/(name+'.png')))[y0:y1,x0:x1])
            if cfg['motion']['execution_mode']=='position_control':assert v['execution']['physics_steps']>0
    print(f'PASS: {len(views)} MuJoCo captures verified; joint paths, optical poses, image alignment, jitter and crop mode.')


if __name__=='__main__':verify(sys.argv[1] if len(sys.argv)>1 else 'outputs/demo')
