"""Re-open exported files and independently verify pose/path/image invariants."""
from pathlib import Path
import json
import sys
import numpy as np
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from viewpoint_sim.config import load_config, crop_settings
from viewpoint_sim.geometry import angle_deg
from viewpoint_sim.motion import path_free
from viewpoint_sim.planner import geometry_check
from viewpoint_sim.world import World


def verify(result):
    folder=Path(result);cfg=load_config(folder/'config.json')
    camera=cfg['camera']['rgb'];crop_enabled=crop_settings(cfg)[0]
    views=json.loads((folder/'views.json').read_text(encoding='utf-8'))
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    logs=[json.loads(line) for line in (folder/'candidates.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len(views)==summary['accepted_total']==sum(x['accepted'] for x in summary['heights'])
    assert len(logs)==sum(h['attempts'] for h in summary['heights'])
    assert sum(x['accepted'] for x in logs)==len(views)
    axes=[];last=np.array(cfg['robot']['home_q_rad'])
    with World(cfg) as world:
        for view in views:
            path=view['path_q_rad'];T=np.array(view['T_BC_actual'])
            np.testing.assert_allclose(path[0],last,atol=1e-9)
            assert path_free(world,path),view['id']
            last=np.array(view['q_actual_rad']);world.set_q(last)
            np.testing.assert_allclose(T,world.camera_pose(),atol=1e-6)
            np.testing.assert_allclose(np.array(view['T_BF_actual'])@view['T_FC'],T,atol=1e-6)
            assert abs(T[2,3]-cfg['roi']['tray_z_B_m']-view['height_above_tray_m']) < cfg['robot']['ik_position_tolerance_m']
            _,reason=geometry_check(cfg,T,view['height_index']);assert reason is None
            for i,d in axes:
                if cfg['sampling']['deduplicate_across_heights'] or i==view['height_index']:
                    assert angle_deg(T[:3,2],d)>=cfg['sampling']['min_axis_angle_deg']-1e-6
            axes.append((view['height_index'],T[:3,2]))
            sample=folder/view['id']
            images={name:np.array(Image.open(sample/(name+'.png'))) for name in ['rgb','depth_mm','instance','food_mask']}
            # Full-frame exports must be uniform and match the configured camera.
            expect=(camera['width'],camera['height'])
            for name,arr in images.items():
                assert arr.shape[1],name
                assert (arr.shape[1],arr.shape[0])==expect,f"{view['id']}:{name} {arr.shape}"
            mask=images['instance']>0
            np.testing.assert_array_equal(mask,images['food_mask']>0)
            # Valid depth covers the whole scene, so only the range contract holds.
            z=np.load(sample/'depth_z_m.npy')
            lo,hi=cfg['camera']['depth_range_m']
            np.testing.assert_array_equal(images['depth_mm']>0,(z>=lo)&(z<=hi))
            np.testing.assert_allclose(images['depth_mm'][images['depth_mm']>0],
                                       np.rint(z[(z>=lo)&(z<=hi)]*1000),atol=1)
            names=sorted(path.name for path in sample.iterdir())
            if crop_enabled:
                x0,y0,x1,y1=view['crop_bbox_xyxy']
                assert (x1-x0,y1-y0)==(view['crop_bbox_xywh'][2],view['crop_bbox_xywh'][3])
                for name in ['rgb','depth_mm','instance','food_mask']:
                    np.testing.assert_array_equal(images[name][y0:y1,x0:x1],
                        np.array(Image.open(sample/('crop_'+name+'.png'))))
                assert any(n.startswith('crop_') for n in names),view['id']
            else:
                assert not any(n.startswith('crop_') for n in names),f"{view['id']}: unexpected crop files"
                assert 'crop_bbox_xyxy' not in view and 'tray_ratio_cropped' not in view
            assert view['image_size_xy']==list(expect)
            assert view['roi_bbox_xyxy'][0]>=0 and view['roi_bbox_xyxy'][2]<=expect[0]
            for q in view['quality']:
                assert q['visible_fraction']>=cfg['quality']['min_visible_fraction_per_food']
                assert q['valid_depth_fraction']>=cfg['quality']['min_valid_depth_fraction']
    print(f'PASS: {len(views)} saved views; uniform {camera["width"]}x{camera["height"]} exports, mask/depth consistency, '
          f'hand-eye transforms, heights, axes, and complete paths verified (crop={"on" if crop_enabled else "off"}).')


if __name__=='__main__':
    verify(sys.argv[1] if len(sys.argv)>1 else 'outputs/demo')
