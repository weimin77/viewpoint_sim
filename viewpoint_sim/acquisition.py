"""Capture rendering and dataset export.

``inspect_capture`` renders one pose and delegates the acceptance verdict to
``quality.assess``; ``save_capture`` writes the dataset files and the metadata
record. The QA thresholds themselves live in ``quality`` so they can be tested
without a simulator.
"""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw
from .geometry import crop_box, cropped_intrinsics, tray_ratio
from .quality import assess


def inspect_capture(world, T_BC):
    """Render one capture, then hand the arrays to the quality assessment.

    Rendering is the only engine-dependent part; ``quality.assess`` is a pure
    function of the arrays, so the acceptance thresholds can be tested without a
    simulator.
    """
    rgb, z, body = world.render(T_BC)
    reference = world.isolated_food_masks(T_BC)
    report = assess(z, body, reference, world.food_ids, world.names, world.cfg)
    return {"rgb": rgb, "z": z.astype(np.float32),
            "depth_mm": report["depth_mm"], "labels": report["labels"],
            "metrics": report["metrics"], "failures": report["failures"]}


def save_capture(folder, capture, record, geometry, cfg):
    folder = Path(folder)
    folder.mkdir(parents=True)
    rgb, depth, labels = (capture[k] for k in ("rgb","depth_mm","labels"))
    food_uv, tray_uv = np.array(geometry["food_uv"]), np.array(geometry["tray_uv"])
    rect = crop_box(food_uv,tray_uv,cfg["camera"]["rgb"],cfg["postprocess"]["crop_padding_px"])
    x0,y0,x1,y1 = rect
    if int(np.count_nonzero(labels[y0:y1,x0:x1])) != int(np.count_nonzero(labels)):
        raise RuntimeError("Crop would remove visible food pixels")
    enabled=cfg['postprocess']['crop_mode']=='tray_roi'
    branches=[('',(rgb,depth,labels))]
    if enabled:branches.append(('crop_',(rgb[y0:y1,x0:x1],depth[y0:y1,x0:x1],labels[y0:y1,x0:x1])))
    for prefix, arrays in branches:
        for name, arr in zip(("rgb","depth_mm","instance"),arrays):
            Image.fromarray(arr).save(folder/(prefix+name+".png"))
        Image.fromarray((arrays[2]>0).astype(np.uint8)*255).save(folder/(prefix+"food_mask.png"))
    np.save(folder/"depth_z_m.npy",capture["z"])
    overlay = Image.fromarray(rgb)
    d = ImageDraw.Draw(overlay)
    # Food ROI edges follow box_vertices' bit order: x/y/z bits 2/1/0.
    for i in range(8):
        for bit in (1,2,4):
            j = i^bit
            if i < j:
                d.line([tuple(food_uv[i]),tuple(food_uv[j])],fill=(40,235,175),width=2)
    d.line([tuple(v) for v in tray_uv]+[tuple(tray_uv[0])],fill=(255,200,60),width=2)
    d.rectangle([x0,y0,x1-1,y1-1],outline=(80,170,255),width=2)
    overlay.save(folder/"overlay.png")
    record.update(geometry=geometry,quality=capture["metrics"],crop_mode=cfg['postprocess']['crop_mode'],crop_bbox_xyxy=rect if enabled else None,
                  crop_bbox_xywh=[x0,y0,x1-x0,y1-y0] if enabled else None,
                  K_crop=cropped_intrinsics(cfg["camera"]["rgb"],rect) if enabled else None,
                  tray_ratio_cropped=tray_ratio(tray_uv,cfg["camera"]["rgb"],rect) if enabled else None,
                  label_definition="Visible surfaces; 0=background, IDs 1..N in config scene.foods order",
                  depth_definition="Optical Z; PNG uint16 millimeters, 0 invalid; NPY geometric float32 meters before range masking")
    (folder/"metadata.json").write_text(json.dumps(record,indent=2,ensure_ascii=False),encoding="utf-8")
    return record
