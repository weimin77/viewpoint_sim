"""Synthetic aligned RGB-D, visible-instance labels, and per-object visibility QA."""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw
from .geometry import crop_box, cropped_intrinsics, tray_ratio


def inspect_capture(world, T_BC):
    rgb, z, body = world.render(T_BC)
    reference = world.isolated_food_masks(T_BC)
    lo, hi = world.cfg["camera"]["depth_range_m"]
    depth_valid = (z >= lo) & (z <= hi) & (body >= 0)
    labels = np.zeros(body.shape, dtype=np.uint16)
    metrics, reasons = [], []
    qc = world.cfg["quality"]
    for instance, (bid, ref) in enumerate(zip(world.food_ids, reference), 1):
        mask = body == bid
        labels[mask] = instance
        nref, count = int(ref.sum()), int(mask.sum())
        visible = count/max(1,nref)
        good_depth = float(depth_valid[mask].mean()) if count else 0.
        ys,xs = np.nonzero(mask)
        bbox = [int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if count else [0]*4
        side = min(bbox[2]-bbox[0],bbox[3]-bbox[1])
        cut = bool(count and (mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any()))
        metrics.append(dict(instance_id=instance,name=world.names[bid],reference_pixels=nref,
                            visible_pixels=count,visible_fraction=visible,
                            valid_depth_fraction=good_depth,bbox_xyxy=bbox,min_bbox_side_px=side))
        if visible < qc["min_visible_fraction_per_food"]:
            reasons.append("occlusion")
        if count < qc["min_food_pixels"] or side < qc["min_food_bbox_side_px"]:
            reasons.append("insufficient_food_pixels")
        if good_depth < qc["min_valid_depth_fraction"]:
            reasons.append("invalid_depth")
        if cut:
            reasons.append("food_cut_by_image_border")
    # Zero represents invalid samples; valid depth stores optical Z in millimeters.
    depth_mm = np.where(depth_valid, np.rint(z*1000),0).astype(np.uint16)
    priority={"occlusion":0,"food_cut_by_image_border":1,"invalid_depth":2,"insufficient_food_pixels":3}
    return {"rgb":rgb, "z":z.astype(np.float32), "depth_mm":depth_mm,
            "labels":labels,"metrics":metrics,"failures":sorted(set(reasons),key=priority.get)}


def save_capture(folder, capture, record, geometry, cfg):
    folder = Path(folder)
    folder.mkdir(parents=True)
    rgb, depth, labels = (capture[k] for k in ("rgb","depth_mm","labels"))
    food_uv, tray_uv = np.array(geometry["food_uv"]), np.array(geometry["tray_uv"])
    rect = crop_box(food_uv,tray_uv,cfg["camera"]["rgb"],cfg["quality"]["crop_padding_px"])
    x0,y0,x1,y1 = rect
    if int(np.count_nonzero(labels[y0:y1,x0:x1])) != int(np.count_nonzero(labels)):
        raise RuntimeError("Crop would remove visible food pixels")
    for prefix, arrays in [("",(rgb,depth,labels)),("crop_",(rgb[y0:y1,x0:x1],depth[y0:y1,x0:x1],labels[y0:y1,x0:x1]))]:
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
    record.update(geometry=geometry,quality=capture["metrics"],crop_bbox_xyxy=rect,
                  crop_bbox_xywh=[x0,y0,x1-x0,y1-y0],
                  K_crop=cropped_intrinsics(cfg["camera"]["rgb"],rect),
                  tray_ratio_cropped=tray_ratio(tray_uv,cfg["camera"]["rgb"],rect),
                  label_definition="Visible surfaces; 0=background, IDs 1..N in config scene.foods order",
                  depth_definition="Optical Z; PNG uint16 millimeters, 0 invalid; NPY geometric float32 meters before range masking")
    (folder/"metadata.json").write_text(json.dumps(record,indent=2,ensure_ascii=False),encoding="utf-8")
    return record
