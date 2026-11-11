"""Synthetic aligned RGB-D, visible-instance labels, and per-object visibility QA."""
from pathlib import Path
import csv
import json
import os
import shutil
import numpy as np
from PIL import Image, ImageDraw
from .geometry import crop_box, cropped_intrinsics, tray_ratio
from .config import crop_settings

IMAGE_KINDS = ('rgb', 'depth_mm', 'instance', 'food_mask', 'overlay', 'overview')
MANIFEST_FIELDS = ('id', 'height_index', 'height_above_tray_m',
                   'z_perturbation_m', 'camera_height_above_tray_m')


def collect_images(result, output, records=None, kinds=('rgb',), mode='copy',
                   force=False):
    """Copy the per-view images of one run into a single flat folder.

    Every accepted view keeps its images in ``hXX_cXXXX/``, which is convenient
    for inspection but awkward for downstream training code that just wants all
    RGB frames. This flattens the requested kinds into ``output`` with an
    acquisition-ordered prefix and writes ``manifest.csv`` so each file can be
    traced back to its view and height. Source files are only read.
    """
    result, output = Path(result), Path(output)
    if records is None:
        views_path = result/'views.json'
        if not views_path.is_file():
            raise FileNotFoundError(f"Not a result directory (no views.json): {result}")
        records = json.loads(views_path.read_text(encoding='utf-8'))
    records = list(records)
    if not records:
        raise ValueError("The result contains no accepted views")
    if output.exists() and any(output.iterdir()) and not force:
        raise FileExistsError(f"Output directory is not empty: {output}. Use --force to overwrite.")
    rows, missing = [], []
    for index, view in enumerate(records):
        folder = result/view['id']
        # Zero padding keeps the alphabetical order equal to the acquisition order.
        stem = f"{index:03d}_{view['id']}"
        for kind in kinds:
            source = folder/(kind+'.png')
            if not source.is_file():
                missing.append(str(source))
                continue
            target = output/(stem+('' if kind == 'rgb' else '_'+kind)+'.png')
            target.parent.mkdir(parents=True, exist_ok=True)
            if mode == 'link':
                if target.exists() or target.is_symlink():
                    target.unlink()
                os.symlink(os.path.relpath(source.resolve(), target.parent.resolve()), target)
            else:
                shutil.copy2(source, target)
            if kind == 'rgb':
                row = {'file': target.name}
                row.update({k: view.get(k, '') for k in MANIFEST_FIELDS})
                row['folder'] = view['id']
                rows.append(row)
    if missing:
        raise FileNotFoundError("Missing image files:\n  "+"\n  ".join(missing[:10]))
    manifest = output/'manifest.csv'
    if rows:
        with manifest.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return rows, manifest


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
    camera = cfg["camera"]["rgb"]
    crop_enabled, padding = crop_settings(cfg)
    # Projected Food+Tray box, recorded for downstream use but never applied to
    # the full-frame exports. Cropping is an opt-in extra output.
    roi_rect = crop_box(food_uv, tray_uv, camera, 0)
    for prefix, arrays in [("",(rgb,depth,labels))]:
        for name, arr in zip(("rgb","depth_mm","instance"),arrays):
            Image.fromarray(arr).save(folder/(prefix+name+".png"))
        Image.fromarray((arrays[2]>0).astype(np.uint8)*255).save(folder/(prefix+"food_mask.png"))
    rect = None
    if crop_enabled:
        rect = crop_box(food_uv, tray_uv, camera, padding)
        x0,y0,x1,y1 = rect
        if int(np.count_nonzero(labels[y0:y1,x0:x1])) != int(np.count_nonzero(labels)):
            raise RuntimeError("Crop would remove visible food pixels")
        for name, arr in zip(("rgb","depth_mm","instance"),(rgb,depth,labels)):
            Image.fromarray(arr[y0:y1,x0:x1]).save(folder/("crop_"+name+".png"))
        Image.fromarray((labels[y0:y1,x0:x1]>0).astype(np.uint8)*255).save(folder/"crop_food_mask.png")
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
    if rect:
        d.rectangle([rect[0],rect[1],rect[2]-1,rect[3]-1],outline=(80,170,255),width=2)
    overlay.save(folder/"overlay.png")
    record.update(geometry=geometry,quality=capture["metrics"],
                  image_size_xy=[camera["width"],camera["height"]],
                  roi_bbox_xyxy=roi_rect,
                  crop_enabled=crop_enabled,
                  label_definition="Visible surfaces; 0=background, IDs 1..N in config scene.foods order",
                  depth_definition="Optical Z; PNG uint16 millimeters, 0 invalid; NPY geometric float32 meters before range masking")
    if rect:
        record.update(crop_bbox_xyxy=rect, crop_bbox_xywh=[rect[0],rect[1],rect[2]-rect[0],rect[3]-rect[1]],
                      K_crop=cropped_intrinsics(camera,rect),
                      tray_ratio_cropped=tray_ratio(tray_uv,camera,rect))
    (folder/"metadata.json").write_text(json.dumps(record,indent=2,ensure_ascii=False),encoding="utf-8")
    return record
