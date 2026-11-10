from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import json
import time
import numpy as np
import pybullet as p
from PIL import Image
from scipy.spatial.transform import Rotation
from .acquisition import inspect_capture, save_capture
from .geometry import box_vertices, look_at, inv, project, image_contains, tray_ratio, angle_deg
from .motion import plan, interpolate


def geometry_check(cfg, T_BC, height_index):
    roi, cam, sampling = cfg["roi"], cfg["camera"], cfg["sampling"]
    pad = roi["margin_m"]
    food = box_vertices(np.array(roi["food_min_B"])-pad, np.array(roi["food_max_B"])+pad)
    uv,z = project(food,T_BC,cam["rgb"])
    margin = np.array(sampling["image_margin_fraction"])*[cam["rgb"]["width"],cam["rgb"]["height"]]
    if not image_contains(uv,z,cam["rgb"],margin):
        return None,"food_out_of_rgb_fov"
    duv,dz = project(food,T_BC@np.array(cam["T_CD"]),cam["depth"])
    if not image_contains(duv,dz,cam["depth"],margin):
        return None,"food_out_of_depth_fov"
    lo,hi = cam["depth_range_m"]
    if dz.min() < lo or dz.max() > hi:
        return None,"depth_range"
    tilt = angle_deg(T_BC[:3,2],[0,0,-1])
    if tilt > sampling["max_tilt_deg"]:
        return None,"tilt"
    tuv,tz = project(np.array(roi["tray_corners_B"]),T_BC,cam["rgb"])
    if np.any(tz <= 0):
        return None,"tray_behind_camera"
    ratio = tray_ratio(tuv,cam["rgb"])
    thresholds = sampling["tray_min_ratio_by_height"]
    if len(thresholds) != len(sampling["heights_above_tray_m"]):
        raise ValueError("tray_min_ratio_by_height must have one entry per configured height")
    if ratio < thresholds[height_index]:
        return None,"tray_ratio"
    return dict(food_uv=uv.tolist(),tray_uv=tuv.tolist(),tray_ratio_raw=ratio,
                roi_depth_min_m=float(dz.min()),roi_depth_max_m=float(dz.max()),
                tilt_deg=tilt,margin_px=margin.tolist()),None


def sample_camera_position(cfg, height, rng):
    """Draw one candidate optical-centre position for a configured base height.

    ``sampling.z_perturbation_m`` (call it amplitude) enables the optional Z
    spread: the added offset is delta ~ U(-amplitude, +amplitude), drawn per
    candidate. With the default amplitude of 0 the RNG stream is left untouched,
    so existing seeds reproduce the pre-perturbation results exactly.
    """
    s = cfg["sampling"]
    x, y = rng.uniform(*s["x_B_m"]), rng.uniform(*s["y_B_m"])
    amplitude = float(s.get("z_perturbation_m", 0.))
    delta = rng.uniform(-amplitude, amplitude) if amplitude > 0 else 0.
    return [x, y, cfg["roi"]["tray_z_B_m"] + height + delta], delta


def execute_path(world, path):
    """Kinematic stop-and-capture simulation, independent of real CAN hardware."""
    for a,b in zip(path,path[1:]):
        for q in interpolate(np.array(a),np.array(b),world.cfg["motion"]["edge_resolution_rad"]):
            reason = world.collision(q)
            if reason:
                raise RuntimeError("Execution aborted: "+reason)
            if world.gui:
                time.sleep(world.cfg["motion"]["replay_seconds_per_edge"])
    world.set_q(path[-1])


def run(world, output):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output}. Choose a new --output.")
    output.mkdir(parents=True,exist_ok=True)
    cfg, s = world.cfg,world.cfg["sampling"]
    (output/"config.json").write_text(json.dumps(cfg,indent=2,ensure_ascii=False),encoding="utf-8")
    # Independent streams keep XY draws unchanged if IK or RRT consumes more randomness.
    seeds = np.random.SeedSequence(cfg["seed"]).spawn(3)
    sample_rng, ik_rng, path_rng = [np.random.default_rng(k) for k in seeds]
    start_time = time.monotonic()
    current = np.array(cfg["robot"]["home_q_rad"],dtype=float)
    initial_collision = world.collision(current)
    if initial_collision:
        raise ValueError("Home configuration is invalid: "+initial_collision)
    accepted, all_axes, heights = [], [], []
    center = (np.array(cfg["roi"]["food_min_B"])+cfg["roi"]["food_max_B"])/2
    with (output/"candidates.jsonl").open("w",encoding="utf-8") as log:
        for height_index,h in enumerate(s["heights_above_tray_m"]):
            failures, count, attempts, axes, deltas = Counter(),0,0,[],[]
            while count < s["target_per_height"] and attempts < s["max_attempts_per_height"]:
                attempts += 1
                candidate_id = f"h{height_index:02d}_c{attempts:04d}"
                eye,delta = sample_camera_position(cfg,h,sample_rng)
                target = look_at(eye,center,s["reference_up_B"])
                state = dict(id=candidate_id,height_above_tray_m=h,z_perturbation_m=delta,eye_B_m=eye)
                geometry, reason = geometry_check(cfg,target,height_index)
                capture, solution, path, ik_quality = None,None,None,None
                if reason is None:
                    # Early duplicate rejection has the same set semantics, avoids costly IK/rendering.
                    compare = all_axes if s["deduplicate_across_heights"] else axes
                    if any(angle_deg(target[:3,2],d) < s["min_axis_angle_deg"] for d in compare):
                        reason = "duplicate_axis"
                if reason is None:
                    solutions = world.kin.solve(target@inv(world.mount),current,ik_rng,cfg["robot"])
                    reason = "ik_or_singularity"
                    for q, info in solutions:
                        collision = world.collision(q)
                        if collision:
                            reason = collision.split(":")[0]
                            state["collision_detail"] = collision
                            continue
                        candidate_path = plan(world,current,q,path_rng)
                        if candidate_path is None:
                            reason = "path_not_found"
                            continue
                        world.set_q(q)
                        actual = world.camera_pose()
                        actual_geometry,geo_error = geometry_check(cfg,actual,height_index)
                        if geo_error:
                            reason = "actual_"+geo_error
                            continue
                        if any(angle_deg(actual[:3,2],d) < s["min_axis_angle_deg"] for d in compare):
                            reason = "duplicate_axis"
                            continue
                        cap = inspect_capture(world,actual)
                        if cap["failures"]:
                            reason = cap["failures"][0]
                            state["capture_failures"] = cap["failures"]
                            state["quality"] = cap["metrics"]
                            continue
                        reason = None
                        capture, solution, path, ik_quality = cap,q,candidate_path,info
                        geometry = actual_geometry
                        break
                if reason is not None:
                    failures[reason] += 1
                    state.update(accepted=False,reason=reason)
                    world.set_q(current)
                else:
                    # Execute from the last ACCEPTED pose, including across height changes.
                    world.set_q(current)
                    execute_path(world,path)
                    actual = world.camera_pose()
                    world.draw_view(actual)
                    # Deterministic renderer at the arrived pose must agree with previewed quality.
                    pe = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
                    re = float(np.degrees(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()))
                    if pe > cfg["robot"]["ik_position_tolerance_m"] or re > cfg["robot"]["ik_rotation_tolerance_deg"]:
                        raise RuntimeError("Arrived camera pose exceeds tolerance")
                    record = dict(id=candidate_id,height_index=height_index,height_above_tray_m=h,
                                  z_perturbation_m=delta,
                                  camera_height_above_tray_m=float(actual[2,3]-cfg["roi"]["tray_z_B_m"]),
                                  capture_mode="kinematic_ideal_rgbd",seed=cfg["seed"],
                                  T_BC_target=target.tolist(),T_BC_actual=actual.tolist(),
                                  T_BF_target=(target@inv(world.mount)).tolist(),T_BF_actual=world.flange_pose().tolist(),
                                  T_FC=world.mount.tolist(),q_actual_rad=solution.tolist(),path_q_rad=path,
                                  camera_position_error_m=pe,camera_rotation_error_deg=re,
                                  ik=ik_quality,
                                  stages=["SAMPLE","GEOMETRY","IK","COLLISION","PATH","ARRIVE","CAPTURE","QUALITY","SAVE"])
                    folder = output/candidate_id
                    record = save_capture(folder,capture,record,geometry,cfg)
                    Image.fromarray(world.overview()).save(folder/"overview.png")
                    accepted.append(record)
                    count += 1
                    current = solution.copy()
                    axes.append(actual[:3,2].copy())
                    all_axes.append(actual[:3,2].copy())
                    deltas.append(delta)
                    state.update(accepted=True,reason="accepted",folder=candidate_id,tray_ratio=geometry["tray_ratio_raw"])
                    print(f"Height {h:.3f} m: accepted {count}/{s['target_per_height']} after {attempts} attempts",flush=True)
                log.write(json.dumps(state,ensure_ascii=False)+"\n")
                log.flush()
            heights.append(dict(height_above_tray_m=h,accepted=count,target=s["target_per_height"],
                                attempts=attempts,rejections=dict(failures),
                                z_perturbations_m=deltas,
                                complete=count==s["target_per_height"],acceptance_rate=count/max(1,attempts)))
            print(f"Height {h:.3f} m finished: {count}/{s['target_per_height']}; rejected {dict(failures)}",flush=True)
    pairs = [angle_deg(a,b) for i,a in enumerate(all_axes) for b in all_axes[i+1:]]
    summary = dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),
                   seed=cfg["seed"],complete=all(x["complete"] for x in heights),
                   accepted_total=len(accepted),heights=heights,
                   minimum_axis_angle_deg=min(pairs) if pairs else None,
                   z_perturbation_m=float(s.get("z_perturbation_m", 0.)),
                   elapsed_seconds=time.monotonic()-start_time,
                   simulation_limits=["Kinematic motion; no dynamics/controller settling",
                                      "Ideal aligned undistorted RGB-D; no stereo matching/noise/material physics",
                                      "Discrete collision checking; not continuous swept-volume certification",
                                      "Example workcell and camera extrinsics; replace with measured values"])
    summary['view_ids_ranked_by_tray_ratio'] = {
        str(h): [v['id'] for v in sorted((v for v in accepted if v['height_above_tray_m']==h),
                                        key=lambda v:v['geometry']['tray_ratio_raw'],reverse=True)]
        for h in s['heights_above_tray_m']}
    from .comparison import compare_heights
    compare_heights(cfg,accepted,output)
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"views.json").write_text(json.dumps(accepted,indent=2,ensure_ascii=False),encoding="utf-8")
    from .report import make_report
    make_report(output,cfg,accepted,summary)
    return summary
