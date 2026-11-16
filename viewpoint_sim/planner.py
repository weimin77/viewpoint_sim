from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import json
import time
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from .acquisition import inspect_capture, save_capture
from .comparison import compare_heights
from .feasibility import check as feasibility_check
from .geometry import inv
from .diversity import duplicate_reason, min_pairwise_angle_deg
from .pose_generation import candidate_pose
from .report import make_report
from .motion import plan, interpolate


def execute_path(world, path):
    return world.execute(path)


def run(world, output):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output}. Choose a new --output.")
    output.mkdir(parents=True,exist_ok=True)
    cfg, s = world.cfg,world.cfg["sampling"]
    (output/"config.json").write_text(json.dumps(cfg,indent=2,ensure_ascii=False),encoding="utf-8")
    # Independent streams keep XY draws unchanged if IK or RRT consumes more randomness.
    seeds = np.random.SeedSequence(cfg["seed"]).spawn(4)
    sample_rng, ik_rng, path_rng, jitter_rng = [np.random.default_rng(k) for k in seeds]
    start_time = time.monotonic()
    current = np.array(cfg["robot"]["home_q_rad"],dtype=float)
    initial_collision = world.collision(current)
    if initial_collision:
        raise ValueError("Home configuration is invalid: "+initial_collision)
    accepted, all_axes, heights = [], [], []
    with (output/"candidates.jsonl").open("w",encoding="utf-8") as log:
        for height_index,h in enumerate(s["heights_above_tray_m"]):
            failures, count, attempts, axes = Counter(),0,0,[]
            while count < s["target_per_height"] and attempts < s["max_attempts_per_height"]:
                attempts += 1
                candidate_id = f"h{height_index:02d}_c{attempts:04d}"
                # Steps 1-2 live in pose_generation: planar sample + height +
                # jitter, then Look-at at the ROI centre. Same draw order as
                # before, so existing seeds still reproduce.
                eye, target, actual_height, jitter = candidate_pose(
                    cfg, height_index, sample_rng, jitter_rng)
                eye = eye.tolist()
                state = dict(id=candidate_id,base_height_above_tray_m=h,height_above_tray_m=actual_height,z_jitter_m=jitter,eye_B_m=eye)
                geometry, reason = feasibility_check(cfg,target,height_index)
                capture, solution, path, ik_quality = None,None,None,None
                if reason is None:
                    # Early duplicate rejection has the same set semantics, avoids costly IK/rendering.
                    reason = duplicate_reason(cfg, target[:3, 2], axes, all_axes)
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
                        actual_geometry,geo_error = feasibility_check(cfg,actual,height_index)
                        if geo_error:
                            reason = "actual_"+geo_error
                            continue
                        reason = duplicate_reason(cfg, actual[:3,2], axes, all_axes)
                        if reason is not None:
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
                    execution = execute_path(world,path)
                    actual = world.camera_pose()
                    world.draw_view(actual)
                    # Deterministic renderer at the arrived pose must agree with previewed quality.
                    pe = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
                    re = float(np.degrees(Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()))
                    if pe > cfg["robot"]["ik_position_tolerance_m"] or re > cfg["robot"]["ik_rotation_tolerance_deg"]:
                        raise RuntimeError("Arrived camera pose exceeds tolerance")
                    geometry, arrived_geometry_error = feasibility_check(cfg,actual,height_index)
                    capture = inspect_capture(world,actual)
                    if arrived_geometry_error or capture['failures']:
                        reason = 'arrived_'+(arrived_geometry_error or capture['failures'][0])
                        failures[reason] += 1
                        state.update(accepted=False,reason=reason,moved=True,q_start_rad=path[0],q_end_rad=world.q.tolist(),path_q_rad=path)
                        current = world.q.copy()
                        log.write(json.dumps(state,ensure_ascii=False)+'\n')
                        log.flush()
                        continue
                    record = dict(id=candidate_id,height_index=height_index,base_height_above_tray_m=h,height_above_tray_m=actual_height,z_jitter_m=jitter,
                                  capture_mode="mujoco_"+cfg['motion']['execution_mode']+"_ideal_rgbd",engine="mujoco",robot_model=cfg['robot']['model'],execution=execution,seed=cfg["seed"],
                                  T_BC_target=target.tolist(),T_BC_actual=actual.tolist(),
                                  T_BF_target=(target@inv(world.mount)).tolist(),T_BF_actual=world.flange_pose().tolist(),
                                  T_FC=world.mount.tolist(),q_actual_rad=world.q.tolist(),q_target_rad=solution.tolist(),path_q_rad=path,
                                  camera_position_error_m=pe,camera_rotation_error_deg=re,
                                  ik=ik_quality,
                                  stages=["SAMPLE","GEOMETRY","IK","COLLISION","PATH","ARRIVE","CAPTURE","QUALITY","SAVE"])
                    folder = output/candidate_id
                    record = save_capture(folder,capture,record,geometry,cfg)
                    Image.fromarray(world.overview()).save(folder/"overview.png")
                    accepted.append(record)
                    count += 1
                    current = world.q.copy()
                    axes.append(actual[:3,2].copy())
                    all_axes.append(actual[:3,2].copy())
                    state.update(accepted=True,reason="accepted",folder=candidate_id,tray_ratio=geometry["tray_ratio_raw"])
                    print(f"Height {h:.3f} m: accepted {count}/{s['target_per_height']} after {attempts} attempts",flush=True)
                log.write(json.dumps(state,ensure_ascii=False)+"\n")
                log.flush()
            heights.append(dict(height_above_tray_m=h,accepted=count,target=s["target_per_height"],
                                attempts=attempts,rejections=dict(failures),
                                complete=count==s["target_per_height"],acceptance_rate=count/max(1,attempts)))
            print(f"Height {h:.3f} m finished: {count}/{s['target_per_height']}; rejected {dict(failures)}",flush=True)
    summary = dict(schema_version=2,engine="mujoco",robot_model=cfg['robot']['model'],created_utc=datetime.now(timezone.utc).isoformat(),
                   seed=cfg["seed"],complete=all(x["complete"] for x in heights),
                   accepted_total=len(accepted),heights=heights,
                   minimum_axis_angle_deg=min_pairwise_angle_deg(all_axes),
                   elapsed_seconds=time.monotonic()-start_time,
                   simulation_limits=["Example inertias/servo gains, not calibrated hardware dynamics",
                                      "Ideal aligned undistorted RGB-D; no stereo matching/noise/material physics",
                                      "Discrete collision checking; not continuous swept-volume certification",
                                      "Example workcell and camera extrinsics; replace with measured values"])
    summary['view_ids_ranked_by_tray_ratio'] = {
        str(h): [v['id'] for v in sorted((v for v in accepted if v['base_height_above_tray_m']==h),
                                        key=lambda v:v['geometry']['tray_ratio_raw'],reverse=True)]
        for h in s['heights_above_tray_m']}
    compare_heights(cfg,accepted,output)
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"views.json").write_text(json.dumps(accepted,indent=2,ensure_ascii=False),encoding="utf-8")
    make_report(output,cfg,accepted,summary)
    return summary
