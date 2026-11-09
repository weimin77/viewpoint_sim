import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image, ImageDraw
from .config import load_config


def replay(result, gui=True, gif=None):
    from contextlib import ExitStack
    from .world import World
    from .motion import interpolate, path_free
    result = Path(result)
    cfg = load_config(result/"config.json")
    views = json.loads((result/"views.json").read_text(encoding="utf-8"))
    if not views:
        raise ValueError("There are no accepted views to replay")
    motion = cfg["motion"]
    # Playback is a constant joint-space speed, independent of the recorded path
    # sampling, so every edge takes time proportional to the distance it covers.
    speed = float(motion.get("replay_speed_rad_per_s", 1.2))
    pause = float(motion.get("replay_pause_seconds", .5))
    resolution = float(motion.get("edge_resolution_rad", .025))
    if speed <= 0 or resolution <= 0 or pause < 0:
        raise ValueError("Invalid replay timing settings")

    frames, durations, interval = [], [], .05
    with ExitStack() as stack:
        # Revalidation needs joint states, and driving the visible robot through a
        # whole trajectory first would teleport it before the animation starts.
        shadow = stack.enter_context(World(cfg, gui=False))
        for view in views:
            if not path_free(shadow, np.asarray(view["path_q_rad"], float)):
                raise RuntimeError("Saved replay path failed collision revalidation")
        world = shadow if not gui else stack.enter_context(World(cfg, gui=True))
        world.set_q(np.array(cfg["robot"]["home_q_rad"], float))
        clock, next_frame, started = 0., 0., time.monotonic()

        def frame(view, captured=False, duration=None):
            canvas=Image.new("RGB",(1000,440),"#101b28")
            scene=Image.fromarray(world.overview()).resize((660,440))
            rgb=Image.fromarray(world.render(world.camera_pose())[0]).resize((320,180))
            canvas.paste(scene,(0,0));canvas.paste(rgb,(670,125))
            draw=ImageDraw.Draw(canvas)
            draw.text((680,30),"MYCOBOT 280 M5 / VIEWPOINT SIM",fill="#72dfbe")
            draw.text((680,62),f"Target height: {view['height_above_tray_m']*1000:.0f} mm",fill="white")
            draw.text((680,86),view["id"],fill="white")
            draw.text((680,340),"KINEMATIC / IDEAL RGB-D",fill="#a0b3c5")
            if captured:
                draw.text((680,286),"CAPTURED",fill="#71dfbf")
            frames.append(canvas)
            durations.append(int(round(duration if duration else interval*1000)))

        def wait():
            if gui or gif:
                delay = clock - (time.monotonic()-started)
                if delay > 0:
                    time.sleep(delay)

        def advance(q, view):
            """Pose the robot at q and keep playback on the shared simulated clock."""
            nonlocal next_frame
            world.set_q(q)
            if gif:
                # Fixed-rate sampling keeps GIF speed independent of step count.
                while next_frame <= clock + 1e-9:
                    frame(view)
                    next_frame += interval
            wait()

        def hold(view):
            """Stop at the capture pose; one long frame keeps the pause visible."""
            nonlocal clock, next_frame
            world.set_q(np.asarray(view["path_q_rad"], float)[-1])
            if pause <= 0:
                return
            clock += pause
            if gif:
                frame(view, captured=True, duration=pause*1000)
                next_frame = clock + interval
            wait()

        for view in views:
            path = np.asarray(view["path_q_rad"], float)
            for a, b in zip(path, path[1:]):
                points = interpolate(a, b, resolution)
                per_step = float(np.max(np.abs(b-a))) / max(1, len(points)-1)
                for q in points[1:]:
                    clock += per_step/speed
                    advance(q, view)
            world.draw_view(np.array(view["T_BC_actual"]))
            hold(view)
        if gif and frames:
            frames[0].save(gif,save_all=True,append_images=frames[1:],
                           duration=durations,loop=0,optimize=True)
        if gui:
            print("Replay complete. Close the PyBullet window or press Ctrl+C to finish.")
            import pybullet as p
            try:
                while p.isConnected(world.cid):
                    time.sleep(.1)
            except KeyboardInterrupt:
                pass


def main(argv=None):
    parser=argparse.ArgumentParser(description="myCobot 280 constrained random viewpoint simulation")
    sub=parser.add_subparsers(dest="command",required=True)
    run=sub.add_parser("run",help="Sample, validate, execute kinematically, and export RGB-D")
    run.add_argument("--config",default=None)
    run.add_argument("--output",default="outputs/demo")
    run.add_argument("--gui",action="store_true",help="Open PyBullet 3D window")
    run.add_argument("--headless",action="store_true",help="Explicit CPU-only DIRECT mode (default)")
    rep=sub.add_parser("replay",help="Replay saved validated trajectories")
    rep.add_argument("--result",default="outputs/demo")
    rep.add_argument("--headless",action="store_true")
    rep.add_argument("--gif",help="Also save a third-person/camera preview GIF")
    check=sub.add_parser("check",help="Validate configuration and initial collisions")
    check.add_argument("--config",default=None)
    args=parser.parse_args(argv)
    try:
        if args.command=="replay":
            replay(args.result,not args.headless,args.gif)
            return 0
        from .world import World
        cfg=load_config(args.config)
        with World(cfg,gui=getattr(args,"gui",False) and not getattr(args,"headless",False)) as world:
            if args.command=="check":
                reason=world.collision(np.array(cfg["robot"]["home_q_rad"]))
                print(json.dumps({"configuration_valid":True,"home_collision":reason,"joint_names":world.kin.names},indent=2))
                return 2 if reason else 0
            from .planner import run as simulate
            summary=simulate(world,args.output)
            print(json.dumps(summary,indent=2,ensure_ascii=False))
            print(f"Open {Path(args.output).resolve()/'report.html'} to inspect results.")
            return 0 if summary["complete"] else 2
    except (ValueError,FileExistsError,RuntimeError) as e:
        print(f"Error: {e}",file=sys.stderr)
        return 1


if __name__=="__main__":
    sys.exit(main())
