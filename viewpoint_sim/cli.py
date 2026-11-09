import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image, ImageDraw
from .config import load_config


def replay(result, gui=True, gif=None):
    from .world import World
    from .motion import interpolate, path_free
    result = Path(result)
    cfg = load_config(result/"config.json")
    views = json.loads((result/"views.json").read_text(encoding="utf-8"))
    if not views:
        raise ValueError("There are no accepted views to replay")
    frames = []
    with World(cfg,gui=gui) as world:
        for view in views:
            path = view["path_q_rad"]
            world.draw_view(np.array(view['T_BC_actual']))
            if not path_free(world,path):
                raise RuntimeError("Saved replay path failed collision revalidation")
            for a,b in zip(path,path[1:]):
                for q in interpolate(np.array(a),np.array(b),.075):
                    world.set_q(q)
                    if gui:
                        time.sleep(.03)
                    if gif:
                        canvas=Image.new("RGB",(1000,440),"#101b28")
                        scene=Image.fromarray(world.overview()).resize((660,440))
                        rgb=Image.fromarray(world.render(world.camera_pose())[0]).resize((320,180))
                        canvas.paste(scene,(0,0));canvas.paste(rgb,(670,125))
                        draw=ImageDraw.Draw(canvas)
                        draw.text((680,30),"MYCOBOT 280 M5 / VIEWPOINT SIM",fill="#72dfbe")
                        draw.text((680,62),f"Target height: {view['height_above_tray_m']*1000:.0f} mm",fill="white")
                        draw.text((680,86),view["id"],fill="white")
                        draw.text((680,340),"KINEMATIC / IDEAL RGB-D",fill="#a0b3c5")
                        frames.append(canvas)
        if gif and frames:
            frames[0].save(gif,save_all=True,append_images=frames[1:],duration=100,loop=0,optimize=True)
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
