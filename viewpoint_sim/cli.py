import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from .config import load_config


FRAME_DT=0.025   # ~40 fps for the kinematic preview


def replay(result,gui=True,gif=None,physics=False,speed=1.0):
    from .world import World
    from .motion import interpolate,path_free
    from .execution import edge_duration
    root=Path(result);cfg=load_config(root/'config.json')
    motion=cfg['motion']
    views=json.loads((root/'views.json').read_text(encoding='utf-8'))
    if not views:raise ValueError('No accepted views to replay')
    frames=[]
    with World(cfg,gui=gui,speed=speed) as world:
        for view in views:
            path=view['path_q_rad']
            if not path_free(world,path):raise RuntimeError('Replay path failed collision revalidation')
            world.set_q(path[0])
            if physics:
                world.execute(path)
            else:
                # Kinematic preview: the SAME schedule as the servo replay, but
                # qpos is written directly so no dynamics are integrated. Sampling
                # uses the planning resolution, so the arm travels the path it
                # actually recorded; only the drawing is thinned to hold ~40 fps.
                for a,b in zip(path,path[1:]):
                    a,b=np.array(a),np.array(b)
                    if gif:
                        samples=interpolate(a,b,.16)   # a GIF needs every frame
                        every,delay=1,0.0
                    else:
                        duration=edge_duration(float(np.max(np.abs(b-a))),motion)/max(speed,.05)
                        want=max(2,int(np.ceil(duration/FRAME_DT))+1)
                        samples=interpolate(a,b,motion['edge_resolution_rad'])
                        if want>=len(samples):
                            # Slow playback: resample FINER so the arm still moves
                            # smoothly, instead of stretching a few poses apart.
                            samples=np.linspace(a,b,want);every=1
                        else:
                            every=max(1,int(round(len(samples)/want)))
                        delay=duration/max(1,int(np.ceil(len(samples)/every)))
                    for i,q in enumerate(samples):
                        world.set_q(q)
                        if (gui or gif) and i%every==0:
                            if gui:world.viewer.sync();time.sleep(delay)
                            if gif:
                                scene=Image.fromarray(world.overview()).resize((660,440))
                                rgb=Image.fromarray(world.render(world.camera_pose())[0]).resize((320,180))
                                canvas=Image.new('RGB',(1000,440),'#0e1825');canvas.paste(scene,(0,0));canvas.paste(rgb,(670,125))
                                d=ImageDraw.Draw(canvas);d.text((680,24),'MUJOCO / myCobot Pro 630',fill='#72dfbe')
                                d.text((680,60),f"Height: {view['height_above_tray_m']*1000:.0f} mm",fill='white')
                                d.text((680,86),view['id'],fill='white');d.text((680,340),'KINEMATIC PATH PREVIEW',fill='#a0b3c5')
                                frames.append(canvas)
            if gif and frames:frames.extend([frames[-1].copy()]*2)
        if gif and frames:frames[0].save(gif,save_all=True,append_images=frames[1:],duration=120,loop=0,optimize=True)
        if gui:
            print('Replay finished. Close the MuJoCo window or press Ctrl+C.')
            try:
                while world.viewer.is_running():world.viewer.sync();time.sleep(.03)
            except KeyboardInterrupt:pass


def main(argv=None):
    p=argparse.ArgumentParser(description='MuJoCo + myCobot Pro 630 constrained viewpoint simulation')
    sub=p.add_subparsers(dest='command',required=True)
    run=sub.add_parser('run',help='Generate and capture new views')
    run.add_argument('--config');run.add_argument('--output',default='outputs/my_run')
    mode=run.add_mutually_exclusive_group();mode.add_argument('--gui',action='store_true');mode.add_argument('--headless',action='store_true')
    run.add_argument('--speed',type=float,default=4.0,help='GUI playback speed multiplier; 1.0 = realtime. Never affects recorded data')
    rep=sub.add_parser('replay',help='Replay saved joint paths')
    rep.add_argument('--result',default='outputs/demo');rep.add_argument('--headless',action='store_true')
    rep.add_argument('--gif');rep.add_argument('--physics',action='store_true',help='Execute with MuJoCo position servos')
    rep.add_argument('--speed',type=float,default=4.0,help='GUI playback speed multiplier; 1.0 = realtime. Never affects recorded data')
    check=sub.add_parser('check',help='Validate model, calibration settings, and initial collision')
    check.add_argument('--config')
    export=sub.add_parser('export-model',help='Export standalone MJCF for the MuJoCo viewer')
    export.add_argument('--config');export.add_argument('--output',default='models/default_scene.xml')
    view=sub.add_parser('view',help='Open interactive MuJoCo model with actuator controls')
    view.add_argument('--config')
    a=p.parse_args(argv)
    try:
        if a.command=='replay':replay(a.result,not a.headless,a.gif,a.physics,a.speed);return 0
        from .world import World
        cfg=load_config(getattr(a,'config',None))
        with World(cfg,gui=getattr(a,'gui',False) or a.command=='view',
                   speed=getattr(a,'speed',4.0)) as world:
            if a.command=='check':
                reason=world.collision(world.q)
                print(json.dumps(dict(engine='mujoco',robot=cfg['robot']['model'],home_collision=reason,
                                      joints=world.kin.names,crop_mode=cfg['postprocess']['crop_mode'],z_jitter_m=cfg['sampling']['z_jitter_m']),indent=2))
                return 2 if reason else 0
            if a.command=='export-model':world.export_scene(a.output);print(Path(a.output).resolve());return 0
            if a.command=='view':
                import mujoco
                while world.viewer.is_running():
                    tic=time.monotonic();mujoco.mj_step(world.model,world.data);world.viewer.sync()
                    time.sleep(max(0,world.model.opt.timestep-(time.monotonic()-tic)))
                return 0
            from .planner import run
            summary=run(world,a.output);print(json.dumps(summary,indent=2,ensure_ascii=False))
            print('Open',Path(a.output).resolve()/'report.html')
            return 0 if summary['complete'] else 2
    except KeyboardInterrupt:return 130
    except (ValueError,FileExistsError,RuntimeError) as error:
        print('Error:',error,file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
