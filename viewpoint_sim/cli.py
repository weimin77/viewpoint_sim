import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from .config import load_config
from .playback import FRAME_DT, Abort, Controls, Playback, Restart


def _running(world):
    return True if world.viewer is None else world.viewer.is_running()


def _kinematic_view(world,motion,path,state,edge_duration):
    """Walk one joint path in virtual time, so the slider acts immediately.

    Progress along an edge is advanced by ``FRAME_DT * speed / duration`` per
    drawn frame instead of by a fixed sample stride: changing the slider then
    changes how fast the arm covers the remaining distance, rather than
    re-timing a ladder that was fixed when the view started.

    Pause, restart and quit all arrive through ``state.step()`` so this mode
    cannot drift from the physics one.
    """
    for a,b in zip(path,path[1:]):
        a,b=np.array(a),np.array(b)
        duration=max(edge_duration(float(np.max(np.abs(b-a))),motion),1e-9)
        u=0.0
        while u<1.0:
            state.step()
            world.set_q(a+(b-a)*u)
            if world.viewer is not None:
                world.viewer.sync()
                time.sleep(FRAME_DT)
            u+=FRAME_DT*state.get_speed()/duration
    world.set_q(np.array(path[-1]))


def replay(result,gui=True,gif=None,physics=False,speed=1.0,controls=True):
    from .world import World
    from .motion import interpolate,path_free
    from .execution import edge_duration
    root=Path(result);cfg=load_config(root/'config.json')
    motion=cfg['motion']
    views=json.loads((root/'views.json').read_text(encoding='utf-8'))
    if not views:raise ValueError('No accepted views to replay')
    state=Playback(speed)
    panel=Controls(state) if (gui and controls and not gif) else None
    if panel is not None and not panel.ok:
        print('控制面板不可用（'+str(panel.error)+'），回放按 --speed 继续。',file=sys.stderr)
        panel=None
    frames=[]
    with World(cfg,gui=gui,speed=state.get_speed()) as world:
        world.speed_source=state.get_speed
        state.stop_check=lambda:_running(world)
        world.frame_hook=state.step if panel is not None else None
        try:
            while True:
                state.start(len(views))
                aborted=restarted=False
                for index,view in enumerate(views):
                    if not _running(world) or state.quit:aborted=True;break
                    path=view['path_q_rad']
                    if not path_free(world,path):raise RuntimeError('Replay path failed collision revalidation')
                    world.set_q(path[0])
                    state.at_view(index+1,view['id'])
                    try:
                        if physics:
                            world.execute(path)
                        elif gif:
                            for a,b in zip(path,path[1:]):
                                for q in interpolate(np.array(a),np.array(b),.16):
                                    world.set_q(q)
                                    scene=Image.fromarray(world.overview()).resize((660,440))
                                    rgb=Image.fromarray(world.render(world.camera_pose())[0]).resize((320,180))
                                    canvas=Image.new('RGB',(1000,440),'#0e1825');canvas.paste(scene,(0,0));canvas.paste(rgb,(670,125))
                                    d=ImageDraw.Draw(canvas);d.text((680,24),'MUJOCO / myCobot Pro 630',fill='#72dfbe')
                                    d.text((680,60),f"Height: {view['height_above_tray_m']*1000:.0f} mm",fill='white')
                                    d.text((680,86),view['id'],fill='white');d.text((680,340),'KINEMATIC PATH PREVIEW',fill='#a0b3c5')
                                    frames.append(canvas)
                            world.set_q(np.array(path[-1]))
                        else:
                            _kinematic_view(world,motion,path,state,edge_duration)
                    except Restart:
                        restarted=True;break
                    except Abort:
                        aborted=True;break
                    if panel is not None:
                        panel.poll()
                    if gif and frames:frames.extend([frames[-1].copy()]*2)
                if gif:
                    if frames:frames[0].save(gif,save_all=True,append_images=frames[1:],duration=120,loop=0,optimize=True)
                    break
                if restarted:
                    # Drop this pass and start over now, without waiting for it
                    # to finish: the button has to feel immediate.
                    state.take_reset()
                    continue
                if aborted or world.viewer is None:break
                # Playback is done; keep the window alive so 重放 still works.
                state.finish()
                again=False
                while _running(world):
                    if panel is not None:
                        panel.poll()
                    # Sync unconditionally: if the panel died (root gone) the
                    # window must keep repainting instead of looking hung.
                    world.viewer.sync()
                    if state.take_reset():again=True;break
                    if state.quit:break
                    time.sleep(.03)
                if not again:break
        finally:
            if panel is not None:panel.close()


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
    rep.add_argument('--no-controls',action='store_true',help='Do not open the speed/reset control window')
    check=sub.add_parser('check',help='Validate model, calibration settings, and initial collision')
    check.add_argument('--config')
    export=sub.add_parser('export-model',help='Export standalone MJCF for the MuJoCo viewer')
    export.add_argument('--config');export.add_argument('--output',default='models/default_scene.xml')
    view=sub.add_parser('view',help='Open interactive MuJoCo model with actuator controls')
    view.add_argument('--config')
    a=p.parse_args(argv)
    try:
        if a.command=='replay':replay(a.result,not a.headless,a.gif,a.physics,a.speed,not a.no_controls);return 0
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
