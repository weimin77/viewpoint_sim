"""MuJoCo + myCobot Pro 630 eye-in-hand acquisition workcell."""
import os,sys
if sys.platform.startswith('linux') and not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
    os.environ.setdefault('MUJOCO_GL','egl')
__version__ = "0.2.0"
