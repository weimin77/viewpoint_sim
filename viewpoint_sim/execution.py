"""Joint-space execution: kinematic stepping or MuJoCo position servos.

The two loops differ in what they claim. ``kinematic`` walks a densely
interpolated joint path and rejects a collision at each sample - it visualises a
path and proves it stays clear, but says nothing about dynamics. ``Servo``
drives ``data.ctrl`` through ``mj_step`` and only reports arrival once the arm
has actually settled, so ``arrived_joint_error_rad`` is measured, not assumed.

Both need the same three things from the workcell - write a joint configuration,
read the current one, check the current state for collisions - and those arrive
as callables. Keeping the collision callback zero-argument in *both* modes is
deliberate: each loop moves the model to the state it cares about and then asks
about that state, so neither can check something other than what it evaluates.
"""
import numpy as np
import mujoco
from .motion import interpolate


def stats_template(mode):
    """The execution record embedded in every accepted view."""
    return {'mode': mode, 'physics_steps': 0, 'max_joint_speed_rad_s': 0.0}


def _cadence(value):
    """``sync_every`` may be a constant or a callable, so speed can change live."""
    return int(round(value() if callable(value) else value))


def edge_duration(delta, motion):
    """Simulated seconds a joint-space edge of span ``delta`` should take.

    Minimum-jerk needs this long to stay under both the speed and the
    acceleration limit; ``dt`` is the floor, never a zero-length edge. Both the
    servo follower and the kinematic preview use it, so the two replays show the
    same timing and ``--speed`` means the same thing in both.
    """
    dt = motion['timestep_s']
    return max(1.5 * delta / motion['max_joint_speed_rad_s'],
               np.sqrt(6 * delta / motion['max_joint_accel_rad_s2']),
               dt)


def kinematic(path, motion, *, set_q, collide, on_frame=None, sync_every=1):
    """Step through interpolated joint states, aborting on any collision.

    ``on_frame`` is called every ``sync_every``-th interpolated state so a caller
    can draw, and it is the only place wall-clock pacing belongs - this module
    never sleeps. Skipped frames still run ``set_q`` and ``collide``: only the
    drawing is strided, never the validation.
    """
    stats = stats_template(motion['execution_mode'])
    resolution = motion['edge_resolution_rad']
    frame = 0
    for start, end in zip(path, path[1:]):
        for q in interpolate(np.array(start), np.array(end), resolution):
            set_q(q)
            reason = collide()
            if reason:
                raise RuntimeError('Execution aborted: ' + reason)
            if on_frame is not None and frame % max(1, _cadence(sync_every)) == 0:
                on_frame()
            frame += 1
    set_q(path[-1])
    return stats


class Servo:
    """Position-servo follower with an explicit settling criterion.

    ``mj_step`` integrates qpos after evaluating kinematics, so the derived
    poses lag one tick behind. Every step therefore runs ``mj_forward`` before
    the collision query or a recorded frame; skipping it makes the check
    describe the previous state while claiming to describe the current one.
    """

    SYNC_EVERY = 12

    def __init__(self, model, data, dof_addresses, motion, *,
                 current_q, collide, on_frame=None, sync_every=SYNC_EVERY):
        self.model = model
        self.data = data
        self.dof_addresses = dof_addresses
        self.motion = motion
        self.current_q = current_q
        self.collide = collide
        self.on_frame = on_frame
        self.sync_every = sync_every
        self.stats = stats_template(motion['execution_mode'])

    def _joint_speed(self):
        return float(np.max(np.abs(self.data.qvel[self.dof_addresses])))

    def step(self, target):
        """One control tick, then a fresh collision query on the new state."""
        self.data.ctrl[:] = target
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.stats['physics_steps'] += 1
        self.stats['max_joint_speed_rad_s'] = max(
            self.stats['max_joint_speed_rad_s'], self._joint_speed())
        reason = self.collide()
        if reason:
            raise RuntimeError('Physics execution aborted: ' + reason)
        if (self.on_frame is not None
                and self.stats['physics_steps'] % max(1, _cadence(self.sync_every)) == 0):
            self.on_frame()

    def follow(self, path):
        """Track the path edge by edge, then hold the goal until settled."""
        dt = self.model.opt.timestep
        for start, end in zip(path, path[1:]):
            start, end = np.array(start), np.array(end)
            duration = edge_duration(float(np.max(np.abs(end - start))), self.motion)
            for u in np.linspace(0, 1, max(2, int(np.ceil(duration / dt)) + 1)):
                self.step(start + (end - start) * (3 * u * u - 2 * u * u * u))
        return self.settle(np.array(path[-1]))

    def settle(self, goal):
        """Hold ``goal`` until error and speed stay below threshold, or fail.

        The hold window is what makes this a measurement: a single quiet tick
        would also occur mid-trajectory while passing through the goal.
        """
        dt = self.model.opt.timestep
        motion = self.motion
        required = max(1, int(motion['settle_hold_s'] / dt))
        stable = 0
        for _ in range(int(motion['settle_timeout_s'] / dt)):
            self.step(goal)
            error = float(np.max(np.abs(self.current_q() - goal)))
            speed = self._joint_speed()
            if (error < motion['settle_joint_error_rad']
                    and speed < motion['settle_velocity_rad_s']):
                stable += 1
            else:
                stable = 0
            if stable >= required:
                self.stats['arrived_joint_error_rad'] = error
                self.stats['arrived_joint_velocity_rad_s'] = speed
                return self.stats
        raise RuntimeError(
            'Position servo did not reach the configured settling criterion')


def execute(model, data, motion, path, *, set_q, current_q, dof_addresses,
            collide, on_frame=None, sync_every=12):
    """Run ``path`` with the configured execution mode."""
    if motion['execution_mode'] == 'kinematic':
        return kinematic(path, motion, set_q=set_q, collide=collide,
                         on_frame=on_frame, sync_every=sync_every)
    return Servo(model, data, dof_addresses, motion, current_q=current_q,
                 collide=collide, on_frame=on_frame,
                 sync_every=sync_every).follow(path)
