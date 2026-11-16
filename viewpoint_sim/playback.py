"""Interactive playback controls for the replay window.

MuJoCo's passive viewer has no widget API. ``mujoco.viewer.Handle`` offers only
``cam``/``opt``, scene injection (``user_scn``, ``set_figures``, ``set_images``)
and text overlays (``set_texts``) - there is no way to add a slider or a button,
short of writing a C viewer plugin. So the controls live in a small Tk window
that sits next to the MuJoCo one.

Both windows are driven from the SAME thread on purpose. Tk is created here and
pumped with ``root.update()`` from the replay loop instead of running
``mainloop()`` in a worker thread: GLFW and Tcl then never touch each other's
state, and the frame delay the loop already pays doubles as the UI refresh.

Everything degrades quietly. If Tk is missing or there is no display, ``Controls``
reports ``ok == False`` and replay falls back to ``--speed``.
"""
import time

FRAME_DT = 0.025          # ~40 fps, the preview's wall-clock frame budget
MIN_SPEED = 0.25
MAX_SPEED = 8.0

BACKDROP = '#141a22'
PANEL = '#1c2430'
TEXT = '#dfe7ef'
MUTED = '#8fa3b8'
ACCENT = '#72dfbe'


class Abort(Exception):
    """Raised out of a running playback loop when the user restarts or quits."""


class Playback:
    """The mutable state the control window writes and the loop reads.

    Kept free of Tk so the loop logic can be exercised without a display.
    """

    def __init__(self, speed=4.0):
        self.speed = self._clamp(speed)
        self.paused = False
        self.reset = False
        self.quit = False
        self.finished = False
        self.view = ''
        self.index = 0
        self.total = 0
        self.ui = None            # set to a Controls instance once one exists

    @staticmethod
    def _clamp(value):
        return min(MAX_SPEED, max(MIN_SPEED, float(value)))

    # -- read by the loop -------------------------------------------------
    def get_speed(self):
        return self._clamp(self.speed)

    def pump(self):
        """Refresh the control window without blocking the loop."""
        if self.ui is not None:
            self.ui.poll()

    def wait_while_paused(self):
        """Block the loop while paused, still responding to reset/quit."""
        while self.paused and not self.quit and not self.reset:
            self.pump()
            time.sleep(.02)

    def take_reset(self):
        if not self.reset:
            return False
        self.reset = False
        self.paused = False
        return True

    # -- written by the loop ----------------------------------------------
    def start(self, total):
        self.finished = False
        self.total = total
        self.index = 0
        self.view = ''

    def at_view(self, index, view_id):
        self.index = index
        self.view = view_id

    def finish(self):
        self.finished = True


class Controls:
    """A Tk window with a speed slider, a replay button and a pause toggle.

    ``ok`` is False when Tk or the display is unavailable; ``error`` then says
    why, so the caller can print it instead of failing silently.
    """

    def __init__(self, state, title='回放控制'):
        self.state = state
        self.root = None
        self.error = None
        self._labels = {}
        try:
            import tkinter as tk
        except Exception as error:                      # pragma: no cover
            self.error = str(error)
            return
        try:
            root = tk.Tk()
        except Exception as error:                      # pragma: no cover
            self.error = str(error)
            return
        try:
            self._build(tk, root, title)
        except Exception as error:                      # pragma: no cover
            self.error = str(error)
            try:
                root.destroy()
            except Exception:
                pass
            self.root = None
            return
        self.root = root
        state.ui = self

    # -- construction ------------------------------------------------------
    def _build(self, tk, root, title):
        root.title(title)
        root.configure(bg=BACKDROP)
        root.resizable(False, False)
        root.protocol('WM_DELETE_WINDOW', self._quit)

        wrap = tk.Frame(root, bg=BACKDROP, padx=16, pady=14)
        wrap.pack(fill='both', expand=True)

        tk.Label(wrap, text='回放控制', bg=BACKDROP, fg=ACCENT,
                 font=('TkDefaultFont', 11, 'bold')).pack(anchor='w')

        self._labels['view'] = tk.Label(wrap, text='准备中', bg=BACKDROP, fg=TEXT,
                                        font=('TkDefaultFont', 10))
        self._labels['view'].pack(anchor='w', pady=(10, 2))

        row = tk.Frame(wrap, bg=BACKDROP)
        row.pack(fill='x', pady=(10, 0))
        tk.Label(row, text='速度', bg=BACKDROP, fg=MUTED).pack(side='left')
        self._labels['speed'] = tk.Label(row, text='', bg=BACKDROP, fg=ACCENT,
                                         font=('TkDefaultFont', 10, 'bold'))
        self._labels['speed'].pack(side='right')

        self.scale = tk.Scale(
            wrap, from_=MIN_SPEED, to=MAX_SPEED, resolution=0.25,
            orient='horizontal', length=300, showvalue=False,
            bg=PANEL, fg=TEXT, troughcolor=BACKDROP, highlightthickness=0,
            activebackground=ACCENT, command=self._on_speed)
        self.scale.set(self.state.get_speed())
        self.scale.pack(fill='x', pady=(4, 0))

        hint = tk.Frame(wrap, bg=BACKDROP)
        hint.pack(fill='x')
        tk.Label(hint, text=f'{MIN_SPEED}×', bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='left')
        tk.Label(hint, text='1× 实时', bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='left', padx=(26, 0))
        tk.Label(hint, text=f'{MAX_SPEED}×', bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='right')

        buttons = tk.Frame(wrap, bg=BACKDROP)
        buttons.pack(fill='x', pady=(14, 0))
        self._button(tk, buttons, '重放 (R)', self._reset).pack(side='left')
        self._pause_button = self._button(tk, buttons, '暂停 (空格)', self._toggle)
        self._pause_button.pack(side='left', padx=6)
        self._button(tk, buttons, '退出 (Esc)', self._quit).pack(side='left')

        tk.Label(wrap, text='拖动滑块即时生效，只改播放速度，不改数据',
                 bg=BACKDROP, fg=MUTED, font=('TkDefaultFont', 8)
                 ).pack(anchor='w', pady=(12, 0))

        root.bind('<space>', lambda _event: self._toggle())
        root.bind('<r>', lambda _event: self._reset())
        root.bind('<R>', lambda _event: self._reset())
        root.bind('<Escape>', lambda _event: self._quit())

        self._refresh()

    def _button(self, tk, parent, text, command):
        return tk.Button(parent, text=text, command=command,
                         bg=PANEL, fg=TEXT, relief='flat', bd=0,
                         padx=12, pady=6, activebackground=ACCENT,
                         activeforeground=BACKDROP,
                         highlightthickness=0, font=('TkDefaultFont', 9))

    # -- callbacks ---------------------------------------------------------
    def _on_speed(self, value):
        try:
            self.state.speed = float(value)
        except (TypeError, ValueError):
            pass
        self._refresh()

    def _reset(self):
        self.state.reset = True
        self.state.paused = False
        self._refresh()

    def _toggle(self):
        self.state.paused = not self.state.paused
        self._refresh()

    def _quit(self):
        self.state.quit = True

    # -- pumping -----------------------------------------------------------
    @property
    def ok(self):
        return self.root is not None

    def poll(self):
        """Process pending Tk events. Never blocks, never raises."""
        if self.root is None:
            return
        try:
            self._refresh()
            self.root.update()
        except Exception:
            # The window went away underneath us; keep the replay running.
            self.root = None

    def step(self):
        """Frame hook: respond to pause, then let the replay continue."""
        self.poll()
        if self.state.quit:
            raise Abort()
        if self.state.paused:
            self.state.wait_while_paused()

    def _refresh(self):
        if self.root is None:
            return
        speed = self.state.get_speed()
        self._labels['speed'].configure(text=f'{speed:.2f}×')
        if self.state.finished:
            view = f'回放结束 — 共 {self.state.total} 个视点'
        elif self.state.view:
            view = (f'{self.state.view}   '
                    f'{self.state.index} / {self.state.total}')
        else:
            view = '准备中'
        self._labels['view'].configure(text=view)
        self._pause_button.configure(text='继续 (空格)' if self.state.paused
                                     else '暂停 (空格)')

    def close(self):
        if self.root is None:
            return
        try:
            self.root.destroy()
        except Exception:
            pass
        self.root = None
        self.state.ui = None
