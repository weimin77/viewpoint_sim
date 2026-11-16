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
reports ``ok == False`` and replay falls back to ``--speed``. If the environment
cannot draw CJK glyphs, the panel switches to ASCII labels instead of showing
tofu; see ``using_chinese``.
"""
import os
import time

FRAME_DT = 0.025          # ~40 fps, the preview's wall-clock frame budget
MIN_SPEED = 0.25
MAX_SPEED = 8.0

BACKDROP = '#141a22'
PANEL = '#1c2430'
TEXT = '#dfe7ef'
MUTED = '#8fa3b8'
ACCENT = '#72dfbe'

# The panel ships two label sets and picks one at runtime. A CJK label on a
# machine without CJK fonts renders as tofu, and Tk cannot fix that: with
# `::tk::pkgconfig get fontsystem` == x11 it only reaches server-side bitmap
# fonts, and a bare container often has no CJK font installed at all. Probing is
# one `measure()` call - an unrenderable string measures 0.
TEXT_ZH = {
    'title': '回放控制', 'speed': '速度', 'replay': '重放 (R)',
    'pause': '暂停 (空格)', 'resume': '继续 (空格)', 'quit': '退出 (Esc)',
    'hint': '拖动滑块即时生效，只改播放速度，不改数据',
    'ready': '准备中', 'done': '回放结束 — 共 {n} 个视点',
    'realtime': '1× 实时',
}
TEXT_EN = {
    'title': 'Replay controls', 'speed': 'Speed', 'replay': 'Replay (R)',
    'pause': 'Pause (Space)', 'resume': 'Resume (Space)', 'quit': 'Quit (Esc)',
    'hint': 'Live playback speed only - recorded data never changes',
    'ready': 'Ready', 'done': 'Finished - {n} views',
    'realtime': '1x realtime',
}


def using_chinese(root):
    """True when the environment can actually draw CJK glyphs.

    Zero width means no glyph, which is exactly the tofu case; a real font
    measures a few multiples of its size. Called once per window, so the cost of
    building a probe font does not matter.

    ``VIEWPOINT_LANG`` overrides the probe: ``zh`` forces Chinese (useful once a
    CJK font is installed), ``en`` forces the ASCII labels.
    """
    forced = os.environ.get('VIEWPOINT_LANG', '').strip().lower()
    if forced in ('zh', 'zh_cn', 'zh-cn', 'cn'):
        return True
    if forced in ('en', 'en_us', 'en-us'):
        return False
    import tkinter.font as tkfont
    try:
        return tkfont.nametofont('TkDefaultFont').measure('回放') > 0
    except Exception:
        return False


class Abort(Exception):
    """Raised out of a running playback loop when the user quits."""


class Restart(Exception):
    """Raised when the user asks to play the run again from the first view.

    Distinct from ``Abort`` because the replay loop must not exit: it drops the
    pass it is in the middle of and starts over immediately. Latching the
    request instead would make the button look broken for as long as the pass
    still has to run - up to ten seconds on the demo dataset.
    """


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
        # Set by the replay loop to "is the 3D window still open?". Without it a
        # pause would outlive the viewer it belongs to and block forever.
        self.stop_check = None

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

    def step(self):
        """Frame hook: honour quit, restart and pause, in that order.

        Called once per drawn frame from the execution loop, which is why it
        takes no arguments - and why every playback mode funnels through it
        instead of each keeping its own copy of the pause logic.
        """
        self.pump()
        if self.quit:
            raise Abort()
        if self.reset:
            raise Restart()
        if self.paused:
            self.wait_while_paused()
            if self.quit:
                raise Abort()
            if self.reset:
                raise Restart()

    def wait_while_paused(self):
        """Block the loop while paused, still responding to reset/quit/close.

        The viewer can go away while we are parked here - the user closes the 3D
        window but leaves the panel open - so the loop asks ``stop_check`` too and
        treats "the window I am driving is gone" as a quit.
        """
        while self.paused and not self.quit and not self.reset:
            if self.stop_check is not None and not self.stop_check():
                self.quit = True
                self.paused = False
                return
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

    def __init__(self, state, title=None):
        self.state = state
        self.root = None
        self.error = None
        self.text = TEXT_EN
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
        self.text = TEXT_ZH if using_chinese(root) else TEXT_EN
        try:
            self._build(tk, root, title or self.text['title'])
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

        tk.Label(wrap, text=self.text['title'], bg=BACKDROP, fg=ACCENT,
                 font=('TkDefaultFont', 11, 'bold')).pack(anchor='w')

        self._labels['view'] = tk.Label(wrap, text=self.text['ready'], bg=BACKDROP,
                                        fg=TEXT, font=('TkDefaultFont', 10))
        self._labels['view'].pack(anchor='w', pady=(10, 2))

        row = tk.Frame(wrap, bg=BACKDROP)
        row.pack(fill='x', pady=(10, 0))
        tk.Label(row, text=self.text['speed'], bg=BACKDROP, fg=MUTED).pack(side='left')
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
        tk.Label(hint, text=f'{MIN_SPEED:g}×', bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='left')
        tk.Label(hint, text=self.text['realtime'], bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='left', padx=(26, 0))
        tk.Label(hint, text=f'{MAX_SPEED:g}×', bg=BACKDROP, fg=MUTED,
                 font=('TkDefaultFont', 8)).pack(side='right')

        buttons = tk.Frame(wrap, bg=BACKDROP)
        buttons.pack(fill='x', pady=(14, 0))
        self._button(tk, buttons, self.text['replay'], self._reset).pack(side='left')
        self._pause_button = self._button(tk, buttons, self.text['pause'], self._toggle)
        self._pause_button.pack(side='left', padx=6)
        self._button(tk, buttons, self.text['quit'], self._quit).pack(side='left')

        tk.Label(wrap, text=self.text['hint'],
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
            # The window went away underneath us; keep the replay running but
            # stop pretending we can still drive it.
            self.root = None
            if self.state.ui is self:
                self.state.ui = None

    def step(self):
        """Compatibility shim: prefer ``Playback.step``, which is a no-op here."""
        self.poll()

    def _refresh(self):
        if self.root is None:
            return
        speed = self.state.get_speed()
        self._labels['speed'].configure(text=f'{speed:.2f}x')
        if self.state.finished:
            view = self.text['done'].format(n=self.state.total)
        elif self.state.view:
            view = (f'{self.state.view}   '
                    f'{self.state.index} / {self.state.total}')
        else:
            view = self.text['ready']
        self._labels['view'].configure(text=view)
        self._pause_button.configure(text=self.text['resume']
                                     if self.state.paused
                                     else self.text['pause'])

    def close(self):
        if self.root is None:
            if self.state.ui is self:
                self.state.ui = None
            return
        try:
            self.root.destroy()
        except Exception:
            pass
        self.root = None
        if self.state.ui is self:
            self.state.ui = None
