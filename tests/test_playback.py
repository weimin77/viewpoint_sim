"""Playback state tests: the part of the replay controls that needs no window."""
import os
import threading
import time
import unittest
from unittest import mock

from viewpoint_sim import playback
from viewpoint_sim.playback import MAX_SPEED, MIN_SPEED, Playback


class PlaybackTests(unittest.TestCase):
    def test_speed_is_clamped_to_the_slider_range(self):
        self.assertEqual(Playback(0.001).get_speed(), MIN_SPEED)
        self.assertEqual(Playback(99).get_speed(), MAX_SPEED)
        self.assertEqual(Playback(2.5).get_speed(), 2.5)

    def test_speed_changes_are_read_live(self):
        state = Playback(1.0)
        self.assertEqual(state.get_speed(), 1.0)
        state.speed = 8.0          # what the slider writes
        self.assertEqual(state.get_speed(), 8.0)

    def test_reset_fires_once(self):
        state = Playback()
        state.reset = True
        self.assertTrue(state.take_reset())
        self.assertFalse(state.take_reset())

    def test_reset_clears_pause(self):
        state = Playback()
        state.paused = True
        state.reset = True
        state.take_reset()
        self.assertFalse(state.paused)

    def test_progress_tracks_the_view_being_played(self):
        state = Playback()
        state.start(3)
        self.assertEqual((state.index, state.total, state.view), (0, 3, ''))
        state.at_view(2, 'h01_c0004')
        self.assertEqual((state.index, state.view), (2, 'h01_c0004'))
        state.finish()
        self.assertTrue(state.finished)

    def test_wait_while_paused_returns_immediately_when_reset_is_pending(self):
        state = Playback()
        state.paused = True
        state.reset = True                       # a reset always outranks a pause
        state.wait_while_paused()                # must not block
        self.assertTrue(state.take_reset())

    def test_wait_while_paused_returns_immediately_when_not_paused(self):
        state = Playback()
        state.wait_while_paused()                # must not block
        self.assertFalse(state.paused)


class LabelTests(unittest.TestCase):
    """The panel must not show tofu on a machine without CJK fonts."""

    def _with_lang(self, value):
        env = {k: v for k, v in os.environ.items() if k != 'VIEWPOINT_LANG'}
        if value is not None:
            env['VIEWPOINT_LANG'] = value
        return mock.patch.dict(os.environ, env, clear=True)

    def test_lang_env_forces_each_language(self):
        with self._with_lang('zh'):
            self.assertTrue(playback.using_chinese(None))
        with self._with_lang('en'):
            self.assertFalse(playback.using_chinese(None))

    def test_detection_without_override_asks_the_font(self):
        fake = mock.Mock()
        fake.measure.return_value = 0                  # no glyphs -> tofu
        with self._with_lang(None), \
                mock.patch('tkinter.font.nametofont', return_value=fake):
            self.assertFalse(playback.using_chinese(None))
        fake.measure.return_value = 24                 # glyphs present
        with self._with_lang(None), \
                mock.patch('tkinter.font.nametofont', return_value=fake):
            self.assertTrue(playback.using_chinese(None))

    def test_ascii_labels_contain_no_cjk(self):
        for value in playback.TEXT_EN.values():
            self.assertFalse([c for c in value if '\u4e00' <= c <= '\u9fff'],
                             f'{value!r} contains CJK')

    def test_both_label_sets_define_the_same_keys(self):
        self.assertEqual(set(playback.TEXT_EN), set(playback.TEXT_ZH))


class StepTests(unittest.TestCase):
    """The frame hook is the only place quit/restart/pause are decided."""

    def test_idle_step_returns(self):
        Playback().step()                     # nothing pending: must not raise

    def test_quit_raises_abort(self):
        state = Playback()
        state.quit = True
        with self.assertRaises(playback.Abort):
            state.step()

    def test_reset_raises_restart_immediately(self):
        state = Playback()
        state.reset = True
        with self.assertRaises(playback.Restart):
            state.step()

    def test_reset_during_pause_raises_restart(self):
        state = Playback()
        state.paused = True

        def press_reset():
            time.sleep(.05)
            state.reset = True

        thread = threading.Thread(target=press_reset, daemon=True)
        thread.start()
        with self.assertRaises(playback.Restart):
            state.step()
        thread.join()

    def test_pause_unblocks_when_the_viewer_closes(self):
        """Closing the 3D window while paused used to block forever."""
        state = Playback()
        state.paused = True
        alive = {'yes': True}
        state.stop_check = lambda: alive['yes']
        finished = threading.Event()

        def waiter():
            state.wait_while_paused()
            finished.set()

        thread = threading.Thread(target=waiter, daemon=True)
        thread.start()
        time.sleep(.1)
        self.assertFalse(finished.is_set())      # parked while the window lives
        alive['yes'] = False
        self.assertTrue(finished.wait(2.0))      # and released when it dies
        self.assertTrue(state.quit)
        thread.join()

    def test_pause_keeps_waiting_while_everything_is_alive(self):
        state = Playback()
        state.paused = True
        state.stop_check = lambda: True
        finished = threading.Event()
        threading.Thread(target=lambda: (state.wait_while_paused(),
                                         finished.set()), daemon=True).start()
        time.sleep(.15)
        self.assertFalse(finished.is_set())
        state.paused = False
        self.assertTrue(finished.wait(2.0))
