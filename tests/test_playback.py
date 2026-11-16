"""Playback state tests: the part of the replay controls that needs no window."""
import unittest

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


if __name__ == '__main__':
    unittest.main()
