"""Viewpoint diversity: optical-axis deduplication and its summary statistic.

The only diversity control in the planner is a minimum angle between optical
axes. Two candidates whose axes are closer than ``sampling.min_axis_angle_deg``
are treated as the same viewpoint, so the second one is rejected as
``duplicate_axis``.

That check happens twice in a run:

  * on the *target* pose, before IK - it is one dot product, while IK plus
    collision plus path planning costs milliseconds to seconds, so rejecting
    early saves the expensive work;
  * on the *actual* arrived pose - the servo lands a fraction of a degree away
    from the target, so a candidate that was unique on paper can become a
    duplicate in reality. Re-checking is stricter, never looser.

Deduplication only guarantees pairwise separation. It says nothing about how
evenly the viewpoints cover the azimuth/tilt space, which is why the run also
reports ``min_pairwise_angle_deg`` - a single number is all the coverage feedback
there is today.

Pure numpy: no config mutation, no engine, no random stream.
"""
import numpy as np

from .geometry import angle_deg


def comparison_axes(cfg, height_axes, all_axes):
    """Which already-accepted axes a new candidate must keep away from.

    ``height_axes`` holds the axes accepted on the current height layer,
    ``all_axes`` every axis accepted so far. ``deduplicate_across_heights``
    chooses between the two; it is off by default, so each height layer is free
    to reuse a direction that an earlier layer already covered.
    """
    return all_axes if cfg["sampling"]["deduplicate_across_heights"] else height_axes


def is_duplicate(cfg, axis, known_axes):
    """True if ``axis`` is closer than the configured minimum to any known axis."""
    threshold = cfg["sampling"]["min_axis_angle_deg"]
    return any(angle_deg(axis, known) < threshold for known in known_axes)


def min_pairwise_angle_deg(axes):
    """Smallest angle between any two accepted axes, or ``None`` if fewer than two.

    Independent of the dedupe threshold: this is the achieved separation, not the
    requested one, so comparing the two tells you whether the threshold is
    actually binding.
    """
    pairs = [angle_deg(a, b) for i, a in enumerate(axes) for b in axes[i + 1:]]
    return min(pairs) if pairs else None


def duplicate_reason(cfg, axis, height_axes, all_axes):
    """``"duplicate_axis"`` when the axis collides with an accepted one, else ``None``.

    Convenience so call sites read as a single gate instead of two lines.
    """
    known = comparison_axes(cfg, height_axes, all_axes)
    return "duplicate_axis" if is_duplicate(cfg, axis, known) else None
