"""Joint-limit and geometry-distance collision checking.

Split so that the part which decides *what to test* needs no simulator:

    excluded_body_pairs(...)     MJCF <contact><exclude> -> non-contact pairs
    geom_pairs(...)              the ordered (g1, g2, kind) test list
    joint_limit_violation(...)   pure numpy bound check
    Checker.violation()          broad phase + exact mj_geomDistance

Two decisions worth keeping explicit, because both were silent bugs waiting to
happen:

  * the robot/environment split is by geom name prefix (``col_`` vs
    ``env_``/``food_``), so a newly modelled obstacle joins the test list by
    being built, not by being registered somewhere else;
  * the arm base against the table is exempted by body name - they are bolted
    together, and a margin test between them reports a permanent collision.

Pairs listed in the generated MJCF's <contact><exclude> block are skipped for
self-collision, so adjacent links that merely share a joint axis are not
reported as touching.
"""
import xml.etree.ElementTree as ElementTree
import numpy as np
import mujoco


def excluded_body_pairs(xml_text, body_id):
    """Body pairs the MJCF declares as non-contacts, as frozensets of ids.

    ``body_id`` maps a body name to its index in the model; the MJCF text is
    parsed again rather than read back from the model so the exclusion list and
    the pair list come from the same source of truth.
    """
    root = ElementTree.fromstring(xml_text)
    return {frozenset((body_id(edge.get('body1')), body_id(edge.get('body2'))))
            for edge in root.findall('contact/exclude')}


def geom_pairs(robot_geoms, env_geoms, body_of, names, excluded):
    """The collision test list, as ``(geom1, geom2, kind)`` triples.

    ``body_of`` maps a geom index to its body index and ``names`` maps a body
    index to its name; both are passed in so this stays free of MuJoCo types.
    Order is deterministic - robot geoms in model order, each against the later
    robot geoms then against the environment - so the first reported violation
    is reproducible.
    """
    pairs = []
    for index, first in enumerate(robot_geoms):
        first_body = body_of(first)
        for second in robot_geoms[index + 1:]:
            second_body = body_of(second)
            if (first_body != second_body
                    and frozenset((first_body, second_body)) not in excluded):
                pairs.append((first, second, 'self_collision'))
        for second in env_geoms:
            if names[first_body] == 'base' and names[body_of(second)] == 'table':
                continue
            pairs.append((first, second, 'environment_collision'))
    return pairs


def joint_limit_violation(q, lower, upper, margin):
    """``'joint_limit'`` when ``q`` is inside the configured safety margin.

    The margin matters because the servo overshoots slightly; a limit check that
    only rejected exactly-on-limit configurations would let a commanded target
    drive the joint past its stop.
    """
    if np.any(q < lower + margin - 1e-8) or np.any(q > upper - margin + 1e-8):
        return 'joint_limit'
    return None


class Checker:
    """Collision query against the live MuJoCo state.

    ``data`` is held by reference, not copied: the caller mutates it through
    ``set_q``/``mj_step`` and the next query must see that state.
    """

    def __init__(self, model, data, pairs, margin_m):
        self.model = model
        self.data = data
        self.pairs = pairs
        self.margin_m = margin_m
        self._first = np.array([p[0] for p in pairs], dtype=int)
        self._second = np.array([p[1] for p in pairs], dtype=int)

    def violation(self):
        """``'kind:geom1:geom2'`` for the first offending pair, else None.

        Bounding-sphere separation is checked first so the exact distance query
        only runs for pairs that can plausibly be within the margin - the whole
        point of a broad phase, and measurable here because the pair list is
        in the thousands.
        """
        if not self.pairs:
            return None
        centres = self.data.geom_xpos
        apart = (np.linalg.norm(centres[self._first] - centres[self._second], axis=1)
                 - self.model.geom_rbound[self._first]
                 - self.model.geom_rbound[self._second])
        for index in np.flatnonzero(apart < self.margin_m):
            first, second, kind = self.pairs[index]
            distance = mujoco.mj_geomDistance(
                self.model, self.data, first, second, self.margin_m, None)
            if distance < self.margin_m - 1e-10:
                return f'{kind}:{self.model.geom(first).name}:{self.model.geom(second).name}'
        return None
