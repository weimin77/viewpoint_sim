"""Run statistics: the summary.json contract.

Named ``run_statistics`` rather than ``statistics`` so it cannot shadow the
standard library module of that name; nothing here needs numpy either.

Everything here is a pure function of what the run already computed, so the
numbers in ``summary.json`` can be reproduced and tested without a simulator:

    height_row(...)            one entry of summary["heights"]
    view_ids_ranked_by_ratio   post-hoc ordering, keyed by height string
    build_summary(...)         the complete summary dict

Two conventions matter because they are persisted:

  * ``heights`` keeps the order the caller appended them, which is
    ``sampling.heights_above_tray_m`` - not the order candidates were accepted;
  * ``rejections`` counts every rejected attempt per reason, so a height that
    needed 12 tries to fill 4 slots still reports all 12 reasons.

The ranking in ``view_ids_ranked_by_ratio`` is advisory: it does not influence
selection (the planner still takes the first N that pass), it only tells a human
which of the accepted views framed the tray best.
"""
from .diversity import min_pairwise_angle_deg

SIMULATION_LIMITS = [
    "Example inertias/servo gains, not calibrated hardware dynamics",
    "Ideal aligned undistorted RGB-D; no stereo matching/noise/material physics",
    "Discrete collision checking; not continuous swept-volume certification",
    "Example workcell and camera extrinsics; replace with measured values",
]


def height_row(height, accepted, target, attempts, failures):
    """One row of ``summary["heights"]``.

    ``failures`` is a Counter keyed by rejection reason. ``complete`` is false
    when the attempt budget ran out before the target count was reached - that is
    a reported outcome, never a silent success.
    """
    return dict(height_above_tray_m=height, accepted=accepted, target=target,
                attempts=attempts, rejections=dict(failures),
                complete=accepted == target,
                acceptance_rate=accepted / max(1, attempts))


def view_ids_ranked_by_ratio(cfg, accepted):
    """Per height, the accepted view ids sorted by raw tray ratio (descending).

    Advisory only - see the module docstring. Kept for the report and for anyone
    who wants to pick a smaller subset from an already-validated run.
    """
    ranked = {}
    for h in cfg["sampling"]["heights_above_tray_m"]:
        same_height = (v for v in accepted if v["base_height_above_tray_m"] == h)
        ranked[str(h)] = [v["id"] for v in
                          sorted(same_height,
                                 key=lambda v: v["geometry"]["tray_ratio_raw"],
                                 reverse=True)]
    return ranked


def build_summary(cfg, heights, accepted, all_axes, elapsed_seconds, created_utc):
    """Assemble the complete summary dict written to summary.json.

    ``created_utc`` is passed in rather than read here so the function stays
    deterministic; the caller owns the clock.
    """
    return dict(
        schema_version=2,
        engine="mujoco",
        robot_model=cfg["robot"]["model"],
        created_utc=created_utc,
        seed=cfg["seed"],
        complete=all(x["complete"] for x in heights),
        accepted_total=len(accepted),
        heights=heights,
        minimum_axis_angle_deg=min_pairwise_angle_deg(all_axes),
        elapsed_seconds=elapsed_seconds,
        simulation_limits=list(SIMULATION_LIMITS),
    )
