"""Per-image quality assessment: visible fractions, depth validity, framing.

This is the acceptance test applied to a rendered capture, expressed as a pure
function over arrays so it can be unit-tested without a simulator:

    assess(z, body, reference, food_ids, names, cfg)
        -> {"labels", "depth_mm", "metrics", "failures"}

Inputs come straight from ``World.render`` plus ``World.isolated_food_masks``:

    z          float depth, optical Z in metres, one value per pixel
    body       int32 per-pixel body id, -1 for background
    reference  one boolean silhouette per food, rendered with every other
               object hidden, so it carries the food's own self-occlusion but
               none of the mutual occlusion

Everything the assessment needs about the scene (which body is which food, what
each is called) arrives as arguments, which is why nothing here imports MuJoCo.

The metrics and the reason strings are the contract with the dataset on disk:
``metrics`` is persisted verbatim into metadata.json, so its keys and their
order matter, and ``failures`` is what the planner's rejection statistics are
built from.

Four checks per food, evaluated for every food before any failure is reported, so
a single capture can fail for several reasons at once:

    occlusion                   visible fraction below the threshold
                                (only when quality.visibility_mode == "enforce")
    food_cut_by_image_border    the mask touches any image edge
    invalid_depth               too few of its pixels fall inside depth_range_m
    insufficient_food_pixels    too few pixels, or a bbox side below the minimum

The reasons are then de-duplicated and ordered by ``FAILURE_PRIORITY`` rather
than alphabetically, so "which reason do we report" is stable across runs.
"""
import numpy as np

FOOD_FAILURE_PRIORITY = {
    "occlusion": 0,
    "food_cut_by_image_border": 1,
    "invalid_depth": 2,
    "insufficient_food_pixels": 3,
}


def depth_valid_mask(z, body, cfg):
    """Pixels whose depth reading is usable.

    A pixel is valid when it hits a body at all and its optical Z falls inside
    the configured sensor range. Background (``body < 0``) is excluded even if it
    is in range, because a background depth is not a measurement of the scene.
    """
    lo, hi = cfg["camera"]["depth_range_m"]
    return (z >= lo) & (z <= hi) & (body >= 0)


def depth_to_millimetres(z, valid):
    """uint16 millimetres, with 0 reserved for "no valid reading"."""
    return np.where(valid, np.rint(z * 1000), 0).astype(np.uint16)


def instance_labels(body, food_ids):
    """uint16 label image: 0 for background, 1..N in ``food_ids`` order.

    The enumeration order is the contract with ``instance.png``: instance 1 is
    the first entry of ``scene.foods``, so reordering the config renumbers the
    mask.
    """
    labels = np.zeros(body.shape, dtype=np.uint16)
    for instance, bid in enumerate(food_ids, 1):
        labels[body == bid] = instance
    return labels


def food_metric(instance, name, mask, reference, valid):
    """Per-food metrics dict, with the keys and order metadata.json expects."""
    nref, count = int(reference.sum()), int(mask.sum())
    ys, xs = np.nonzero(mask)
    bbox = ([int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]
            if count else [0, 0, 0, 0])
    side = min(bbox[2] - bbox[0], bbox[3] - bbox[1])
    return dict(instance_id=instance, name=name, reference_pixels=nref,
                visible_pixels=count,
                visible_fraction=count / max(1, nref),
                valid_depth_fraction=float(valid[mask].mean()) if count else 0.,
                bbox_xyxy=bbox, min_bbox_side_px=side)


def food_failures(metric, mask, cfg):
    """Reason strings for one food, in no particular order (sorted later)."""
    qc = cfg["quality"]
    reasons = []
    if qc["visibility_mode"] == "enforce" and \
            metric["visible_fraction"] < qc["min_visible_fraction_per_food"]:
        reasons.append("occlusion")
    if metric["visible_pixels"] < qc["min_food_pixels"] or \
            metric["min_bbox_side_px"] < qc["min_food_bbox_side_px"]:
        reasons.append("insufficient_food_pixels")
    if metric["valid_depth_fraction"] < qc["min_valid_depth_fraction"]:
        reasons.append("invalid_depth")
    if metric["visible_pixels"] and (mask[0].any() or mask[-1].any()
                                     or mask[:, 0].any() or mask[:, -1].any()):
        reasons.append("food_cut_by_image_border")
    return reasons


def assess(z, body, reference, food_ids, names, cfg):
    """Score one rendered capture and produce the exported label/depth images.

    ``names`` maps body id to a human-readable name (``World.names``); it is only
    used to label the metrics, never to decide anything.
    """
    valid = depth_valid_mask(z, body, cfg)
    labels = instance_labels(body, food_ids)
    metrics, reasons = [], []
    for instance, (bid, ref) in enumerate(zip(food_ids, reference), 1):
        mask = body == bid
        metric = food_metric(instance, names[bid], mask, ref, valid)
        metrics.append(metric)
        reasons.extend(food_failures(metric, mask, cfg))
    return {
        "labels": labels,
        "depth_mm": depth_to_millimetres(z, valid),
        "metrics": metrics,
        # Deduplicate across foods, then order deterministically.
        "failures": sorted(set(reasons), key=FOOD_FAILURE_PRIORITY.get),
    }
