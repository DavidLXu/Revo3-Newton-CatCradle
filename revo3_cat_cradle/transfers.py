"""Geometric planning for contact-only string transfers after Opening A."""

import numpy as np
from scipy.optimize import least_squares
from .contact_geometry import finger_capsule
from .bimanual_checks import gaps


def fingertip(robot, q, side, finger):
    c, axis, radius, half = finger_capsule(finger, "DIP", side)
    direction = -1 if finger == "thumb" and side == "left" else 1
    local = c + direction * axis * (half + radius)
    return (robot.fk(q)[f"{side}_{finger}_DIP_link"] @ np.r_[local, 1])[:3]


def thumb_arc_sweep(transforms, points, side, segment="DIP"):
    """Unsigned angular sweep of an ordered open strand about a thumb axis.

    A planning diagnostic, not a topological certificate: the finger is bent,
    and this projection alone cannot prove axial overlap or exclude crossings.
    No artificial closing chord is added to the open material arc.
    """
    center, _, _, _ = finger_capsule("thumb", segment, side)
    tf = transforms[f"{side}_thumb_{segment}_link"]
    local = (np.asarray(points) - tf[:3, 3]) @ tf[:3, :3] - center
    angles = np.unwrap(np.arctan2(local[:, 2], local[:, 0]))
    return float(abs(angles[-1] - angles[0]))


def digit_target(robot, q, side, finger, target, avoid_points=None):
    """Move one digit only; bounded IK cannot move or constrain rope particles."""
    ids = [i for i, n in enumerate(robot.names) if n.startswith(f"{side}_{finger}_")]
    initial = q[ids].copy()
    trial = q.copy()

    def residual(x):
        trial[ids] = x
        result = [
            (fingertip(robot, trial, side, finger) - target) * 100,
            0.01 * (x - initial),
        ]
        if avoid_points is not None:
            for u in (0.25, 0.5, 0.75, 1.0):
                intermediate = q.copy()
                intermediate[ids] = (1 - u) * initial + u * x
                result.append(
                    np.minimum(
                        gaps(robot.fk(intermediate), avoid_points, side, finger)
                        - 0.002,
                        0,
                    )
                    * 150
                )
        return np.concatenate(result)

    fit = least_squares(
        residual,
        initial,
        bounds=(robot.limits[ids, 0], robot.limits[ids, 1]),
        max_nfev=300,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )
    trial[ids] = fit.x
    error = np.linalg.norm(fingertip(robot, trial, side, finger) - target)
    if error > 0.003:
        raise ValueError(f"{side} {finger} target misses by {error * 1000:.1f} mm")
    if avoid_points is not None:
        clearance = min(
            gaps(robot.fk((1 - u) * q + u * trial), avoid_points, side, finger).min()
            for u in np.linspace(0, 1, 21)
        )
        if clearance < 0.0015:
            raise ValueError(
                f"{side} {finger} sweep misses near-string clearance: {clearance * 1000:.1f} mm"
            )
    return trial.copy()


def support_contact_patch(indices, count):
    """Planning patch, separating isolated grazes from a contiguous support.

    This does not certify a loop or suppress contacts in the physics/audits.
    Multiple extended patches remain ambiguous and must be inspected.
    """
    indices = np.asarray(indices, dtype=int)
    if not len(indices):
        return indices
    patches = list(np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1))
    if len(patches) > 1 and patches[0][0] == 0 and patches[-1][-1] == count - 1:
        patches = [np.r_[patches[-1], patches[0]]] + patches[1:-1]
    patches.sort(key=len, reverse=True)
    if len(patches) > 1 and (
        len(patches[0]) < 3 or any(len(p) > 1 for p in patches[1:])
    ):
        raise ValueError("Ambiguous multiple support contact patches")
    return patches[0]


def track_support_patch(indices, count, previous_center, max_shift=8):
    """Follow a support's material patch through sliding, without relabeling jumps.

    Prefer a patch containing the previous representative material sample.
    When growth/merging moves that patch's midpoint beyond max_shift, retain
    the still-contacting representative instead of treating midpoint drift as
    material loss. Otherwise the original midpoint continuity bound applies.
    Other contact patches are returned explicitly. This tracks a known support,
    not a topology certificate; an extra patch can be a new unwanted capture.
    Call sequentially, once per saved frame. Lost or ambiguous tracks fail closed.
    """
    indices = np.sort(np.unique(np.asarray(indices, dtype=int)))
    if not len(indices):
        raise ValueError("Tracked support has no contact")
    patches = list(np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1))
    if len(patches) > 1 and patches[0][0] == 0 and patches[-1][-1] == count - 1:
        patches = [np.r_[patches[-1], patches[0]]] + patches[1:-1]
    centers = [int(p[(len(p) - 1) // 2]) for p in patches]
    distances = [
        min((c - previous_center) % count, (previous_center - c) % count)
        for c in centers
    ]
    containing = next((i for i, patch in enumerate(patches)
                       if previous_center in patch), None)
    if containing is not None:
        center = centers[containing] if distances[containing] <= max_shift else previous_center
        return (
            center,
            patches[containing],
            [patch for i, patch in enumerate(patches) if i != containing],
        )
    order = np.argsort(distances)
    if distances[order[0]] > max_shift:
        raise ValueError("Tracked support moved beyond continuity bound")
    if len(order) > 1 and distances[order[0]] == distances[order[1]]:
        raise ValueError("Ambiguous support track")
    selected = int(order[0])
    return (
        centers[selected],
        patches[selected],
        [p for i, p in enumerate(patches) if i != selected],
    )


def directed_support_arc(start, end, count, direction):
    """Keep the original cyclic orientation as the supporting contacts slide."""
    if direction not in (-1, 1):
        raise ValueError("Arc direction must be -1 or +1")
    length = ((end - start) * direction) % count
    return (start + direction * np.arange(length + 1)) % count


def support_arc(robot, q, points, start, end, meshes=None):
    """Find the cyclic arc between supports without passing a third support."""
    tf = robot.fk(q)

    def contact_indices(side, finger):
        if meshes is None:
            return np.flatnonzero(gaps(tf, points, side, finger) < 0.0015)
        from .mesh_geometry import support_contact_indices

        return support_contact_indices(tf, points, side, finger, meshes)

    labels = [(s, f) for s in ("left", "right") for f in ("thumb", "index", "little")]
    contacts = {
        label: support_contact_patch(contact_indices(*label), len(points))
        for label in labels
    }
    if any(not len(v) for v in contacts.values()):
        raise ValueError("Opening A support missing before transfer")
    a, b = [
        int(contacts[label][(len(contacts[label]) - 1) // 2]) for label in (start, end)
    ]
    n = len(points)
    arcs = [
        np.arange(a, a + (b - a) % n + 1) % n,
        np.arange(a, a - (a - b) % n - 1, -1) % n,
    ]
    others = np.concatenate([v for k, v in contacts.items() if k not in (start, end)])
    valid = [arc for arc in arcs if not np.intersect1d(arc, others).size]
    if len(valid) != 1:
        raise ValueError(f"Ambiguous support arc: {start} -> {end}")
    return valid[0]


def original_thumb_loop(robot, q, points, side, meshes=None):
    """Both branches of an Opening A thumb loop, in material coordinates.

    Call before transfers: contact particles slide around the finger later.
    One branch leaving the finger is insufficient evidence of a release.
    """
    other = "right" if side == "left" else "left"
    return np.union1d(
        support_arc(robot, q, points, (side, "thumb"), (other, "thumb"), meshes=meshes),
        support_arc(robot, q, points, (side, "thumb"), (other, "index"), meshes=meshes),
    )


def over_strand_margin(point, strand, width=0.015, height=0.003):
    """Tip clearance over a strand's vertical curtain in world XY/Z coordinates.

    Only a planning constraint: rope contacts still require physical replay.
    Interpolate every segment at the tip's Y, retaining multiple intersections.
    A point outside the curtain's X band is unrestricted.
    """
    strand = np.asarray(strand)
    a, b = strand[:-1], strand[1:]
    dy = b[:, 1] - a[:, 1]
    variable = abs(dy) > 1e-10
    t = np.divide(point[1] - a[:, 1], dy, out=np.zeros_like(dy), where=variable)
    selected = variable & (t >= 0) & (t <= 1)
    crossings = a[selected] + t[selected, None] * (b[selected] - a[selected])
    # Horizontal segments occupy the whole X interval at their Y.
    horizontal = (~variable) & (abs(point[1] - a[:, 1]) < 1e-6)
    for start, end in zip(a[horizontal], b[horizontal]):
        if min(start[0], end[0]) - width <= point[0] <= max(start[0], end[0]) + width:
            crossings = np.vstack(
                [crossings, [point[0], point[1], max(start[2], end[2])]]
            )
    crossings = crossings[abs(crossings[:, 0] - point[0]) <= width]
    return (
        float(point[2] - crossings[:, 2].max() - height)
        if len(crossings)
        else float("inf")
    )


def plan_digit_path(
    robot,
    q,
    goal,
    side,
    finger,
    obstacles,
    seed=7,
    attempts=5000,
    min_tip_height=None,
    over_points=None,
):
    """Deterministic joint-space search around a measured strand, without rope edits."""
    from .bimanual_checks import hand_clearance

    ids = [i for i, n in enumerate(robot.names) if n.startswith(f"{side}_{finger}_")]
    rng = np.random.default_rng(seed)
    lo = robot.limits[ids, 0]
    hi = robot.limits[ids, 1]

    def valid(x):
        trial = q.copy()
        trial[ids] = x
        if (
            over_points is not None
            and over_strand_margin(fingertip(robot, trial, side, finger), over_points)
            < 0
        ):
            return False
        if (
            min_tip_height is not None
            and fingertip(robot, trial, side, finger)[2] < min_tip_height
        ):
            return False
        tf = robot.fk(trial)
        if hand_clearance(tf)[0] < -0.0005:
            return False
        if gaps(tf, obstacles, side, finger).min() < 0.0015:
            return False
        own = [
            g
            for g, a, b in hand_clearance(tf, side, all_pairs=True)
            if a.startswith(finger + "_") or b.startswith(finger + "_")
        ]
        return min(own) > -0.0005

    def edge(a, b):
        count = max(2, int(np.ceil(np.linalg.norm(a - b) / 0.07)))
        return all(valid((1 - u) * a + u * b) for u in np.linspace(0, 1, count + 1)[1:])

    if not valid(q[ids]) or not valid(goal[ids]):
        raise ValueError("Digit path endpoints are not clear")
    nodes = [q[ids].copy()]
    parent = [-1]
    for _ in range(attempts):
        target = goal[ids] if rng.random() < 0.2 else rng.uniform(lo, hi)
        near = int(np.argmin(np.linalg.norm(np.array(nodes) - target, axis=1)))
        delta = target - nodes[near]
        trial = nodes[near] + delta * min(1.0, 0.25 / max(np.linalg.norm(delta), 1e-9))
        if not edge(nodes[near], trial):
            continue
        nodes.append(trial)
        parent.append(near)
        if np.linalg.norm(trial - goal[ids]) < 0.4 and edge(trial, goal[ids]):
            path = [goal[ids]]
            k = len(nodes) - 1
            while k >= 0:
                path.append(nodes[k])
                k = parent[k]
            path = path[::-1]
            simplified = [path[0]]
            k = 0
            while k < len(path) - 1:
                j = len(path) - 1
                while j > k + 1 and not edge(path[k], path[j]):
                    j -= 1
                simplified.append(path[j])
                k = j
            result = []
            for x in simplified[1:]:
                trial = q.copy()
                trial[ids] = x
                result.append(trial)
            return result
    raise ValueError("No collision-free digit sweep found")
