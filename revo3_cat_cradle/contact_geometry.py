"""Capsules fitted to official visual bounds, shared by contact and placement."""
import numpy as np


def finger_capsule(finger, segment, side="left"):
    # center, radius, half cylinder length; meters in the URDF link frame.
    if finger == "thumb":
        center, radius, half_height = {
            "MCP": ((-.00135, -.0218, .0001), .0145, .0140),
            "PIP": ((-.0006, -.0161, .00015), .0116, .0120),
            "DIP": ((-.0018, -.0139, .0003), .0115, .0090),
        }[segment]
        center = np.array(center)
        if side == "right":
            center[1] *= -1
        axis = np.array([0., 1., 0.])
    else:
        center, radius, half_height = {
            "MCP": ((-.00025, 0., .0230), .0123, .0191),
            "PIP": ((-.00025, 0., .0150), .0108, .0113),
            "DIP": ((-.0011, 0., .0115), .0105, .0080),
        }[segment]
        center = np.array(center)
        axis = np.array([0., 0., 1.])
    return center, axis, radius, half_height


def clear_initial_intersections(points, transforms, side, rope_radius):
    """One-time geometric placement, never called while integrating motion."""
    p = points.astype(np.float64).copy()
    for _ in range(8):
        for finger in ("thumb", "index", "middle", "ring", "little"):
            for segment in ("MCP", "PIP", "DIP"):
                center, axis, radius, half_height = finger_capsule(finger, segment, side)
                tf = transforms[f"{side}_{finger}_{segment}_link"]
                local = (p-tf[:3,3]) @ tf[:3,:3]
                along = np.clip((local-center) @ axis, -half_height, half_height)
                closest = center+along[:,None]*axis
                delta = local-closest
                distance = np.linalg.norm(delta,axis=1)
                required = radius+rope_radius+.00015
                inside = distance < required
                if np.any(inside):
                    direction = delta[inside]/np.maximum(distance[inside,None],1e-12)
                    p[inside] += ((required-distance[inside])[:,None]*direction) @ tf[:3,:3].T
    return p.astype(np.float32)
