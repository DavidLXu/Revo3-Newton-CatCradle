"""Active rest-length control using rope strain only, with no hand targets."""
import numpy as np


def tensioned_lengths(edge_lengths, stiffness, force=.08, take_up_m=0.):
    """Apply a loop-wide take-up offset to the nominal tension target."""
    target=np.maximum(edge_lengths-force/stiffness, edge_lengths/1.08)
    target *= 1.-take_up_m/target.sum()
    return target


def adjust_lengths(points, rest, initial, stiffness, dt, take_up_m=0.):
    lengths=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1)
    target=tensioned_lengths(lengths,stiffness,take_up_m=take_up_m)
    target=np.clip(target,initial*.25,initial*2.)
    # Take-up rate is total loop length per second, distributed by initial
    # edge length. Payout immediately releases excess load (an ideal active
    # actuator), instead of fighting a rapidly moving kinematic finger.
    weights=initial/initial.sum()
    result=rest+np.maximum(target-rest,-.30*dt*weights)
    total=result.sum()
    result *= np.clip(total,.20,.70)/total
    return result.astype(np.float32)
