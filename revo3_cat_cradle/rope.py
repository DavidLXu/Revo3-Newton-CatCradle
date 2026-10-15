"""Initial closed-loop geometry; no point is attached during simulation."""
import numpy as np
from .contact_geometry import finger_capsule

ROUTE = ("thumb", "index", "middle", "ring", "little")
WRAP_ANGLES = (np.pi/2, 0., -np.pi/2)


def wrap_arc(finger, theta, side="left", rope_radius=.001):
    center, _, radius, _ = finger_capsule(finger,"DIP",side)
    radius += rope_radius+.00015
    theta = np.asarray(theta)
    points = np.broadcast_to(center,theta.shape+(3,)).copy()
    if finger == "thumb":
        points[...,0] += radius*np.sin(theta)
        # Seat just behind the fingertip, beside the DIP hinge's metal band.
        points[...,1] = -.001 if side == "left" else .001
        points[...,2] -= radius*np.cos(theta)
    else:
        points[...,0] += (1 if finger == "middle" else -1)*radius*np.cos(theta)
        points[...,1] += (-1 if side == "left" else 1)*radius*np.sin(theta)
        points[...,2] = -.004
    return points


def wrap_sites(side="left", rope_radius=.001):
    sites=[]
    for finger in ROUTE:
        # The little-finger return enters the thumb on +X and exits toward
        # the index on -X. Reversing these ends crosses the two free spans.
        for point in wrap_arc(finger,np.array(WRAP_ANGLES),side,rope_radius):
            sites.append((f"{side}_{finger}_DIP_link",point))
    return sites


def site_positions(transforms, sites):
    return np.array([(transforms[name] @ np.r_[point, 1])[:3] for name, point in sites])


def build_rope(transforms, spacing=.003, side="left", span_lengths=None, rope_radius=.001):
    sites = wrap_sites(side,rope_radius)
    positions = site_positions(transforms,sites)
    particles, site_indices = [], []
    for i,a in enumerate(positions):
        b = positions[(i+1)%len(positions)]
        site_indices.append(len(particles))
        finger = ROUTE[i//3]
        if i%3 != 2:
            # Sample the actual surface arc; chords would cut through the finger.
            radius = finger_capsule(finger,"DIP",side)[2]+rope_radius+.00015
            count=max(3,int(np.ceil(radius*np.pi/2/spacing)))
            angles=np.array(WRAP_ANGLES)
            theta=np.linspace(angles[i%3],angles[i%3+1],count,endpoint=False)
            local=wrap_arc(finger,theta,side,rope_radius)
            tf=transforms[f"{side}_{finger}_DIP_link"]
            particles.extend(local@tf[:3,:3].T+tf[:3,3])
            continue
        distance=np.linalg.norm(b-a)
        target=max(distance,span_lengths[i] if span_lengths is not None else distance)
        count=max(2,int(np.ceil(target/spacing)))
        u=np.linspace(0,1,201)
        direction=(b-a)/distance
        sag=np.array([.7,0,-1.]);sag-=np.dot(sag,direction)*direction;sag/=np.linalg.norm(sag)
        low,high=0.,target
        for _ in range(30):
            amplitude=(low+high)/2
            curve=(1-u[:,None])*a+u[:,None]*b+amplitude*np.sin(np.pi*u[:,None])*sag
            arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
            if arc[-1]<target:low=amplitude
            else:high=amplitude
        samples=np.linspace(0,arc[-1],count,endpoint=False)
        particles.extend(np.column_stack([np.interp(samples,arc,curve[:,k]) for k in range(3)]))
    return np.array(particles,dtype=np.float32),np.array(site_indices,dtype=np.int32),sites
