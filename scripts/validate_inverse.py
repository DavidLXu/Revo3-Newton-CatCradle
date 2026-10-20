"""Validate physical continuity and the restored open-loop inverse end state.

Palm crossings are a fixed front-view diagnostic, not a general knot invariant.
The point-cloud comparison tolerates material sliding and particle reindexing.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from revo3_cat_cradle.hand import Hand
from revo3_cat_cradle.motion import RecordedMotion,TimeScaledMotion
from revo3_cat_cradle.contact_geometry import finger_capsule


def palm_crossings(points):
    # Revo3's palm is viewed along +X, so the image plane is YZ.
    q=points[:,1:];hits=[];n=len(q)
    def cross(a,b):return a[0]*b[1]-a[1]*b[0]
    for i in range(n):
        a=q[i];u=q[(i+1)%n]-a
        for j in range(i+3,n):
            if min(j-i,n-j+i)<=2:continue
            b=q[j];v=q[(j+1)%n]-b;denominator=cross(u,v)
            if abs(denominator)<1e-12:continue
            t=cross(b-a,v)/denominator;s=cross(b-a,u)/denominator
            if 0<t<1 and 0<s<1:
                hit=a+t*u
                if .075<hit[1]<.175 and abs(hit[0])<.07 and all(np.linalg.norm(hit-x)>.004 for x in hits):
                    hits.append(hit)
    return len(hits)


def finger_gaps(hand, joints, points, rope_radius):
    transforms=hand.fk(joints);result={}
    for finger in ('thumb','index','middle','ring','little'):
        best=float('inf')
        for segment in ('MCP','PIP','DIP'):
            center,axis,radius,half=finger_capsule(finger,segment,hand.side)
            tf=transforms[f'{hand.side}_{finger}_{segment}_link']
            local=(points-tf[:3,3])@tf[:3,:3]-center
            local-=np.clip(local@axis,-half,half)[:,None]*axis
            best=min(best,float((np.linalg.norm(local,axis=1)-radius-rope_radius).min()))
        result[finger]=best
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inverse',type=Path,default=ROOT/'output/inverse_speed_1_5')
    parser.add_argument('--forward',type=Path,default=ROOT/'output/speed_1_5')
    args=parser.parse_args()
    data=np.load(args.inverse/'trajectory.npz');forward=np.load(args.forward/'trajectory.npz')
    payload=json.loads((args.inverse/'trajectory.json').read_text());config=payload['config']
    hand=Hand();motion=TimeScaledMotion(RecordedMotion(hand,ROOT/'data/inverse_motion.json'),config['motion_speed'])
    np.testing.assert_allclose(data['joints'],[motion.sample(t)[0] for t in data['time']],atol=1e-5)
    np.testing.assert_array_equal(data['rope'][0],forward['rope'][-1])
    np.testing.assert_array_equal(data['joints'][0],forward['joints'][-1])
    np.testing.assert_array_equal(data['particle_masses'],forward['particle_masses'])
    assert data['anchors'].size==0 and np.all(data['particle_masses']>0)
    assert payload['interaction']=='contact_friction_only'
    assert np.isfinite(data['rope']).all() and payload['validation']['deformation_bounds_passed']
    assert data['time'][-1]>=motion.duration+1
    reference_index=np.argmin(abs(forward['time']-.5/config['motion_speed']))
    reference=forward['rope'][reference_index];final=data['rope'][-1]
    distance=np.r_[cKDTree(reference).query(final)[0],cKDTree(final).query(reference)[0]]
    contacts=finger_gaps(hand,data['joints'][-1],final,config['radius'])
    tail=np.flatnonzero(data['time']>=motion.duration)
    counts=[palm_crossings(data['rope'][i]) for i in tail]
    contact_ok=all(all(-.001<gap<.001 for gap in finger_gaps(hand,data['joints'][i],data['rope'][i],config['radius']).values()) for i in tail)
    passed=bool(palm_crossings(data['rope'][0])>=3 and max(counts)==0 and contact_ok
                and np.mean(distance)<.003 and np.quantile(distance,.95)<.005 and distance.max()<.007)
    report=dict(passed=passed,motion_speed=config['motion_speed'],command_seconds=motion.duration,
                simulated_seconds=float(data['time'][-1]),initial_state_continuity='exact',
                anchors=0,initial_palm_crossings=palm_crossings(data['rope'][0]),
                final_palm_crossings=counts[-1],entire_terminal_hold_clear=max(counts)==0,
                five_fingers_hold_contact=contact_ok,final_finger_gaps_m=contacts,
                symmetric_mean_distance_to_start_m=float(np.mean(distance)),
                p95_distance_to_start_m=float(np.quantile(distance,.95)),
                max_distance_to_start_m=float(distance.max()),
                max_stretch_ratio=payload['validation']['max_stretch_ratio'])
    (args.inverse/'inverse_validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    if not passed:raise RuntimeError('Inverse did not restore the reference loop; inspect the diagnostics')


if __name__=='__main__':main()
