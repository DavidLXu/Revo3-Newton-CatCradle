"""Named joint commands and a local, hardcoded strand-pickup correction."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .hand import ROOT


class RecordedMotion:
    """Replay normalized URDF angles; no calibration or hardware interface."""
    def __init__(self, hand, path=ROOT/'data/forward_motion.json'):
        self.hand = hand
        path = Path(path)
        raw = json.loads(path.read_text(encoding='utf-8'))
        if raw.get('schema_version') != 1 or raw.get('angle_unit') != 'radian':
            raise ValueError('Expected schema_version=1 and angle_unit=radian')
        names = raw['joint_names']
        if len(names) != len(set(names)) or set(names) != set(hand.names):
            raise ValueError('Joint names must match the hand exactly')
        self.steps = raw['frames']
        if not self.steps:
            raise ValueError('Motion must contain at least one frame')
        positions = np.asarray([f['positions'] for f in self.steps], dtype=float)
        durations = np.asarray([f['duration_s'] for f in self.steps], dtype=float)
        if positions.shape != (len(self.steps), len(names)):
            raise ValueError('Each frame must contain one angle per joint')
        if not np.isfinite(positions).all() or not np.isfinite(durations).all() or np.any(durations <= 0):
            raise ValueError('Angles must be finite and durations positive')
        self.q = positions[:, [names.index(n) for n in hand.names]]
        if np.any(self.q < hand.limits[:, 0]) or np.any(self.q > hand.limits[:, 1]):
            raise ValueError('Motion exceeds URDF joint limits')
        self.ends = np.cumsum(durations)
        self.starts = np.r_[0., self.ends[:-1]]
        self.duration = float(self.ends[-1])
        self.direction=raw.get('direction','正向')
        self.metadata = dict(source=path.name, name=raw['name'], direction=self.direction,
            duration=self.duration, command_count=len(self.steps),
            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_kind=raw.get('source_kind','command_targets'),
            normalization=raw.get('normalization','none'),
            interpolation='quintic rest-to-rest; first target held for its duration')
        if 'joint_limit_adjustments' in raw:
            self.metadata['joint_limit_adjustments']=raw['joint_limit_adjustments']

    def sample(self, t):
        t = float(np.clip(t, 0, self.duration))
        i = min(int(np.searchsorted(self.ends,t,side='right')), len(self.steps)-1)
        u = np.clip((t-self.starts[i])/(self.ends[i]-self.starts[i]),0,1)
        u = u*u*u*(10+u*(-15+6*u))
        return (1-u)*self.q[max(0,i-1)]+u*self.q[i], f'{self.direction}翻 · {i+1:02d}/{len(self.steps)}'


class TimeScaledMotion:
    """Speed up kinematic commands while physics continues in real seconds."""
    def __init__(self, base, speed):
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError('motion speed must be finite and positive')
        self.base, self.hand, self.speed = base, base.hand, float(speed)
        self.duration = base.duration / self.speed
        self.metadata = dict(base.metadata, duration=self.duration,
            motion_speed=self.speed, source_duration=base.duration,
            timing='source_time = simulation_time * motion_speed; adjustment timestamps use source time')

    def sample(self, t):
        return self.base.sample(float(np.clip(t, 0, self.duration)) * self.speed)


class PickupMotion:
    """Scoop the middle-to-thumb strand; the supplied command file is intact."""
    # Absolute URDF joint angles, degrees: MPR, MCP, PIP, DIP.
    knots = (
        (4.00, None),
        (4.13, (-17., 85., 90., 90.)),
        (4.35, (-17., 85., 90., 90.)),
        (4.65, (-17., 45., 70., 30.)),
        (5.00, (-20., 40., 40., 0.4)),
        (5.50, None),
    )

    def __init__(self, base):
        self.base=base
        self.hand=base.hand
        self.duration=base.duration
        self.indices=[self.hand.names.index(f'{self.hand.side}_little_{j}_joint')
                      for j in ('MPR','MCP','PIP','DIP')]
        self.thumb_indices=[i for i,n in enumerate(self.hand.names) if '_thumb_' in n]
        self.times=np.array([t for t,_ in self.knots])
        self.targets=[]
        for t,angles in self.knots:
            original=base.sample(t)[0][self.indices]
            target=original if angles is None else np.deg2rad(angles)
            limits=self.hand.limits[self.indices]
            if np.any(target < limits[:,0]) or np.any(target > limits[:,1]):
                raise ValueError('Little-finger insertion pose exceeds joint limits')
            self.targets.append(target)
        self.targets=np.array(self.targets)
        self.metadata=dict(base.metadata)
        self.metadata['simulation_joint_adjustments']={
            'purpose':'Pick up the middle-to-thumb strand: tuck little finger, deliver rope, then extend',
            'joints':[self.hand.names[i] for i in self.indices],
            'knots':[{'time_s':t,'target_degrees':angles} for t,angles in self.knots],
            'blend':'quintic joint poses; zero-velocity joins to original motion at 4.0 and 5.5 s',
            'thumb_retiming': {
                'joints':[self.hand.names[i] for i in self.thumb_indices],
                'window_s':[4.0,4.5], 'delivery_complete_s':4.3,
                'source_window_s':[4.0,4.5],
                'description':'Replay the same thumb poses earlier with quintic time mapping; hold the 4.5 s pose from 4.3 to 4.5 s',
            },
        }

    def sample(self,t):
        q,phase=self.base.sample(t)
        if self.times[0] < t < self.times[-1]:
            i=min(np.searchsorted(self.times,t,side='right')-1,len(self.times)-2)
            u=(t-self.times[i])/(self.times[i+1]-self.times[i])
            u=u*u*u*(10+u*(-15+6*u))
            q[self.indices]=(1-u)*self.targets[i]+u*self.targets[i+1]
        # Deliver the strand while the little fingertip is still tucked below
        # it. This changes joint timing only, never rope positions or contacts.
        if 4.0 < t < 4.5:
            u=float(np.clip((t-4.0)/.3,0.,1.))
            u=u*u*u*(10+u*(-15+6*u))
            q[self.thumb_indices]=self.base.sample(4.0+.5*u)[0][self.thumb_indices]
        return q,phase
