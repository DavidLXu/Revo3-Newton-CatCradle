"""Settle the rope before recording one uninterrupted forward/inverse cycle."""
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.sim import Config, Simulation
from revo3_cat_cradle.motion import RecordedMotion, TimeScaledMotion
from revo3_cat_cradle.__main__ import export_run
from validate_inverse import finger_gaps, palm_crossings


def distances(a, b):
    # Sample the rope segments densely so sliding particles do not falsely
    # register as shape motion at half of the 3 mm particle spacing.
    u=np.arange(20)/20
    a=(a[:,None,:]*(1-u[None,:,None])+np.roll(a,-1,axis=0)[:,None,:]*u[None,:,None]).reshape(-1,3)
    b=(b[:,None,:]*(1-u[None,:,None])+np.roll(b,-1,axis=0)[:,None,:]*u[None,:,None]).reshape(-1,3)
    return np.r_[cKDTree(a).query(b)[0], cKDTree(b).query(a)[0]]


class CycleMotion:
    def __init__(self, forward, inverse, opening=.5, closing=3.):
        self.hand = forward.hand
        self.forward, self.inverse = forward, inverse
        self.opening, self.closing = opening, closing
        self.turn = opening + forward.duration
        self.finish = self.turn + inverse.duration
        self.duration = self.finish + closing
        np.testing.assert_allclose(forward.sample(forward.duration)[0], inverse.sample(0)[0], atol=1e-8)
        np.testing.assert_allclose(forward.sample(0)[0], inverse.sample(inverse.duration)[0], atol=1e-8)
        self.metadata = dict(source='forward_motion.json + inverse_motion.json',
            direction='循环', duration=self.duration, motion_speed=forward.speed,
            forward=forward.metadata, inverse=inverse.metadata,
            opening_hold_seconds=opening, terminal_hold_seconds=closing,
            forward_end_seconds=self.turn, inverse_end_seconds=self.finish,
            continuity='One physical run; no rope state reset at motion boundaries')

    def sample(self, t):
        if t < self.turn:
            return self.forward.sample(t-self.opening)
        return self.inverse.sample(t-self.turn)


def main():
    out = ROOT/'output/cycle_speed_1_5'
    out.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    sim = Simulation(Config(motion_speed=1.5))
    history = [sim.state.particle_q.numpy().copy()]
    settling = []
    # Geometry convergence tolerates tangential particle sliding. Require a
    # full second of quiet geometry, not a single coincident pair of samples.
    quiet = 0
    for i in range(1, 12*sim.config.fps+1):
        sim.advance(0, move=False)
        history.append(sim.state.particle_q.numpy().copy())
        if i >= 15:
            d = distances(history[-1], history[-16])
            quiet = quiet+1 if np.quantile(d,.95)<.00075 and d.max()<.0015 else 0
            settling.append(dict(time=i/30, p95_m=float(np.quantile(d,.95)), max_m=float(d.max())))
        if i%30 == 0:
            print('Settling', settling[-1], 'quiet frames', quiet, flush=True)
            sim.save_checkpoint(out/'preroll_checkpoint.npz')
            np.savez_compressed(out/'preroll.npz',rope=history)
        if i>=90 and quiet>=30:
            break
    if quiet<30:
        raise RuntimeError('Initial rope did not reach the geometry settling tolerance')
    sim.save_checkpoint(out/'settled_start.npz')
    forward = sim.motion
    inverse = TimeScaledMotion(RecordedMotion(sim.hand, ROOT/'data/inverse_motion.json'),1.5)
    sim.motion = CycleMotion(forward,inverse)
    sim.motion.metadata['preroll_seconds'] = i/30
    sim.motion.metadata['settling_criterion'] = 'Every frame for 1 s: 0.5 s shape drift p95 < 0.75 mm, max < 1.5 mm'
    sim.initial_positions = sim.state.particle_q.numpy().copy()
    frames, metrics = [sim.snapshot()], [sim.metrics()]
    for i in range(1,round(sim.motion.duration*30)+1):
        sim.advance(i/30)
        frames.append(sim.snapshot()); metrics.append(sim.metrics())
        if i%30==0:
            print(f'Cycle {sim.t:.1f}s | {sim.phase} | stretch {metrics[-1]["max_stretch_ratio"]:.3f}',flush=True)
        if i == round(sim.motion.turn*30):
            sim.save_checkpoint(out/'forward_end.npz')
    export_run(sim,frames,metrics,out,no_meshes=True,start=start)
    first, final = np.array(frames[0]['rope']), np.array(frames[-1]['rope'])
    d = distances(first,final)
    terminal_drift = distances(final,np.array(frames[-16]['rope']))
    star = np.array(frames[round(sim.motion.turn*30)]['rope'])
    report = dict(preroll_seconds=sim.motion.metadata['preroll_seconds'],settling=settling,
        start_crossings=palm_crossings(first),star_crossings=palm_crossings(star),end_crossings=palm_crossings(final),
        start_finger_gaps_m=finger_gaps(sim.hand,frames[0]['joints'],first,sim.config.radius),
        end_finger_gaps_m=finger_gaps(sim.hand,frames[-1]['joints'],final,sim.config.radius),
        seam_mean_m=float(d.mean()),seam_p95_m=float(np.quantile(d,.95)),seam_max_m=float(d.max()),
        terminal_half_second_shape_drift_m=dict(p95=float(np.quantile(terminal_drift,.95)),max=float(terminal_drift.max())),
        joint_seam_max_rad=float(np.max(np.abs(np.array(frames[0]['joints'])-frames[-1]['joints']))))
    (out/'cycle_validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='settling'},indent=2),flush=True)
    assert report['start_crossings']==report['end_crossings']==0
    assert report['star_crossings']>=3
    assert report['seam_p95_m']<.005 and report['seam_max_m']<.007
    assert all(abs(v)<.001 for v in report['end_finger_gaps_m'].values())
    assert terminal_drift.max()<.0015


if __name__ == '__main__':
    main()
