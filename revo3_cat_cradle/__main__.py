import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np
from .sim import Config, Simulation
from .hand import ROOT


def main():
    parser = argparse.ArgumentParser(description="Hardcoded Revo3 + Newton XPBD cat's cradle")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--motion-speed", type=float, default=1.,
                        help="Joint command speed during physics integration; not video playback speed")
    parser.add_argument("--direction", choices=("forward","inverse"), default="forward")
    parser.add_argument("--seed-state", type=Path, default=None,
                        help="Forward-end checkpoint to seed inverse physics; computed if missing")
    parser.add_argument("--checkpoint-every", type=int, default=0, help="Save physical state every N frames")
    parser.add_argument("--original-joints", action="store_true", help="Disable the simulation-only little-finger pickup and thumb timing correction")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--hold-after", type=float, default=0.,
                        help="Continue physics with the final joint pose for this many seconds")
    parser.add_argument("--substeps", type=int, default=96)
    parser.add_argument("--iterations", type=int, default=Config.iterations)
    parser.add_argument("--stretch-stiffness", type=float, default=Config.stretch_stiffness)
    parser.add_argument("--fixed-length", action="store_true", help="Disable active rope take-up/payout")
    parser.add_argument("--take-up", type=float, default=Config.adaptive_take_up_m,
                        help="Extra loop-wide shortening of the automatic length target, in meters (default 0.01)")
    parser.add_argument("--friction", type=float, default=.65,
                        help="Particle/hand contact friction coefficient; 0 disables tangential friction")
    parser.add_argument("--rope-length", type=float, default=Config.rope_length_m,
                        help="Initial material length budget in meters; held constant with --fixed-length")
    parser.add_argument("--output", type=Path, default=ROOT/"output")
    parser.add_argument("--no-meshes", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("duration must be positive")
    if not np.isfinite(args.hold_after) or args.hold_after<0:
        parser.error("hold-after must be finite and nonnegative")
    start = time.perf_counter()
    sim = Simulation(Config(fps=args.fps, device=args.device, motion_speed=args.motion_speed,
        pickup_correction=not args.original_joints, friction=args.friction,
        adaptive_length=not args.fixed_length, adaptive_take_up_m=args.take_up,
        substeps=args.substeps, iterations=args.iterations,
        stretch_stiffness=args.stretch_stiffness, rope_length_m=args.rope_length))
    if args.direction == "inverse":
        from .motion import RecordedMotion, TimeScaledMotion
        checkpoint=args.seed_state or args.output/"forward_seed.npz"
        if checkpoint.exists():
            sim.load_checkpoint(checkpoint)
            print(f"Loaded forward-end physical state: {checkpoint}",flush=True)
        else:
            print("Preparing forward-end rope state with full physics...",flush=True)
            for i in range(1,round(sim.motion.duration*args.fps)+1):
                sim.advance(i/args.fps)
                if i%args.fps==0:print(f"Prepare {sim.t:.1f}s",flush=True)
            sim.save_checkpoint(checkpoint)
        inverse=TimeScaledMotion(RecordedMotion(sim.hand,ROOT/'data/inverse_motion.json'),args.motion_speed)
        np.testing.assert_allclose(sim.q,inverse.sample(0)[0],atol=1e-6,
                                   err_msg="Inverse first pose must match the physical seed")
        inverse.metadata['initial_state']='Continuous forward-end physics checkpoint; rope velocities retained'
        sim.motion=inverse
        sim.t=0.;sim.phase=inverse.sample(0)[1]
        sim.initial_positions=sim.state.particle_q.numpy().copy()
        sim.contact_peak=0
    maximum_duration=sim.motion.duration+args.hold_after
    duration = maximum_duration if args.duration is None else args.duration
    if duration > maximum_duration:
        parser.error(f"duration must be <= motion plus hold duration ({maximum_duration}s)")
    if args.hold_after:
        sim.motion.metadata['terminal_hold_seconds']=args.hold_after
    frames, metrics = [sim.snapshot()], [sim.metrics()]
    for i in range(1, round(duration*args.fps)+1):
        sim.advance(i/args.fps)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        if args.checkpoint_every and i%args.checkpoint_every==0:
            sim.save_checkpoint(args.output/'checkpoints'/f'{i:04d}.npz')
        if i % args.fps == 0:
            print(f"{sim.t:.1f}s | {sim.phase} | stretch {metrics[-1]['max_stretch_ratio']:.3f}", flush=True)
    export_run(sim,frames,metrics,args.output,no_meshes=args.no_meshes,start=start)


def export_run(sim, frames, metrics, output, *, no_meshes=False, start=None):
    output=Path(output)
    output.mkdir(parents=True, exist_ok=True)
    sim.save_checkpoint(output/"checkpoint.npz")
    payload = dict(config=asdict(sim.config), links=sim.hand.links, joint_names=sim.hand.names,
        rope_rest_length_m=float(sim.initial_rest_lengths.sum()),
        length_control="active_strain_feedback" if sim.config.adaptive_length else "fixed",
        anchor_indices=sim.anchor_indices.tolist(), frames=frames,
        motion=sim.motion.metadata,
        interaction="contact_friction_only", initial_wrap_indices=sim.initial_wrap_indices.tolist(),
        description="All rope particles are dynamic. The kinematic hand interacts through collision and friction only; no attachments or adhesion.")
    payload['validation'] = dict(
        deformation_bounds_passed=max(m['max_stretch_ratio'] for m in metrics) <= 1.2
            and max(m['max_coordinate_m'] for m in metrics) <= .6,
        max_stretch_ratio=max(m['max_stretch_ratio'] for m in metrics))
    (output/"motion_audit.json").write_text(json.dumps(sim.motion.metadata,ensure_ascii=False,indent=2),encoding="utf-8")
    (output/"trajectory.json").write_text(json.dumps(payload, separators=(",",":")), encoding="utf-8")
    np.savez_compressed(output/"trajectory.npz", time=[f["time"] for f in frames],
        joints=[f["joints"] for f in frames], rope=[f["rope"] for f in frames],
        rest_lengths=[f['rest_lengths'] for f in frames], anchors=sim.anchor_indices,
        initial_rest_lengths=sim.initial_rest_lengths,
        particle_masses=sim.model.particle_mass.numpy(), initial_wrap_indices=sim.initial_wrap_indices,
        initial_rope=sim.initial_positions)
    if not no_meshes:
        (output/"meshes.json").write_text(json.dumps(sim.hand.visuals(),separators=(",",":")))
    summary = dict(particles=sim.model.particle_count, anchors=len(sim.anchor_indices),
        rope_rest_length_m=float(sim.rest_lengths.sum()),
        initial_rope_rest_length_m=float(sim.initial_rest_lengths.sum()),
        min_rope_rest_length_m=min(m['rope_rest_length_m'] for m in metrics),
        max_rope_rest_length_m=max(m['rope_rest_length_m'] for m in metrics),
        length_control=payload['length_control'],
        frames=len(frames), simulated_seconds=sim.t,
        wall_seconds=None if start is None else round(time.perf_counter()-start,2),
        max_stretch_ratio=max(m["max_stretch_ratio"] for m in metrics),
        interaction="contact_friction_only", friction=sim.config.friction,
        min_height_m=min(m["min_height_m"] for m in metrics),
        max_coordinate_m=max(m["max_coordinate_m"] for m in metrics),
        peak_sampled_soft_contacts=sim.contact_peak, per_frame=metrics)
    (output/"metrics.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k != "per_frame"},indent=2))
    if summary["max_stretch_ratio"] > 1.2 or summary["max_coordinate_m"] > .6:
        raise RuntimeError("Simulation failed deformation bounds; inspect metrics.json")


if __name__ == "__main__":
    main()
