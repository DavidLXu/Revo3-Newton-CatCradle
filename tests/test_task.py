"""Kinematic validity and physical behavior, not just rendering smoke tests."""
import numpy as np
from revo3_cat_cradle.hand import Hand
from revo3_cat_cradle.motion import RecordedMotion, PickupMotion, TimeScaledMotion
import pytest
from revo3_cat_cradle.hand import ROOT
from revo3_cat_cradle.sim import Simulation, Config
from revo3_cat_cradle.rope import ROUTE, wrap_sites
from revo3_cat_cradle.contact_geometry import finger_capsule


def test_speed_scales_corrected_commands_and_velocity_in_physical_time():
    base = PickupMotion(RecordedMotion(Hand()))
    fast = TimeScaledMotion(base, 1.5)
    assert fast.duration == pytest.approx(11.5/1.5)
    for source_time in (0., .7, 4.13, 4.3, 4.65, 5., 11.5):
        np.testing.assert_allclose(fast.sample(source_time/1.5)[0], base.sample(source_time)[0], atol=1e-12)
    dt = 1e-5
    source_time = 4.5
    velocity = (base.sample(source_time+dt)[0]-base.sample(source_time-dt)[0])/(2*dt)
    fast_velocity = (fast.sample(source_time/1.5+dt)[0]-fast.sample(source_time/1.5-dt)[0])/(2*dt)
    assert np.linalg.norm(velocity) > .1
    np.testing.assert_allclose(fast_velocity, 1.5*velocity, atol=1e-6)
    for speed in (0., -1., np.inf, np.nan):
        with pytest.raises(ValueError, match='finite and positive'):
            TimeScaledMotion(base, speed)


def test_inverse_endpoints_match_forward_and_limit_changes_are_audited():
    hand=Hand()
    forward=RecordedMotion(hand)
    inverse=RecordedMotion(hand,ROOT/'data/inverse_motion.json')
    assert inverse.duration==12. and len(inverse.steps)==24
    np.testing.assert_array_equal(inverse.sample(0)[0],forward.sample(forward.duration)[0])
    np.testing.assert_array_equal(inverse.sample(inverse.duration)[0],forward.sample(0)[0])
    assert inverse.metadata['direction']=='反向'
    assert len(inverse.metadata['joint_limit_adjustments'])==4
    assert np.all(inverse.q>=hand.limits[:,0]) and np.all(inverse.q<=hand.limits[:,1])


def test_checkpoint_preserves_rope_velocity_and_subsequent_dynamics(tmp_path):
    continuous=Simulation(Config(motion_speed=1.5))
    for i in range(1,4):continuous.advance(i/30)
    assert np.linalg.norm(continuous.state.particle_qd.numpy()) > .001
    path=tmp_path/'state.npz'
    continuous.save_checkpoint(path)
    resumed=Simulation(Config(motion_speed=1.5))
    resumed.load_checkpoint(path)
    np.testing.assert_array_equal(resumed.state.particle_qd.numpy(),continuous.state.particle_qd.numpy())
    continuous.advance(4/30);resumed.advance(4/30)
    np.testing.assert_array_equal(resumed.state.particle_q.numpy(),continuous.state.particle_q.numpy())
    np.testing.assert_array_equal(resumed.rest_lengths,continuous.rest_lengths)


def test_initial_wrap_arcs_follow_contact_surfaces():
    assert ROUTE == ("thumb", "index", "middle", "ring", "little")
    for side in ("left", "right"):
        sites = wrap_sites(side)
        for i,finger in enumerate(ROUTE):
            points=np.array([p for _,p in sites[3*i:3*i+3]])
            center,axis,radius,_=finger_capsule(finger,"DIP",side)
            offset=points-center
            radial=offset-np.outer(offset@axis,axis)
            np.testing.assert_allclose(np.linalg.norm(radial,axis=1),radius+.00115,atol=1e-12)
            if finger=="thumb":
                assert radial[1,2] < 0
            elif finger=="middle":
                assert radial[1,0] > 0
            else:
                assert radial[1,0] < 0
    for (_,left),(_,right) in zip(wrap_sites("left"),wrap_sites("right")):
        np.testing.assert_allclose(left,right*np.array([1,-1,1]),atol=1e-12)


def test_joint_limits_and_motion_continuity():
    hand = Hand()
    assert len(hand.names) == 21
    motion = PickupMotion(RecordedMotion(hand))
    q = np.array([motion.sample(t)[0] for t in np.linspace(0,11.5,3451)])
    assert np.all(q >= hand.limits[:,0])
    assert np.all(q <= hand.limits[:,1])
    # Check continuity at joins directly; a velocity cap from the removed
    # four-second authored motion does not apply to the recorded sequence.
    for t in sorted(set(motion.base.ends) | set(motion.times) | {4.3}):
        eps=1e-6
        left=motion.sample(t-eps)[0]; center=motion.sample(t)[0]; right=motion.sample(t+eps)[0]
        np.testing.assert_allclose(left,right,atol=3e-5)
        np.testing.assert_allclose((center-left)/eps,(right-center)/eps,atol=.01)
    assert np.max(np.ptp(q,axis=0)) > .5
    for t in (0,.55,1.1,1.65,2.25,2.9,3.55,4):
        for tf in hand.fk(motion.sample(t)[0]).values():
            np.testing.assert_allclose(tf[:3,:3].T@tf[:3,:3],np.eye(3),atol=1e-12)


def test_free_loop_mass_rest_lengths_gravity_and_friction():
    sim = Simulation(Config(adaptive_length=False))
    n = sim.model.particle_count
    edges = sim.model.spring_indices.numpy().reshape(-1,2)[:n]
    assert set(map(tuple, edges)) == {(i,(i+1)%n) for i in range(n)}
    assert len(sim.anchor_indices) == 0
    assert np.all(sim.model.particle_mass.numpy() > 0)
    assert np.all(sim.model.particle_inv_mass.numpy() > 0)
    assert sim.model.particle_adhesion == 0
    assert sim.model.particle_cohesion == 0
    # In the palm view, the little→thumb and thumb→index legs must not
    # cross into a loop over the thumb. Swapping the thumb ends fails this.
    a,b,c,d=sim.initial_positions[sim.initial_wrap_indices[[14,0,2,3]]][:,1:]
    def side_of_line(start,end,point):
        u,v=end-start,point-start
        return u[0]*v[1]-u[1]*v[0]
    assert (side_of_line(a,b,c)*side_of_line(a,b,d) > 0
            or side_of_line(c,d,a)*side_of_line(c,d,b) > 0)
    # The seated wraps stay proximal to the official fingertip rubber bounds,
    # including after the one-time collision-clearance projection.
    for i,finger in enumerate(ROUTE):
        tf=sim.transforms[f'{sim.hand.side}_{finger}_DIP_link']
        points=sim.initial_positions[sim.initial_wrap_indices[3*i:3*i+3]]
        local=(points-tf[:3,3])@tf[:3,:3]
        if finger == 'thumb':
            assert np.all(local[:,1] > -.010)
        else:
            assert np.all(local[:,2] < .002)
    # The displayed initial loop is seated outside each collider and tensioned.
    assert 1.01 < sim.metrics()['mean_stretch_ratio'] < 1.2
    for finger in ROUTE:
        closest = float('inf')
        for segment in ('MCP','PIP','DIP'):
            center,axis,radius,half=finger_capsule(finger,segment,sim.hand.side)
            tf=sim.transforms[f'{sim.hand.side}_{finger}_{segment}_link']
            local=(sim.initial_positions-tf[:3,3])@tf[:3,:3]
            on_axis=center+np.clip((local-center)@axis,-half,half)[:,None]*axis
            gap=np.linalg.norm(local-on_axis,axis=1)-radius-sim.config.radius
            assert gap.min() > -1e-6
            closest=min(closest,gap.min())
        assert closest < .0003
    rest = sim.model.spring_rest_length.numpy().copy()
    np.testing.assert_allclose(rest[:n], sim.rest_lengths)
    np.testing.assert_allclose(rest[:n].sum(), .4417950868606567, atol=1e-7)
    for _ in range(6):
        sim.advance(0, move=False)
    assert sim.metrics()['max_coordinate_m'] < .5
    assert sim.contact_peak > 0
    assert sim.metrics()['initial_wrap_displacement_m'] > .001
    np.testing.assert_array_equal(sim.model.spring_rest_length.numpy(),rest)

    for config in (Config(gravity=0.,adaptive_length=False), Config(friction=0.,adaptive_length=False)):
        comparison = Simulation(config)
        for _ in range(6):
            comparison.advance(0, move=False)
        delta = sim.state.particle_q.numpy()-comparison.state.particle_q.numpy()
        assert np.max(np.linalg.norm(delta,axis=1)) > .0001


def test_auto_take_up_keeps_starting_palm_span_taut():
    sim=Simulation()
    initial=sim.rest_lengths.copy()
    for i in range(1,16):
        sim.advance(i/30)
    assert sim.rest_lengths.sum() < initial.sum()-.02
    assert sim.metrics()['max_stretch_ratio'] < 1.2
    np.testing.assert_allclose(sim.model.spring_rest_length.numpy()[:len(initial)],sim.rest_lengths)
    p=sim.state.particle_q.numpy()
    span=np.vstack((p[sim.initial_wrap_indices[14]:],p[0]))
    chord=span[-1]-span[0]
    u=(span-span[0])@chord/(chord@chord)
    deviation=np.linalg.norm(span-span[0]-u[:,None]*chord,axis=1)
    assert deviation.max() < .003
    assert len(sim.anchor_indices) == 0


def test_self_contact_excludes_strand_neighbors_but_not_distant_strands():
    import warp as wp
    from newton.solvers import SolverXPBD
    from revo3_cat_cradle.rope_solver import RopeSolverXPBD,rope_self_contacts
    wp.init()
    # Ordinary Newton solvers must retain their original contact kernel.
    assert SolverXPBD.step.__globals__['solve_particle_particle_contacts'] is not rope_self_contacts
    assert RopeSolverXPBD.step.__globals__['solve_particle_particle_contacts'] is rope_self_contacts
    points=np.zeros((8,3),dtype=np.float32)
    points[:,0]=np.arange(8)*.05
    points[1]=[.0005,0,0]
    points[7]=[-.0005,0,0]
    for remote_contact in (False,True):
        if remote_contact:
            points[4]=[0,.0005,0]
        x=wp.array(points,dtype=wp.vec3,device='cpu')
        grid=wp.HashGrid(16,16,16,device='cpu');grid.build(x,radius=.002)
        delta=wp.zeros(8,dtype=wp.vec3,device='cpu')
        wp.launch(rope_self_contacts,dim=8,inputs=[grid.id,x,wp.zeros_like(x),
            wp.ones(8,dtype=float,device='cpu'),wp.full(8,.001,dtype=float,device='cpu'),
            wp.ones(8,dtype=wp.int32,device='cpu'),.25,0.,.001,.001,.35],outputs=[delta],device='cpu')
        if remote_contact:
            assert delta.numpy()[0,1] < 0
            assert delta.numpy()[4,1] > 0
        else:
            np.testing.assert_array_equal(delta.numpy(),0)


def test_hand_cannot_drag_rope_without_contact():
    moving = Simulation(Config(gravity=0.))
    static = Simulation(Config(gravity=0.))
    for sim in (moving, static):
        # Place the entire rope away from every hand collider and the ground.
        detached = sim.initial_positions+np.array([.4,0,.5], dtype=np.float32)
        sim.state.particle_q.assign(detached)
        sim.next.particle_q.assign(detached)
    for t in (.7,.8,.9,1.):
        moving.advance(t)
        static.advance(0, move=False)
    assert moving.contact_peak == static.contact_peak == 0
    # Joint motion has no effect at a distance: no hidden tracking/attachment.
    np.testing.assert_allclose(moving.state.particle_q.numpy(),static.state.particle_q.numpy(),atol=1e-7)


def test_rotating_collider_velocity_is_at_its_center_of_mass():
    previous = np.eye(4)
    current = np.eye(4)
    current[:3,:3] = [[0,-1,0],[1,0,0],[0,0,1]]
    velocity = Simulation.body_velocity(current, previous, np.array([1.,0,0]), .5)
    np.testing.assert_allclose(velocity, [-2,2,0,0,0,np.pi], atol=1e-12)
    np.testing.assert_allclose(Simulation.body_velocity(current,current,np.array([1.,0,0]),.5),0)


@pytest.mark.integration
def test_full_export_regression():
    """Check the complete recorded replay, including every transition."""
    import json
    from revo3_cat_cradle.hand import ROOT
    data = np.load(ROOT/'output/trajectory.npz')
    p = data['rope']
    assert p.shape[0] == 346
    assert data['time'][-1] == 11.5
    np.testing.assert_array_equal(p[0],data['initial_rope'])
    motion = PickupMotion(RecordedMotion(Hand()))
    expected = np.array([motion.sample(t)[0] for t in data['time']])
    np.testing.assert_allclose(data['joints'],expected,atol=1e-5)
    assert np.isfinite(p).all()
    assert np.isfinite(data['joints']).all()
    assert np.abs(p).max() < .5
    assert p[:,:,2].min() > 0
    ratio = np.linalg.norm(np.roll(p,-1,axis=1)-p,axis=2)/data['rest_lengths']
    assert ratio.max() < 1.2
    assert data['rest_lengths'].shape == p.shape[:2]
    assert np.ptp(data['rest_lengths'].sum(axis=1)) > .02
    summary = json.loads((ROOT/'output/metrics.json').read_text())
    assert summary['anchors'] == 0
    assert data['anchors'].size == 0
    assert np.all(data['particle_masses'] > 0)
    assert summary['interaction'] == 'contact_friction_only'
    assert summary['peak_sampled_soft_contacts'] > 0


def test_little_insertion_is_local_smooth_and_clear_of_ring_finger():
    from scipy.optimize import minimize
    hand=Hand()
    base=RecordedMotion(hand)
    motion=PickupMotion(base)
    times=np.linspace(0,11.5,2301)
    q=np.array([motion.sample(t)[0] for t in times])
    original=np.array([base.sample(t)[0] for t in times])
    others=[i for i in range(21) if i not in motion.indices+motion.thumb_indices]
    np.testing.assert_array_equal(q[:,others],original[:,others])
    thumb_outside=(times<=4.)|(times>=4.5)
    np.testing.assert_array_equal(q[thumb_outside][:,motion.thumb_indices],
                                  original[thumb_outside][:,motion.thumb_indices])
    for t in (4.05,4.15,4.25,4.3,4.4):
        u=np.clip((t-4.)/.3,0,1)
        source_time=4.+.5*u**3*(10-15*u+6*u*u)
        np.testing.assert_allclose(motion.sample(t)[0][motion.thumb_indices],
                                  base.sample(source_time)[0][motion.thumb_indices])
    outside=(times<=4.)|(times>=5.5)
    np.testing.assert_array_equal(q[outside],original[outside])
    assert np.all(q>=hand.limits[:,0]) and np.all(q<=hand.limits[:,1])
    for t,_ in motion.knots:
        np.testing.assert_allclose(motion.sample(t-1e-7)[0][motion.indices],
                                   motion.sample(t+1e-7)[0][motion.indices],atol=1e-8)
    # Hand bodies are kinematic: explicitly guard against placing two fingers
    # through each other while optimizing a rope insertion.
    for t in np.linspace(4.,5.5,31):
        transforms=hand.fk(motion.sample(t)[0])
        capsules={}
        for finger in ('little','ring','thumb'):
            for segment in ('MCP','PIP','DIP'):
                c,axis,r,half=finger_capsule(finger,segment)
                tf=transforms[f'left_{finger}_{segment}_link']
                endpoints=np.array([(tf@np.r_[c-axis*half,1])[:3],(tf@np.r_[c+axis*half,1])[:3]])
                capsules[finger,segment]=(endpoints,r)
        for a in ('MCP','PIP','DIP'):
            for other,b in [(f,b) for f in ('ring','thumb') for b in ('MCP','PIP','DIP')]:
                aa,ra=capsules['little',a];bb,rb=capsules[other,b]
                def distance_squared(u):
                    return np.linalg.norm(aa[0]+u[0]*(aa[1]-aa[0])-bb[0]-u[1]*(bb[1]-bb[0]))**2
                result=minimize(distance_squared,[.5,.5],bounds=[(0,1),(0,1)],method='L-BFGS-B',options={'gtol':1e-12})
                assert np.sqrt(result.fun)-ra-rb > (.001 if other=='ring' else .0005)


@pytest.mark.integration
def test_full_export_little_finger_picks_middle_to_thumb_strand():
    data=np.load(ROOT/'output/trajectory.npz')
    hand=Hand()
    # Material particles on the steep middle-to-thumb free span at 4.13 s.
    # The other strand originally around the little finger is near 112–120;
    # contact with that strand alone must not count as the requested pickup.
    strand=np.r_[175:180,0:20]
    frames=np.flatnonzero((data['time']>=4.6)&(data['time']<=5.2))
    center,axis,radius,half=finger_capsule('little','DIP')
    for i in frames:
        tf=hand.fk(data['joints'][i])['left_little_DIP_link']
        local=(data['rope'][i,strand]-tf[:3,3])@tf[:3,:3]-center
        local-=np.clip(local@axis,-half,half)[:,None]*axis
        gap=np.linalg.norm(local,axis=1)-radius-.001
        assert -.0005 < gap.min() < .001
    before=np.argmin(abs(data['time']-4.35))
    raised=np.argmin(abs(data['time']-5.))
    # Follow the same material points, not whichever part of the loop happens
    # to be highest after a different strand is lifted.
    assert np.median(data['rope'][raised,5:16,2]-data['rope'][before,5:16,2]) > .025


def test_named_recording_targets_and_order():
    import json
    motion=RecordedMotion(Hand())
    raw=json.loads((ROOT/'data/forward_motion.json').read_text())
    assert len(motion.steps)==23 and motion.duration==11.5
    np.testing.assert_array_equal(motion.q, [f['positions'] for f in raw['frames']])
    np.testing.assert_array_equal(motion.sample(.25)[0], motion.q[0])
    for i,t in enumerate(motion.ends):
        np.testing.assert_allclose(motion.sample(t)[0],motion.q[i],atol=1e-12)
        np.testing.assert_allclose(motion.sample(t-1e-7)[0],motion.sample(t+1e-7)[0],atol=1e-9)


def test_invalid_motion_rejected(tmp_path):
    import json
    raw=json.loads((ROOT/'data/forward_motion.json').read_text())
    raw['frames'][0]['positions'][0]=100.
    path=tmp_path/'invalid.json';path.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match='exceeds URDF joint limits'):
        RecordedMotion(Hand(),path)
