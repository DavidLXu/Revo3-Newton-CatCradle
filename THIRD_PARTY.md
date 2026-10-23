# Third-party components

| Component | Revision | License / distribution |
| --- | --- | --- |
| [Newton](https://github.com/newton-physics/newton) | `4a9a27a7a3419c64201ca1fb0c3660c4a0f9ac58` | Apache-2.0; fetched by setup, not bundled. The adapted particle-contact kernel in `rope_solver.py` retains its upstream copyright and records the modification. |
| [BrainCo description](https://github.com/BrainCoTech/brainco-description) | `6db7e2ed1d864ee0a3ab6ff62d43a303787137f6` | No standalone model license found in this pinned tree. Fetched separately; URDFs, meshes and generated mesh exports are not committed or covered by the project's license. |
| [Three.js](https://github.com/mrdoob/three.js/tree/r170) | r170 / 0.170.0 | MIT; `three.module.js`, `OrbitControls.js` and `THREE-LICENSE.txt` are included under `web/`. |
| [BrainCo RevoTron bimanual assembly](https://github.com/BrainCoTech/brainco-description/tree/bf114a1b421ca00b80f517f5e2b074591940ddc0) | `bf114a1b421ca00b80f517f5e2b074591940ddc0` | Optional download via `scripts/setup_revotron.sh`; `revotron_system/urdf/revotron_bimanual_revo3.urdf` and its Revo3 meshes remain external assets. No model redistribution or relicensing. |

The joint-name ordering was checked against [RevoLab's deployment mapping](https://github.com/BrainCoTech/RevoLab/blob/bb3f6f32b1e55d870b399aa54f6cda7ac8b7519f/deploy/revo3/config/revo3_right.yaml) and [the SDK joint map](https://github.com/BrainCoTech/brainco-revo3-sdk/blob/3fcf173ac8e0c020ba9cc48f4a74c7459d074a93/python/revo3/revo3_joint_map.md). Those repositories are not copied into this project.

Python packages are installed from the pinned `requirements.lock` under their respective licenses. `docs/preview.png` shows a render of the external BrainCo model and the simulated rope; it does not include reusable model meshes. Original reference media and hardware calibration records are not distributed.
