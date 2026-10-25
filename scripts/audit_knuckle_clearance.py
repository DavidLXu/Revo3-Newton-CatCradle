"""Audit rope centers against the actual, concave URDF MPR collision meshes.

Uses triangle winding numbers rather than convex-hull containment. This audits
one checkpoint, not continuous collision avoidance or the figure topology.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.hand import origin
from revo3_cat_cradle.mesh_geometry import signed_surface_clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    saved = np.load(args.checkpoint)
    robot = RevoTron()
    transforms = robot.fk(saved["joints"])
    radius = json.loads(str(saved["config"]))["radius"]
    rows = []
    for side in ("left", "right"):
        for finger in ("index", "middle", "ring", "little"):
            name = f"{side}_{finger}_MPR_link"
            tf = transforms[name]
            points = (saved["particle_q"] - tf[:3, 3]) @ tf[:3, :3]
            link = robot.xml.find(f"link[@name='{name}']")
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                surface = trimesh.load_mesh(robot.path.parent / mesh.get("filename"))
                surface.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
                surface.apply_transform(origin(collision.find("origin")))
                ids = np.flatnonzero(
                    np.all(
                        (points >= surface.bounds[0] - radius)
                        & (points <= surface.bounds[1] + radius),
                        axis=1,
                    )
                )
                signed = (
                    signed_surface_clearance(surface, points[ids])
                    if len(ids)
                    else np.array([])
                )
                rows.append(
                    dict(
                        link=name,
                        particle_indices=ids.tolist(),
                        surface_clearance_m=signed.tolist(),
                        rope_clearance_m=(signed - radius).tolist(),
                        inside_particle_indices=ids[signed < -1e-7].tolist(),
                    )
                )
    report = dict(
        checkpoint=str(args.checkpoint),
        time=float(saved["time"]),
        scope="Checkpoint only; actual URDF MPR triangles, winding-number interior test",
        inside_count=sum(len(row["inside_particle_indices"]) for row in rows),
        minimum_rope_clearance_m=min(
            (v for row in rows for v in row["rope_clearance_m"]), default=None
        ),
        surfaces=rows,
    )
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "surfaces"}))


if __name__ == "__main__":
    main()
