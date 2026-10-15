"""Read the official URDF: exact joint axes, limits, FK and visual geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/brainco-description/revo3_system/urdf/revo3_left.urdf"


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
        t[:3, :3] = Rotation.from_euler("xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return t


class Hand:
    def __init__(self, path=URDF):
        self.path = Path(path)
        self.side = "left" if "left" in self.path.stem else "right"
        if not self.path.exists():
            raise FileNotFoundError(f"Missing official model: {path}. Run scripts/setup.sh first.")
        self.xml = ET.parse(self.path).getroot()
        self.links = [x.get("name") for x in self.xml.findall("link")]
        self.joints = []
        for j in self.xml.findall("joint"):
            axis = j.find("axis")
            limit = j.find("limit")
            self.joints.append(dict(name=j.get("name"), kind=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                origin=origin(j.find("origin")),
                axis=np.fromstring(axis.get("xyz"), sep=" ") if axis is not None else np.zeros(3),
                limits=(float(limit.get("lower")), float(limit.get("upper"))) if limit is not None else (0, 0)))
        self.moving = [j for j in self.joints if j["kind"] == "revolute"]
        self.names = [j["name"] for j in self.moving]
        self.limits = np.array([j["limits"] for j in self.moving])

    def fk(self, q):
        """All URDF link poses, meters, z-up. The wrist sits on a 22 mm pedestal."""
        angles = dict(zip(self.names, q, strict=True))
        transforms = {"world": np.eye(4)}
        transforms["world"][2, 3] = 0.022
        pending = list(self.joints)
        while pending:
            ready = [j for j in pending if j["parent"] in transforms]
            if not ready:
                raise ValueError("URDF contains a disconnected or cyclic joint tree")
            for j in ready:
                delta = np.eye(4)
                if j["kind"] == "revolute":
                    delta[:3, :3] = Rotation.from_rotvec(j["axis"] * angles[j["name"]]).as_matrix()
                transforms[j["child"]] = transforms[j["parent"]] @ j["origin"] @ delta
                pending.remove(j)
        return transforms

    def visuals(self, max_faces=1400):
        import trimesh
        materials = {m.get("name"): [float(x) for x in m.find("color").get("rgba").split()][:3]
                     for m in self.xml.findall("material")}
        result = []
        for link in self.xml.findall("link"):
            for visual in link.findall("visual"):
                item = visual.find("geometry/mesh")
                if item is None:
                    continue
                mesh = trimesh.load_mesh(self.path.parent / item.get("filename"))
                if len(mesh.faces) > max_faces:
                    mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
                mesh.apply_scale(np.fromstring(item.get("scale", "1 1 1"), sep=" "))
                mesh.apply_transform(origin(visual.find("origin")))
                mat = visual.find("material")
                color = materials.get(mat.get("name"), [.8, .82, .85]) if mat is not None else [.8]*3
                result.append(dict(link=link.get("name"), color=color,
                    vertices=np.round(mesh.vertices, 7).tolist(), faces=mesh.faces.tolist()))
        return result
