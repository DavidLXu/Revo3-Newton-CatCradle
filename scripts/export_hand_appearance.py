"""Export RevoBench USD materials/UVs onto the demo's unchanged visual meshes.

Requires the optional usd-core package. Source hashes must match before UVs
are assigned; this exporter never imports USD joints or collision geometry.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdShade

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.hand import Hand, origin


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export(bundle, output):
    manifest = json.loads((bundle / 'manifest.json').read_text())
    stage = Usd.Stage.Open(str(bundle / 'hand_appearance.usdc'))
    records = {(item['link'], item['visual_index']): item for item in manifest['visuals']}
    hand = Hand()
    output.mkdir(parents=True, exist_ok=True)
    texture_dir = output / 'textures'
    texture_dir.mkdir(exist_ok=True)
    textures, materials = {}, {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Material):
            continue
        shader = UsdShade.Material(prim).ComputeSurfaceSource()[0]
        material = dict(name=prim.GetName(), inputs={})
        for value in shader.GetInputs():
            name = value.GetBaseName()
            if name not in ('diffuseColor', 'metallic', 'roughness', 'normal',
                            'emissiveColor', 'opacity', 'ior', 'clearcoat', 'clearcoatRoughness'):
                continue
            data = {}
            scalar = value.Get()
            if scalar is not None:
                data['value'] = float(scalar) if isinstance(scalar, (float, int)) else list(scalar)
            if value.HasConnectedSource():
                texture_shader, channel, _ = value.GetConnectedSource()
                texture_shader = UsdShade.Shader(texture_shader)
                if texture_shader.GetIdAttr().Get() != 'UsdUVTexture':
                    raise ValueError(f'Unsupported texture network: {texture_shader.GetPath()}')
                asset = texture_shader.GetInput('file').Get()
                source = Path(asset.resolvedPath) if asset.resolvedPath else bundle / asset.path
                if digest(source) != manifest['textures_sha256'][source.name]:
                    raise ValueError(f'Texture hash mismatch: {source}')
                shutil.copy2(source, texture_dir / source.name)
                textures[source.name] = digest(source)
                data.update(texture='textures/' + source.name, channel=str(channel),
                            color_space=str(texture_shader.GetInput('sourceColorSpace').Get()),
                            wrap_s=str(texture_shader.GetInput('wrapS').Get()),
                            wrap_t=str(texture_shader.GetInput('wrapT').Get()))
                # Three.js applies the standard tangent-space normal remap.
                if name == 'normal':
                    scale = texture_shader.GetInput('scale').Get()
                    bias = texture_shader.GetInput('bias').Get()
                    if not np.allclose(scale, (2, 2, 2, 1)) or not np.allclose(bias, (-1, -1, -1, 0)):
                        raise ValueError('Unsupported normal texture scale/bias')
            material['inputs'][name] = data
        materials[material['name']] = material
    chunks, offset, meshes, checks = [], 0, [], []

    def add_array(array):
        nonlocal offset
        array = np.asarray(array, dtype='<f4')
        if not np.isfinite(array).all():
            raise ValueError('Visual attribute has nonfinite values')
        result = dict(byte_offset=offset, length=array.size, item_size=array.shape[-1])
        raw = array.tobytes()
        chunks.append(raw)
        offset += len(raw)
        return result

    for link in hand.xml.findall('link'):
        name = link.get('name')
        matched_name = name.replace('plam', 'palm').replace('tubber', 'rubber')
        for index, visual in enumerate(link.findall('visual')):
            specification = visual.find('geometry/mesh')
            if specification is None:
                continue
            record = records[(matched_name, index)]
            source = hand.path.parent / specification.get('filename')
            source_hash = digest(source)
            if source_hash != record['source_visual_sha256']:
                raise ValueError(f'Visual mesh does not match the UV source: {name}/{index}')
            mesh = trimesh.load(source, force='mesh', process=False)
            mesh.merge_vertices()
            uv = UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(record['target'])).GetPrimvar('st')
            if uv.GetInterpolation() != UsdGeom.Tokens.faceVarying:
                raise ValueError(f'Expected face-varying UVs: {name}/{index}')
            coordinates = np.asarray(uv.ComputeFlattened(), dtype=float)
            if coordinates.shape != (mesh.faces.size, 2):
                raise ValueError(f'UV topology mismatch: {name}/{index}')
            mesh.apply_scale(np.fromstring(specification.get('scale', '1 1 1'), sep=' '))
            mesh.apply_transform(origin(visual.find('origin')))
            corners = mesh.faces.reshape(-1)
            meshes.append(dict(link=name, visual_index=index, material=record['material'],
                               attributes=dict(position=add_array(mesh.vertices[corners]),
                                               normal=add_array(mesh.vertex_normals[corners]),
                                               uv=add_array(coordinates))))
            checks.append(dict(link=name, visual_index=index, triangles=len(mesh.faces),
                               source_visual_sha256=source_hash, uv_count=len(coordinates)))
    binary = output / 'geometry.bin'
    binary.write_bytes(b''.join(chunks))
    provenance = dict(
        source='RevoBench revosim_bionic_silver appearance layer',
        upstream_repository=manifest['source_repository'], upstream_commit=manifest['source_commit'],
        layer_sha256=digest(bundle / 'hand_appearance.usdc'),
        urdf_sha256=digest(hand.path), geometry_sha256=digest(binary),
        textures_sha256=textures, visual_count=len(meshes),
        triangles=sum(record['triangles'] for record in checks), visual_checks=checks,
        geometry='Original demo URDF visuals at full resolution; exact source hashes checked before assigning USD UVs.',
        physics_changed=False, notes='The screen texture is the supplied static frame.',
    )
    (output / 'meshes.json').write_text(json.dumps(dict(
        format='revo3-textured-visuals-v1', buffer='geometry.bin', materials=materials,
        meshes=meshes, provenance=provenance), indent=2) + '\n')
    for filename in ('RevoSim.LICENSE', 'SOURCE_NOTICES.md'):
        shutil.copy2(bundle / filename, output / filename)
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps({key: provenance[key] for key in ('visual_count', 'triangles', 'physics_changed')}
                     | dict(textures=len(textures), output=str(output)), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True,
                        help='RevoBench assets/robots/revotron/appearance/revosim_bionic_silver')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/hand_appearance')
    args = parser.parse_args()
    export(args.bundle.resolve(), args.output.resolve())
