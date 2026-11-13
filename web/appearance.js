import * as THREE from 'three';

// Interpret the USD PreviewSurface textures exported by export_hand_appearance.py.
// This module creates visual meshes only; link transforms still come from Newton.
export async function texturedVisuals(asset, assetURL, renderer) {
  if (asset.format !== 'revo3-textured-visuals-v1') throw new Error('Unsupported visual asset format');
  const resolve = path => new URL(path, assetURL).href;
  const response = await fetch(resolve(asset.buffer));
  if (!response.ok) throw new Error(`Visual geometry: ${response.status}`);
  const buffer = await response.arrayBuffer();
  const loader = new THREE.TextureLoader(), textures = new Map();
  async function texture(spec) {
    const key = `${spec.texture}:${spec.color_space}`;
    if (!textures.has(key)) textures.set(key, loader.loadAsync(resolve(spec.texture)).then(map => {
      map.colorSpace = spec.color_space === 'sRGB' ? THREE.SRGBColorSpace : THREE.NoColorSpace;
      map.wrapS = spec.wrap_s === 'repeat' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
      map.wrapT = spec.wrap_t === 'repeat' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
      map.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
      return map;
    }));
    return textures.get(key);
  }
  const materials = new Map();
  await Promise.all(Object.entries(asset.materials).map(async ([name, definition]) => {
    const values = definition.inputs;
    const options = {name, color:0xffffff, metalness:0, roughness:.5};
    const slots = {diffuseColor:'map', metallic:'metalnessMap', roughness:'roughnessMap',
                   normal:'normalMap', emissiveColor:'emissiveMap'};
    for (const [input, slot] of Object.entries(slots)) {
      const spec = values[input];
      if (!spec) continue;
      if (spec.texture) {
        options[slot] = await texture(spec);
        if (input === 'metallic') options.metalness = 1;
        if (input === 'roughness') options.roughness = 1;
        if (input === 'emissiveColor') options.emissive = 0xffffff;
      } else if (spec.value !== undefined) {
        if (input === 'diffuseColor') options.color = new THREE.Color(...spec.value);
        if (input === 'emissiveColor') options.emissive = new THREE.Color(...spec.value);
        if (input === 'metallic') options.metalness = spec.value;
        if (input === 'roughness') options.roughness = spec.value;
      }
    }
    for (const name of ['ior','clearcoat','clearcoatRoughness','opacity']) {
      if (values[name]?.value !== undefined) options[name] = values[name].value;
    }
    if (options.opacity < 1) options.transparent = true;
    materials.set(name, new THREE.MeshPhysicalMaterial(options));
  }));
  return asset.meshes.map(item => {
    const geometry = new THREE.BufferGeometry();
    for (const [name, field] of Object.entries(item.attributes)) {
      geometry.setAttribute(name, new THREE.BufferAttribute(
        new Float32Array(buffer, field.byte_offset, field.length), field.item_size));
    }
    const mesh = new THREE.Mesh(geometry, materials.get(item.material));
    mesh.castShadow = true; mesh.receiveShadow = true;
    return {link:item.link, mesh};
  });
}
