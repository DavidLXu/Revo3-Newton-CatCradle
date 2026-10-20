import * as THREE from 'three';

// Rendering only: the Newton particle positions and hand poses are unchanged.
export function studio(scene, renderer, camera, controls) {
  document.body.classList.add('cinematic');
  const style = document.createElement('style');
  style.textContent = `.cinematic header,.cinematic .badge,.cinematic #readout,
    .cinematic .hint,.cinematic .controls {display:none}
    .film-title{position:absolute;left:64px;top:56px;color:#343d43;pointer-events:none}
    .film-title small{font-size:24px;letter-spacing:5px;color:#68777d}
    .film-title h2{font-size:44px;font-weight:400;letter-spacing:1px;margin:14px 0}
    .film-title .english{font-size:34px;font-weight:400;letter-spacing:.5px;color:#68777d}
    .film-note{position:absolute;bottom:42px;left:64px;font-size:22px;letter-spacing:2px;color:#7e898b}`;
  document.head.appendChild(style);
  const title = document.createElement('div');
  title.className = 'film-title';
  title.innerHTML = `<small>Revo3</small><h2>指间翻花绳</h2><div class="english" lang="en">Cat's Cradle</div>`;
  document.body.appendChild(title);
  const note = document.createElement('div');
  note.className = 'film-note';
  note.textContent = 'Newton physics';
  document.body.appendChild(note);
  if(new URLSearchParams(location.search).has('stacked')) {
    title.style.display='none'; note.style.display='none';
  }

  scene.background = new THREE.Color('#dce1e0');
  scene.fog = new THREE.Fog('#dce1e0', .8, 2.4);
  renderer.toneMappingExposure = .85;
  renderer.shadowMap.type = THREE.VSMShadowMap;
  for (const object of [...scene.children]) if (object.isLight) scene.remove(object);
  scene.add(new THREE.HemisphereLight('#eaf2ff', '#9e9485', .8));

  // Large luminous panels provide smooth reflections on the metal housing.
  const environment = new THREE.Scene();
  environment.background = new THREE.Color('#525c67');
  function panel(position, size, color, intensity) {
    const material = new THREE.MeshBasicMaterial({color, side:THREE.DoubleSide});
    material.color.multiplyScalar(intensity);
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(...size), material);
    mesh.position.set(...position); mesh.lookAt(0,0,0); environment.add(mesh);
  }
  panel([2,-2,3], [3,4], '#fff4e4', 4);
  panel([1,3,1], [2,4], '#e6f0ff', 2.5);
  panel([-2,0,2], [1,3], '#ffffff', 4);
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(environment, .06).texture;
  scene.environmentIntensity = .75;
  pmrem.dispose();
  environment.traverse(object => {object.geometry?.dispose();object.material?.dispose();});

  function light(position, color, intensity, shadow=false) {
    const source = new THREE.DirectionalLight(color, intensity);
    source.position.set(...position); source.target.position.set(0,0,.12);
    source.castShadow = shadow;
    if (shadow) {
      source.shadow.mapSize.set(2048,2048);
      Object.assign(source.shadow.camera, {left:-.22,right:.22,top:.30,bottom:-.18,near:.01,far:2});
      source.shadow.bias = -.00008; source.shadow.normalBias = .0005;
      source.shadow.radius = 4; source.shadow.blurSamples = 8;
    }
    scene.add(source, source.target);
  }
  light([.3,-.32,.62], '#fff2df', 2.0, true);
  light([.25,.42,.32], '#dceaff', 1.1);
  light([-.3,.05,.5], '#ffffff', 2.2);
  // Matte floor with a subtle contact shadow beneath the pedestal.
  const floor = scene.children.find(x => x.isMesh && x.geometry.type === 'BoxGeometry' && x.geometry.parameters.width === 3);
  floor.material.color.set('#bcc6c5'); floor.material.roughness = .95; floor.material.metalness = 0;
  const canvas = document.createElement('canvas'); canvas.width = canvas.height = 256;
  const context = canvas.getContext('2d');
  const gradient = context.createRadialGradient(128,128,20,128,128,128);
  gradient.addColorStop(0,'rgba(25,32,34,.32)'); gradient.addColorStop(.4,'rgba(25,32,34,.18)'); gradient.addColorStop(1,'rgba(25,32,34,0)');
  context.fillStyle = gradient; context.fillRect(0,0,256,256);
  const contact = new THREE.Mesh(new THREE.PlaneGeometry(.25,.25),new THREE.MeshBasicMaterial({map:new THREE.CanvasTexture(canvas),transparent:true,depthWrite:false}));
  contact.position.z = -.0008; scene.add(contact);
  controls.enableDamping = false;
  camera.fov = 34; camera.updateProjectionMatrix();
  return (progress, azimuth) => {
    // Constant-speed −15° to +15° front orbit, with gentle vertical elevation.
    const theta = THREE.MathUtils.degToRad(azimuth ?? (-15 + 30 * Math.max(0,Math.min(1,progress))));
    controls.target.set(.012,0,.120);
    camera.position.set(.012 + .52*Math.cos(theta), .52*Math.sin(theta), .210);
    camera.lookAt(controls.target);
  };
}
