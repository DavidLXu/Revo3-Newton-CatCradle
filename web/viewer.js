import * as THREE from 'three';
import { OrbitControls } from './OrbitControls.js';
import { studio } from './cinematic.js';
const cinematic = new URLSearchParams(location.search).has('cinematic');

const scene = new THREE.Scene();
scene.background = new THREE.Color('#e9e7e2');
scene.fog = new THREE.Fog('#e9e7e2', 1.1, 2.6);
const renderer = new THREE.WebGLRenderer({antialias: true, preserveDrawingBuffer: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
document.querySelector('#scene').appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(34, innerWidth/innerHeight, .001, 10);
camera.up.set(0,0,1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(.01,0,.137);
camera.position.set(.62,-.14,.28);
controls.enableDamping = true;
controls.minDistance = .21;
controls.maxDistance = 1.1;
controls.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x858477, 2.2));
function light(pos, intensity) {
  const light = new THREE.DirectionalLight(0xffffff, intensity);
  light.position.set(...pos); scene.add(light); return light;
}
const key = light([.35,-.4,.65], 3.0);
key.castShadow = true; key.shadow.mapSize.set(2048,2048);
Object.assign(key.shadow.camera, {left:-.3, right:.3, top:.4, bottom:-.3, near:.01, far:2});
key.shadow.bias = -.00005; key.shadow.normalBias = .0004;
light([-.3,.3,.4], 1.8);
function solid(geometry, color, position, roughness=.7, metalness=.1) {
  const m = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color,roughness,metalness}));
  m.position.set(...position);m.castShadow = true;m.receiveShadow = true;scene.add(m);return m;
}
solid(new THREE.BoxGeometry(3,3,.018), '#c9c8bd',[0,0,-.010]);
solid(new THREE.BoxGeometry(.095,.125,.009), '#272d2b',[0,0,.0045],.4,.5);
const stand = solid(new THREE.CylinderGeometry(.023,.027,.015,64),'#535b57',[0,0,.016],.3,.6);
stand.rotation.x = Math.PI/2;
for(const x of [-.037,.037]) for(const y of [-.05,.05]) {
  const screw = solid(new THREE.CylinderGeometry(.0023,.0023,.001,12),'#8c938c',[x,y,.0095],.3,.7);
  screw.rotation.x = Math.PI/2;
}
window.addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});
const orbit = cinematic ? studio(scene, renderer, camera, controls) : null;
try {
  const [data, meshes] = await Promise.all(['trajectory',cinematic?'meshes_cinematic':'meshes'].map(async name=>{
    const path=name==='trajectory' ? (new URLSearchParams(location.search).get('trajectory') ?? '../output/trajectory.json') : `../output/${name}.json`;
    const response=await fetch(path,{cache:'no-store'});if(!response.ok)throw new Error(`${name}.json: ${response.status}`);return response.json();
  }));
  const motion=data.motion;
  if(cinematic && motion?.direction==='反向')document.querySelector('.film-title h2').textContent='反向翻花绳';
  document.querySelector('h1').textContent=`指间翻花绳 · ${motion?.direction ?? '正向'}`;
  document.querySelector('#source').textContent=`${motion?.source ?? '关节记录'} · ${motion?.command_count ?? ''} 组目标 · ${data.frames.at(-1).time.toFixed(1)} s`;
  if(motion?.simulation_joint_adjustments) document.querySelector('#source').textContent+=' · 小指挑线 · 拇指送线时序调整';
  const sourceCaption=document.querySelector('#source').textContent;
  const lengthMode=data.config.adaptive_length?'自动收放绳':'固定绳长';
  document.querySelector('.footer span').textContent=`${lengthMode} · 无固定挂点 · 碰撞与摩擦（μ = ${data.config.friction ?? .65}）· Newton 柔性绳求解`;
  if(data.validation?.deformation_bounds_passed === false) {
    const status=document.querySelector('.footer span');
    status.textContent=`物理检查未通过：最大局部拉伸 ${((data.validation.max_stretch_ratio-1)*100).toFixed(0)}% · 当前为预览`;
    status.style.color='#a32f25';
  }
  const groups = new Map();
  data.links.forEach(name=>{const group=new THREE.Group();group.matrixAutoUpdate=false;groups.set(name,group);scene.add(group);});
  for(const item of meshes) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.Float32BufferAttribute(item.vertices.flat(),3));
    geo.setIndex(item.faces.flat());geo.computeVertexNormals();
    const silicone=item.color[0]<.65;
    const mat=new THREE.MeshStandardMaterial({color:new THREE.Color(...item.color),metalness:silicone?.05:(cinematic?.65:.48),roughness:silicone?.86:(cinematic?.28:.32)});
    const mesh=new THREE.Mesh(geo,mat);mesh.castShadow=true;mesh.receiveShadow=true;groups.get(item.link).add(mesh);
  }
  const n = data.frames[0].rope.length;
  const ropeMat = new THREE.MeshStandardMaterial({color:'#bd1238',roughness:.68,metalness:.02});
  const cylinders = new THREE.InstancedMesh(new THREE.CylinderGeometry(data.config.radius,data.config.radius,1,10),ropeMat,n);
  const balls = new THREE.InstancedMesh(new THREE.SphereGeometry(data.config.radius,8,6),ropeMat,n);
  cylinders.instanceMatrix.setUsage(THREE.DynamicDrawUsage);balls.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  cylinders.castShadow=true;balls.castShadow=true;cylinders.frustumCulled=false;balls.frustumCulled=false;scene.add(cylinders,balls);
  const dummy=new THREE.Object3D(), a=new THREE.Vector3(), b=new THREE.Vector3(), direction=new THREE.Vector3(), up=new THREE.Vector3(0,1,0);
  let frame=0, playing=false, clock=0, last=performance.now();
  const seek=document.querySelector('#seek');seek.max=data.frames.length-1;
  const poseA=new THREE.Matrix4(), poseB=new THREE.Matrix4();
  const posA=new THREE.Vector3(), posB=new THREE.Vector3(), scaleA=new THREE.Vector3(), scaleB=new THREE.Vector3();
  const quatA=new THREE.Quaternion(), quatB=new THREE.Quaternion();
  function setFrame(index, interpolate=false) {
    const sample=Math.max(0,Math.min(data.frames.length-1,index));
    const lo=Math.floor(sample), mix=interpolate?sample-lo:0;
    frame=Math.max(0,Math.min(data.frames.length-1,Math.round(index)));
    const f=data.frames[interpolate?lo:frame];
    const next=data.frames[Math.min(lo+1,data.frames.length-1)];
    const restLength=f.rope_rest_length_m ?? data.rope_rest_length_m;
    document.querySelector('#source').textContent=`${sourceCaption} · ${lengthMode} · 当前未拉伸长度 ${(restLength*100).toFixed(1)} cm`;
    data.links.forEach((name,i)=>{
      const matrix=groups.get(name).matrix;
      if(mix===0) matrix.fromArray(f.transforms[i]);
      else {
        poseA.fromArray(f.transforms[i]).decompose(posA,quatA,scaleA);
        poseB.fromArray(next.transforms[i]).decompose(posB,quatB,scaleB);
        matrix.compose(posA.lerp(posB,mix),quatA.slerp(quatB,mix),scaleA.lerp(scaleB,mix));
      }
    });
    for(let i=0;i<n;i++) {
      a.fromArray(f.rope[i]);b.fromArray(f.rope[(i+1)%n]);
      if(mix){a.lerp(posA.fromArray(next.rope[i]),mix);b.lerp(posB.fromArray(next.rope[(i+1)%n]),mix);}
      direction.subVectors(b,a);
      dummy.position.copy(a).add(b).multiplyScalar(.5);dummy.quaternion.setFromUnitVectors(up,direction.clone().normalize());
      dummy.scale.set(1,direction.length(),1);dummy.updateMatrix();cylinders.setMatrixAt(i,dummy.matrix);
      dummy.position.copy(a);dummy.scale.set(1,1,1);dummy.quaternion.identity();dummy.updateMatrix();balls.setMatrixAt(i,dummy.matrix);
    }
    cylinders.instanceMatrix.needsUpdate=true;balls.instanceMatrix.needsUpdate=true;
    seek.value=frame;document.querySelector('#time').textContent=`${f.time.toFixed(2)} / ${data.frames.at(-1).time.toFixed(2)} s`;
    document.querySelector('#phase').textContent=f.phase;
    document.querySelector('#meta').textContent=`21 JOINTS  /  ${n} PARTICLES  /  1 CLOSED LOOP`;
    renderer.render(scene,camera);
  }
  document.querySelector('#play').onclick=()=>{playing=!playing;document.querySelector('#play').textContent=playing?'暂停':'播放';};
  seek.oninput=()=>{playing=false;document.querySelector('#play').textContent='播放';clock=Number(seek.value)/data.config.fps;setFrame(Number(seek.value));};
  let front=false;
  document.querySelector('#camera').onclick=()=>{front=!front;camera.position.set(...(front?[.65,0,.21]:[.62,-.14,.28]));controls.target.set(.01,0,.137);controls.update();document.querySelector('#camera').textContent=front?'斜侧面':'正面';};
  window.renderCinematic=(time,progress)=>{if(!orbit)throw new Error('Cinematic mode required');orbit(progress);setFrame(time*data.config.fps,true);};
  window.renderCinematicView=(time,azimuth)=>{if(!orbit)throw new Error('Cinematic mode required');orbit(0,azimuth);setFrame(time*data.config.fps,true);};
  window.setFrame=(index)=>{playing=false;document.querySelector('#play').textContent='播放';setFrame(index);};
  window.setFront=()=>{front=true;camera.position.set(.65,0,.21);controls.update();renderer.render(scene,camera);};
  window.setSide=()=>{camera.position.set(.12,-.65,.24);controls.update();renderer.render(scene,camera);};
  window.revo3={frameCount:data.frames.length,particles:n,meshes:meshes.length,fps:data.config.fps,motionSource:motion?.source,direction:motion?.direction,interaction:data.interaction,anchors:data.anchor_indices.length};
  const requestedTime=Number(new URLSearchParams(location.search).get('t') ?? 0);
  clock=Number.isFinite(requestedTime)?Math.max(0,Math.min(data.frames.at(-1).time,requestedTime)):0;
  setFrame(clock*data.config.fps);window.viewerReady=true;
  function animate(now) {
    requestAnimationFrame(animate);
    const delta=Math.min(.1,(now-last)/1000);last=now;
    if(playing){clock=(clock+delta*Number(document.querySelector('#speed').value))%(data.frames.at(-1).time+.7);setFrame(Math.min(data.frames.length-1,Math.floor(clock*data.config.fps)));}
    controls.update();renderer.render(scene,camera);
  }
  if(!new URLSearchParams(location.search).has('capture')) requestAnimationFrame(animate);
} catch(error) {document.querySelector('#error').textContent=`加载失败：${error.message}。请先运行仿真，再通过 HTTP 打开此页面。`;console.error(error);}
