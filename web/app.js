import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = id => document.getElementById(id);
let running = false, busy = false, initialized = false, frames = [], latest = null;
let arena = { radius: .6096, height: .0254, border: .0508, timeout: 120 };
const models = [], bodyObjects = [];
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#dbe7f0');
const camera = new THREE.PerspectiveCamera(35, 1, .01, 20);
camera.position.set(1, 1.3, 1.3);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.25;
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, .03, 0);
controls.minDistance = .35; controls.maxDistance = 3;
controls.maxPolarAngle = Math.PI / 2 - .03;
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x718398, 2.5));
const light = new THREE.DirectionalLight(0xfff8e9, 3);
light.position.set(-.5, 1.5, .4); light.castShadow = true;
light.shadow.mapSize.set(2048, 2048);
Object.assign(light.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: .1, far: 4 });
light.shadow.bias = -.0001;
scene.add(light);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(10, 10), new THREE.MeshStandardMaterial({ color: '#dbe7f0', roughness: 1 }));
floor.rotation.x = -Math.PI / 2; floor.position.y = -.001; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(3, 60, 0xb3c7d7, 0xc6d6e2);
grid.position.y = -.0005; scene.add(grid);
const ring = new THREE.Mesh(new THREE.CylinderGeometry(.385, .385, .025, 128), new THREE.MeshStandardMaterial({ color: '#263139', roughness: .9 }));
ring.position.y = .0125; ring.receiveShadow = true; ring.castShadow = true; scene.add(ring);
const surface = new THREE.Mesh(new THREE.RingGeometry(.36, .385, 128), new THREE.MeshStandardMaterial({ color: '#f3f4f0', roughness: 1 }));
surface.rotation.x = -Math.PI / 2; surface.position.y = .0253; surface.receiveShadow = true; scene.add(surface);
const halos = [0x1767c7, 0xcb5146].map(color => {
  const mesh = new THREE.Mesh(new THREE.RingGeometry(.085, .088, 64), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: .8, depthWrite: false }));
  mesh.rotation.x = -Math.PI / 2; scene.add(mesh); return mesh;
});
new ResizeObserver(() => { const { width, height } = viewport.getBoundingClientRect(); renderer.setSize(width, height); camera.aspect = width/height; camera.updateProjectionMatrix(); }).observe(viewport);
function draw() { controls.update(); renderer.render(scene, camera); requestAnimationFrame(draw); }
draw();
$('iso').onclick = () => { camera.position.set(1, 1.3, 1.3); controls.target.set(0, .03, 0); };
$('top').onclick = () => { camera.position.set(0, 2.3, .001); controls.target.set(0, 0, 0); };
$('details').onclick = () => $('build-dialog').showModal();
$('close-details').onclick = () => $('build-dialog').close();

async function api(path, body) {
  const response = await fetch(`/api/${path}`, body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const data = await response.json();
  if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : JSON.stringify(data.detail));
  return data;
}
function fail(error) { running = false; $('run').textContent = '▶ Run match'; $('error').textContent = error.message; }
function display(frame) {
  $('clock').textContent = frame.time.toFixed(2);
  const result = frame.result;
  $('match-state').textContent = result ? `${result.winner === null ? 'DRAW' : result.winner === 0 ? 'BLUE WINS' : 'RED WINS'} · ${result.reason.replaceAll('_',' ')}` : running ? 'MATCH IN PROGRESS' : 'PAUSED / INSPECT';
  frame.bots.forEach((bot, i) => {
    for (const [name, pose] of Object.entries(bot.bodies)) {
      const object = bodyObjects[i]?.[name];
      if (object) { object.position.fromArray(pose.position); object.quaternion.fromArray(pose.rotation); }
    }
    const p = bot.bodies.chassis.position;
    halos[i].position.set(p[0], arena.height+.001, p[2]);
    const cells = [['Eyes', `${bot.sensors.distance}`, 'mm'], ['Reflection', `${bot.sensors.reflection}`, '%'], ['Left / right', bot.motors.map(m => Math.round(m.speed)).join(' / '), '°/s'], ['Battery', bot.battery.toFixed(2), 'V']];
    $('telemetry'+i).replaceChildren(...cells.map(([label,value,unit]) => { const div = document.createElement('div'); div.className = 'reading'; div.textContent = label; const b = document.createElement('b'); b.textContent = value+' '; const small = document.createElement('small'); small.textContent = unit; b.append(small); div.append(b); return div; }));
    const program = frame.programs?.[i];
    const opening = frame.rules?.bots?.[i];
    $('status'+i).textContent = program?.error || (opening ? `${opening.phase}${opening.opening ? ' · '+opening.opening : ''}` : program?.status) || 'Ready';
    $('status'+i).classList.toggle('error', !!program?.error);
  });
}
function record(frame) {
  latest = frame;
  if (!frames.length || frame.time !== frames.at(-1).time) frames.push(frame);
  else frames[frames.length-1] = frame;
  $('scrub').max = Math.max(0, frames.length-1); $('scrub').value = frames.length-1;
  display(frame);
  if (frame.result) { running = false; $('run').textContent = '▶ Run match'; }
}
async function reset() {
  running = false; $('run').textContent = '▶ Run match'; $('error').textContent = '';
  const seed = Number($('seed').value);
  if (!Number.isInteger(seed) || seed < 0) throw new Error('Seed must be a nonnegative integer.');
  const frame = await api('reset', { programs: [$('code0').value, $('code1').value], seed, timeout: arena.timeout });
  frames = []; initialized = true; record(frame);
  $('match-state').textContent = 'READY TO TEST';
}
async function guarded(action) {
  if (busy) return;
  busy = true;
  try { await action(); } catch (error) { fail(error); } finally { busy = false; }
}
async function control(action) {
  running = false; $('run').textContent = '▶ Run match';
  while (busy) await new Promise(resolve => setTimeout(resolve, 10));
  await guarded(action);
}
$('reset').onclick = () => control(reset);
$('run').onclick = () => {
  if (running) { running = false; $('run').textContent = '▶ Run match'; if (latest) display(latest); return; }
  control(async () => {
  if (!initialized || latest?.result) await reset();
  running = !running; $('run').textContent = running ? 'Ⅱ Pause' : '▶ Run match';
  if (latest) display(latest);
  });
};
$('step').onclick = () => control(async () => { if (!initialized) await reset(); record(await api('step', { seconds: .05 })); });
$('scrub').oninput = () => { running = false; $('run').textContent = '▶ Run match'; if (frames.length) display(frames[Number($('scrub').value)]); };
$('download').onclick = () => guarded(async () => { const replay = await api('replay'); const url = URL.createObjectURL(new Blob([JSON.stringify(replay)], { type: 'application/json' })); const link = document.createElement('a'); link.href = url; link.download = `spike-sumo-seed-${replay.metadata.seed ?? 0}.json`; link.click(); URL.revokeObjectURL(url); });
document.querySelectorAll('input[type=file]').forEach(input => input.onchange = async () => { if (input.files[0]) { $('code'+input.dataset.bot).value = await input.files[0].text(); initialized = false; running = false; $('run').textContent = '▶ Run match'; } });
for (const i of [0, 1]) $('code'+i).oninput = () => { initialized = false; running = false; $('run').textContent = '▶ Run match'; };
$('seed').onchange = () => { initialized = false; running = false; $('run').textContent = '▶ Run match'; };
async function pump() {
  if (running && !busy) await guarded(async () => record(await api('step', { seconds: .05 })));
  setTimeout(pump, 20);
}
async function boot() {
  const info = await api('info');
  arena = info.arena;
  ring.geometry.dispose(); ring.geometry = new THREE.CylinderGeometry(arena.radius, arena.radius, arena.height, 128); ring.position.y = arena.height/2;
  surface.geometry.dispose(); surface.geometry = new THREE.RingGeometry(arena.radius-arena.border, arena.radius, 128); surface.position.y = arena.height+.0003;
  document.querySelector('.scene-label').textContent = `Ø ${(arena.radius*2000).toFixed(1)} mm · ${(arena.border*1000).toFixed(1)} mm white border`;
  $('rules-description').textContent = 'Facing away · wait 3 seconds · touch white and turn, or pivot 180° on one stationary wheel. Provisional Atomic Eagles 2022 arena; user-specified pivot.';
  const gltf = await new GLTFLoader().loadAsync('/assets/model.glb');
  for (const i of [0, 1]) {
    const model = gltf.scene.clone(true); scene.add(model); models.push(model);
    const objects = Object.fromEntries(info.robot.bodies.map(body => [body.name, model.getObjectByName(body.name)]));
    // Physics supplies world transforms, so flatten only the articulated body groups.
    for (const object of Object.values(objects)) { if (!object) throw new Error('Missing named robot body in GLB'); scene.attach(object); }
    bodyObjects.push(objects);
    for (const object of Object.values(objects)) object.traverse(child => { if (child.isMesh) { child.castShadow = true; child.receiveShadow = true; } });
    const names = Object.keys(info.examples);
    for (const name of names) { const option = document.createElement('option'); option.value = name; option.textContent = name.replaceAll('_',' '); $('preset'+i).append(option); }
    $('preset'+i).selectedIndex = Math.min(i, names.length-1);
    $('code'+i).value = info.examples[$('preset'+i).value] || '';
    $('preset'+i).onchange = () => { $('code'+i).value = info.examples[$('preset'+i).value]; initialized = false; running = false; $('run').textContent = '▶ Run match'; };
  }
  $('mass').textContent = `${(info.robot.totalMassKg*1000).toFixed(2)} g`;
  $('dimensions').textContent = `${(info.robot.totalMassKg*1000).toFixed(2)} g · ${info.robot.bounds.max.map((v,i) => ((v-info.robot.bounds.min[i])*1000).toFixed(1)).join(' × ')} mm (width × height × length)`;
  const table = document.createElement('table');
  for (const part of info.robot.parts) { const row = table.insertRow(); row.insertCell().textContent = part.name.replaceAll('_',' '); row.insertCell().textContent = part.ldraw; row.insertCell().textContent = `${(part.massKg*1000).toFixed(2)} g`; }
  $('bom').append(table);
  await guarded(reset); $('loading').remove(); pump();
}
boot().catch(fail);
