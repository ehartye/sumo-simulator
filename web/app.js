import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = id => document.getElementById(id);
let running = false, busy = false, initialized = false, frames = [], latest = null;
let events = [], eventViewKey = '';
let loadedReplay = null, liveArena = null;
let bestSelection = null, appliedRevision = null, automaticPrograms = null, examples = {};
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
let showSensors = true;
const sensorRays = [0x1767c7, 0xcb5146].map(color => {
  const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const ray = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity: .9 }));
  const hit = new THREE.Mesh(new THREE.SphereGeometry(.005, 8, 6), new THREE.MeshBasicMaterial({ color }));
  scene.add(ray, hit); return { ray, hit };
});
new ResizeObserver(() => { const { width, height } = viewport.getBoundingClientRect(); renderer.setSize(width, height); camera.aspect = width/height; camera.updateProjectionMatrix(); }).observe(viewport);
function draw() { controls.update(); renderer.render(scene, camera); requestAnimationFrame(draw); }
draw();
$('iso').onclick = () => { camera.position.set(1, 1.3, 1.3); controls.target.set(0, .03, 0); };
$('top').onclick = () => { camera.position.set(0, 2.3, .001); controls.target.set(0, 0, 0); };
$('details').onclick = () => $('build-dialog').showModal();
$('close-details').onclick = () => $('build-dialog').close();
$('sensor-view').onclick = () => {
  showSensors = !showSensors; $('sensor-view').setAttribute('aria-pressed', String(showSensors));
  if (latest) display(frames[Number($('scrub').value)] || latest);
};

async function api(path, body) {
  const response = await fetch(`/api/${path}`, body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const data = await response.json();
  if (!response.ok) throw new Error(typeof data.detail === 'string' ? data.detail : JSON.stringify(data.detail));
  return data;
}
function fail(error) { running = false; $('run').textContent = '▶ Run match'; $('error').textContent = error.message; }
function setArena(value) {
  arena = value;
  ring.geometry.dispose(); ring.geometry = new THREE.CylinderGeometry(arena.radius, arena.radius, arena.height, 128); ring.position.y = arena.height/2;
  surface.geometry.dispose(); surface.geometry = new THREE.RingGeometry(arena.radius-arena.border, arena.radius, 128); surface.position.y = arena.height+.0003;
  document.querySelector('.scene-label').textContent = `Ø ${(arena.radius*2000).toFixed(1)} mm · ${(arena.border*1000).toFixed(1)} mm white border`;
}
function validateReplay(replay) {
  const invalid = () => { throw new Error('Invalid replay. Load a version 1 saved run with arena, programs, frames, and events.'); };
  const vector = (value, size) => Array.isArray(value) && value.length === size && value.every(Number.isFinite);
  const result = value => value === null || (value && [null, 0, 1].includes(value.winner) && typeof value.reason === 'string');
  const metadata = replay?.metadata, area = metadata?.arena;
  if (replay?.version !== 1 || !area || !['radius', 'height', 'border', 'timeout'].every(key => Number.isFinite(area[key]) && area[key] > 0) || area.border >= area.radius ||
      !Number.isInteger(metadata.seed) || metadata.seed < 0 || metadata.seed > 2147483647 ||
      !Array.isArray(metadata.programs) || metadata.programs.length !== 2 || !metadata.programs.every(program => typeof program === 'string') ||
      !Array.isArray(replay.frames) || !replay.frames.length || !Array.isArray(replay.events)) invalid();
  let previousTime = -1;
  for (const frame of replay.frames) {
    if (!Number.isFinite(frame.time) || frame.time < previousTime || !result(frame.result) ||
        !Number.isInteger(frame.eventCount) || frame.eventCount < 0 || frame.eventCount > replay.events.length ||
        !Array.isArray(frame.bots) || frame.bots.length !== 2) invalid();
    previousTime = frame.time;
    for (const bot of frame.bots) {
      if (!bot.bodies?.chassis || !Object.values(bot.bodies).every(pose => vector(pose?.position, 3) && vector(pose?.rotation, 4)) ||
          !Number.isFinite(bot.battery) || !Number.isFinite(bot.sensors?.distance) || !Number.isFinite(bot.sensors?.reflection) ||
          !Array.isArray(bot.motors) || bot.motors.length !== 2 || !bot.motors.every(motor => Number.isFinite(motor.speed))) invalid();
      const sensor = bot.ultrasonic;
      if (sensor && (!Number.isFinite(sensor.sampleTime) || !sensor.counts || typeof sensor.reason !== 'string' ||
          (sensor.hitObject != null && typeof sensor.hitObject !== 'string') ||
          (sensor.hitPoint && (!vector(sensor.hitPoint, 3) || !vector(sensor.origin, 3))))) invalid();
    }
  }
  previousTime = -1;
  replay.events.forEach((event, i) => {
    // Events round simulation time to six decimals; snapshots retain step accumulation.
    if (event.id !== i || !Number.isFinite(event.time) || event.time < previousTime || event.time > replay.frames.at(-1).time + 1e-6 || typeof event.type !== 'string' ||
        (event.type === 'match_ended' && (!event.result || !result(event.result))) ||
        (event.parts != null && (!Array.isArray(event.parts) || !event.parts.every(part => typeof part === 'string')))) invalid();
    previousTime = event.time;
  });
  return replay;
}
async function loadReplay(file) {
  let replay;
  try { replay = validateReplay(JSON.parse(await file.text())); }
  catch (error) { throw new Error(error instanceof SyntaxError ? 'Invalid JSON. Choose a saved run JSON file.' : error.message); }
  loadedReplay = replay;
  initialized = false; frames = replay.frames; events = replay.events; eventViewKey = ''; latest = frames.at(-1);
  setArena(replay.metadata.arena);
  replay.metadata.programs.forEach((program, i) => { $('code'+i).value = program; $('preset'+i).selectedIndex = -1; });
  $('seed').value = replay.metadata.seed;
  $('error').textContent = '';
  $('replay-source').textContent = `Loaded replay · ${file.name} · Run or Step starts a new match`;
  $('scrub').max = frames.length-1; $('scrub').value = frames.length-1;
  display(latest);
}
function eventText(event) {
  const bot = event.bot === 0 ? 'Blue' : 'Red';
  switch (event.type) {
    case 'opponent_spotted': return `${bot} · Opponent spotted · ${event.distanceMm} mm`;
    case 'opponent_lost': return `${bot} · Opponent echo lost`;
    case 'contact_started': return 'Robots made contact';
    case 'contact_ended': return 'Robots separated';
    case 'program_log': return `${bot} program · ${event.message}`;
    case 'opening_phase': return `${bot} · ${event.phase}${event.opening ? ' · '+event.opening : ''}`;
    case 'match_ended': return `${event.result.winner === null ? 'Draw' : event.result.winner === 0 ? 'Blue wins' : 'Red wins'} · ${event.result.reason.replaceAll('_', ' ')}`;
    default: return event.type;
  }
}
function displayEvents(frame) {
  const filter = $('event-filter').value;
  const occurred = events.filter(event => event.id < frame.eventCount);
  const key = `${occurred.length}:${filter}`;
  if (key === eventViewKey) return;
  eventViewKey = key;
  const first = type => occurred.find(event => event.type === type);
  const spotted = first('opponent_spotted'), contact = first('contact_started');
  $('event-summary').textContent = `${spotted ? 'First opponent echo '+spotted.time.toFixed(3)+' s' : 'No opponent spotted'} · ${contact ? 'First robot contact '+contact.time.toFixed(3)+' s' : 'No robot contact'} · ${occurred.length} events recorded`;
  const filtered = occurred.filter(event => filter === 'all' ||
    (filter === 'sensor' && event.type.startsWith('opponent_')) ||
    (filter === 'contact' && event.type.startsWith('contact_')) ||
    (filter === 'program' && event.type === 'program_log'));
  $('event-list').replaceChildren(...filtered.slice(-300).map(event => {
    const li = document.createElement('li'), button = document.createElement('button');
    button.dataset.eventId = event.id;
    button.className = event.bot === 0 ? 'event-blue' : event.bot === 1 ? 'event-red' : 'event-shared';
    const time = document.createElement('time'); time.textContent = event.time.toFixed(3)+' s';
    const text = document.createElement('span'); text.textContent = eventText(event);
    button.append(time, text);
    button.title = event.parts?.join(' ↔ ') || event.hitObject || 'Inspect nearest recorded frame';
    button.onclick = () => control(async () => {
      let nearest = 0;
      frames.forEach((candidate, i) => { if (Math.abs(candidate.time-event.time) < Math.abs(frames[nearest].time-event.time)) nearest = i; });
      $('scrub').value = nearest; display(frames[nearest]);
    });
    li.append(button); return li;
  }));
  if (!filtered.length) {
    const li = document.createElement('li'); li.textContent = 'No matching events at this replay time.'; $('event-list').append(li);
  } else if (filtered.length > 300) {
    const li = document.createElement('li'); li.textContent = `Showing latest 300 of ${filtered.length} matching events. Save run includes all events.`; $('event-list').prepend(li);
  }
}
$('event-filter').onchange = () => { if (latest) displayEvents(frames[Number($('scrub').value)] || latest); };
function display(frame) {
  displayEvents(frame);
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
    const sensor = bot.ultrasonic;
    const { ray, hit } = sensorRays[i];
    ray.visible = hit.visible = false;
    if (sensor) {
      const labels = { echo: 'Echo', no_echo: 'No echo', below_minimum: 'Below rated range (model policy)' };
      $('sensor-status'+i).textContent = `${labels[sensor.reason]}${sensor.hitObject ? ' · '+sensor.hitObject.replace(/^b[01]_/, '').replace(/_\d+$/, '').replaceAll('_',' ') : ''}`;
      const savedDetail = loadedReplay && !loadedReplay.ultrasonicSamples ? 'This replay contains diagnostics at each recorded frame.' : 'Save run includes the full diagnostic log.';
      $('sensor-status'+i).title = `Sample ${sensor.sampleTick} at ${sensor.sampleTime.toFixed(3)} s; ${sensor.counts.inRange} in-range rays, ${sensor.counts.grazing} grazing rays rejected. ${savedDetail}`;
      if (sensor.hitPoint) {
        const positions = ray.geometry.attributes.position;
        positions.setXYZ(0, ...sensor.origin); positions.setXYZ(1, ...sensor.hitPoint);
        positions.needsUpdate = true; ray.geometry.computeBoundingSphere();
        hit.position.fromArray(sensor.hitPoint); ray.visible = hit.visible = showSensors;
      }
    }
  });
}
function record(frame) {
  events.push(...(frame.events || []));
  delete frame.events;
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
  const frame = await api('reset', { programs: [$('code0').value, $('code1').value], seed, timeout: liveArena.timeout });
  loadedReplay = null; setArena(liveArena); $('replay-source').textContent = 'Live match';
  frames = []; events = []; eventViewKey = ''; initialized = true; record(frame);
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
function showBestSelection() {
  $('load-best').disabled = $('inspect-best').disabled = !bestSelection;
  if (!bestSelection) { $('best-label').textContent = 'No tested strategy selected yet'; return; }
  const { champion, opponent, loop, score, revision } = bestSelection;
  $('best-label').textContent = `${revision === appliedRevision ? 'Loaded' : 'Available'} incumbent · ${champion.name.replaceAll('_', ' ')} · loop ${loop} · ${score.wins}W / ${score.draws}D / ${score.losses}L vs ${opponent.name.replaceAll('_', ' ')}`;
}
function applyBestSelection() {
  if (!bestSelection) return;
  [bestSelection.champion, bestSelection.opponent].forEach(({ name, source }, i) => {
    examples[name] = source;
    for (const corner of [0, 1]) {
      if (![...$('preset'+corner).options].some(option => option.value === name)) {
        const option = document.createElement('option'); option.value = name; option.textContent = name.replaceAll('_', ' '); $('preset'+corner).append(option);
      }
    }
    $('preset'+i).value = name; $('code'+i).value = source;
  });
  automaticPrograms = [$('code0').value, $('code1').value];
  appliedRevision = bestSelection.revision; initialized = false; $('error').textContent = '';
  showBestSelection();
}
async function refreshBestSelection(manual = false) {
  bestSelection = await api('strategy-selection');
  showBestSelection();
  const unchanged = automaticPrograms?.every((program, i) => $('code'+i).value === program);
  if (manual) applyBestSelection();
  else if (bestSelection && bestSelection.revision !== appliedRevision && !running && !busy && !loadedReplay && unchanged) await control(applyBestSelection);
}
$('load-best').onclick = () => control(() => refreshBestSelection(true));
$('inspect-best').onclick = () => control(async () => {
  if (!bestSelection) return;
  const response = await fetch(bestSelection.replayUrl);
  if (!response.ok) throw new Error('Winning replay is unavailable. Try Load latest best to refresh.');
  await loadReplay({ name: `loop ${bestSelection.loop} winning bout`, text: () => response.text() });
});
async function pollBestSelection() {
  try { await refreshBestSelection(); }
  catch { /* Keep the current match and retry if the local server is restarting. */ }
  setTimeout(pollBestSelection, 10000);
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
$('download').onclick = () => guarded(async () => { const replay = loadedReplay || await api('replay'); const url = URL.createObjectURL(new Blob([JSON.stringify(replay)], { type: 'application/json' })); const link = document.createElement('a'); link.href = url; link.download = `spike-sumo-seed-${replay.metadata.seed ?? 0}.json`; link.click(); URL.revokeObjectURL(url); });
$('load-replay').onchange = () => {
  const file = $('load-replay').files[0]; $('load-replay').value = '';
  if (file) control(() => loadReplay(file));
};
document.querySelectorAll('input[data-bot]').forEach(input => input.onchange = async () => { if (input.files[0]) { $('code'+input.dataset.bot).value = await input.files[0].text(); initialized = false; running = false; $('run').textContent = '▶ Run match'; } });
for (const i of [0, 1]) $('code'+i).oninput = () => { initialized = false; running = false; $('run').textContent = '▶ Run match'; };
$('seed').onchange = () => { initialized = false; running = false; $('run').textContent = '▶ Run match'; };
async function pump() {
  if (running && !busy) await guarded(async () => record(await api('step', { seconds: .05 })));
  setTimeout(pump, 20);
}
async function boot() {
  const info = await api('info');
  examples = { ...info.examples };
  liveArena = info.arena; setArena(liveArena);
  $('rules-description').textContent = 'Facing away · wait 3 seconds · touch white and turn, or pivot 180° on one stationary wheel. Provisional Atomic Eagles 2022 arena; user-specified pivot.';
  const gltf = await new GLTFLoader().loadAsync('/assets/model.glb');
  for (const i of [0, 1]) {
    const model = gltf.scene.clone(true); scene.add(model); models.push(model);
    const objects = Object.fromEntries(info.robot.bodies.map(body => [body.name, model.getObjectByName(body.name)]));
    // Physics supplies world transforms, so flatten only the articulated body groups.
    for (const object of Object.values(objects)) { if (!object) throw new Error('Missing named robot body in GLB'); scene.attach(object); }
    bodyObjects.push(objects);
    for (const object of Object.values(objects)) object.traverse(child => { if (child.isMesh) { child.castShadow = true; child.receiveShadow = true; } });
    const names = Object.keys(examples);
    for (const name of names) { const option = document.createElement('option'); option.value = name; option.textContent = name.replaceAll('_',' '); $('preset'+i).append(option); }
    $('preset'+i).selectedIndex = Math.min(i, names.length-1);
    $('code'+i).value = examples[$('preset'+i).value] || '';
    $('preset'+i).onchange = () => { $('code'+i).value = examples[$('preset'+i).value]; initialized = false; running = false; $('run').textContent = '▶ Run match'; };
  }
  $('mass').textContent = `${(info.robot.totalMassKg*1000).toFixed(2)} g`;
  $('dimensions').textContent = `${(info.robot.totalMassKg*1000).toFixed(2)} g · ${info.robot.bounds.max.map((v,i) => ((v-info.robot.bounds.min[i])*1000).toFixed(1)).join(' × ')} mm (width × height × length)`;
  const table = document.createElement('table');
  for (const part of info.robot.parts) { const row = table.insertRow(); row.insertCell().textContent = part.name.replaceAll('_',' '); row.insertCell().textContent = part.ldraw; row.insertCell().textContent = `${(part.massKg*1000).toFixed(2)} g`; }
  $('bom').append(table);
  automaticPrograms = [$('code0').value, $('code1').value];
  try { await refreshBestSelection(); } catch { /* Selection is optional on older servers. */ }
  await guarded(reset); $('loading').remove(); pump(); setTimeout(pollBestSelection, 10000);
}
boot().catch(fail);
