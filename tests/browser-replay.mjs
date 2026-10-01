// Saved-run inspection uses real snapshots; no full simulation is needed.
import { chromium } from '../../agent-meshes/node_modules/playwright/index.mjs';
import assert from 'node:assert/strict';
import { mkdir, readFile } from 'node:fs/promises';

await mkdir('artifacts/screenshots', { recursive: true });
const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1500, height: 1100 } });
const errors = [];
page.on('pageerror', error => errors.push(error.message));
let resets = 0;
page.on('request', request => { if (request.url().endsWith('/api/reset')) resets++; });
try {
  await page.goto('http://127.0.0.1:8765');
  await page.waitForFunction(() => !document.querySelector('#loading'));
  const source = await page.evaluate(() => fetch('/api/replay').then(response => response.json()));
  const report = { version: 1, metadata: { ...source.metadata, seed: 123, arena: { ...source.metadata.arena, radius: .5 }, programs: ['# saved blue\npass', '# saved red\npass'] }, frames: [], events: [
    { id: 0, time: 0, type: 'opening_phase', bot: 0, phase: 'waiting' },
    { id: 1, time: 1, type: 'contact_started', parts: ['b0_hub', 'b1_hub'] },
    { id: 2, time: 2, type: 'match_ended', result: { winner: 0, reason: 'ring_out' } },
  ] };
  for (let time = 0; time <= 2; time++) {
    const frame = structuredClone(source.frames[0]);
    frame.time = time; frame.eventCount = time + 1;
    for (const pose of Object.values(frame.bots[0].bodies)) pose.position[2] += time * .05;
    frame.result = time === 2 ? report.events[2].result : null;
    report.frames.push(frame);
  }
  assert.equal(await page.locator('#load-replay').count(), 1, 'Load replay control exists');
  const load = async (data, name = 'saved-bout.json') => page.locator('#load-replay').setInputFiles({ name, mimeType: 'application/json', buffer: Buffer.from(typeof data === 'string' ? data : JSON.stringify(data)) });
  await load(report);
  await page.waitForFunction(() => document.querySelector('#clock').textContent === '2.00');
  assert.match(await page.locator('#replay-source').textContent(), /Loaded replay.*saved-bout.json/);
  assert.equal(await page.locator('#seed').inputValue(), '123');
  assert.equal(await page.locator('#code0').inputValue(), report.metadata.programs[0]);
  assert.match(await page.locator('.scene-label').textContent(), /1000.0 mm/);
  assert.match(await page.locator('#match-state').textContent(), /BLUE WINS/);
  await page.screenshot({ path: 'artifacts/screenshots/loaded-replay.png', fullPage: true });
  await page.locator('[data-event-id="1"]').click();
  assert.equal(await page.locator('#clock').textContent(), '1.00');
  await page.locator('#scrub').evaluate(input => { input.value = 0; input.dispatchEvent(new Event('input')); });
  assert.equal(await page.locator('#clock').textContent(), '0.00');
  await page.locator('#scrub').evaluate(input => { input.value = input.max; input.dispatchEvent(new Event('input')); });
  for (const invalid of ['{broken', { ...report, frames: [{ time: 0 }] }, { ...report, frames: [...report.frames].reverse() }]) {
    await load(invalid, 'invalid.json');
    await page.waitForFunction(() => document.querySelector('#error').textContent.length > 0);
    assert.equal(await page.locator('#clock').textContent(), '2.00', 'Bad file retains previous replay');
    assert.match(await page.locator('#replay-source').textContent(), /saved-bout.json/);
  }
  await page.context().setOffline(true);
  const downloadPromise = page.waitForEvent('download');
  await page.click('#download');
  const download = await downloadPromise;
  assert.deepEqual(JSON.parse(await readFile(await download.path(), 'utf8')), report, 'Save loaded run without live API');
  await page.context().setOffline(false);
  await page.setViewportSize({ width: 390, height: 844 });
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
  await page.setViewportSize({ width: 1500, height: 1100 });
  let before = resets;
  await page.click('#step');
  await page.waitForFunction(() => document.querySelector('#clock').textContent === '0.05');
  assert.equal(resets, before + 1, 'Step creates a fresh backend run');
  assert.match(await page.locator('.scene-label').textContent(), /1219.2 mm/);
  await load(report);
  await page.waitForFunction(() => document.querySelector('#clock').textContent === '2.00');
  before = resets;
  await page.click('#run');
  await page.waitForFunction(() => Number(document.querySelector('#clock').textContent) > 0 && Number(document.querySelector('#clock').textContent) < 2);
  await page.click('#run');
  assert.equal(resets, before + 1, 'Run creates a fresh backend run');
  await load(report);
  await page.waitForFunction(() => document.querySelector('#clock').textContent === '2.00');
  await page.click('#reset');
  await page.waitForFunction(() => document.querySelector('#match-state').textContent === 'READY TO TEST');
  assert.equal(await page.locator('#replay-source').textContent(), 'Live match');
  assert.deepEqual(errors, []);
  console.log(JSON.stringify({ savedReplay: 'passed', checks: ['load', 'scrub', 'event seek', 'malformed file retention', 'offline save', 'mobile', 'fresh step/run/reset'], errors }));
} finally { await browser.close(); }
