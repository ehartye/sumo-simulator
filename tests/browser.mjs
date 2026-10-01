// Requires the local server and the sibling Agent Meshes Playwright installation.
import { chromium } from '../../agent-meshes/node_modules/playwright/index.mjs';
import assert from 'node:assert/strict';
import { mkdir } from 'node:fs/promises';

await mkdir('artifacts/screenshots', { recursive: true });
const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1500, height: 1000 } });
const errors = [];
page.on('pageerror', error => errors.push(error.message));
try {
  // Bound the end-to-end bout while exercising the real controls and referee.
  await page.route('**/api/reset', async route => {
    const body = route.request().postDataJSON();
    await route.continue({ postData: JSON.stringify({ ...body, timeout: 12 }) });
  });
  await page.goto('http://127.0.0.1:8765');
  await page.waitForFunction(() => document.querySelectorAll('.reading').length === 8 && !document.querySelector('#loading'));
  assert.match(await page.locator('#sensor-status0').textContent(), /No echo/);
  await page.click('#sensor-view');
  assert.equal(await page.locator('#sensor-view').getAttribute('aria-pressed'), 'false');
  await page.click('#sensor-view');
  await page.screenshot({ path: 'artifacts/screenshots/workbench.png', fullPage: true });
  await page.click('#run');
  await page.waitForFunction(() => Number(document.querySelector('#clock').textContent) > .5);
  await page.click('#run');
  await page.waitForTimeout(250);
  const paused = await page.locator('#clock').textContent();
  await page.waitForTimeout(150);
  assert.equal(await page.locator('#clock').textContent(), paused, 'Pause must stop pumping');
  await page.click('#step');
  await page.waitForFunction(value => Number(document.querySelector('#clock').textContent) > Number(value), paused);
  await page.click('#run');
  await page.waitForFunction(() => /WINS|DRAW/.test(document.querySelector('#match-state').textContent), null, { timeout: 240000 });
  const replay = await page.evaluate(() => fetch('/api/replay').then(r => r.json()));
  assert(replay.frames.at(-1).result, 'Completed bout must be saved');
  assert(replay.ultrasonicSamples.length > 2000, 'Sensors log at 100 Hz independently of browser pumping');
  assert(replay.ultrasonicSamples.some(sample => sample.reason === 'echo' && sample.hitPoint && sample.hitObject));
  assert(replay.metadata.sensorLogFile && replay.metadata.sensorMetadataFile);
  const diagnostic = await page.evaluate(() => fetch('/api/diagnostics').then(r => r.json()));
  assert.equal(diagnostic.samples.length, replay.ultrasonicSamples.length);
  assert(replay.frames.at(-1).rules.bots.every(bot => bot.phase === 'engaged' && !bot.violation), 'Both opening paths complete');
  await page.screenshot({ path: 'artifacts/screenshots/completed-bout.png', fullPage: true });
  await page.locator('#code0').fill('raise ValueError("intentional browser test fault")');
  await page.click('#reset');
  await page.waitForFunction(() => document.querySelector('#clock').textContent === '0.00');
  await page.click('#step');
  await page.waitForFunction(() => document.querySelector('#match-state').textContent.includes('program error'));
  const fault = await page.evaluate(() => fetch('/api/replay').then(r => r.json()));
  assert.equal(fault.frames.at(-1).result.reason, 'program_error');
  await page.locator('#preset0').selectOption('aggressive');
  await page.click('#reset');
  await page.waitForFunction(() => document.querySelector('#match-state').textContent === 'READY TO TEST');
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: 'artifacts/screenshots/mobile.png', fullPage: true });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
  await page.goto('http://127.0.0.1:8765/ui/robot.html');
  await page.waitForFunction(() => document.body.dataset.loaded === 'true');
  assert.match(await page.locator('#connection-stats').textContent(), /43 kit parts.*61 mating connections.*One connected assembly/);
  assert.deepEqual(errors, []);
  console.log(JSON.stringify({ browser: 'passed', result: replay.frames.at(-1).result, frames: replay.frames.length, errors }));
} catch (error) {
  console.error(await page.evaluate(() => ({ clock: document.querySelector('#clock')?.textContent, error: document.querySelector('#error')?.textContent, state: document.querySelector('#match-state')?.textContent })));
  await page.screenshot({ path: 'artifacts/screenshots/browser-failure.png', fullPage: true });
  throw error;
} finally { await browser.close(); }
