// Requires the local server; failure status must refresh even without new events.
import { chromium } from '../../agent-meshes/node_modules/playwright/index.mjs';
import assert from 'node:assert/strict';

const browser = await chromium.launch({ headless: true });
const page = await browser.newPage();
try {
  // Historical-label coverage must survive later revalidation of the live selection.
  await page.route('**/api/strategy-selection', async route => {
    const response = await route.fetch();
    const data = await response.json();
    data.testedRuntime = 'sync-subset-v3';
    data.currentRuntime = 'sync-subset-v4';
    await route.fulfill({ response, json: data });
  });
  await page.goto('http://127.0.0.1:8765');
  await page.waitForFunction(() => !document.querySelector('#loading'));
  assert.match(await page.locator('#best-label').textContent(), /older logging timing; current timing unvalidated/);
  for (const bot of [0, 1]) await page.locator('#code'+bot).fill('from pybricks.tools import wait\nwait(100000)');
  await page.click('#reset');
  await page.waitForFunction(() => document.querySelector('#match-state').textContent === 'READY TO TEST');
  await page.click('#step');
  await page.waitForFunction(() => Number(document.querySelector('#clock').textContent) > .01);
  const before = await page.evaluate(() => fetch('/api/replay').then(r => r.json()));
  assert.equal(before.metadata.runtime, 'sync-subset-v4');
  await page.route('**/api/step', async route => {
    const response = await route.fetch();
    const data = await response.json();
    assert.equal(data.eventCount, before.frames.at(-1).eventCount, 'No new event can trigger the refresh');
    data.observationLog = { droppedBatches: 1, error: 'Controlled UI fixture' };
    await route.fulfill({ response, json: data });
  });
  await page.click('#step');
  await page.waitForFunction(() => document.querySelector('#event-summary').textContent.includes('Disk logging incomplete'));
  await page.unrouteAll({ behavior: 'wait' });
  await page.click('#run');
  await page.waitForFunction(() => Number(document.querySelector('#clock').textContent) > .1);
  assert.match(await page.locator('#run').textContent(), /Pause/);
  await page.click('#run');
  await page.click('#load-best');
  await page.click('#reset');
  await page.waitForFunction(() => document.querySelector('#match-state').textContent === 'READY TO TEST');
  console.log(JSON.stringify({ observationStatus: 'refreshes without new events', reactions: 'continue', historicalScore: 'labeled' }));
} finally {
  await page.unrouteAll({ behavior: 'ignoreErrors' });
  await browser.close();
}
