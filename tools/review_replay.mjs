// Usage: node tools/review_replay.mjs path/to/bout.json [more-bouts.json ...]
// Uses the running local UI; screenshots are saved beside each replay.
import { chromium } from '../../agent-meshes/node_modules/playwright/index.mjs';
import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';

if (!process.argv[2]) throw new Error('Pass at least one saved replay JSON file.');
const browser = await chromium.launch({ headless: true });
const page = await browser.newPage({ viewport: { width: 1000, height: 900 }, deviceScaleFactor: 1 });
const errors = [];
page.on('pageerror', error => errors.push(error.message));
try {
  await page.goto('http://127.0.0.1:8765');
  await page.waitForFunction(() => !document.querySelector('#loading'));
  await page.addStyleTag({ content: '#viewport{min-height:430px;max-height:430px}.competitor,.intro,header{display:none}.workbench{display:block}.arena-panel{border:0}main{padding:12px}' });
  const montage = await browser.newPage({ viewport: { width: 1200, height: 900 } });
  for (const path of process.argv.slice(2)) {
    const replay = JSON.parse(await readFile(path, 'utf8'));
    await page.locator('#load-replay').setInputFiles(resolve(path));
    await page.waitForFunction(expected => document.querySelector('#replay-source').textContent.includes(expected.name) &&
      document.querySelector('#code0').value === expected.programs[0] && document.querySelector('#code1').value === expected.programs[1] &&
      Number(document.querySelector('#scrub').max) === expected.last && document.querySelector('#clock').textContent === expected.end,
    { name: path.split(/[\\/]/).at(-1), programs: replay.metadata.programs, last: replay.frames.length-1, end: replay.frames.at(-1).time.toFixed(2) });
    if (await page.locator('#error').textContent()) throw new Error(await page.locator('#error').textContent());
    const end = replay.frames.at(-1).time;
    const firstContact = replay.events.find(event => event.type === 'contact_started');
    const moments = [['First contact', firstContact?.time ?? Math.min(5, end)], ['2 seconds before resolution', Math.max(0, end - 2)], ['0.2 seconds before resolution', Math.max(0, end - .2)], ['Resolution', end]];
    const captures = [];
    for (const [label, target] of moments) {
      let index = 0;
      replay.frames.forEach((frame, i) => { if (Math.abs(frame.time-target) < Math.abs(replay.frames[index].time-target)) index = i; });
      await page.locator('#scrub').evaluate((input, value) => { input.value = value; input.dispatchEvent(new Event('input')); }, index);
      await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
      const png = await page.locator('#viewport').screenshot();
      const time = replay.frames[index].time;
      const nearby = replay.events.filter(event => event.time <= time && ['program_log', 'contact_started', 'contact_ended', 'match_ended'].includes(event.type)).slice(-2);
      captures.push({ label: `${label} · ${time.toFixed(3)} s`, image: png.toString('base64'), events: nearby.map(event => `${event.time.toFixed(3)} s ${event.bot === 0 ? 'Blue ' : event.bot === 1 ? 'Red ' : ''}${event.message || event.type}`).join(' | ') });
    }
    await montage.setContent('<style>*{box-sizing:border-box}body{margin:0;padding:16px;background:#eaf0f5;color:#182c3c;font:14px Consolas,monospace}h1{font-size:18px}main{display:grid;grid-template-columns:1fr 1fr;gap:14px}figure{margin:0;background:#fff;border:1px solid #b9c9d5}img{width:100%;display:block}figcaption{padding:10px}p{font-size:11px;line-height:1.5;margin:8px 0 0;min-height:32px}</style><h1></h1><main></main>');
    await montage.evaluate(({ title, captures }) => {
      document.querySelector('h1').textContent = title;
      for (const capture of captures) {
        const figure = document.createElement('figure'), image = document.createElement('img'), caption = document.createElement('figcaption'), events = document.createElement('p');
        image.src = `data:image/png;base64,${capture.image}`;
        caption.textContent = capture.label; events.textContent = capture.events;
        caption.append(events); figure.append(image, caption); document.querySelector('main').append(figure);
      }
    }, { title: `${path} · seed ${replay.metadata.seed} · ${JSON.stringify(replay.frames.at(-1).result)}`, captures });
    await montage.evaluate(() => Promise.all([...document.images].map(image => image.decode())));
    const output = path.replace(/\.json$/i, '') + '-sequence.png';
    await montage.screenshot({ path: output, fullPage: true });
    console.log(JSON.stringify({ replay: path, output, moments: captures.map(capture => capture.label) }));
  }
  if (errors.length) throw new Error(errors.join('\n'));
} finally { await browser.close(); }
