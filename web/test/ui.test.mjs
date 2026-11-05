// Drives the built demo in a headless browser, served under /pymoveit2/ as on
// GitHub Pages, with every request outside that path blocked.
//
//   DEMO_SITE      built site (default: build into a temporary directory)
//   DEMO_ENGINE    chromium (default), firefox or webkit, as installed by
//                  `npx playwright-core install`
//   DEMO_BROWSER   a Chrome or Chromium executable to use instead of Playwright's
//   DEMO_EVIDENCE  directory for screenshots (default: a temporary directory)
//   DEMO_ONLINE=1  also runs a script in real Python (needs jsDelivr for Pyodide)

import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { mkdirSync, mkdtempSync, readFileSync, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { tmpdir } from 'node:os';
import { dirname, extname, join, resolve, sep } from 'node:path';
import { after, before, describe, it } from 'node:test';
import { fileURLToPath } from 'node:url';

import { chromium, firefox, webkit } from 'playwright-core';

const here = dirname(fileURLToPath(import.meta.url));
const ROUTE = '/pymoveit2/';
const ENGINE = process.env.DEMO_ENGINE || 'chromium';
const ONLINE = process.env.DEMO_ONLINE === '1';
const evidence = resolve(process.env.DEMO_EVIDENCE ?? mkdtempSync(join(tmpdir(), 'pymoveit2-demo-ui-')));
const KIT = {
  'demo-kit/theme.js': 'f4fbec7f0ac721f498451ed395ae8d8d553711e0731e99313b750146879bbc3f',
  'demo-kit/tokens.css': 'ce241c3377d7c6803a9489c1c2b035af7e855caabfdb69a70b0d5bae2688d5d2',
};
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json',
  '.glb': 'model/gltf-binary',
  '.txt': 'text/plain; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.woff2': 'font/woff2',
};

let site;
let server;
let origin;
let browser;

function serve(request, response) {
  const path = decodeURIComponent(new URL(request.url, 'http://localhost').pathname);
  if (!path.startsWith(ROUTE)) {
    response.writeHead(404).end();
    return;
  }
  let file = resolve(site, `.${path.slice(ROUTE.length - 1)}`);
  if (file !== site && !file.startsWith(`${site}${sep}`)) {
    response.writeHead(403).end();
    return;
  }
  try {
    if (statSync(file).isDirectory()) file = join(file, 'index.html');
    const body = readFileSync(file);
    response.writeHead(200, { 'content-type': TYPES[extname(file)] ?? 'application/octet-stream' });
    response.end(body);
  } catch {
    response.writeHead(404).end();
  }
}

// Engine notes about code the demo does not own: GPU drivers report headless
// Chromium reading WebGL frames back as stalls; Firefox notes once that
// filtering shadow-map depth textures is driver-defined, and that Pyodide's
// WebAssembly uses the older exception-handling instructions.
const ENGINE_NOTE = /GL Driver Message \(OpenGL, Performance,|WebGL warning: drawArraysInstanced: Depth texture comparison requests|The WebAssembly exception handling 'try' instruction is deprecated/;

function recordProblems(page) {
  const problems = [];
  page.on('console', (message) => {
    if (message.type() !== 'error' && message.type() !== 'warning') return;
    const source = message.location()?.url ?? '';
    if (message.type() === 'warning' && ENGINE_NOTE.test(message.text())) return;
    problems.push(`${message.type()}: ${message.text()}${source ? ` (${source})` : ''}`);
  });
  page.on('pageerror', (error) => problems.push(`pageerror: ${error.message}`));
  return problems;
}

async function openDemo({ width = 1440, height = 900, theme = 'light', query = '', webgl = true, touch = false, delayedPyodide = false } = {}) {
  const context = await browser.newContext({ viewport: { width, height }, deviceScaleFactor: 1, reducedMotion: 'reduce', hasTouch: touch, isMobile: touch });
  // A browser with WebGL turned off returns no context for it.
  if (!webgl) {
    await context.addInitScript(() => {
      const getContext = HTMLCanvasElement.prototype.getContext;
      HTMLCanvasElement.prototype.getContext = function (kind, ...rest) {
        return /^webgl/.test(kind) ? null : getContext.call(this, kind, ...rest);
      };
    });
  }
  const page = await context.newPage();
  const problems = recordProblems(page);
  const external = [];
  let releasePyodide;
  let signalPyodideRequest;
  const pyodideRequest = new Promise((resolve) => { signalPyodideRequest = resolve; });
  const pyodideGate = new Promise((resolve) => { releasePyodide = resolve; });
  await page.route('**/*', (route) => {
    const url = new URL(route.request().url());
    if (url.origin === origin) return route.continue();
    if (delayedPyodide && url.hostname === 'cdn.jsdelivr.net' && url.pathname.endsWith('/pyodide.mjs')) {
      signalPyodideRequest();
      return pyodideGate.then(() => route.fulfill({
        status: 200,
        contentType: 'text/javascript',
        body: `export async function loadPyodide(){return {version:'test',FS:{mkdirTree(){},writeFile(){}},globals:{set(){}},registerJsModule(){},runPython(){return '{"error":null,"exit":null}'},setStdout(){},setStderr(){}}}`,
      })).catch(() => {});
    }
    if (ONLINE && url.hostname === 'cdn.jsdelivr.net') return route.continue();
    external.push(url.href);
    return route.abort();
  });
  await page.goto(`${origin}${ROUTE}${query}#${theme}`);
  await page.waitForFunction(() => document.body.dataset.ready === 'true', null, { timeout: 30_000 });
  return { context, page, problems, external, pyodideRequest, releasePyodide };
}

/** Distinct colors in a screenshot of the scene, decoded by the browser. */
async function paintedColors(page) {
  const png = await page.locator('#scene').screenshot();
  return page.evaluate(async (base64) => {
    const blob = await (await fetch(`data:image/png;base64,${base64}`)).blob();
    const bitmap = await createImageBitmap(blob);
    const canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
    const ctx = canvas.getContext('2d');
    ctx.drawImage(bitmap, 0, 0);
    const { data } = ctx.getImageData(0, 0, bitmap.width, bitmap.height);
    const colors = new Set();
    for (let i = 0; i < data.length; i += 4 * 97) colors.add((data[i] >> 3) * 1024 + (data[i + 1] >> 3) * 32 + (data[i + 2] >> 3));
    return colors.size;
  }, png.toString('base64'));
}

async function shot(page, name) {
  await page.screenshot({ path: join(evidence, `${ENGINE}-${name}.png`) });
}

before(async () => {
  site = process.env.DEMO_SITE ? resolve(process.env.DEMO_SITE) : join(mkdtempSync(join(tmpdir(), 'pymoveit2-demo-site-')), 'site');
  if (!process.env.DEMO_SITE) execFileSync(process.execPath, [join(here, '..', 'scripts', 'build.mjs'), '--out', site], { stdio: 'inherit' });
  mkdirSync(evidence, { recursive: true });
  server = createServer(serve);
  await new Promise((done) => server.listen(0, '127.0.0.1', done));
  origin = `http://127.0.0.1:${server.address().port}`;
  const engine = { chromium, firefox, webkit }[ENGINE];
  browser = await engine.launch({ headless: true, executablePath: ENGINE === 'chromium' ? process.env.DEMO_BROWSER || undefined : undefined });
  console.log(`evidence: ${evidence}`);
});

after(async () => {
  await browser?.close();
  server?.close();
});

describe('pymoveit2 demo', () => {
  it('ships the demo kit unchanged, with relative URLs and no personal links', () => {
    for (const [path, sha] of Object.entries(KIT)) {
      assert.equal(createHash('sha256').update(readFileSync(join(site, path))).digest('hex'), sha, path);
    }
    const html = readFileSync(join(site, 'index.html'), 'utf8');
    assert.ok(html.indexOf('demo-kit/theme.js') < html.indexOf('demo-kit/tokens.css'), 'theme.js loads before tokens.css');
    assert.doesNotMatch(html, /\{\{/);
    for (const match of html.matchAll(/(?:src|href)="([^"]+)"/g)) {
      const url = match[1];
      assert.ok(!url.startsWith('/'), `root-relative URL ${url} would break under ${ROUTE}`);
    }
    // The demo stands on its own: no author's name, and no links to a personal site.
    assert.doesNotMatch(html, /Andrej Orsula/);
    for (const [, url] of html.matchAll(/<(?:a|link rel="icon")[^>]*href="([^"]+)"/g)) {
      assert.doesNotMatch(url, /andrejorsula\.github\.io/, url);
    }
  });

  for (const theme of ['light', 'dark']) {
    it(`loads and paints the robot (${theme}, 1440 px)`, async () => {
      const { context, page, problems, external } = await openDemo({ theme });
      assert.equal(await page.locator('#status-text').textContent(), 'Ready · simulated MoveIt 2');
      assert.equal(await page.evaluate(() => document.documentElement.dataset.theme), theme);
      await page.waitForTimeout(500);
      assert.ok((await paintedColors(page)) > 40, 'the 3D scene is painted');
      await shot(page, `overview-${theme}`);
      // Nothing hidden covers the scene: the canvas and its buttons take the pointer.
      const center = await page.evaluate(() => {
        const box = document.querySelector('#viewport canvas').getBoundingClientRect();
        return document.elementFromPoint(box.x + box.width / 2, box.y + box.height / 2)?.tagName;
      });
      assert.equal(center, 'CANVAS');
      const goal = page.locator('[data-layer="goal"]');
      await goal.click({ timeout: 5_000 });
      assert.equal(await goal.getAttribute('aria-pressed'), 'false');
      assert.deepEqual(problems, []);
      assert.deepEqual(external, []);
      await context.close();
    });
  }

  it('runs the first lesson step and shows its Python and ROS 2 calls', async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('button', { name: 'Start: Hello, robot' }).click();
    await page.locator('[data-run="0"]').click();
    await page.locator('#step-0-result').filter({ hasText: 'True' }).waitFor({ timeout: 30_000 });
    assert.match(await page.locator('#last-call-code').textContent(), /robot\.move_to_pose\(\[0\.4, 0\.0, 0\.4\], \[1\.0, 0\.0, 0\.0, 0\.0\]\)/);
    // A passed step hands focus to the next one, so Enter alone walks the lesson.
    assert.equal(await page.evaluate(() => document.activeElement?.dataset.run), '1');
    await page.keyboard.press('Enter');
    await page.locator('#step-1[data-state="done"]').waitFor({ timeout: 30_000 });
    await page.keyboard.press('Enter');
    await page.locator('#lesson-win').filter({ hasText: '✓ Lesson done · 1 of 12' }).waitFor({ timeout: 30_000 });
    assert.equal(await page.evaluate(() => document.activeElement?.id), 'lesson-next');
    assert.match(await page.locator('#last-call-ros').textContent(), /plan_kinematic_path/);
    assert.match(await page.locator('#timeline-meta').textContent(), /s ·/);
    await shot(page, 'lesson-1');
    assert.equal(await page.locator('#step-0').getAttribute('data-state'), 'done');
    // A box around the hand: the same step now prints False and stays open.
    await page.getByRole('tab', { name: 'Console' }).click();
    await page.locator('#console-input').fill('add box b 0.4 0 0.4');
    await page.locator('#console-input').press('Enter');
    await page.locator('#object-list li[data-id="b"]').waitFor({ state: 'attached' });
    await page.getByRole('tab', { name: 'Learn' }).click();
    await page.locator('[data-run="0"]').click();
    await page.locator('#step-0[data-state="failed"]').waitFor({ timeout: 30_000 });
    assert.equal(await page.locator('#step-0-result').getAttribute('data-ok'), 'false');
    assert.match(await page.locator('#lesson-status').textContent(), /^Step 1 did not give the expected result/);
    assert.equal(await page.locator('#step-0-after').isHidden(), true);
    // A two-line step goes to the console one line per Enter.
    await page.locator('nav [data-open="joints"]').click();
    await page.locator('[data-console="0"]').click();
    const input = page.locator('#console-input');
    assert.equal(await input.inputValue(), 'robot.joint_names');
    await input.press('Enter');
    await page.waitForFunction(() => document.querySelector('#console-input').value === 'robot.joint_state.position', null, { timeout: 30_000 });
    assert.match(await page.locator('#console-output').textContent(), /2 lines: Enter runs each in turn\.[\s\S]*panda_joint1/);
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('switches to the OpenMANIPULATOR-X and runs its lessons in world', async () => {
    const { context, page, problems } = await openDemo({ query: '?lesson=cartesian' });
    await page.locator('#robot').selectOption('open_manipulator_x');
    await page.waitForURL(/\?robot=open_manipulator_x#light$/);
    await page.waitForFunction(() => document.body.dataset.ready === 'true', null, { timeout: 30_000 });
    assert.equal(await page.locator('#robot').inputValue(), 'open_manipulator_x');
    await page.waitForTimeout(500);
    assert.ok((await paintedColors(page)) > 40, 'the 3D scene is painted');
    await page.getByRole('button', { name: 'Start: Hello, robot' }).click();
    await page.locator('[data-run="0"]').click();
    await page.locator('#step-0-result').filter({ hasText: 'True' }).waitFor({ timeout: 30_000 });
    assert.match(await page.locator('#last-call-code').textContent(), /frame_id='world', tolerance_orientation=3\.1416\)/);
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.locator('[data-preset="pick"]').click();
    await page.locator('#object-list li[data-id="cube"]').waitFor();
    await shot(page, 'omx');
    await page.getByRole('tab', { name: 'Learn' }).click();
    await page.locator('[data-overview]').first().click();
    assert.match(await page.locator('#panel-learn').textContent(), /9 lessons · \d+ min/);
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('console runs Python and commander shortcuts', async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('tab', { name: 'Console' }).click();
    const input = page.locator('#console-input');
    await input.fill('robot.arm.max_velocity = 1.0');
    await input.press('Enter');
    await input.fill('go extended');
    await input.press('Enter');
    await page.locator('#console-output .out').filter({ hasText: 'True' }).first().waitFor({ timeout: 30_000 });
    const text = await page.locator('#console-output').textContent();
    assert.match(text, /robot\.move_to_configuration\(robot\.joint_positions\('extended'\)\)/);
    await input.fill('robot.arm.compute_');
    await input.press('Tab');
    assert.ok(await page.locator('#console-completions li').count() >= 2, 'completions are listed');
    await input.press('Escape');
    // help lists each shortcut on its own line, with its description.
    await input.fill('help');
    await input.press('Enter');
    await page.locator('#console-output .help').filter({ hasText: 'go random — move to random joint positions' }).waitFor({ timeout: 5_000 });
    // Enter runs a name typed out in full, though the completion list is open.
    await input.fill('');
    await input.pressSequentially('robot.arm.joint_names');
    await page.locator('#console-completions').waitFor();
    await input.press('Enter');
    await page.locator('#console-output .out').filter({ hasText: 'panda_joint7' }).waitFor({ timeout: 5_000 });
    // A call that raised stays out of the session script.
    await input.fill('robot.move_to_pose(1)');
    await input.press('Enter');
    await page.locator('#console-output .err').filter({ hasText: 'ValueError' }).waitFor({ timeout: 5_000 });
    // Ctrl+C stops a running motion even after typing ahead in the input.
    await input.fill('robot.arm.max_velocity = 0.05');
    await input.press('Enter');
    await input.fill('go ready');
    await input.press('Enter');
    await page.locator('#state-chip[data-state="executing"]').waitFor({ timeout: 30_000 });
    await input.pressSequentially('robot.');
    await input.press('Control+c');
    await page.locator('#state-chip[data-state="idle"]').waitFor({ timeout: 5_000 });
    assert.match(await page.locator('#console-output').textContent(), /KeyboardInterrupt/);
    await shot(page, 'console');
    await page.getByRole('tab', { name: 'Script' }).click();
    const script = await page.locator('#script-source').inputValue();
    // Console lines go into the session script as typed, settings included.
    assert.match(script, /\n {4}robot\.move_to_configuration\(robot\.joint_positions\('extended'\)\)\n/);
    assert.match(script, /\n {4}robot\.arm\.max_velocity = 0\.05\n/);
    assert.doesNotMatch(script, /move_to_pose\(1\)/);
    // The Control tab shows a speed set in the console.
    assert.equal(await page.locator('#max-velocity-out').textContent(), '0.05');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('control panel adds an obstacle and the scene lists it', async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.locator('[data-add="box"]').click();
    await page.locator('#object-list li[data-id="box1"]').waitFor();
    assert.match(await page.locator('#last-call-code').textContent(), /robot\.add_collision_box\('box1'/);
    await page.getByRole('tab', { name: 'Script' }).click();
    assert.match(await page.locator('#script-source').inputValue(), /robot\.add_collision_box\('box1'/);
    await shot(page, 'control');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('keeps the selected object gizmo attached when its mesh is replaced', async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.locator('[data-add="box"]').click();
    await page.locator('#object-list li[data-id="box1"]').waitFor();
    const attached = await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      app.selectObject('box1');
      app.sim.attachObject('box1');
      const attachedHidden = app.viewport.transform.object !== app.viewport.objects.get('box1').mesh;
      app.sim.detachObject('box1');
      const detachedVisible = app.viewport.transform.object === app.viewport.objects.get('box1').mesh;
      const oldMesh = app.viewport.objects.get('box1').mesh;
      const transform = app.viewport.transform;
      let commits = 0;
      app.viewport.addEventListener('object-moved', () => { commits += 1; });
      transform.axis = 'X';
      transform.dragging = true;
      const beforeAttach = {
        position: oldMesh.position.toArray(),
        quaternion: oldMesh.quaternion.toArray(),
      };
      oldMesh.position.set(0.73, -0.21, 0.62);
      oldMesh.quaternion.set(0, Math.sin(Math.PI / 6), 0, Math.cos(Math.PI / 6));
      app.sim.attachObject('box1');
      const activeAttachCanceled = !transform.dragging && transform.object !== oldMesh && commits === 0;
      const attachedPoseReset = oldMesh.position.toArray().every((value, i) => Math.abs(value - beforeAttach.position[i]) < 1e-9)
        && Math.abs(oldMesh.quaternion.toArray().reduce((sum, value, i) => sum + value * beforeAttach.quaternion[i], 0)) > 1 - 1e-9;
      app.sim.detachObject('box1');
      const gizmoRestored = transform.object === oldMesh;
      transform.axis = 'X';
      transform.dragging = true;
      const object = app.sim.getObjects().find(({ id }) => id === 'box1');
      app.sim.applyCollisionObject({ ...object, dimensions: [0.4, 0.4, 0.4] });
      const newMesh = app.viewport.objects.get('box1').mesh;
      const replacedAttached = oldMesh !== newMesh && transform.object === newMesh;
      const canceledDrag = !transform.dragging && commits === 0;
      transform.axis = 'X';
      transform.dragging = true;
      transform.pointerUp(null);
      const nextDragCommits = commits;
      let changedId = null;
      app.viewport.addEventListener('object-change', ({ detail }) => { changedId = detail.id; }, { once: true });
      newMesh.position.x += 0.1;
      transform.dispatchEvent({ type: 'objectChange' });
      app.sim.applyCollisionObject({ id: 'box1', operation: 'remove' });
      return { attachedHidden, detachedVisible, activeAttachCanceled, attachedPoseReset, gizmoRestored, replacedAttached, canceledDrag, nextDragCommits, changedId, removedSelectionCleared: app.selectedObject === null };
    });
    assert.deepEqual(attached, { attachedHidden: true, detachedVisible: true, activeAttachCanceled: true, attachedPoseReset: true, gizmoRestored: true, replacedAttached: true, canceledDrag: true, nextDragCommits: 1, changedId: 'box1', removedSelectionCleared: true });
    assert.deepEqual(problems, []);
    await context.close();
  });

  for (const robot of ['panda', 'open_manipulator_x']) {
    it(`keeps attached ${robot} meshes on the moving robot and leaves world objects editable`, async () => {
      const { context, page, problems } = await openDemo({ query: robot === 'panda' ? '' : '?robot=open_manipulator_x' });
      const motion = await page.evaluate(() => {
        const app = window.pymoveit2Demo;
        const link = app.model.end_effector_name;
        const before = app.sim.getState();
        const objectPose = structuredClone(before.ee);
        objectPose.position[0] += 0.15 * app.model.view.scale;
        app.sim.applyCollisionObject({
          id: `attached-${app.model.id}`,
          type: 'box',
          dimensions: [0.04, 0.04, 0.04],
          allowed: true,
          pose: objectPose,
        });
        app.sim.attachObject(`attached-${app.model.id}`, link);
        const mesh = app.viewport.objects.get(`attached-${app.model.id}`).mesh;
        let stepSceneEvents = 0;
        app.addEventListener('scene', () => { stepSceneEvents += 1; });
        app.sim.servoEnable();
        app.sim.servoJog([app.model.joint_names[0]], [0.5]);
        const after = app.sim.step(0.05);
        const firstStepSceneEvents = stepSceneEvents;
        const expected = app.sim.getObjects().find(({ id }) => id === `attached-${app.model.id}`).pose;
        const pose = () => ({ position: mesh.position.toArray(), quaternion: mesh.quaternion.toArray() });
        const matches = (actual, target) => actual.position.every((value, i) => Math.abs(value - target.position[i]) < 1e-7)
          && Math.abs(actual.quaternion.reduce((sum, value, i) => sum + value * target.quat_xyzw[i], 0)) > 1 - 1e-7;
        const attachedTracksSimulation = matches(pose(), expected);
        const movedRobot = before.q.some((value, i) => Math.abs(value - after.q[i]) > 1e-6);
        app.sim.attachObject(`attached-${app.model.id}`, app.model.fingers.links[0]);
        app.sim.servoDisable();
        const fingerStart = app.sim.fingers;
        const fingerNames = app.model.gripper.joint_names;
        const fingerStartPositions = fingerNames.map(() => fingerStart);
        const fingerEndPositions = fingerNames.map(() => app.model.fingers.upper);
        app.sim.execute({
          joint_names: fingerNames,
          points: [
            { positions: fingerStartPositions, velocities: fingerNames.map(() => 0), accelerations: fingerNames.map(() => 0), time_from_start: 0 },
            { positions: fingerEndPositions, velocities: fingerNames.map(() => 0), accelerations: fingerNames.map(() => 0), time_from_start: 1 },
          ],
        }, 'gripper');
        const fingerStep = app.sim.step(0.05);
        const fingerExpected = app.sim.getObjects().find(({ id }) => id === `attached-${app.model.id}`).pose;
        const fingerAttachmentTracksSimulation = matches(pose(), fingerExpected);
        const movedFingers = Math.abs(fingerStep.fingers - fingerStart) > 1e-6;
        app.sim.cancel('gripper');
        app.sim.servoEnable();
        app.sim.detachObject(`attached-${app.model.id}`);
        const detachedPose = pose();
        app.sim.servoJog([app.model.joint_names[0]], [-0.5]);
        app.sim.step(0.05);
        const detachedStaysInWorld = matches(pose(), {
          position: detachedPose.position,
          quat_xyzw: detachedPose.quaternion,
        });

        const worldId = `world-${app.model.id}`;
        app.sim.applyCollisionObject({ id: worldId, type: 'box', dimensions: [0.04, 0.04, 0.04], pose: structuredClone(app.sim.getState().ee) });
        app.selectObject(worldId);
        const worldMesh = app.viewport.objects.get(worldId).mesh;
        app.viewport.transform.axis = 'X';
        app.viewport.transform.dragging = true;
        worldMesh.position.x += 0.12;
        worldMesh.quaternion.set(Math.sin(0.2), 0, 0, Math.cos(0.2));
        const dragged = { position: worldMesh.position.toArray(), quaternion: worldMesh.quaternion.toArray() };
        app.sim.servoJog([app.model.joint_names[0]], [0.5]);
        const sceneEventsBeforeDragStep = stepSceneEvents;
        app.sim.step(0.05);
        const dragStepSceneEvents = stepSceneEvents - sceneEventsBeforeDragStep;
        const dragSurvivesState = worldMesh.position.toArray().every((v, i) => Math.abs(v - dragged.position[i]) < 1e-9)
          && Math.abs(worldMesh.quaternion.toArray().reduce((sum, value, i) => sum + value * dragged.quaternion[i], 0)) > 1 - 1e-9
          && app.viewport.transform.dragging;
        app.viewport.transform.pointerUp(null);
        app.sim.servoDisable();
        app.sim.attachObject(`attached-${app.model.id}`, link);
        return { attachedTracksSimulation, movedRobot, fingerAttachmentTracksSimulation, movedFingers, detachedStaysInWorld, dragSurvivesState, firstStepSceneEvents, dragStepSceneEvents };
      });
      assert.deepEqual(motion, {
        attachedTracksSimulation: true,
        movedRobot: true,
        fingerAttachmentTracksSimulation: true,
        movedFingers: true,
        detachedStaysInWorld: true,
        dragSurvivesState: true,
        firstStepSceneEvents: 0,
        dragStepSceneEvents: 0,
      });
      // Let the viewport paint the state changes before capturing its canvas.
      await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
      await shot(page, `attached-motion-${robot}`);
      assert.deepEqual(problems, []);
      await context.close();
    });
  }

  it('resolves attached meshes against replay frames while scrubbing', async () => {
    const { context, page, problems } = await openDemo();
    const replay = await page.evaluate(async () => {
      const app = window.pymoveit2Demo;
      const initial = app.sim.snapshot();
      const link = app.model.end_effector_name;
      const initialState = app.sim.getState();
      app.sim.applyCollisionObject({ id: 'replay-attached', type: 'box', dimensions: [0.04, 0.04, 0.04], pose: structuredClone(initialState.ee) });
      app.sim.attachObject('replay-attached', link);
      const sceneObject = structuredClone(app.sim.getObjects().find(({ id }) => id === 'replay-attached'));
      const start = [...app.sim.q];
      const end = [...start];
      end[0] += 0.1;
      const checkpoint = app.sim.snapshot();
      app.sim.reset({ ...checkpoint, q: end });
      const expected = app.sim.getObjects().find(({ id }) => id === 'replay-attached').pose;
      app.sim.reset(checkpoint);
      const result = {
        duration: 0.1,
        frames: [
          { t: 0, q: start, fingers: app.sim.fingers, line: 0 },
          { t: 0.1, q: end, fingers: app.sim.fingers, line: 0 },
        ],
        scenes: [{ t: 0, objects: [sceneObject] }],
        plans: [], executions: [], calls: [], logs: [], output: [], snapshot: initial,
      };
      const finished = app.replay(result, { from: 0.1 });
      await Promise.resolve();
      const mesh = app.viewport.objects.get('replay-attached').mesh;
      const displayed = { position: mesh.position.toArray(), quaternion: mesh.quaternion.toArray() };
      const agrees = displayed.position.every((value, i) => Math.abs(value - expected.position[i]) < 1e-7)
        && Math.abs(displayed.quaternion.reduce((sum, value, i) => sum + value * expected.quat_xyzw[i], 0)) > 1 - 1e-7;
      const changedFromStaleScene = displayed.position.some((value, i) => Math.abs(value - sceneObject.pose.position[i]) > 1e-5);
      app.stopReplay();
      await finished;
      return { agrees, changedFromStaleScene };
    });
    assert.deepEqual(replay, { agrees: true, changedFromStaleScene: true });
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('waits for true idle, lets Stop cancel waits and Python startup, and restarts', async () => {
    const { context, page, problems, pyodideRequest, releasePyodide } = await openDemo({ delayedPyodide: true });
    await page.getByRole('tab', { name: 'Script' }).click();
    await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      window.__originalNow = performance.now.bind(performance);
      let now = 0;
      performance.now = () => (now += 3000);
      window.__busyPolls = 0;
      window.__busy = true;
      app.sim.isBusy = () => {
        window.__busyPolls += 1;
        return window.__busy;
      };
    });
    await page.locator('#script-run').click();
    await page.waitForFunction(() => window.__busyPolls >= 5 && window.pymoveit2Demo.scriptPanel.phase === 'waiting', null, { timeout: 5_000 });
    assert.equal(await page.evaluate(() => Boolean(window.pymoveit2Demo.scriptPanel.python.worker)), false);
    await page.locator('#script-stop').click();
    await page.locator('#script-status').filter({ hasText: /^Stopped\./ }).waitFor({ timeout: 5_000 });
    await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      performance.now = window.__originalNow;
      window.__busy = false;
    });
    assert.equal(await page.locator('#script-run').isDisabled(), false, 'Run recovers after Stop during idle wait');

    await page.locator('#script-run').click();
    await pyodideRequest;
    assert.equal(await page.evaluate(() => window.pymoveit2Demo.scriptPanel.phase), 'python', 'Python starts once the robot is idle');
    await page.locator('#script-stop').click();
    await page.locator('#script-status').filter({ hasText: /^Stopped\./ }).waitFor({ timeout: 5_000 });
    assert.equal(await page.locator('#script-run').isDisabled(), false, 'Run recovers after Stop during Python startup');

    releasePyodide();
    await page.locator('#script-run').click();
    await page.locator('#script-status').filter({ hasText: /^Finished/ }).waitFor({ timeout: 10_000 });
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('snapshots the live scene after Python finishes loading', async () => {
    const { context, page, problems, pyodideRequest, releasePyodide } = await openDemo({ delayedPyodide: true });
    await page.getByRole('tab', { name: 'Script' }).click();
    await page.locator('#script-run').click();
    await pyodideRequest;
    const changed = await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      const snapshot = app.sim.snapshot();
      const q = [...snapshot.q];
      q[0] += 0.2;
      app.sim.reset({ ...snapshot, q });
      app.sim.applyCollisionObject({ id: 'during-python-load', type: 'box', dimensions: [0.05, 0.05, 0.05], pose: structuredClone(app.sim.getState().ee) });
      return { q: app.sim.q[0], object: app.sim.getObjects().some(({ id }) => id === 'during-python-load') };
    });
    releasePyodide();
    await page.locator('#script-status').filter({ hasText: /^Finished/ }).waitFor({ timeout: 10_000 });
    const after = await page.evaluate(() => ({
      q: window.pymoveit2Demo.sim.q[0],
      object: window.pymoveit2Demo.sim.getObjects().some(({ id }) => id === 'during-python-load'),
    }));
    assert.ok(Math.abs(after.q - changed.q) < 1e-9, 'the robot state changed during Python loading survives the run');
    assert.equal(after.object, true, 'the scene object added during Python loading survives the run');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('waits for idle after Python loads, and Stop cancels that wait before replay', async () => {
    const { context, page, problems, pyodideRequest, releasePyodide } = await openDemo({ delayedPyodide: true });
    await page.getByRole('tab', { name: 'Script' }).click();
    await page.evaluate(() => {
      window.__scriptRunsPosted = 0;
      const postMessage = Worker.prototype.postMessage;
      Worker.prototype.postMessage = function (message, ...args) {
        if (message?.type === 'run') window.__scriptRunsPosted += 1;
        return postMessage.call(this, message, ...args);
      };
    });
    await page.locator('#script-run').click();
    await pyodideRequest;
    await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      window.__originalNow = performance.now.bind(performance);
      let now = 0;
      performance.now = () => (now += 3000);
      window.__busyPolls = 0;
      window.__busy = true;
      app.sim.isBusy = () => {
        window.__busyPolls += 1;
        return window.__busy;
      };
    });
    releasePyodide();
    await page.waitForFunction(() => window.__busyPolls >= 5, null, { timeout: 5_000 });
    assert.equal(await page.evaluate(() => window.__scriptRunsPosted), 0, 'Python has loaded, but the script has not started while the robot is busy');
    await page.locator('#script-stop').click();
    await page.locator('#script-status').filter({ hasText: /^Stopped\./ }).waitFor({ timeout: 5_000 });
    assert.equal(await page.evaluate(() => window.__scriptRunsPosted), 0, 'Stop prevents the waiting script from reaching the worker');
    assert.equal(await page.evaluate(() => window.pymoveit2Demo.replaying), false, 'Stop does not begin replay');
    await page.evaluate(() => {
      performance.now = window.__originalNow;
      window.__busy = false;
    });
    assert.equal(await page.locator('#script-run').isDisabled(), false, 'Run recovers after Stop during the post-load idle wait');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('holds servo pad commands on Enter and Space, stops on release or focus loss, and quiets repeated announcements', async () => {
    const { context, page, problems } = await openDemo({ webgl: false });
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.locator('#panel-control').waitFor({ state: 'visible' });
    await page.locator('#panel-control summary').filter({ hasText: /^Servo$/ }).click();
    await page.locator('#servo-enable').click();
    await page.waitForFunction(() => window.pymoveit2Demo.sim.servo.enabled, null, { timeout: 5_000 });
    await page.evaluate(() => {
      const app = window.pymoveit2Demo;
      window.servoCalls = [];
      window.servoCallEvents = 0;
      const call = app.call.bind(app);
      app.call = (...args) => {
        if (args[0] === 'robot.servo') window.servoCalls.push({ at: performance.now(), args });
        return call(...args);
      };
      app.addEventListener('call', ({ detail }) => { if (detail.python.startsWith('robot.servo(')) window.servoCallEvents += 1; });
      window.lastCallMutations = 0;
      new MutationObserver(() => { window.lastCallMutations += 1; }).observe(document.querySelector('#last-call'), { subtree: true, childList: true, characterData: true });
    });
    const pad = page.locator('#servo-pad [data-pad="0"]');
    await pad.focus();
    await page.keyboard.down('Enter');
    await page.waitForFunction(() => window.servoCalls.length >= 1, null, { timeout: 5_000 });
    await page.waitForFunction(() => window.pymoveit2Demo.lastPython?.startsWith('robot.servo('), null, { timeout: 5_000 });
    const repeatKeyCalls = await page.evaluate(() => {
      const before = window.servoCalls.length;
      document.activeElement.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true, repeat: true }));
      return { before, after: window.servoCalls.length };
    });
    assert.equal(repeatKeyCalls.after, repeatKeyCalls.before, 'a repeated keydown does not start a duplicate servo timer');
    const beforeRelease = await page.evaluate(() => ({ count: window.servoCalls.length, events: window.servoCallEvents, recorded: window.pymoveit2Demo.recorder.reduce((sum, item) => sum + item.count, 0), text: document.querySelector('#last-call').textContent, mutations: window.lastCallMutations }));
    await page.waitForTimeout(180);
    const duringHold = await page.evaluate(() => ({ count: window.servoCalls.length, events: window.servoCallEvents, recorded: window.pymoveit2Demo.recorder.reduce((sum, item) => sum + item.count, 0), text: document.querySelector('#last-call').textContent, mutations: window.lastCallMutations }));
    assert.ok(duringHold.count > beforeRelease.count, 'servo calls continue while Enter is held');
    assert.ok(duringHold.events > beforeRelease.events, 'successful quiet calls still emit their normal call events');
    assert.ok(duringHold.recorded > beforeRelease.recorded, 'successful quiet calls still update the session recorder');
    assert.equal(duringHold.mutations, beforeRelease.mutations, 'quiet repeats do not rewrite the live announcement');
    assert.equal(duringHold.text, beforeRelease.text, 'the last-call text stays stable during quiet repeats');
    await page.keyboard.up('Enter');
    const releasedAt = await page.evaluate(() => window.servoCalls.length);
    await page.waitForTimeout(130);
    assert.equal(await page.evaluate(() => window.servoCalls.length), releasedAt, 'keyup stops the held command');

    const beforeSpace = await page.evaluate(() => window.servoCalls.length);
    await pad.focus();
    await page.keyboard.down('Space');
    await page.waitForFunction((before) => window.servoCalls.length > before, beforeSpace, { timeout: 5_000 });
    await page.evaluate(() => document.activeElement.blur());
    const blurredAt = await page.evaluate(() => window.servoCalls.length);
    await page.waitForTimeout(130);
    assert.equal(await page.evaluate(() => window.servoCalls.length), blurredAt, 'focus loss stops the held command');
    await page.keyboard.up('Space');
    await page.locator('#servo-disable').click();
    await page.waitForFunction(() => !window.pymoveit2Demo.sim.servo.enabled, null, { timeout: 5_000 });
    await page.evaluate(async () => {
      await window.pymoveit2Demo.call('robot.servo', [], { linear: [0.1, 0, 0], enable_if_disabled: false }, { quiet: true });
    });
    await page.waitForFunction(() => document.querySelector('#last-call-code').dataset.ok === 'false', null, { timeout: 5_000 });
    assert.match(await page.locator('#last-call-code').textContent(), /robot\.servo\(/, 'a quiet failed command remains visible');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('script editor takes clicks anywhere on its text', async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('tab', { name: 'Script' }).click();
    const border = () => page.locator('.editor').evaluate((editor) => getComputedStyle(editor).borderColor);
    const idle = await border();
    const body = await page.locator('.editor-body').boundingBox();
    await page.mouse.click(body.x + 40, body.y + body.height - 20);
    assert.equal(await page.evaluate(() => document.activeElement?.id), 'script-source');
    assert.notEqual(await border(), idle, 'the editor shows that it has focus');
    await page.keyboard.press('Control+End');
    await page.keyboard.type('    robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0], frame_id="panda_link0", cartesian=True)');
    const [area, text] = await page.evaluate(() => {
      const source = document.querySelector('#script-source');
      const pre = source.parentElement.querySelector('pre');
      return [[source.offsetWidth, source.offsetHeight], [pre.scrollWidth, pre.scrollHeight]];
    });
    assert.ok(area[0] >= text[0] && area[1] >= text[1], `the textarea (${area}) covers its text (${text})`);
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('works from the keyboard, and fields keep their last valid value', async () => {
    const { context, page, problems } = await openDemo();
    const focused = () => page.evaluate(() => document.activeElement?.id || document.activeElement?.dataset.run || document.activeElement?.tagName);
    // The arrow keys walk all the tabs; the console's input does not take focus on the way.
    await page.getByRole('tab', { name: 'Learn' }).focus();
    for (let i = 0; i < 4; i++) await page.keyboard.press('ArrowRight');
    assert.equal(await focused(), 'tab-log');
    assert.equal(await page.locator('#tab-log').getAttribute('aria-selected'), 'true');
    await page.keyboard.press('Home');
    // A lesson opened from the keyboard puts focus on its first step, after any scene setup.
    await page.getByRole('button', { name: 'Start: Hello, robot' }).focus();
    await page.keyboard.press('Enter');
    await page.waitForFunction(() => document.activeElement?.dataset.run === '0', null, { timeout: 5_000 });
    for (const id of ['joints', 'poses', 'cartesian']) {
      await page.locator(`nav [data-open="${id}"]`).focus();
      await page.keyboard.press('Enter');
      await page.waitForFunction(() => document.activeElement?.dataset.run === '0' && !document.activeElement.disabled, null, { timeout: 30_000 });
    }
    assert.match(await page.locator('#panel-learn h3').textContent(), /^4\. /);
    await page.locator('[data-overview]').first().focus();
    await page.keyboard.press('Enter');
    await page.waitForFunction(() => document.activeElement?.matches('.hero .btn.primary'), null, { timeout: 5_000 });
    // Clicking the Console tab readies its input.
    await page.getByRole('tab', { name: 'Console' }).click();
    assert.equal(await focused(), 'console-input');
    // An emptied pose field goes back to the goal's value instead of moving the goal to 0.
    await page.getByRole('tab', { name: 'Control' }).click();
    const x = page.locator('#goal-x');
    const before = await x.inputValue();
    await x.fill('');
    await x.press('Tab');
    assert.equal(await x.inputValue(), before);
    // Planning settings stay within the bounds the fields show.
    for (const [id, typed, kept] of [['planning-time', '-1', '0.1'], ['planning-attempts', '50', '20'], ['planning-attempts', '', '20']]) {
      await page.locator(`#${id}`).fill(typed);
      await page.locator(`#${id}`).press('Tab');
      await page.waitForFunction(([id, kept]) => document.getElementById(id).value === kept, [id, kept], { timeout: 5_000 });
    }
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('links open a lesson, and the address bar follows the tabs', async () => {
    const { context, page, problems } = await openDemo({ query: '?lesson=cartesian' });
    await page.locator('#panel-learn h3').filter({ hasText: 'Straight lines' }).waitFor({ timeout: 30_000 });
    await page.locator('[data-run="0"]').click();
    await page.locator('#step-0-result').filter({ hasText: 'True' }).waitFor({ timeout: 30_000 });
    await page.locator('#state-chip[data-state="idle"]').waitFor({ timeout: 30_000 });
    // Pointing at the trajectory plot previews that moment of the motion.
    const plot = await page.locator('#plot').boundingBox();
    await page.mouse.move(plot.x + plot.width * 0.5, plot.y + plot.height * 0.5);
    await page.waitForTimeout(100);
    await shot(page, 'lesson-link-preview');
    await page.mouse.move(0, 0);
    await page.getByRole('tab', { name: 'Console' }).click();
    assert.equal(new URL(page.url()).search, '?tab=console');
    assert.equal(new URL(page.url()).hash, '#light', 'the theme hash survives');
    await page.getByRole('tab', { name: 'Learn' }).click();
    assert.equal(new URL(page.url()).search, '?lesson=cartesian');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('fits a 390 px phone without horizontal scrolling', async () => {
    const { context, page, problems } = await openDemo({ width: 390, height: 844 });
    const overflow = await page.evaluate(() => document.scrollingElement.scrollWidth - window.innerWidth);
    assert.ok(overflow <= 0, `page is ${overflow} px wider than the viewport`);
    assert.ok((await paintedColors(page)) > 40, 'the 3D scene is painted');
    await shot(page, 'phone');
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.screenshot({ path: join(evidence, `${ENGINE}-phone-control.png`), fullPage: true });
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('keeps the lessons and console without WebGL', async () => {
    const { context, page, problems } = await openDemo({ webgl: false });
    assert.match(await page.locator('#viewport-message').textContent(), /WebGL is off/);
    assert.equal(await page.locator('#view-reset').isVisible(), false, 'scene buttons are hidden');
    await page.getByRole('button', { name: 'Start: Hello, robot' }).click();
    await page.locator('[data-run="0"]').click();
    await page.locator('#step-0-result').filter({ hasText: 'True' }).waitFor({ timeout: 30_000 });
    await page.getByRole('tab', { name: 'Control' }).click();
    await page.locator('[data-add="box"]').click();
    await page.locator('#object-list li[data-id="box1"]').waitFor();
    await shot(page, 'no-webgl');
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('a pinch that starts on the goal gizmo zooms rather than moving the goal', { skip: ENGINE !== 'chromium' && 'touch events are sent through Chromium' }, async () => {
    const { context, page, problems } = await openDemo({ width: 412, height: 915, touch: true });
    const marker = () => page.evaluate(() => {
      const viewport = window.pymoveit2Demo.viewport;
      const p = viewport.goal.position.clone().project(viewport.camera);
      const r = viewport.canvas.getBoundingClientRect();
      return [r.left + ((p.x + 1) / 2) * r.width, r.top + ((1 - p.y) / 2) * r.height];
    });
    const view = () => page.evaluate(() => {
      const { camera, controls } = window.pymoveit2Demo.viewport;
      return { distance: camera.position.distanceTo(controls.target), goal: ['x', 'y', 'z'].map((k) => document.querySelector(`#goal-${k}`).value) };
    });
    const before = await view();
    const cdp = await context.newCDPSession(page);
    const touch = (type, points) => cdp.send('Input.dispatchTouchEvent', { type, touchPoints: points.map(([x, y], id) => ({ x, y, id })) });
    const [x, y] = await marker();
    await touch('touchStart', [[x, y]]);
    await touch('touchMove', [[x, y + 6]]);
    await touch('touchStart', [[x, y + 6], [x, y + 80]]);
    for (let i = 1; i <= 10; i++) await touch('touchMove', [[x, y + 6 - 6 * i], [x, y + 80 + 10 * i]]);
    await touch('touchEnd', []);
    await page.waitForTimeout(300);
    const after = await view();
    assert.deepEqual(after.goal, before.goal, 'the goal stays where it was');
    assert.ok(after.distance < before.distance * 0.9, `the view zoomed in (${before.distance.toFixed(3)} -> ${after.distance.toFixed(3)})`);
    // The gizmo works again once the fingers lift.
    const [x2, y2] = await marker();
    await touch('touchStart', [[x2, y2]]);
    for (let i = 1; i <= 5; i++) await touch('touchMove', [[x2 + 8 * i, y2]]);
    await touch('touchEnd', []);
    assert.notDeepEqual((await view()).goal, before.goal, 'one finger drags the goal');
    // A two-finger tap selects nothing, whichever finger lifts first.
    await page.evaluate(() => {
      window.selections = [];
      window.pymoveit2Demo.viewport.addEventListener('select', (event) => window.selections.push(event.detail.kind));
    });
    const box = await page.locator('#scene').boundingBox();
    const [a, b] = [[box.x + 40, box.y + box.height - 40], [box.x + 120, box.y + box.height - 40]];
    const lift = (point, id) => cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [{ x: point[0], y: point[1], id }] });
    await touch('touchStart', [a]);
    await touch('touchStart', [a, b]);
    await lift(b, 1);
    await lift(a, 0);
    await page.waitForTimeout(100);
    assert.deepEqual(await page.evaluate(() => window.selections), []);
    // A one-finger tap on empty floor still clears the selection.
    await touch('touchStart', [a]);
    await touch('touchEnd', []);
    await page.waitForTimeout(100);
    assert.deepEqual(await page.evaluate(() => window.selections), ['none']);
    // On a touch screen the Console tab leaves the keyboard closed until its input is tapped.
    await page.getByRole('tab', { name: 'Console' }).tap();
    assert.notEqual(await page.evaluate(() => document.activeElement?.id), 'console-input');
    // Text fields use 16 px text there, below which iOS Safari zooms the page in on focus.
    const small = await page.evaluate(() => [...document.querySelectorAll('input:not([type="range"], [type="checkbox"], [type="radio"]), textarea, select')]
      .filter((field) => parseFloat(getComputedStyle(field).fontSize) < 16)
      .map((field) => field.id || field.className || field.tagName));
    assert.deepEqual(small, []);
    // A finger on the trajectory plot shows the robot at that moment, and the pose stays after it lifts.
    await page.locator('#console-input').fill('go extended');
    await page.locator('#console-input').press('Enter');
    await page.locator('#state-chip[data-state="idle"]').waitFor({ timeout: 30_000 });
    await page.waitForFunction(() => window.pymoveit2Demo.plot.trajectory && !window.pymoveit2Demo.execution, null, { timeout: 30_000 });
    await page.locator('#plot').scrollIntoViewIfNeeded();
    const plot = await page.locator('#plot').boundingBox();
    const preview = () => page.evaluate(() => [window.pymoveit2Demo.plot.hover !== null, window.pymoveit2Demo.viewport.ghost.root.visible]);
    const mid = plot.y + plot.height / 2;
    await touch('touchStart', [[plot.x + plot.width * 0.2, mid]]);
    for (let i = 1; i <= 8; i++) await touch('touchMove', [[plot.x + plot.width * (0.2 + i * 0.05), mid]]);
    assert.deepEqual(await preview(), [true, true]);
    await touch('touchEnd', []);
    assert.deepEqual(await preview(), [true, true]);
    await page.getByRole('tab', { name: 'Learn' }).tap();
    assert.deepEqual(await preview(), [false, false]);
    assert.deepEqual(problems, []);
    await context.close();
  });

  it('runs a script in real Python and replays it', { skip: !ONLINE && 'set DEMO_ONLINE=1 to download Pyodide' }, async () => {
    const { context, page, problems } = await openDemo();
    await page.getByRole('tab', { name: 'Script' }).click();
    await page.locator('#script-choice').selectOption('quick-start');
    await page.locator('#script-run').click();
    await page.locator('#script-status').filter({ hasText: /Finished/ }).waitFor({ timeout: 180_000 });
    await shot(page, 'script');
    // Dragging the slider back shows the run at that moment; letting go plays it to the end.
    const shown = () => page.evaluate(() => window.pymoveit2Demo.viewport.state.q.map((v) => v.toFixed(3)).join());
    const end = await shown();
    const slider = await page.locator('#scrubber').boundingBox();
    await page.mouse.move(slider.x + slider.width - 4, slider.y + slider.height / 2);
    await page.mouse.down();
    await page.mouse.move(slider.x + slider.width * 0.4, slider.y + slider.height / 2, { steps: 8 });
    assert.equal(await page.evaluate(() => window.pymoveit2Demo.replaying), true);
    assert.notEqual(await shown(), end);
    assert.ok(await page.locator('.editor-body .line-active').count(), 'the running line is marked');
    await page.mouse.up();
    await page.waitForFunction(() => !window.pymoveit2Demo.replaying, null, { timeout: 10_000 });
    assert.equal(await shown(), end);
    // Run ends a replay of the last run, and Stop then stops the new run rather than that replay.
    await page.mouse.click(slider.x + slider.width * 0.3, slider.y + slider.height / 2);
    assert.equal(await page.evaluate(() => window.pymoveit2Demo.replaying), true);
    await page.locator('#script-run').click();
    await page.locator('#script-stop').click();
    await page.locator('#script-status').filter({ hasText: /^Stopped/ }).waitFor({ timeout: 10_000 });
    assert.equal(await page.evaluate(() => window.pymoveit2Demo.replaying), false);
    assert.deepEqual(problems, []);
    await context.close();
  });
});
