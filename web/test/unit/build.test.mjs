import test from 'node:test';
import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import { createServer } from 'node:net';
import {
  cpSync, existsSync, lstatSync, mkdirSync, mkdtempSync, readFileSync, rmSync, symlinkSync, writeFileSync,
} from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const repositoryWeb = fileURLToPath(new URL('../..', import.meta.url));
const repositoryNodeModules = join(repositoryWeb, 'node_modules');

function fixture(t, { broken = false } = {}) {
  const root = mkdtempSync(join(tmpdir(), 'pymoveit2-build-'));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const web = join(root, 'web');
  const scripts = join(web, 'scripts');
  mkdirSync(join(web, 'src', 'py'), { recursive: true });
  mkdirSync(join(web, 'static'), { recursive: true });
  mkdirSync(join(web, 'node_modules'), { recursive: true });
  mkdirSync(scripts, { recursive: true });
  writeFileSync(join(web, 'package.json'), '{"type":"module"}\n');
  for (const dependency of ['esbuild', 'three', 'pyodide']) {
    symlinkSync(join(repositoryNodeModules, dependency), join(web, 'node_modules', dependency), 'dir');
  }
  writeFileSync(join(root, 'pyproject.toml'), 'version = "5.0.1"\n');
  writeFileSync(join(web, 'index.html'), '<!doctype html><title>{{PYMOVEIT2_VERSION}}</title>');
  writeFileSync(join(web, 'src', 'main.js'), broken ? 'export default {' : 'console.log("demo");');
  writeFileSync(join(web, 'src', 'py', 'worker.js'), 'console.log("worker");');
  writeFileSync(join(web, 'src', 'sentinel'), 'source survives');
  writeFileSync(join(web, 'static', 'sentinel'), 'static survives');
  cpSync(join(repositoryWeb, 'scripts', 'build.mjs'), join(scripts, 'build.mjs'));
  cpSync(join(repositoryWeb, 'scripts', 'build-output.js'), join(scripts, 'build-output.js'));
  return { root, web, build: join(scripts, 'build.mjs') };
}

function runBuild(build, args) {
  return spawnSync(process.execPath, [build, ...args], { encoding: 'utf8' });
}

async function availablePort() {
  const server = createServer();
  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', resolve);
  });
  const { port } = server.address();
  await new Promise((resolve, reject) => server.close(error => error ? reject(error) : resolve()));
  return port;
}

async function waitFor(predicate, child, output, timeoutMessage) {
  const deadline = Date.now() + 8000;
  while (Date.now() < deadline) {
    if (child.exitCode !== null) throw new Error(`${timeoutMessage}: child exited (${child.exitCode}); ${output()}`);
    const value = await predicate();
    if (value) return value;
    await new Promise(resolve => setTimeout(resolve, 20));
  }
  throw new Error(`${timeoutMessage}: timed out; ${output()}`);
}

function assertRejectedWithoutChanges(build, out, sentinel, args = []) {
  const result = runBuild(build, ['--out', out, ...args]);
  assert.notEqual(result.status, 0, result.stdout);
  assert.match(result.stderr, /unsafe build output path/i);
  assert.equal(readFileSync(sentinel, 'utf8'), 'source survives');
}

test('CLI rejects source, marker-owned source, node_modules and canonical parent aliases safely', (t) => {
  const { root, web, build } = fixture(t);
  const source = join(web, 'src');
  const sentinel = join(source, 'sentinel');
  const marker = join(source, '.pymoveit2-demo-build');
  const nodeModulesSentinel = join(web, 'node_modules', 'sentinel');
  writeFileSync(nodeModulesSentinel, 'dependency survives');
  try {
    assertRejectedWithoutChanges(build, source, sentinel);
    writeFileSync(marker, `${JSON.stringify({
      format: 'pymoveit2-demo-build', version: 1, path: source, state: 'success',
    })}\n`);
    assertRejectedWithoutChanges(build, source, sentinel);
    const alias = join(root, 'web-alias');
    symlinkSync(web, alias, 'dir');
    assertRejectedWithoutChanges(build, join(alias, 'src'), sentinel);
    const outputAlias = join(root, 'src-alias');
    symlinkSync(source, outputAlias, 'dir');
    assertRejectedWithoutChanges(build, outputAlias, sentinel);
    assertRejectedWithoutChanges(build, root, sentinel);
    const modulesResult = runBuild(build, ['--out', join(web, 'node_modules')]);
    assert.notEqual(modulesResult.status, 0);
    assert.match(modulesResult.stderr, /unsafe build output path/i);
    assert.equal(readFileSync(nodeModulesSentinel, 'utf8'), 'dependency survives');
  } finally {
    rmSync(nodeModulesSentinel, { force: true });
    rmSync(marker, { force: true });
  }
});

test('CLI rejects populated unowned output and malformed marker without damage', (t) => {
  const { root, build } = fixture(t);
  const out = join(root, 'populated');
  mkdirSync(out);
  const sentinel = join(out, 'sentinel');
  writeFileSync(sentinel, 'keep');
  const result = runBuild(build, ['--out', out]);
  assert.notEqual(result.status, 0);
  assert.match(result.stderr, /populated and is not a recognized pymoveit2 demo build/i);
  assert.equal(readFileSync(sentinel, 'utf8'), 'keep');

  rmSync(sentinel);
  mkdirSync(join(out, '.pymoveit2-demo-build'));
  const malformed = runBuild(build, ['--out', out]);
  assert.notEqual(malformed.status, 0);
  assert.match(malformed.stderr, /malformed build marker/i);
  assert.equal(lstatSync(join(out, '.pymoveit2-demo-build')).isDirectory(), true);

  rmSync(join(out, '.pymoveit2-demo-build'), { recursive: true });
  const markerTarget = join(root, 'marker-target');
  writeFileSync(markerTarget, 'pymoveit2-demo-build');
  symlinkSync(markerTarget, join(out, '.pymoveit2-demo-build'));
  const symlinkMarker = runBuild(build, ['--out', out]);
  assert.notEqual(symlinkMarker.status, 0);
  assert.match(symlinkMarker.stderr, /malformed build marker/i);
  assert.equal(readFileSync(markerTarget, 'utf8'), 'pymoveit2-demo-build');
});

test('CLI writes its own marker and repeats a successful build', (t) => {
  const { root, build } = fixture(t);
  const empty = join(root, 'empty-dist');
  mkdirSync(empty, { mode: 0o750 });
  const emptyBefore = lstatSync(empty);
  const emptyBuild = runBuild(build, ['--out', empty]);
  assert.equal(emptyBuild.status, 0, emptyBuild.stderr);
  assert.equal(existsSync(join(empty, 'index.html')), true);
  const emptyAfter = lstatSync(empty);
  assert.equal(emptyAfter.ino, emptyBefore.ino);
  assert.equal(emptyAfter.dev, emptyBefore.dev);
  assert.equal(emptyAfter.mode, emptyBefore.mode);

  const out = join(root, 'dist');
  const first = runBuild(build, ['--out', out]);
  assert.equal(first.status, 0, first.stderr);
  const successMarker = JSON.parse(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8'));
  assert.equal(successMarker.state, 'success');
  assert.equal(successMarker.path, out);
  writeFileSync(join(out, 'stale-output'), 'remove on rebuild');
  const beforeRebuild = lstatSync(out);
  const second = runBuild(build, ['--out', out]);
  assert.equal(second.status, 0, second.stderr);
  const afterRebuild = lstatSync(out);
  assert.equal(afterRebuild.ino, beforeRebuild.ino);
  assert.equal(afterRebuild.dev, beforeRebuild.dev);
  assert.equal(afterRebuild.mode, beforeRebuild.mode);
  assert.equal(existsSync(join(out, 'stale-output')), false);
  assert.equal(existsSync(join(out, 'index.html')), true);

  const copied = join(root, 'copied-build');
  cpSync(out, copied, { recursive: true });
  writeFileSync(join(copied, 'sentinel'), 'copied output survives');
  const copiedResult = runBuild(build, ['--out', copied]);
  assert.notEqual(copiedResult.status, 0);
  assert.match(copiedResult.stderr, /populated and is not a recognized pymoveit2 demo build/i);
  assert.equal(readFileSync(join(copied, 'sentinel'), 'utf8'), 'copied output survives');
});

test('CLI failed first build stays recognized and can be retried', (t) => {
  const { root, web, build } = fixture(t, { broken: true });
  const out = join(root, 'dist');
  const failed = runBuild(build, ['--out', out]);
  assert.notEqual(failed.status, 0);
  assert.equal(JSON.parse(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8')).state, 'in-progress');
  writeFileSync(join(web, 'src', 'main.js'), 'console.log("fixed");');
  const retried = runBuild(build, ['--out', out]);
  assert.equal(retried.status, 0, retried.stderr);
  assert.equal(JSON.parse(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8')).state, 'success');
});

test('CLI cleanup failure keeps the output recognized for retry', (t) => {
  const { root, build } = fixture(t);
  const out = join(root, 'dist');
  const initial = runBuild(build, ['--out', out]);
  assert.equal(initial.status, 0, initial.stderr);

  const blocked = join(out, 'blocked-cleanup');
  mkdirSync(blocked);
  writeFileSync(join(blocked, 'stale'), 'retry removes this');
  const preload = join(root, 'fail-rm-sync.cjs');
  writeFileSync(preload, `
    const fs = require('node:fs');
    const { syncBuiltinESMExports } = require('node:module');
    const rmSync = fs.rmSync;
    const readdirSync = fs.readdirSync;
    fs.readdirSync = function (path, options) {
      const entries = readdirSync.call(this, path, options);
      if (String(path) !== process.env.PYMOVEIT2_ORDER_READDIR || entries.length === 0) return entries;
      const marker = entries.findIndex(entry => String(entry) === '.pymoveit2-demo-build');
      return marker < 0 ? entries : [entries[marker], ...entries.slice(0, marker), ...entries.slice(marker + 1)];
    };
    fs.rmSync = function (path, options) {
      if (String(path) === process.env.PYMOVEIT2_FAIL_RM) {
        const error = new Error('injected cleanup failure');
        error.code = 'EACCES';
        throw error;
      }
      return rmSync.call(this, path, options);
    };
    syncBuiltinESMExports();
  `);

  const failedCleanup = spawnSync(process.execPath, [build, '--out', out], {
    encoding: 'utf8',
    env: {
      ...process.env,
      NODE_OPTIONS: `${process.env.NODE_OPTIONS ?? ''} --require ${preload}`.trim(),
      PYMOVEIT2_FAIL_RM: blocked,
      PYMOVEIT2_ORDER_READDIR: out,
    },
  });
  assert.notEqual(failedCleanup.status, 0);
  assert.match(failedCleanup.stderr, /injected cleanup failure/);
  assert.equal(JSON.parse(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8')).state, 'in-progress');
  assert.equal(readFileSync(join(blocked, 'stale'), 'utf8'), 'retry removes this');

  const retried = runBuild(build, ['--out', out]);
  assert.equal(retried.status, 0, retried.stderr);
  assert.equal(existsSync(blocked), false);
  assert.equal(existsSync(join(out, 'index.html')), true);
  assert.equal(JSON.parse(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8')).state, 'success');
});

test('serve CLI serves initial output and watch rebuilds with an idempotent marker', async (t) => {
  const { root, web, build } = fixture(t);
  const out = join(root, 'serve-dist');
  const port = await availablePort();
  const child = spawn(process.execPath, [build, '--serve', String(port), '--out', out], {
    cwd: root,
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  let stdout = '';
  let stderr = '';
  child.stdout.setEncoding('utf8').on('data', chunk => { stdout += chunk; });
  child.stderr.setEncoding('utf8').on('data', chunk => { stderr += chunk; });
  const output = () => `${stdout}${stderr}`;
  const closed = new Promise(resolve => child.once('close', (code, signal) => resolve({ code, signal })));

  try {
    await waitFor(() => stdout.includes(`:${port}/`), child, output, 'serve did not become ready');
    const url = `http://127.0.0.1:${port}/app.js`;
    const initial = await fetch(url);
    assert.equal(initial.status, 200);
    assert.match(await initial.text(), /console\.log\("demo"\)/);
    const markerBefore = readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8');
    assert.equal(JSON.parse(markerBefore).state, 'success');

    writeFileSync(join(web, 'src', 'main.js'), 'console.log("watch-rebuild-token");');
    const rebuilt = await waitFor(async () => {
      const response = await fetch(url);
      if (!response.ok) return null;
      const body = await response.text();
      return body.includes('watch-rebuild-token') ? body : null;
    }, child, output, 'serve did not publish the watched rebuild');
    assert.match(rebuilt, /watch-rebuild-token/);
    assert.equal(readFileSync(join(out, '.pymoveit2-demo-build'), 'utf8'), markerBefore);
  } finally {
    if (child.exitCode === null) child.kill('SIGTERM');
    const result = await Promise.race([
      closed,
      new Promise(resolve => setTimeout(() => resolve(null), 2000)),
    ]);
    if (result === null && child.exitCode === null) {
      child.kill('SIGKILL');
      await closed;
    }
  }
});

test('missing option values fail before build or serve output changes', (t) => {
  const { root, build } = fixture(t);
  for (const mode of [[], ['--serve']]) {
    const out = join(root, mode.length ? 'serve-out' : 'build-out');
    mkdirSync(out);
    writeFileSync(join(out, 'sentinel'), 'keep');
    const result = runBuild(build, [...mode, '--out', out, '--url']);
    assert.notEqual(result.status, 0);
    assert.match(result.stderr, /--url requires a value/);
    assert.equal(readFileSync(join(out, 'sentinel'), 'utf8'), 'keep');
  }
});
