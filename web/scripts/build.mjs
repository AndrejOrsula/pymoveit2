// Builds the static demo into web/dist (or --out DIR): bundles src/ with
// esbuild, copies index.html and static/, and writes build-info.json.
//
//   node scripts/build.mjs                 build once
//   node scripts/build.mjs --serve [PORT]  rebuild on change and serve on PORT (8000)
//   node scripts/build.mjs --out DIR       build into DIR
//   node scripts/build.mjs --url URL       set the canonical URL of the published site
//
// Every URL in the output is relative, so the site works under any path,
// such as /pymoveit2/ on GitHub Pages. Only the canonical and og:url links
// name the published address, https://andrejorsula.github.io/pymoveit2/ unless
// --url says otherwise.

import { execFileSync } from 'node:child_process';
import { cpSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import * as esbuild from 'esbuild';
import { markSuccessfulBuild, prepareOutput } from './build-output.js';

const web = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const serve = args.includes('--serve');
const outIndex = args.indexOf('--out');
const urlIndex = args.indexOf('--url');
function optionValue(index, option) {
  const value = args[index + 1];
  if (!value || value.startsWith('--')) throw new Error(`${option} requires a value`);
  return value;
}
const siteUrl = new URL(urlIndex >= 0 ? optionValue(urlIndex, '--url') : 'https://andrejorsula.github.io/pymoveit2/').href;
const port = serve ? Number(args[args.indexOf('--serve') + 1]) || 8000 : 0;
const output = outIndex >= 0 ? optionValue(outIndex, '--out') : join(web, 'dist');

function version(pkg) {
  return JSON.parse(readFileSync(join(web, 'node_modules', pkg, 'package.json'), 'utf8')).version;
}

function pymoveit2Version() {
  const match = readFileSync(join(web, '..', 'pyproject.toml'), 'utf8').match(/^version = "([^"]+)"/m);
  if (!match) throw new Error('pyproject.toml has no version');
  return match[1];
}

function commit() {
  if (process.env.GITHUB_SHA) return process.env.GITHUB_SHA.slice(0, 12);
  try {
    const git = (...rest) => execFileSync('git', ['-C', join(web, '..'), ...rest], { encoding: 'utf8' }).trim();
    // A build of uncommitted work says so instead of claiming the last commit.
    const dirty = git('status', '--porcelain', '--', 'web', 'pymoveit2', 'pyproject.toml', 'examples/assets') !== '';
    return git('rev-parse', '--short=12', 'HEAD') + (dirty ? '-dirty' : '');
  } catch {
    return 'unknown';
  }
}

const info = {
  pymoveit2: pymoveit2Version(),
  commit: commit(),
  three: version('three'),
  pyodide: version('pyodide'),
};
const out = prepareOutput(output, web);

function copyStatic() {
  cpSync(join(web, 'static'), out, { recursive: true });
  const html = readFileSync(join(web, 'index.html'), 'utf8')
    .replaceAll('{{SITE_URL}}', siteUrl)
    .replaceAll('{{PYMOVEIT2_VERSION}}', info.pymoveit2)
    .replaceAll('{{COMMIT}}', info.commit);
  writeFileSync(join(out, 'index.html'), html);
  writeFileSync(join(out, 'build-info.json'), `${JSON.stringify(info, null, 2)}\n`);
  writeFileSync(join(out, '.nojekyll'), '');
  markSuccessfulBuild(out);
}

const options = {
  absWorkingDir: web,
  entryPoints: { app: 'src/main.js', 'py-worker': 'src/py/worker.js' },
  outdir: out,
  bundle: true,
  format: 'esm',
  target: 'es2022',
  minify: !serve,
  sourcemap: serve ? 'inline' : false,
  legalComments: 'linked',
  loader: { '.py': 'text', '.glb': 'file' },
  define: {
    __PYODIDE_VERSION__: JSON.stringify(info.pyodide),
    __BUILD_INFO__: JSON.stringify(info),
  },
  logLevel: 'info',
  plugins: [
    {
      name: 'static-files',
      setup(build) {
        build.onEnd((result) => {
          if (result.errors.length === 0) copyStatic();
        });
      },
    },
  ],
};

if (serve) {
  const context = await esbuild.context(options);
  await context.watch();
  const { hosts, port: actual } = await context.serve({ servedir: out, port });
  console.log(`Serving ${out} at http://${hosts[0]}:${actual}/`);
} else {
  await esbuild.build(options);
  console.log(`Built ${out} (pymoveit2 ${info.pymoveit2}, ${info.commit})`);
}
