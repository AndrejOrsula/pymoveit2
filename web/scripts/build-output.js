import { lstatSync, mkdirSync, realpathSync, rmSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, isAbsolute, join, relative, resolve, sep } from 'node:path';

export const BUILD_MARKER = '.pymoveit2-demo-build';
const BUILD_MARKER_FORMAT = 'pymoveit2-demo-build';

function markerContent(path, state) {
  return `${JSON.stringify({ format: BUILD_MARKER_FORMAT, version: 1, path, state })}\n`;
}

function canonicalPath(path) {
  const absolute = resolve(path);
  let existing = absolute;
  const suffix = [];
  while (true) {
    try {
      const real = realpathSync(existing);
      return resolve(real, ...suffix.reverse());
    } catch (error) {
      if (error.code !== 'ENOENT' && error.code !== 'ENOTDIR') throw error;
      const parent = dirname(existing);
      if (parent === existing) throw error;
      suffix.push(existing.slice(parent.length + (parent.endsWith(sep) ? 0 : 1)));
      existing = parent;
    }
  }
}

function overlaps(path, protectedPath) {
  const within = (parent, child) => {
    const fromParent = relative(parent, child);
    return fromParent === '' || (fromParent !== '..' && !fromParent.startsWith(`..${sep}`) && !isAbsolute(fromParent));
  };
  return within(path, protectedPath) || within(protectedPath, path);
}

export function prepareOutput(output, webRoot) {
  const out = canonicalPath(output);
  const web = canonicalPath(webRoot);
  const protectedPaths = [...['src', 'static', 'scripts', 'test', 'examples', 'node_modules'].map(name => join(web, name)),
    ...['index.html', 'package.json', 'package-lock.json'].map(name => join(web, name))];
  if (protectedPaths.some(path => overlaps(out, canonicalPath(path)))) {
    throw new Error(`Unsafe build output path: ${output} overlaps the project or its source files`);
  }

  let exists = false;
  try {
    const stat = lstatSync(out);
    if (!stat.isDirectory()) throw new Error(`Unsafe build output path: ${output} is not a directory`);
    exists = true;
  } catch (error) {
    if (error.code !== 'ENOENT') throw error;
  }

  let markerWritten = false;
  if (exists) {
    const entries = readdirSync(out);
    let marker = null;
    if (entries.includes(BUILD_MARKER)) {
      const markerPath = join(out, BUILD_MARKER);
      const markerStat = lstatSync(markerPath);
      if (!markerStat.isFile() || markerStat.isSymbolicLink()) {
        throw new Error(`Unsafe build output path: ${output} has a malformed build marker`);
      }
      marker = readFileSync(markerPath, 'utf8');
    }
    let metadata;
    try {
      metadata = marker === null ? null : JSON.parse(marker);
    } catch {
      metadata = null;
    }
    const marked = metadata?.format === BUILD_MARKER_FORMAT && metadata.version === 1
      && metadata.path === out && ['in-progress', 'success'].includes(metadata.state);
    if (entries.length !== 0 && !marked) {
      throw new Error(`Unsafe build output path: ${output} is populated and is not a recognized pymoveit2 demo build`);
    }
    if (marked) {
      writeFileSync(join(out, BUILD_MARKER), markerContent(out, 'in-progress'));
      markerWritten = true;
    }
    for (const entry of entries) {
      if (entry !== BUILD_MARKER) rmSync(join(out, entry), { recursive: true, force: true });
    }
  }
  mkdirSync(out, { recursive: true });
  if (!markerWritten) {
    writeFileSync(join(out, BUILD_MARKER), markerContent(out, 'in-progress'));
  }
  return out;
}

export function markSuccessfulBuild(output) {
  writeFileSync(join(output, BUILD_MARKER), markerContent(output, 'success'));
}
