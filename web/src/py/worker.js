// Web Worker that runs Python scripts with Pyodide, loaded from jsDelivr the
// first time a visitor runs a script. Messages:
//   in:  {type: 'load'} | {type: 'run', id, source, snapshot}
//   out: {type: 'loading', text} | {type: 'ready', version} | {type: 'progress', id, line, time}
//        {type: 'result', id, result} | {type: 'failed', id?, message}

import { ScriptRunner } from './runner.js';
import SHIM from './pymoveit2_shim.py';

// eslint-disable-next-line no-undef
const PYODIDE_URL = `https://cdn.jsdelivr.net/pyodide/v${__PYODIDE_VERSION__}/full/`;

let runner = null;
let loading = null;

function load() {
  loading ??= (async () => {
    self.postMessage({ type: 'loading', text: 'Downloading Python (Pyodide, about 10 MB)…' });
    const { loadPyodide } = await import(`${PYODIDE_URL}pyodide.mjs`);
    const pyodide = await loadPyodide({ indexURL: PYODIDE_URL });
    self.postMessage({ type: 'loading', text: 'Starting Python…' });
    runner = new ScriptRunner(pyodide, {
      shim: SHIM,
      onProgress: (progress) => self.postMessage({ type: 'progress', id: current, ...progress }),
    });
    self.postMessage({ type: 'ready', version: pyodide.version });
  })();
  return loading;
}

let current = null;

self.addEventListener('message', async ({ data }) => {
  try {
    if (data.type === 'load') {
      await load();
    } else if (data.type === 'run') {
      await load();
      current = data.id;
      const result = runner.run(data.source, data.snapshot);
      self.postMessage({ type: 'result', id: data.id, result });
    }
  } catch (error) {
    loading = runner ? loading : null;
    self.postMessage({ type: 'failed', id: data.id, message: String(error?.message ?? error) });
  }
});
