// The Script tab: edit a whole pymoveit2 program and run it in real Python
// (Pyodide, in a Web Worker), then watch the robot replay it line by line.

import { EXAMPLES } from './examples.js';
import { escapeHtml, highlightPython } from './highlight.js';

const SESSION = 'session';
// Keep the main-thread watchdog aligned with ScriptRunner.WALL_TIME_LIMIT.
const SCRIPT_WALL_TIMEOUT_MS = 30_000;
const EMPTY_SESSION = `# Your calls on this page appear here.
from pymoveit2 import connect

with connect() as robot:
    pass
`;

/** Talks to the Pyodide worker (src/py/worker.js), created on first use. */
export class PythonWorker {
  constructor({ onStatus }) {
    this.onStatus = onStatus;
    this.worker = null;
    this.pending = new Map();
    this.ready = null;
    this.nextId = 1;
  }

  #ensure() {
    if (this.worker) return;
    const worker = new Worker(new URL('./py-worker.js', import.meta.url), { type: 'module' });
    this.worker = worker;
    worker.addEventListener('message', ({ data }) => {
      if (this.worker === worker) this.#message(data);
    });
    worker.addEventListener('error', (event) => {
      if (this.worker !== worker) return;
      event.preventDefault?.();
      this.#fail(event.message || 'the Python worker failed to start');
    });
  }

  #message(data) {
    if (data.type === 'loading') this.onStatus({ phase: 'loading', text: data.text });
    else if (data.type === 'ready') {
      this.version = data.version;
      this.readyResolve?.();
      this.onStatus({ phase: 'ready', text: `Python ${data.version} ready.` });
    } else if (data.type === 'progress') this.onStatus({ phase: 'running', line: data.line, time: data.time });
    else if (data.type === 'result') {
      this.pending.get(data.id)?.resolve(data.result);
      this.pending.delete(data.id);
    } else if (data.type === 'failed') this.#fail(data.message, data.id);
  }

  #fail(message, id) {
    const error = new Error(message);
    if (id !== undefined && this.pending.has(id)) {
      this.pending.get(id).reject(error);
      this.pending.delete(id);
      return;
    }
    this.readyReject?.(error);
    for (const { reject } of this.pending.values()) reject(error);
    this.pending.clear();
    this.terminate();
  }

  load() {
    this.#ensure();
    this.ready ??= new Promise((resolve, reject) => {
      this.readyResolve = resolve;
      this.readyReject = reject;
      this.worker.postMessage({ type: 'load' });
    });
    return this.ready;
  }

  async run(source, snapshot) {
    const ready = this.load();
    const worker = this.worker;
    await ready;
    if (this.worker !== worker) throw new Error('stopped');
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        const pending = this.pending.get(id);
        if (!pending) return;
        this.pending.delete(id);
        const error = new Error('The script exceeded the 30-second wall limit.');
        error.name = 'TimeoutError';
        pending.reject(error);
        this.terminate();
      }, SCRIPT_WALL_TIMEOUT_MS);
      const finish = (callback) => (value) => {
        clearTimeout(timer);
        callback(value);
      };
      this.pending.set(id, { resolve: finish(resolve), reject: finish(reject) });
      try {
        worker.postMessage({ type: 'run', id, source, snapshot });
      } catch (error) {
        this.pending.get(id)?.reject(error);
        this.pending.delete(id);
      }
    });
  }

  terminate() {
    const stopped = new Error('stopped');
    this.readyReject?.(stopped);
    this.readyResolve = null;
    this.readyReject = null;
    this.worker?.terminate();
    this.worker = null;
    this.ready = null;
    for (const { reject } of this.pending.values()) reject(stopped);
    this.pending.clear();
  }
}

export class ScriptPanel {
  constructor(panel, app) {
    this.app = app;
    this.examples = EXAMPLES[app.model.id];
    this.choice = SESSION;
    this.edited = false;
    this.running = false;
    this.activeLine = null;
    this.errorLine = null;
    const options = [`<option value="${SESSION}">Your session</option>`, ...this.examples.map((example) => `<option value="${example.id}">${escapeHtml(example.title)}</option>`)];
    panel.innerHTML = `
      <div class="btn-row">
        <label class="field" style="flex:1 1 10rem"><select id="script-choice" aria-label="Script">${options.join('')}</select></label>
        <button type="button" class="btn primary" id="script-run">Run</button>
        <button type="button" class="btn danger" id="script-stop" disabled>Stop</button>
      </div>
      <div class="editor">
        <div class="editor-scroll">
          <pre class="editor-gutter" id="script-gutter" aria-hidden="true"></pre>
          <div class="editor-body">
            <pre aria-hidden="true"><code id="script-highlight"></code></pre>
            <textarea id="script-source" spellcheck="false" autocomplete="off" autocapitalize="off" wrap="off" aria-label="Python script"></textarea>
          </div>
        </div>
      </div>
      <div class="btn-row">
        <button type="button" class="btn small" id="script-copy">Copy</button>
        <button type="button" class="btn small" id="script-download">Download .py</button>
        <button type="button" class="btn small" id="script-revert">Revert</button>
        <span class="hint" id="script-status" role="status">Real Python (Pyodide), downloaded on first run.</span>
      </div>
      <div class="run-output" id="script-output" aria-live="polite"></div>`;
    this.$ = (selector) => panel.querySelector(selector);
    this.source = this.$('#script-source');
    this.highlight = this.$('#script-highlight');
    this.gutter = this.$('#script-gutter');
    this.output = this.$('#script-output');
    this.status = this.$('#script-status');
    this.python = new PythonWorker({ onStatus: (status) => this.#onWorker(status) });

    this.$('#script-choice').addEventListener('change', (event) => this.select(event.target.value));
    this.$('#script-run').addEventListener('click', () => this.run());
    this.$('#script-stop').addEventListener('click', () => this.stop());
    this.$('#script-copy').addEventListener('click', () => navigator.clipboard?.writeText(this.source.value).then(() => app.toast('Copied'), () => {}));
    this.$('#script-download').addEventListener('click', () => this.#download());
    this.$('#script-revert').addEventListener('click', () => {
      this.edited = false;
      this.select(this.choice);
    });
    this.source.addEventListener('input', () => {
      this.edited = true;
      this.errorLine = null;
      this.#render();
    });
    this.source.addEventListener('keydown', (event) => this.#key(event));
    // A hidden panel measures as empty, so the editor fits again once shown.
    new ResizeObserver(() => this.#fit()).observe(this.highlight.parentElement.parentElement);
    app.addEventListener('recorded', () => {
      if (this.choice === SESSION && !this.edited && !this.running) this.#setSource(this.#sessionSource());
    });
    this.select(SESSION);
  }

  #sessionSource() {
    const script = this.app.sessionScript();
    return script ?? EMPTY_SESSION;
  }

  select(choice) {
    this.choice = choice;
    this.$('#script-choice').value = choice;
    this.app.updateLink?.();
    this.edited = false;
    this.errorLine = null;
    this.activeLine = null;
    const example = this.examples.find((e) => e.id === choice);
    this.#setSource(example ? example.source : this.#sessionSource());
    this.output.textContent = '';
    this.$('.editor-scroll').scrollTo(0, 0);
  }

  #setSource(text) {
    this.source.value = text;
    this.#render();
  }

  #render() {
    const lines = this.source.value.split('\n');
    this.highlight.innerHTML = lines
      .map((line, i) => {
        const html = highlightPython(line) || ' ';
        return i + 1 === this.activeLine || i + 1 === this.errorLine ? `<span class="line-active"${i + 1 === this.errorLine ? ' data-error="true"' : ''}>${html}</span>` : html;
      })
      .join('\n');
    this.gutter.innerHTML = lines.map((_, i) => `<span${i + 1 === this.activeLine ? ' data-active="true"' : ''}${i + 1 === this.errorLine ? ' data-error="true"' : ''}>${i + 1}</span>`).join('');
    this.#fit();
  }

  /** Sizes the textarea to its text, so the shared scroller scrolls both and every character takes clicks. */
  #fit() {
    const text = this.highlight.parentElement;
    this.source.style.width = `${text.scrollWidth}px`;
    this.source.style.height = `${text.scrollHeight}px`;
  }

  setActiveLine(line) {
    if (line === this.activeLine) return;
    this.activeLine = line;
    this.#render();
    // Scrolls the editor alone: scrollIntoView would also move the page away from the robot.
    const element = this.gutter.querySelector('[data-active="true"]');
    const scroller = this.$('.editor-scroll');
    if (!element) return;
    const box = element.getBoundingClientRect(), view = scroller.getBoundingClientRect();
    if (box.top < view.top) scroller.scrollTop += box.top - view.top;
    else if (box.bottom > view.bottom) scroller.scrollTop += box.bottom - view.bottom;
  }

  #key(event) {
    const area = this.source;
    if (event.key === 'Tab' && !event.shiftKey && !event.ctrlKey && !event.altKey) {
      event.preventDefault();
      this.#insert('    ');
    } else if (event.key === 'Enter' && !event.ctrlKey && !event.metaKey) {
      event.preventDefault();
      const before = area.value.slice(0, area.selectionStart);
      const current = before.slice(before.lastIndexOf('\n') + 1);
      let indent = current.match(/^\s*/)[0];
      if (/:\s*(#.*)?$/.test(current)) indent += '    ';
      this.#insert(`\n${indent}`);
    } else if (event.key === 'Enter' && (event.ctrlKey || event.metaKey)) {
      event.preventDefault();
      this.run();
    }
  }

  #insert(text) {
    const area = this.source;
    const start = area.selectionStart;
    area.setRangeText(text, start, area.selectionEnd, 'end');
    this.edited = true;
    this.#render();
  }

  #download() {
    const blob = new Blob([this.source.value], { type: 'text/x-python' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = this.choice === SESSION ? 'my_robot.py' : `${this.choice.replaceAll('-', '_')}.py`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  #onWorker(status) {
    if (status.phase === 'loading' || status.phase === 'ready') this.status.textContent = status.text;
    if (status.phase === 'running') this.status.textContent = `Running line ${status.line}… (${status.time.toFixed(1)} s simulated)`;
  }

  #print(stream, text) {
    const span = document.createElement('span');
    if (stream !== 'out') span.className = stream;
    span.textContent = text.endsWith('\n') ? text : `${text}\n`;
    this.output.append(span);
    this.output.scrollTop = this.output.scrollHeight;
  }

  async run() {
    if (this.running) return;
    this.running = true;
    // An earlier run being replayed again from the slider gives way to this one.
    this.app.stopReplay();
    this.errorLine = null;
    this.activeLine = null;
    this.#render();
    this.output.textContent = '';
    this.$('#script-run').disabled = true;
    this.$('#script-stop').disabled = false;
    this.status.textContent = this.python.version ? 'Running…' : 'Loading Python…';
    this.app.setBusy('python', true);
    this.stopRequested = false;
    let result, source;
    try {
      this.phase = 'waiting';
      await this.app.whenIdle({ shouldStop: () => this.stopRequested });
      // Stop pressed while the robot was still moving: never start the script.
      if (this.stopRequested) throw new Error('stopped');
      this.phase = 'python';
      source = this.source.value;
      await this.python.load();
      if (this.stopRequested) throw new Error('stopped');
      this.phase = 'waiting';
      await this.app.whenIdle({ shouldStop: () => this.stopRequested });
      if (this.stopRequested) throw new Error('stopped');
      this.phase = 'python';
      const snapshot = this.app.sim.snapshot();
      result = await this.python.run(source, snapshot);
      if (this.stopRequested) throw new Error('stopped');
    } catch (error) {
      if (error.name === 'TimeoutError') {
        this.status.textContent = 'Script stopped after 30 seconds.';
      } else if (error.message !== 'stopped') {
        this.status.textContent = `Python is unavailable right now: ${error.message}. Check your connection and run again.`;
      } else this.status.textContent = 'Stopped.';
      this.#finish();
      return;
    }
    this.status.textContent = 'Replaying…';
    this.phase = 'replay';
    const ended = await this.app.replay(result, {
      // Replayed again later, the run marks lines only while the editor still holds its script.
      onLine: (line) => this.setActiveLine(this.source.value === source ? line : null),
      onOutput: (entry) => this.#print(entry.stream, entry.text),
    });
    if (result.error) {
      this.errorLine = result.error.line;
      this.#print('err', `${result.error.traceback}${result.error.line ? ` (line ${result.error.line})` : ''}`);
    }
    const exit = result.exit ? ` with exit code ${result.exit}` : '';
    const summary = result.error ? `${result.error.type} on line ${result.error.line ?? '?'}` : `Finished${exit}`;
    // A script that stops before any simulated time passes leaves nothing to replay.
    const replay = result.duration > 0 ? ' Drag the timeline to replay.' : '';
    this.status.textContent = `${ended === 'stopped' ? 'Stopped' : summary} · ${result.duration.toFixed(1)} s simulated.${replay}`;
    this.#finish();
  }

  #finish() {
    this.running = false;
    this.phase = null;
    this.activeLine = null;
    this.#render();
    this.$('#script-run').disabled = false;
    this.$('#script-stop').disabled = true;
    this.app.setBusy('python', false);
  }

  stop() {
    this.stopRequested = true;
    if (this.phase === 'replay') this.app.stopReplay();
    // While waiting for the robot, keep the loaded Python; run() checks stopRequested.
    else if (this.phase === 'python') this.python.terminate();
  }
}
