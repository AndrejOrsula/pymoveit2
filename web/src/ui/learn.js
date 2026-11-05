// The Learn tab: an overview, then one lesson at a time. Each step shows
// pymoveit2 Python and runs exactly that code against the simulated robot.

import { PythonConsole } from '../console/python.js';
import { LESSONS } from './lessons.js';
import { escapeHtml, highlightPython } from './highlight.js';

const STORAGE_KEY = 'pymoveit2-demo-lessons';
const POSITION_TOLERANCE = 0.001;
const POINT_REPR = /^geometry_msgs\.msg\.Point\(x=([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?), y=([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?), z=([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\)$/;
const QUICK_START = {
  panda: `from pymoveit2 import connect

with connect() as robot:
    if not robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]):
        print(robot.last_failure())
    robot.gripper.close()
    robot.move_to_configuration()`,
  open_manipulator_x: `from pymoveit2 import connect

with connect() as robot:
    if not robot.move_to_pose(
        [0.0, -0.05, 0.12], [0.0, 0.0, 0.0, 1.0],
        frame_id='world', tolerance_orientation=3.1416,
    ):
        print(robot.last_failure())
    robot.gripper.close()
    robot.move_to_configuration()`,
};

/**
 * Whether a step did what its lesson says, from its printed lines: its last
 * line matches `expect` or `expectPosition` (else no error), and every `False`
 * before it is followed by the failure it explains. The lesson tests apply the
 * same rule.
 */
export function stepPassed(step, lines) {
  const texts = lines.map((line) => line.text);
  if (texts.slice(0, -1).some((text, i) => text === 'False' && !texts[i + 1].startsWith("'"))) return false;
  if (step.expectPosition) {
    const match = POINT_REPR.exec(texts.at(-1) ?? '');
    if (!match) return false;
    const actual = match.slice(1).map(Number);
    return actual.every((coordinate, i) => Number.isFinite(coordinate) && Math.abs(coordinate - step.expectPosition[i]) < POSITION_TOLERANCE);
  }
  return step.expect ? step.expect.test(texts.at(-1) ?? '') : !lines.some((line) => line.kind === 'err');
}

/** Minutes a lesson takes: about 40 s per step to read, run and look. */
const minutes = (lesson) => Math.max(2, Math.round(lesson.steps.length * 0.7));

function loadDone(key) {
  try {
    return new Set(JSON.parse(localStorage.getItem(key) ?? '[]'));
  } catch {
    return new Set();
  }
}

export class LearnPanel {
  constructor(panel, app) {
    this.panel = panel;
    this.app = app;
    this.lessons = LESSONS[app.model.id];
    // The Panda keeps the key it always had, so earlier progress survives.
    this.key = app.model.id === 'panda' ? STORAGE_KEY : `${STORAGE_KEY}-${app.model.id}`;
    this.done = loadDone(this.key);
    this.lesson = null;
    this.running = false;
    this.python = null;
    panel.addEventListener('click', (event) => this.#click(event));
    this.showOverview();
  }

  #save() {
    try {
      localStorage.setItem(this.key, JSON.stringify([...this.done]));
    } catch {
      // Private browsing: progress just isn't kept.
    }
  }

  showOverview() {
    // Rendering removes the control that had focus; a keyboard user lands on the next lesson instead.
    const refocus = this.panel.contains(document.activeElement);
    this.lesson = null;
    this.app.updateLink();
    const lessons = this.lessons, model = this.app.model;
    const finished = lessons.filter((lesson) => this.done.has(lesson.id));
    const next = lessons.find((lesson) => !this.done.has(lesson.id));
    const left = lessons.reduce((sum, lesson) => sum + (this.done.has(lesson.id) ? 0 : minutes(lesson)), 0);
    const item = (lesson) => `
      <li class="${this.done.has(lesson.id) ? 'done' : ''}">
        <button type="button" data-open="${lesson.id}">
          <span class="lesson-num" aria-hidden="true">${this.done.has(lesson.id) ? '✓' : lessons.indexOf(lesson) + 1}</span>
          <span class="lesson-title">${escapeHtml(lesson.title)}</span>
          <span class="lesson-time">${minutes(lesson)} min</span>
        </button>
      </li>`;
    const parts = [...new Set(lessons.map((lesson) => lesson.part))].map((part) => {
      const group = lessons.filter((lesson) => lesson.part === part);
      const count = group.filter((lesson) => this.done.has(lesson.id)).length;
      return `<h4>${escapeHtml(part)} <span class="hint">${count}/${group.length}</span></h4><ol class="lesson-list">${group.map(item).join('')}</ol>`;
    }).join('');
    const action = next
      ? `<button type="button" class="btn primary" data-open="${next.id}">${finished.length ? 'Continue' : 'Start'}: ${escapeHtml(next.title)}</button>
         <span class="hint">${finished.length ? `${finished.length} of ${lessons.length} done · ${left} min left` : `${lessons.length} lessons · ${left} min`}</span>`
      : `<span class="hint">All ${lessons.length} lessons done.</span>`;
    this.panel.innerHTML = `
      <div class="hero">
        <h3>Move a robot arm with a few lines of Python</h3>
        <div class="btn-row">${action}</div>
        <p><strong>pymoveit2</strong> is a Python interface to MoveIt&nbsp;2. Here it moves a simulated ${escapeHtml(model.label)}. The lessons build up to this:</p>
        <div class="code"><pre><code>${highlightPython(QUICK_START[model.id])}</code></pre></div>
        <p class="hint">On a real robot: <code>pip install pymoveit2</code> <button type="button" class="link-button" data-copy-text="pip install pymoveit2">Copy</button>, launch MoveIt&nbsp;2 and run the same code.</p>
      </div>
      ${parts}`;
    if (refocus) this.panel.querySelector('.hero .btn.primary, [data-open]')?.focus({ preventScroll: true });
  }

  async open(id) {
    const lessons = this.lessons;
    const index = lessons.findIndex((lesson) => lesson.id === id);
    const lesson = lessons[index];
    if (!lesson || this.running) return;
    this.lesson = lesson;
    this.app.updateLink();
    const refocus = this.panel.contains(document.activeElement);
    this.python = new PythonConsole(this.app.pythonSession(), { write: (kind, text) => this.#write(kind, text) });
    const steps = lesson.steps.map((step, i) => this.#stepMarkup(step, i, lesson.steps.length)).join('');
    const setup = lesson.setup ? `<details class="card"><summary>Scene setup</summary><div class="code"><pre><code>${highlightPython(lesson.setup.join('\n'))}</code></pre></div></details>` : '';
    this.panel.innerHTML = `
      <nav class="lesson-nav" aria-label="Lessons">
        <button type="button" class="btn small" data-overview>All lessons</button>
        <span class="btn-row" style="margin:0">
          <button type="button" class="btn small" data-open="${lessons[index - 1]?.id ?? ''}" ${index === 0 ? 'disabled' : ''}>Previous</button>
          <button type="button" class="btn small" data-open="${lessons[index + 1]?.id ?? ''}" ${index === lessons.length - 1 ? 'disabled' : ''}>Next</button>
        </span>
      </nav>
      <h3 tabindex="-1">${index + 1}. ${escapeHtml(lesson.title)}</h3>
      ${lesson.intro}
      ${setup}
      <div class="lesson-status hint" id="lesson-status" role="status"></div>
      ${steps}
      <div class="callout" id="lesson-outro" hidden><p id="lesson-win"></p>${lesson.outro ?? ''}</div>
      <div class="btn-row">
        ${index < lessons.length - 1 ? `<button type="button" class="btn primary" data-open="${lessons[index + 1].id}" id="lesson-next" hidden>Next: ${escapeHtml(lessons[index + 1].title)}</button>` : `<button type="button" class="btn primary" data-overview id="lesson-next" hidden>All lessons</button>`}
      </div>`;
    this.panel.scrollTop = 0;
    // The control that opened the lesson is gone: focus waits on the title while the
    // scene is set up, then moves to the first step, so Enter runs it.
    const heading = this.panel.querySelector('h3');
    if (refocus) heading.focus({ preventScroll: true });
    if (lesson.reset || lesson.setup) await this.#prepare(lesson);
    if (refocus && document.activeElement === heading) this.panel.querySelector('.step .btn.primary')?.focus({ preventScroll: true });
  }

  async #prepare(lesson) {
    const status = this.panel.querySelector('#lesson-status');
    this.running = true;
    this.#setRunButtons(true);
    status.textContent = 'Setting up the scene…';
    try {
      if (lesson.reset) await this.app.resetWorld();
      const quiet = new PythonConsole(this.app.pythonSession(), { write: () => {} });
      for (const line of lesson.setup ?? []) await quiet.run(line);
      status.textContent = '';
    } catch (error) {
      status.textContent = `Setup failed (${error.type ?? 'Error'}: ${error.message}). Open the lesson again to retry.`;
    } finally {
      this.running = false;
      this.#setRunButtons(false);
    }
  }

  #stepMarkup(step, i, count) {
    const code = step.code
      ? `<div class="code"><pre><code>${highlightPython(step.code)}</code></pre>
          <div class="code-actions"><button type="button" class="btn small" data-copy="${i}">Copy</button></div></div>
          <div class="btn-row"><button type="button" class="btn primary small" data-run="${i}">Run</button>
          <button type="button" class="btn small" data-console="${i}" title="Paste into the console">Try in console</button></div>
          <pre class="result" id="step-${i}-result" hidden></pre>`
      : '';
    const extra = step.script
      ? `<div class="btn-row"><button type="button" class="btn primary small" data-script="${escapeHtml(step.script)}" data-step="${i}">Open in Script</button></div>`
      : step.tab
        ? `<div class="btn-row"><button type="button" class="btn primary small" data-tab="${escapeHtml(step.tab)}" data-step="${i}">Open ${escapeHtml(step.tab[0].toUpperCase() + step.tab.slice(1))}</button></div>`
        : '';
    return `<section class="step" id="step-${i}" data-state="todo" aria-label="Step ${i + 1} of ${count}" data-label="Step ${i + 1} of ${count}">
        <p>${step.text}</p>${code}${extra}
        <p class="hint after" id="step-${i}-after" hidden>${step.after ?? ''}</p>
      </section>`;
  }

  #write(kind, text) {
    if (!this.output) return;
    this.output.push({ kind, text });
  }

  #setRunButtons(disabled) {
    for (const button of this.panel.querySelectorAll('[data-run], [data-open], [data-overview]')) {
      if (disabled) {
        button.dataset.wasDisabled = String(button.disabled);
        button.disabled = true;
      } else if (button.dataset.wasDisabled !== undefined) {
        button.disabled = button.dataset.wasDisabled === 'true';
        delete button.dataset.wasDisabled;
      }
    }
  }

  async runStep(i) {
    const lesson = this.lesson;
    const step = lesson?.steps[i];
    if (!step?.code || this.running) return;
    const section = this.panel.querySelector(`#step-${i}`);
    const result = this.panel.querySelector(`#step-${i}-result`);
    this.running = true;
    this.#setRunButtons(true);
    section.dataset.state = 'running';
    result.hidden = false;
    result.textContent = 'Running…';
    result.removeAttribute('data-ok');
    if (step.layer) this.app.setLayer(step.layer, true);
    this.output = [];
    try {
      for (const line of step.code.split('\n')) await this.python.run(line);
    } catch (error) {
      this.output.push({ kind: 'err', text: `${error.type ?? 'Error'}: ${error.message}` });
    }
    const lines = this.output.filter((entry) => entry.kind !== 'note');
    result.textContent = lines.map((entry) => entry.text).join('\n') || '(no output)';
    // A failed step keeps the lesson open and its explanation hidden.
    const passed = stepPassed(step, lines);
    result.dataset.ok = String(passed);
    section.dataset.state = passed ? 'done' : 'failed';
    this.panel.querySelector('#lesson-status').textContent = passed ? '' : `Step ${i + 1} did not give the expected result. Run it again, or reopen the lesson.`;
    this.output = null;
    this.running = false;
    this.#setRunButtons(false);
    this.panel.querySelector(`#step-${i}-after`).hidden = !(passed && step.after);
    this.#progress();
    if (passed) this.#focusNext(section);
  }

  /** Focuses the next step's main button, or the next lesson once all are done. */
  #focusNext(section) {
    let target = null;
    for (let next = section.nextElementSibling; next?.classList.contains('step') && !target; next = next.nextElementSibling) {
      target = next.querySelector('.btn.primary');
    }
    target ??= this.panel.querySelector('#lesson-next:not([hidden])');
    target?.scrollIntoView({ block: 'nearest' });
    target?.focus({ preventScroll: true });
  }

  #progress() {
    const lesson = this.lesson;
    const finished = lesson.steps.every((step, i) => {
      const state = this.panel.querySelector(`#step-${i}`)?.dataset.state;
      return state === 'done' || (!step.code && state === 'visited');
    });
    if (!finished) return;
    if (!this.done.has(lesson.id)) {
      this.done.add(lesson.id);
      this.#save();
    }
    const count = this.lessons.filter((entry) => this.done.has(entry.id)).length;
    this.panel.querySelector('#lesson-win').innerHTML = `<strong>✓ Lesson done</strong> · ${count} of ${this.lessons.length}`;
    this.panel.querySelector('#lesson-outro').hidden = false;
    const next = this.panel.querySelector('#lesson-next');
    if (next) next.hidden = false;
  }

  #click(event) {
    const target = event.target.closest('button');
    if (!target || target.disabled) return;
    const data = target.dataset;
    if (data.open) this.open(data.open);
    else if ('overview' in data) this.showOverview();
    else if (data.run) this.runStep(Number(data.run));
    else if (data.copyText) {
      navigator.clipboard?.writeText(data.copyText).then(() => this.app.toast('Copied'), () => {});
    } else if (data.copy) {
      navigator.clipboard?.writeText(this.lesson.steps[Number(data.copy)].code).then(() => this.app.toast('Copied'), () => {});
    } else if (data.console) {
      this.app.openConsole(this.lesson.steps[Number(data.console)].code);
    } else if (data.script) {
      this.#visit(data.step);
      this.app.openScript(data.script);
    } else if (data.tab) {
      if (data.step !== undefined) this.#visit(data.step);
      this.app.showTab(data.tab);
    }
  }

  #visit(i) {
    if (i === undefined || !this.lesson) return;
    const section = this.panel.querySelector(`#step-${i}`);
    if (section) section.dataset.state = 'visited';
    this.#progress();
  }
}
