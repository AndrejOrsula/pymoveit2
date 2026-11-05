// The Console tab: a Python prompt with `robot` connected, autocomplete from
// pymoveit2's signatures, history, and moveit_commander-style shortcuts that
// print the Python they stand for.

import { PythonConsole } from '../console/python.js';
import { commands, expand, isCommand } from '../console/commander.js';
import { escapeHtml, highlightPython } from './highlight.js';

/** Example buttons; `go` names the robot's first named state other than its start. */
const quick = (model) => {
  const state = Object.keys(model.named_states[model.group_name]).find((name) => name !== model.default_state);
  return [['help', 'help'], [`go ${state}`, `go ${state}`], ['close', 'close'], ['open', 'open'], ['compute_fk()', 'robot.compute_fk()']];
};

// One sentence per line, so narrow screens wrap them rather than the lines breaking mid-sentence.
const WELCOME = [
  '# Python with `robot` connected. Type help.',
  '# Tab completes · ↑ ↓ history · Ctrl+C stops.',
];

export class ConsolePanel {
  constructor(panel, app) {
    this.app = app;
    this.history = [];
    this.historyIndex = 0;
    // Lines of a lesson step still to come, one per Enter.
    this.queue = [];
    this.busy = false;
    this.completion = null;
    panel.innerHTML = `
      <div class="console">
        <div class="console-output" id="console-output" role="log" aria-live="polite" tabindex="0"></div>
        <div class="console-input-row">
          <span class="prompt" aria-hidden="true">&gt;&gt;&gt;</span>
          <input class="console-input" id="console-input" type="text" spellcheck="false" autocomplete="off" autocapitalize="off"
            aria-label="Python or a shortcut command" aria-autocomplete="list" aria-controls="console-completions">
          <ul class="completions" id="console-completions" role="listbox" hidden></ul>
        </div>
        <div class="quick" role="group" aria-label="Examples">
          ${quick(app.model).map(([label, code]) => `<button type="button" data-code="${escapeHtml(code)}">${escapeHtml(label)}</button>`).join('')}
        </div>
      </div>`;
    this.output = panel.querySelector('#console-output');
    this.input = panel.querySelector('#console-input');
    this.list = panel.querySelector('#console-completions');
    this.python = new PythonConsole(app.pythonSession(), { write: (kind, text) => this.write(kind, text) });
    for (const line of WELCOME) this.write('note', line);

    panel.querySelector('.quick').addEventListener('click', (event) => {
      const code = event.target.closest('button')?.dataset.code;
      if (code) this.submit(code);
    });
    this.input.addEventListener('keydown', (event) => this.#key(event));
    this.input.addEventListener('input', () => this.#refreshCompletions(false));
    this.input.addEventListener('blur', () => setTimeout(() => this.#closeCompletions(), 150));
    this.list.addEventListener('mousedown', (event) => {
      const item = event.target.closest('li');
      if (!item) return;
      event.preventDefault();
      this.#accept(Number(item.dataset.index));
    });
    app.addEventListener('hint', (event) => this.write('note', `# ${event.detail.text}`));
  }

  focus() {
    this.input.focus();
  }

  /** Puts code in the input; with several lines, each Enter runs one and shows the next. */
  prefill(lines) {
    [this.input.value, ...this.queue] = lines;
    if (this.queue.length) this.write('note', `# ${lines.length} lines: Enter runs each in turn.`);
    this.focus();
  }

  write(kind, text, detail) {
    const line = document.createElement('div');
    line.className = kind;
    if (kind === 'in' || kind === 'gen') line.innerHTML = `<span class="muted">${kind === 'in' ? '&gt;&gt;&gt; ' : '... '}</span>${highlightPython(text)}`;
    else line.textContent = text;
    if (detail) line.insertAdjacentHTML('beforeend', `<span class="muted"> — ${escapeHtml(detail)}</span>`);
    this.output.append(line);
    while (this.output.childElementCount > 600) this.output.firstElementChild.remove();
    this.output.scrollTop = this.output.scrollHeight;
  }

  /** Runs one entry as if typed, used by the quick buttons and lessons. */
  async submit(text) {
    const source = text.trim();
    if (!source || this.busy) return;
    this.history = this.history.filter((entry) => entry !== source);
    this.history.push(source);
    this.historyIndex = this.history.length;
    this.input.value = '';
    this.#closeCompletions();
    this.write('in', source);
    this.busy = true;
    this.input.placeholder = 'Running… Ctrl+C stops the motion';
    try {
      if (isCommand(source)) await this.#command(source);
      else await this.python.run(source);
    } catch (error) {
      this.#error(error);
    } finally {
      this.busy = false;
      this.input.placeholder = '';
      if (this.queue.length && !this.input.value) this.input.value = this.queue.shift();
    }
  }

  async #command(source) {
    let expansion;
    try {
      expansion = expand(source, { state: () => this.app.sim.getState(), model: this.app.model });
    } catch (error) {
      this.write('err', error.message);
      return;
    }
    if (expansion.help) {
      this.write('note', '# Shortcuts; each prints the Python it runs:');
      for (const [usage, text] of commands(this.app.model)) this.write('help', usage, text);
      this.write('note', '# Anything else runs as Python, such as help(robot).');
      return;
    }
    for (const line of expansion.python) {
      this.write('gen', line);
      await this.python.run(line);
    }
  }

  #error(error) {
    if (error?.type) this.write('err', `${error.type}: ${error.message}`);
    else {
      this.write('err', `RuntimeError: ${error?.message ?? error}`);
      console.error(error);
    }
  }

  #key(event) {
    const open = !this.list.hidden && this.completion?.items.length;
    // Ctrl+C copies a selection; otherwise, as in a terminal, it drops the
    // typed line and any queued ones and stops the motion.
    if (event.key === 'c' && event.ctrlKey && this.input.selectionStart === this.input.selectionEnd) {
      event.preventDefault();
      this.input.value = '';
      this.queue = [];
      this.#closeCompletions();
      this.interrupt();
      return;
    }
    if (open && (event.key === 'ArrowDown' || event.key === 'ArrowUp')) {
      event.preventDefault();
      const n = this.completion.items.length;
      this.completion.selected = (this.completion.selected + (event.key === 'ArrowDown' ? 1 : n - 1)) % n;
      this.#renderCompletions();
      return;
    }
    if (open && (event.key === 'Tab' || event.key === 'Enter')) {
      event.preventDefault();
      const typed = this.input.value;
      this.#accept(this.completion.selected);
      // A name already typed out in full has nothing to complete: Enter runs it.
      if (event.key === 'Enter' && this.input.value === typed) this.submit(typed);
      return;
    }
    if (event.key === 'Escape') {
      this.#closeCompletions();
      return;
    }
    if (event.key === 'Tab' || (event.key === ' ' && event.ctrlKey)) {
      event.preventDefault();
      this.#refreshCompletions(true);
      return;
    }
    if (event.key === 'Enter') {
      event.preventDefault();
      this.submit(this.input.value);
      return;
    }
    if (event.key === 'ArrowUp' || event.key === 'ArrowDown') {
      if (!this.history.length) return;
      event.preventDefault();
      this.historyIndex = Math.max(0, Math.min(this.history.length, this.historyIndex + (event.key === 'ArrowUp' ? -1 : 1)));
      this.input.value = this.history[this.historyIndex] ?? '';
      this.input.setSelectionRange(this.input.value.length, this.input.value.length);
    }
  }

  interrupt() {
    this.write('err', 'KeyboardInterrupt');
    this.app.interrupt();
  }

  #refreshCompletions(explicit) {
    const text = this.input.value.slice(0, this.input.selectionStart ?? this.input.value.length);
    const afterDot = /\.\w*$/.test(text);
    if (!explicit && !afterDot) {
      this.#closeCompletions();
      return;
    }
    const result = this.python.complete(text);
    if (!result.items.length) {
      this.#closeCompletions();
      return;
    }
    if (explicit && result.items.length === 1) {
      this.completion = { ...result, selected: 0 };
      this.#accept(0);
      return;
    }
    this.completion = { ...result, selected: 0 };
    this.#renderCompletions();
  }

  #renderCompletions() {
    const { items, selected } = this.completion;
    this.list.innerHTML = items
      .slice(0, 60)
      .map((item, index) => `<li role="option" id="completion-${index}" data-index="${index}" aria-selected="${index === selected}">
        <span>${escapeHtml(item.name)}${item.kind === 'method' ? '()' : ''}</span><span class="sig">${escapeHtml(item.detail ?? '')}</span></li>`)
      .join('');
    this.list.hidden = false;
    this.input.setAttribute('aria-activedescendant', `completion-${selected}`);
    this.list.querySelector('[aria-selected="true"]')?.scrollIntoView({ block: 'nearest' });
  }

  #accept(index) {
    const item = this.completion?.items[index];
    if (!item) return;
    const value = this.input.value;
    const cursor = this.input.selectionStart ?? value.length;
    const insert = item.kind === 'method' ? `${item.name}(` : item.name;
    this.input.value = value.slice(0, this.completion.from) + insert + value.slice(cursor);
    const position = this.completion.from + insert.length;
    this.input.setSelectionRange(position, position);
    this.#closeCompletions();
    this.input.focus();
  }

  #closeCompletions() {
    this.list.hidden = true;
    this.completion = null;
    this.input.removeAttribute('aria-activedescendant');
  }
}
