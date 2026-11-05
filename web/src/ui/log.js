// The ROS 2 tab: every request pymoveit2 sent and every line move_group,
// servo_node and the controllers logged, newest last.

import { escapeHtml } from './highlight.js';

const LIMIT = 400;

export class LogPanel {
  constructor(panel, app) {
    this.panel = panel;
    this.filter = 'all';
    this.entries = [];
    this.counts = new Map();
    panel.innerHTML = `
      <p>ROS&nbsp;2 requests behind each call, and the simulated node logs.</p>
      <div class="btn-row">
        <div class="segmented small" role="group" aria-label="Show">
          <button type="button" data-filter="all" aria-pressed="true">All</button>
          <button type="button" data-filter="ros" aria-pressed="false">Requests</button>
          <button type="button" data-filter="log" aria-pressed="false">Logs</button>
        </div>
        <button type="button" class="btn small" id="log-clear">Clear</button>
      </div>
      <ul class="trace" id="trace" aria-live="off"></ul>
      <details class="card" style="margin-top:0.75rem">
        <summary>Interfaces used so far</summary>
        <ul class="trace" id="interfaces"></ul>
      </details>`;
    this.list = panel.querySelector('#trace');
    this.interfaces = panel.querySelector('#interfaces');
    panel.querySelector('[role="group"]').addEventListener('click', (event) => {
      const button = event.target.closest('[data-filter]');
      if (!button) return;
      this.filter = button.dataset.filter;
      for (const other of panel.querySelectorAll('[data-filter]')) other.setAttribute('aria-pressed', String(other === button));
      this.#render();
    });
    panel.querySelector('#log-clear').addEventListener('click', () => {
      this.entries = [];
      this.#render();
    });
    app.addEventListener('ros', (event) => this.add({ ...event.detail, source: 'ros' }));
    app.addEventListener('log', (event) => this.add({ ...event.detail, source: 'log' }));
  }

  add(entry) {
    if (entry.source === 'ros') {
      const key = entry.name;
      const known = this.counts.get(key);
      this.counts.set(key, { ...entry, count: (known?.count ?? 0) + 1 });
      this.#renderInterfaces();
    }
    // Servo streams one command every 50 ms; keep one line per burst.
    const last = this.entries.at(-1);
    if (entry.source === 'ros' && entry.name?.includes('delta_') && last?.name === entry.name) {
      last.repeat = (last.repeat ?? 1) + 1;
      last.time = entry.time;
      this.#renderLast();
      return;
    }
    this.entries.push(entry);
    if (this.entries.length > LIMIT) this.entries.shift();
    if (this.filter === 'all' || this.filter === entry.source) {
      this.list.insertAdjacentHTML('beforeend', this.#row(entry));
      while (this.list.childElementCount > LIMIT) this.list.firstElementChild.remove();
      if (!this.panel.hidden) this.list.lastElementChild?.scrollIntoView({ block: 'nearest' });
    }
  }

  #row(entry) {
    const time = `${entry.time?.toFixed(2) ?? '0.00'} s`;
    if (entry.source === 'ros') {
      const repeat = entry.repeat > 1 ? ` ×${entry.repeat}` : '';
      const status = entry.status ? ` <span class="badge">${escapeHtml(entry.status)}</span>` : '';
      return `<li><span class="t">${time}</span><span class="lvl" data-level="ros">${escapeHtml(entry.kind ?? 'ros')}</span>
        <span class="msg"><code>${escapeHtml(entry.name)}</code>${repeat}${status} <span class="node">${escapeHtml(entry.type ?? '')}</span><br>${escapeHtml(entry.summary ?? '')}</span></li>`;
    }
    return `<li><span class="t">${time}</span><span class="lvl" data-level="${escapeHtml(entry.level)}">${escapeHtml(entry.level.toUpperCase())}</span>
      <span class="msg"><span class="node">[${escapeHtml(entry.node)}]</span> ${escapeHtml(entry.text)}</span></li>`;
  }

  #renderLast() {
    if (this.filter === 'log') return;
    const last = this.list.lastElementChild;
    if (last) last.outerHTML = this.#row(this.entries.at(-1));
  }

  #render() {
    this.list.innerHTML = this.entries
      .filter((entry) => this.filter === 'all' || entry.source === this.filter)
      .map((entry) => this.#row(entry))
      .join('');
  }

  #renderInterfaces() {
    this.interfaces.innerHTML = [...this.counts.values()]
      .sort((a, b) => b.count - a.count)
      .map((entry) => `<li><span class="t">${entry.count}×</span><span class="lvl" data-level="ros">${escapeHtml(entry.kind ?? '')}</span>
        <span class="msg"><code>${escapeHtml(entry.name)}</code> <span class="node">${escapeHtml(entry.type ?? '')}</span></span></li>`)
      .join('');
  }
}
