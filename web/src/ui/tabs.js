// The side panel's tabs (WAI-ARIA tabs pattern with arrow-key navigation).

export class Tabs extends EventTarget {
  constructor(tablist) {
    super();
    this.tabs = [...tablist.querySelectorAll('[role="tab"]')];
    for (const tab of this.tabs) {
      tab.addEventListener('click', () => this.select(tab.id.replace('tab-', '')));
    }
    tablist.addEventListener('keydown', (event) => {
      const index = this.tabs.indexOf(document.activeElement);
      if (index < 0) return;
      let next = null;
      if (event.key === 'ArrowRight') next = (index + 1) % this.tabs.length;
      if (event.key === 'ArrowLeft') next = (index - 1 + this.tabs.length) % this.tabs.length;
      if (event.key === 'Home') next = 0;
      if (event.key === 'End') next = this.tabs.length - 1;
      if (next === null) return;
      event.preventDefault();
      this.tabs[next].focus();
      this.select(this.tabs[next].id.replace('tab-', ''), { keys: true });
    });
  }

  get current() {
    return this.tabs.find((tab) => tab.getAttribute('aria-selected') === 'true')?.id.replace('tab-', '');
  }

  /** Shows a tab's panel; `keys` marks a selection made with the arrow keys, which keeps focus on the tabs. */
  select(name, { keys = false } = {}) {
    for (const tab of this.tabs) {
      const selected = tab.id === `tab-${name}`;
      tab.setAttribute('aria-selected', String(selected));
      tab.tabIndex = selected ? 0 : -1;
      document.getElementById(tab.getAttribute('aria-controls')).hidden = !selected;
    }
    this.dispatchEvent(new CustomEvent('change', { detail: { name, keys } }));
  }
}
