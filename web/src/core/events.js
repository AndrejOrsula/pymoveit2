// Small synchronous event source shared by the simulation and API facade.
/** Event subscriptions with independent unsubscribe closures. */
export class Events {
  constructor() { this.listeners = new Map(); }
  /** Subscribe to an event type. */
  on(type, listener) {
    if (!this.listeners.has(type)) this.listeners.set(type, new Set());
    this.listeners.get(type).add(listener);
    return () => this.listeners.get(type)?.delete(listener);
  }
  /** Deliver an event to a stable snapshot of its listeners. */
  emit(type, value) { for (const listener of [...(this.listeners.get(type) ?? [])]) listener(value); }
}
