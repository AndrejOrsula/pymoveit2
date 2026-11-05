// Python exception values transported by the JavaScript facade.
/** An exception carrying the Python type and its exact message. */
export class PyError extends Error {
  constructor(type, message) { super(message); this.name = type; this.type = type; }
  /** Serializable exception shape for the browser console and worker. */
  toJSON() { return { type: this.type, message: this.message }; }
}
