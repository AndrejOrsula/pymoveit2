// A small Python interpreter for the console: one-line statements over the
// pymoveit2 facade (calls, attribute reads and assignments, variables, lists,
// arithmetic, print/help/dir, math and time.sleep). Full scripts run in real
// Python (Pyodide) from the Script tab; this subset answers instantly.

import { PyError } from '../api/errors.js';
import { pyRepr } from '../api/codegen.js';

/** Interfaces reachable from `robot`, and the class each one is. */
export const OBJECT_PATHS = {
  robot: 'RobotSession',
  'robot.arm': 'MoveIt2',
  'robot.gripper': 'GripperInterface',
  'robot.servo': 'MoveIt2Servo',
  'robot.description': 'RobotDescription',
};

class Ref {
  constructor(path) {
    this.path = path;
  }
}

class Method {
  constructor(path) {
    this.path = path;
  }
}

class Builtin {
  constructor(name, fn, doc, returns = '') {
    this.name = name;
    this.fn = fn;
    this.doc = doc;
    this.returns = returns;
  }
}

class Module {
  constructor(name, members) {
    this.name = name;
    this.members = members;
  }
}

// ----------------------------------------------------------------- tokenizer

const OPERATORS = ['**', '//', '==', '!=', '<=', '>=', '+=', '-=', '*=', '/=', '(', ')', '[', ']', '{', '}', ',', ':', '.', '=', '+', '-', '*', '/', '%', '<', '>', ';'];

function tokenize(line) {
  const tokens = [];
  let i = 0;
  while (i < line.length) {
    const c = line[i];
    if (c === ' ' || c === '\t') {
      i += 1;
      continue;
    }
    if (c === '#') break;
    const rest = line.slice(i);
    let m = rest.match(/^(?:\d[\d_]*\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?)/);
    if (m) {
      const text = m[0].replaceAll('_', '');
      tokens.push({ kind: 'number', value: Number(text), float: /[.eE]/.test(text), pos: i });
      i += m[0].length;
      continue;
    }
    m = rest.match(/^[A-Za-z_][A-Za-z0-9_]*/);
    if (m) {
      tokens.push({ kind: 'name', value: m[0], pos: i });
      i += m[0].length;
      continue;
    }
    if (c === '"' || c === "'") {
      let j = i + 1;
      let value = '';
      while (j < line.length && line[j] !== c) {
        if (line[j] === '\\' && j + 1 < line.length) {
          const next = line[j + 1];
          value += { n: '\n', t: '\t', '\\': '\\', "'": "'", '"': '"' }[next] ?? `\\${next}`;
          j += 2;
        } else {
          value += line[j];
          j += 1;
        }
      }
      if (j >= line.length) throw new PyError('SyntaxError', 'unterminated string literal');
      tokens.push({ kind: 'string', value, pos: i });
      i = j + 1;
      continue;
    }
    const op = OPERATORS.find((candidate) => rest.startsWith(candidate));
    if (!op) throw new PyError('SyntaxError', `invalid character '${c}'`);
    tokens.push({ kind: 'op', value: op, pos: i });
    i += op.length;
  }
  tokens.push({ kind: 'end', value: '', pos: line.length });
  return tokens;
}

// -------------------------------------------------------------------- parser

class Parser {
  constructor(tokens) {
    this.tokens = tokens;
    this.i = 0;
  }

  peek(offset = 0) {
    return this.tokens[this.i + offset];
  }

  next() {
    return this.tokens[this.i++];
  }

  at(value, kind = 'op') {
    const token = this.peek();
    return token.kind === kind && token.value === value;
  }

  accept(value, kind = 'op') {
    if (this.at(value, kind)) {
      this.i += 1;
      return true;
    }
    return false;
  }

  expect(value, kind = 'op') {
    if (!this.accept(value, kind)) {
      const token = this.peek();
      throw new PyError('SyntaxError', token.kind === 'end' ? `expected '${value}'` : `invalid syntax near '${token.value}'`);
    }
  }

  statement() {
    if (this.peek().kind === 'end') return { type: 'pass' };
    if (this.accept('pass', 'name')) return { type: 'pass' };
    if (this.at('import', 'name')) return this.importStatement();
    if (this.at('from', 'name')) return this.fromStatement();
    if (this.at('with', 'name')) {
      this.next();
      const value = this.expression();
      this.expect('as', 'name');
      const name = this.next();
      if (name.kind !== 'name') throw new PyError('SyntaxError', 'invalid syntax');
      this.expect(':');
      return { type: 'with', value, name: name.value };
    }
    for (const keyword of ['for', 'while', 'if', 'def', 'class', 'try']) {
      if (this.at(keyword, 'name')) {
        throw new PyError('SyntaxError', `the console runs one line at a time; write '${keyword}' blocks in the Script tab, which runs real Python`);
      }
    }
    const target = this.expression();
    for (const op of ['=', '+=', '-=', '*=', '/=']) {
      if (this.accept(op)) {
        const value = this.expression();
        return { type: 'assign', target, value, op };
      }
    }
    return { type: 'expr', value: target };
  }

  importStatement() {
    this.next();
    const names = [];
    do {
      const module = this.dotted();
      let alias = module.split('.')[0];
      if (this.accept('as', 'name')) alias = this.next().value;
      names.push({ module, alias: module.includes('.') && alias === module.split('.')[0] ? alias : alias });
    } while (this.accept(','));
    return { type: 'import', names };
  }

  fromStatement() {
    this.next();
    const module = this.dotted();
    this.expect('import', 'name');
    const names = [];
    const paren = this.accept('(');
    do {
      const name = this.next().value;
      let alias = name;
      if (this.accept('as', 'name')) alias = this.next().value;
      names.push({ name, alias });
    } while (this.accept(','));
    if (paren) this.expect(')');
    return { type: 'from', module, names };
  }

  dotted() {
    let name = this.next().value;
    while (this.accept('.')) name += `.${this.next().value}`;
    return name;
  }

  expression() {
    return this.or();
  }

  or() {
    let left = this.and();
    while (this.accept('or', 'name')) left = { type: 'bool', op: 'or', left, right: this.and() };
    return left;
  }

  and() {
    let left = this.not();
    while (this.accept('and', 'name')) left = { type: 'bool', op: 'and', left, right: this.not() };
    return left;
  }

  not() {
    if (this.accept('not', 'name')) return { type: 'unary', op: 'not', value: this.not() };
    return this.comparison();
  }

  comparison() {
    // Python chains comparisons: a < b < c means a < b and b < c.
    const operands = [this.sum()], ops = [];
    for (;;) {
      const token = this.peek();
      if (token.kind === 'op' && ['==', '!=', '<', '>', '<=', '>='].includes(token.value)) {
        this.next();
        ops.push(token.value);
      } else if (token.kind === 'name' && (token.value === 'is' || token.value === 'in')) {
        this.next();
        const negate = token.value === 'is' && this.accept('not', 'name');
        ops.push(negate ? 'is not' : token.value);
      } else break;
      operands.push(this.sum());
    }
    if (!ops.length) return operands[0];
    if (ops.length === 1) return { type: 'binary', op: ops[0], left: operands[0], right: operands[1] };
    return { type: 'compare', ops, operands };
  }

  sum() {
    let left = this.term();
    while (this.at('+') || this.at('-')) left = { type: 'binary', op: this.next().value, left, right: this.term() };
    return left;
  }

  term() {
    let left = this.unary();
    while (this.at('*') || this.at('/') || this.at('//') || this.at('%')) {
      left = { type: 'binary', op: this.next().value, left, right: this.unary() };
    }
    return left;
  }

  unary() {
    if (this.at('-') || this.at('+')) return { type: 'unary', op: this.next().value, value: this.unary() };
    return this.power();
  }

  power() {
    const base = this.postfix();
    if (this.accept('**')) return { type: 'binary', op: '**', left: base, right: this.unary() };
    return base;
  }

  postfix() {
    let node = this.atom();
    for (;;) {
      if (this.accept('.')) {
        const name = this.next();
        if (name.kind !== 'name') throw new PyError('SyntaxError', 'invalid syntax');
        node = { type: 'attr', value: node, name: name.value };
      } else if (this.accept('(')) {
        const args = [];
        const kwargs = [];
        while (!this.at(')')) {
          if (this.peek().kind === 'name' && this.peek(1).kind === 'op' && this.peek(1).value === '=') {
            const name = this.next().value;
            this.next();
            kwargs.push([name, this.expression()]);
          } else {
            if (kwargs.length) throw new PyError('SyntaxError', 'positional argument follows keyword argument');
            args.push(this.expression());
          }
          if (!this.accept(',')) break;
        }
        this.expect(')');
        node = { type: 'call', fn: node, args, kwargs };
      } else if (this.accept('[')) {
        const index = this.expression();
        this.expect(']');
        node = { type: 'index', value: node, index };
      } else return node;
    }
  }

  atom() {
    const token = this.next();
    if (token.kind === 'number') return { type: 'const', value: token.value, float: token.float };
    if (token.kind === 'string') {
      let value = token.value;
      while (this.peek().kind === 'string') value += this.next().value;
      return { type: 'const', value };
    }
    if (token.kind === 'name') {
      if (token.value === 'True') return { type: 'const', value: true };
      if (token.value === 'False') return { type: 'const', value: false };
      if (token.value === 'None') return { type: 'const', value: null };
      return { type: 'name', name: token.value };
    }
    if (token.kind === 'op' && token.value === '(') {
      if (this.accept(')')) return { type: 'tuple', items: [] };
      const first = this.expression();
      if (this.accept(')')) return first;
      const items = [first];
      while (this.accept(',')) {
        if (this.at(')')) break;
        items.push(this.expression());
      }
      this.expect(')');
      return { type: 'tuple', items };
    }
    if (token.kind === 'op' && token.value === '[') {
      const items = [];
      while (!this.at(']')) {
        items.push(this.expression());
        if (!this.accept(',')) break;
      }
      this.expect(']');
      return { type: 'list', items };
    }
    if (token.kind === 'op' && token.value === '{') {
      const entries = [];
      while (!this.at('}')) {
        const key = this.expression();
        this.expect(':');
        entries.push([key, this.expression()]);
        if (!this.accept(',')) break;
      }
      this.expect('}');
      return { type: 'dict', entries };
    }
    if (token.kind === 'end') throw new PyError('SyntaxError', 'unexpected end of line');
    throw new PyError('SyntaxError', `invalid syntax near '${token.value}'`);
  }
}

/** Splits a line on top-level semicolons and parses each statement. */
const definesState = (statement) => statement.type === 'assign' || statement.type === 'import' || (statement.type === 'from' && statement.module !== 'pymoveit2');

export function parse(line) {
  const tokens = tokenize(line);
  const statements = [];
  let start = 0;
  let depth = 0;
  for (let i = 0; i < tokens.length; i += 1) {
    const t = tokens[i];
    if (t.kind === 'op' && '([{'.includes(t.value)) depth += 1;
    if (t.kind === 'op' && ')]}'.includes(t.value)) depth -= 1;
    if ((t.kind === 'op' && t.value === ';' && depth === 0) || t.kind === 'end') {
      const parser = new Parser([...tokens.slice(start, i), { kind: 'end', value: '', pos: t.pos }]);
      const statement = parser.statement();
      if (parser.peek().kind !== 'end') throw new PyError('SyntaxError', `invalid syntax near '${parser.peek().value}'`);
      statements.push(statement);
      start = i + 1;
    }
  }
  return statements;
}

// ----------------------------------------------------------------- evaluator

function typeName(value) {
  if (value === null || value === undefined) return 'NoneType';
  if (typeof value === 'boolean') return 'bool';
  if (typeof value === 'number') return Number.isInteger(value) ? 'int' : 'float';
  if (typeof value === 'string') return 'str';
  if (Array.isArray(value)) return value.isTuple ? 'tuple' : 'list';
  if (value instanceof Ref) return OBJECT_PATHS[value.path] ?? 'object';
  if (value instanceof Method || value instanceof Builtin) return 'method';
  if (value instanceof Module) return 'module';
  return value.__class__ ?? 'dict';
}

// ROS message fields that are messages themselves, so `pose.pose.position`
// stays a geometry_msgs Point as in rclpy instead of becoming a dict.
const FIELD_CLASSES = {
  PoseStamped: { header: 'Header', pose: 'Pose' },
  Pose: { position: 'Point', orientation: 'Quaternion' },
  Header: { stamp: 'Time' },
  JointState: { header: 'Header' },
  JointTrajectory: { header: 'Header', points: 'JointTrajectoryPoint' },
  JointTrajectoryPoint: { time_from_start: 'Duration' },
};

function field(owner, name) {
  const value = owner[name], cls = FIELD_CLASSES[owner.__class__]?.[name];
  const typed = (item) => (cls && item && typeof item === 'object' && !Array.isArray(item) && !item.__class__ ? { __class__: cls, ...item } : item);
  return Array.isArray(value) ? value.map(typed) : typed(value);
}

function tuple(items) {
  const result = items.slice();
  Object.defineProperty(result, 'isTuple', { value: true });
  return result;
}

/** Python's str() of a console value. */
export function pyStr(value) {
  if (typeof value === 'string') return value;
  return repr(value);
}

/**
 * Python's repr() of a console value. JavaScript numbers do not remember
 * whether they were floats, so `hint` (a type from pymoveit2's signatures)
 * marks floats, and a list holding any non-integer prints as floats.
 */
export function repr(value, hint = '') {
  if (value instanceof Ref) return `<pymoveit2 ${OBJECT_PATHS[value.path]} object (${value.path})>`;
  if (value instanceof Method) return `<bound method ${value.path.split('.').pop()} of ${value.path.split('.').slice(0, -1).join('.')}>`;
  if (value instanceof Builtin) return `<built-in function ${value.name}>`;
  if (value instanceof Module) return `<module '${value.name}'>`;
  if (typeof value === 'number') return pyRepr(value, /float/.test(hint) && !/\bint\b/.test(hint) ? 'float' : '');
  if (Array.isArray(value)) {
    const floats = /float/.test(hint) || (value.some((v) => typeof v === 'number' && !Number.isInteger(v)) && value.every((v) => typeof v === 'number'));
    const inner = value.map((v) => repr(v, floats ? 'float' : '')).join(', ');
    if (!value.isTuple) return `[${inner}]`;
    return value.length === 1 ? `(${inner},)` : `(${inner})`;
  }
  if (value && typeof value === 'object' && !value.__class__) {
    return `{${Object.entries(value).map(([k, v]) => `${repr(k)}: ${repr(v)}`).join(', ')}}`;
  }
  return pyRepr(value);
}

function truthy(value) {
  if (value === null || value === undefined || value === false || value === 0 || value === '') return false;
  if (Array.isArray(value)) return value.length > 0;
  return true;
}

function number(value, op) {
  if (typeof value === 'boolean') return value ? 1 : 0;
  if (typeof value !== 'number') throw new PyError('TypeError', `unsupported operand type(s) for ${op}: '${typeName(value)}'`);
  return value;
}

const RANGE_LIMIT = 100_000;

function binary(op, a, b) {
  switch (op) {
    case '+':
      if (typeof a === 'string' && typeof b === 'string') return a + b;
      if (Array.isArray(a) && Array.isArray(b)) return a.isTuple ? tuple([...a, ...b]) : [...a, ...b];
      return number(a, op) + number(b, op);
    case '-':
      return number(a, op) - number(b, op);
    case '*':
      if (Array.isArray(a) && typeof b === 'number') return Array.from({ length: b }, () => a).flat();
      if (typeof a === 'string' && typeof b === 'number') return a.repeat(b);
      return number(a, op) * number(b, op);
    case '/':
      if (number(b, op) === 0) throw new PyError('ZeroDivisionError', 'division by zero');
      return number(a, op) / number(b, op);
    case '//':
      if (number(b, op) === 0) throw new PyError('ZeroDivisionError', 'integer division or modulo by zero');
      return Math.floor(number(a, op) / number(b, op));
    case '%':
      if (number(b, op) === 0) throw new PyError('ZeroDivisionError', Number.isInteger(a) && Number.isInteger(b) ? 'integer modulo by zero' : 'float modulo by zero');
      return ((number(a, op) % number(b, op)) + number(b, op)) % number(b, op);
    case '**':
      return number(a, op) ** number(b, op);
    case '==':
      return JSON.stringify(a) === JSON.stringify(b);
    case '!=':
      return JSON.stringify(a) !== JSON.stringify(b);
    case '<':
      return a < b;
    case '>':
      return a > b;
    case '<=':
      return a <= b;
    case '>=':
      return a >= b;
    case 'is':
      return a === b || (a == null && b == null);
    case 'is not':
      return !(a === b || (a == null && b == null));
    case 'in':
      if (Array.isArray(b) || typeof b === 'string') return b.includes(a);
      if (b && typeof b === 'object') return a in b;
      throw new PyError('TypeError', `argument of type '${typeName(b)}' is not iterable`);
    default:
      throw new PyError('SyntaxError', `unsupported operator ${op}`);
  }
}

/**
 * Interprets console lines against a pymoveit2 facade session.
 * `write(kind, text)` receives output: kind is "out", "err" or "note".
 */
export class PythonConsole {
  constructor(session, { write = () => {} } = {}) {
    this.session = session;
    this.write = write;
    this.globals = new Map();
    this.#installBuiltins();
  }

  #installBuiltins() {
    const g = this.globals;
    const math = new Module('math', {
      pi: Math.PI,
      tau: 2 * Math.PI,
      e: Math.E,
      sin: new Builtin('sin', (x) => Math.sin(x), undefined, 'float'),
      cos: new Builtin('cos', (x) => Math.cos(x), undefined, 'float'),
      tan: new Builtin('tan', (x) => Math.tan(x), undefined, 'float'),
      atan2: new Builtin('atan2', (y, x) => Math.atan2(y, x), undefined, 'float'),
      sqrt: new Builtin('sqrt', (x) => Math.sqrt(x), undefined, 'float'),
      radians: new Builtin('radians', (x) => (x * Math.PI) / 180, undefined, 'float'),
      degrees: new Builtin('degrees', (x) => (x * 180) / Math.PI, undefined, 'float'),
      hypot: new Builtin('hypot', (...xs) => Math.hypot(...xs), undefined, 'float'),
    });
    const time = new Module('time', {
      sleep: new Builtin('sleep', (seconds) => this.session.sleep(number(seconds, 'sleep')), 'sleep(seconds): pause; the simulated robot keeps moving.'),
    });
    this.modules = { math, time };
    g.set('print', new Builtin('print', (...args) => {
      this.write('out', args.map(pyStr).join(' '));
      return null;
    }));
    g.set('len', new Builtin('len', (x) => {
      if (Array.isArray(x) || typeof x === 'string') return x.length;
      throw new PyError('TypeError', `object of type '${typeName(x)}' has no len()`);
    }));
    g.set('range', new Builtin('range', (a, b, step = 1) => {
      const [start, stop] = b === undefined ? [0, a] : [a, b];
      for (const value of [start, stop, step]) if (!Number.isInteger(value)) throw new PyError('TypeError', `'${typeName(value)}' object cannot be interpreted as an integer`);
      if (step === 0) throw new PyError('ValueError', 'range() arg 3 must not be zero');
      // The console builds the whole list, so it bounds what Python would iterate lazily.
      if (Math.ceil((stop - start) / step) > RANGE_LIMIT) throw new PyError('ValueError', `the console's range() is limited to ${RANGE_LIMIT} items; use the Script tab for more`);
      const out = [];
      for (let i = start; step > 0 ? i < stop : i > stop; i += step) out.push(i);
      return out;
    }));
    g.set('round', new Builtin('round', (x, n = 0) => Number(number(x, 'round').toFixed(n))));
    g.set('abs', new Builtin('abs', (x) => Math.abs(number(x, 'abs'))));
    g.set('min', new Builtin('min', (...xs) => Math.min(...(xs.length === 1 ? xs[0] : xs))));
    g.set('max', new Builtin('max', (...xs) => Math.max(...(xs.length === 1 ? xs[0] : xs))));
    g.set('sum', new Builtin('sum', (xs) => xs.reduce((a, b) => a + b, 0)));
    g.set('float', new Builtin('float', (x) => Number(x), undefined, 'float'));
    g.set('int', new Builtin('int', (x) => Math.trunc(Number(x))));
    g.set('str', new Builtin('str', (x) => pyStr(x)));
    g.set('list', new Builtin('list', (x = []) => Array.from(x)));
    g.set('tuple', new Builtin('tuple', (x = []) => tuple(Array.from(x))));
    g.set('type', new Builtin('type', (x) => `<class '${typeName(x)}'>`));
    g.set('repr', new Builtin('repr', (x) => repr(x)));
    g.set('help', new Builtin('help', (x) => {
      this.write('note', this.help(x));
      return null;
    }));
    g.set('dir', new Builtin('dir', (x) => this.dir(x)));
    g.set('connect', new Builtin('connect', () => new Ref('robot'), 'connect(node_name="pymoveit2", *, namespace=None, group_name=None, ...) -> RobotSession'));
    g.set('robot', new Ref('robot'));
  }

  /**
   * Runs one console entry, which may hold several lines. A session with
   * `statement(source, run, { keep })` runs each line through it, so the page
   * can record the line as typed; `keep` marks lines a later one may depend on
   * (assignments and imports other than pymoveit2's).
   */
  async run(source) {
    for (const raw of source.split('\n')) {
      const line = raw.trim();
      if (!line) continue;
      const statements = parse(line);
      const run = async () => {
        for (const statement of statements) await this.exec(statement);
      };
      if (!this.session.statement) await run();
      else await this.session.statement(line, run, { keep: statements.some(definesState) });
    }
  }

  async exec(statement) {
    switch (statement.type) {
      case 'pass':
        return;
      case 'import':
        for (const { module, alias } of statement.names) this.globals.set(alias, this.#importModule(module));
        return;
      case 'from': {
        const module = this.#importModule(statement.module);
        for (const { name, alias } of statement.names) {
          if (module === 'pymoveit2') {
            if (name !== 'connect' && name !== 'RobotSession') this.write('note', `# '${name}' is not needed here: the console already has \`robot\`.`);
            this.globals.set(alias, this.globals.get('connect'));
          } else if (module instanceof Module && name in module.members) this.globals.set(alias, module.members[name]);
          else throw new PyError('ImportError', `cannot import name '${name}' from '${statement.module}'`);
        }
        return;
      }
      case 'with': {
        const value = await this.eval(statement.value);
        this.globals.set(statement.name, value);
        this.write('note', `# \`${statement.name}\` is connected; type the body's lines one at a time.`);
        return;
      }
      case 'assign':
        return this.#assign(statement);
      case 'expr': {
        this.lastHint = '';
        const value = await this.eval(statement.value);
        if (value !== null && value !== undefined) this.write('out', repr(value, this.lastHint));
        return;
      }
      default:
        throw new PyError('SyntaxError', 'invalid syntax');
    }
  }

  #importModule(name) {
    if (name === 'pymoveit2') return 'pymoveit2';
    if (this.modules[name]) return this.modules[name];
    if (name === 'rclpy' || name.startsWith('rclpy.') || name.endsWith('_msgs') || name.includes('_msgs.')) {
      throw new PyError('ModuleNotFoundError', `No module named '${name}' in this console. ROS is simulated here; \`robot\` is all you need.`);
    }
    throw new PyError('ModuleNotFoundError', `No module named '${name}'`);
  }

  async #assign({ target, value, op }) {
    let result = await this.eval(value);
    if (op !== '=') result = binary(op[0], await this.eval(target), result);
    if (target.type === 'name') {
      this.globals.set(target.name, result);
      return;
    }
    if (target.type === 'attr') {
      const owner = await this.eval(target.value);
      if (owner instanceof Ref) {
        await this.session.setattr(`${owner.path}.${target.name}`, result);
        return;
      }
      if (owner && typeof owner === 'object' && !Array.isArray(owner) && target.name in owner) {
        owner[target.name] = result;
        return;
      }
      throw new PyError('AttributeError', `'${typeName(owner)}' object has no attribute '${target.name}'`);
    }
    if (target.type === 'index') {
      const owner = await this.eval(target.value);
      const index = await this.eval(target.index);
      if (Array.isArray(owner) && !owner.isTuple) {
        owner[index < 0 ? owner.length + index : index] = result;
        return;
      }
      throw new PyError('TypeError', `'${typeName(owner)}' object does not support item assignment`);
    }
    throw new PyError('SyntaxError', 'cannot assign to expression');
  }

  async eval(node) {
    switch (node.type) {
      case 'const':
        return node.value;
      case 'name':
        if (this.globals.has(node.name)) return this.globals.get(node.name);
        if (this.modules[node.name]) {
          throw new PyError('NameError', `name '${node.name}' is not defined. Did you forget to import '${node.name}'?`);
        }
        throw new PyError('NameError', `name '${node.name}' is not defined`);
      case 'list':
        return Promise.all(node.items.map((item) => this.eval(item)));
      case 'tuple':
        return tuple(await Promise.all(node.items.map((item) => this.eval(item))));
      case 'dict': {
        const out = {};
        for (const [k, v] of node.entries) out[pyStr(await this.eval(k))] = await this.eval(v);
        return out;
      }
      case 'unary': {
        const value = await this.eval(node.value);
        if (node.op === 'not') return !truthy(value);
        return node.op === '-' ? -number(value, 'unary -') : number(value, 'unary +');
      }
      case 'bool': {
        const left = await this.eval(node.left);
        if (node.op === 'and') return truthy(left) ? this.eval(node.right) : left;
        return truthy(left) ? left : this.eval(node.right);
      }
      case 'binary':
        return binary(node.op, await this.eval(node.left), await this.eval(node.right));
      case 'compare': {
        let left = await this.eval(node.operands[0]);
        for (let i = 0; i < node.ops.length; i++) {
          const right = await this.eval(node.operands[i + 1]);
          if (!binary(node.ops[i], left, right)) return false;
          left = right;
        }
        return true;
      }
      case 'attr':
        return this.#getattr(await this.eval(node.value), node.name);
      case 'index': {
        const value = await this.eval(node.value);
        const index = await this.eval(node.index);
        if (Array.isArray(value) || typeof value === 'string') {
          const i = index < 0 ? value.length + index : index;
          if (!Number.isInteger(i) || i < 0 || i >= value.length) throw new PyError('IndexError', `${typeName(value)} index out of range`);
          return value[i];
        }
        if (value?.__class__) throw new PyError('TypeError', `'${value.__class__}' object is not subscriptable`);
        if (value && typeof value === 'object' && index in value) return value[index];
        throw new PyError('KeyError', repr(index));
      }
      case 'call':
        return this.#call(node);
      default:
        throw new PyError('SyntaxError', 'invalid syntax');
    }
  }

  #members(path) {
    try {
      return this.session.members(path) ?? [];
    } catch {
      return [];
    }
  }

  #member(path, name) {
    const own = this.#members(path).find((m) => m.name === name);
    if (own || path !== 'robot') return own ? { path: `${path}.${name}`, info: own } : null;
    // RobotSession forwards attributes it lacks to MoveIt2.
    const forwarded = this.#members('robot.arm').find((m) => m.name === name);
    return forwarded ? { path: `robot.${name}`, info: forwarded } : null;
  }

  #getattr(owner, name) {
    this.lastHint = '';
    if (owner instanceof Ref) {
      const path = `${owner.path}.${name}`;
      if (OBJECT_PATHS[path]) return new Ref(path);
      const member = this.#member(owner.path, name);
      if (member?.info.kind === 'method') return new Method(member.path);
      this.lastHint = member?.info.type ?? '';
      return this.session.getattr(path);
    }
    if (owner instanceof Module) {
      if (name in owner.members) return owner.members[name];
      throw new PyError('AttributeError', `module '${owner.name}' has no attribute '${name}'`);
    }
    if (owner && typeof owner === 'object' && !Array.isArray(owner) && name in owner && name !== '__class__') return field(owner, name);
    throw new PyError('AttributeError', `'${typeName(owner)}' object has no attribute '${name}'`);
  }

  async #call(node) {
    const fn = await this.eval(node.fn);
    const args = [];
    for (const arg of node.args) args.push(await this.eval(arg));
    const kwargs = {};
    for (const [name, value] of node.kwargs) kwargs[name] = await this.eval(value);
    this.lastHint = fn instanceof Method ? (this.#memberOf(fn.path)?.returns ?? '') : (fn?.returns ?? '');
    if (fn instanceof Builtin) {
      if (Object.keys(kwargs).length && fn.name !== 'connect' && fn.name !== 'print') {
        throw new PyError('TypeError', `${fn.name}() takes no keyword arguments`);
      }
      return fn.fn(...args);
    }
    if (fn instanceof Method) return this.session.invoke(fn.path, args, kwargs);
    // Calling the servo object is MoveIt2Servo.__call__, a gripper's is toggle().
    if (fn instanceof Ref && (fn.path === 'robot.servo' || fn.path === 'robot.gripper')) return this.session.invoke(fn.path, args, kwargs);
    throw new PyError('TypeError', `'${typeName(fn)}' object is not callable`);
  }

  #memberOf(path) {
    const dot = path.lastIndexOf('.');
    return this.#member(path.slice(0, dot), path.slice(dot + 1))?.info;
  }

  /** help(x) text: the signature and docstring of a pymoveit2 member. */
  help(value) {
    if (value instanceof Method) {
      const [owner, name] = [value.path.split('.').slice(0, -1).join('.'), value.path.split('.').pop()];
      const info = this.#member(owner, name)?.info;
      if (!info) return `No help for ${value.path}.`;
      return `${info.python_signature ?? info.signature ?? name}\n\n    ${info.doc ?? ''}`.trimEnd();
    }
    if (value instanceof Ref) {
      const members = this.#members(value.path);
      const lines = members.map((m) => `  ${m.kind === 'method' ? `${m.name}()` : m.name}  ${m.doc ? `- ${m.doc.split('. ')[0].replace(/\.$/, '')}` : ''}`);
      return `class ${OBJECT_PATHS[value.path]} (${value.path})\n\n${lines.join('\n')}`;
    }
    if (value instanceof Builtin) return value.doc ?? `${value.name}(...)`;
    if (value instanceof Module) return `module ${value.name}: ${Object.keys(value.members).join(', ')}`;
    return `${typeName(value)}: ${repr(value)}`;
  }

  dir(value) {
    if (value === undefined) return [...this.globals.keys()].sort();
    if (value instanceof Ref) return this.#members(value.path).map((m) => m.name).sort();
    if (value instanceof Module) return Object.keys(value.members).sort();
    if (value && typeof value === 'object' && !Array.isArray(value)) return Object.keys(value).filter((k) => k !== '__class__').sort();
    return [];
  }

  /**
   * Completions for the text before the cursor: {from, items: [{name, kind, detail}]}.
   * `from` is the offset where the completed word starts.
   */
  complete(text) {
    const m = text.match(/([A-Za-z_][\w.]*)\.([A-Za-z_]\w*)?$/);
    if (m) {
      const path = m[1];
      const prefix = m[2] ?? '';
      let items = [];
      if (OBJECT_PATHS[path]) {
        items = this.#members(path).map((info) => ({ name: info.name, kind: info.kind, detail: info.python_signature ?? info.signature ?? info.type ?? '' }));
        if (path === 'robot') {
          for (const sub of ['arm', 'gripper', 'servo', 'description']) {
            if (!items.some((i) => i.name === sub)) items.push({ name: sub, kind: 'property', detail: OBJECT_PATHS[`robot.${sub}`] });
          }
        }
      } else if (this.globals.get(path) instanceof Module) {
        items = Object.keys(this.globals.get(path).members).map((name) => ({ name, kind: 'function', detail: '' }));
      }
      items = items.filter((i) => i.name.startsWith(prefix) && !i.name.startsWith('_')).sort((a, b) => a.name.localeCompare(b.name));
      return { from: text.length - prefix.length, items };
    }
    const word = text.match(/([A-Za-z_]\w*)$/)?.[1] ?? '';
    if (!word) return { from: text.length, items: [] };
    const names = [...this.globals.keys(), 'import', 'from', 'with', 'True', 'False', 'None', 'math', 'time'];
    const items = [...new Set(names)]
      .filter((name) => name.startsWith(word) && name !== word)
      .sort()
      .map((name) => ({ name, kind: 'name', detail: '' }));
    return { from: text.length - word.length, items };
  }
}
