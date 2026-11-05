// Python bound-method argument binding and CPython-compatible call errors.
import { PyError } from './errors.js';

const namesText = names => names.length < 2 ? `'${names[0]}'` : names.length === 2 ? `'${names[0]}' and '${names[1]}'` : names.slice(0, -1).map(n => `'${n}'`).join(', ') + `, and '${names.at(-1)}'`;

/** Bind explicit positional/keyword arguments, collecting *args and **kwargs. */
export function bindArguments(qualname, spec, args = [], kwargs = {}) {
  const name = qualname.split('.').at(-1), params = spec.params;
  const fail = message => { throw new PyError('TypeError', `${name}() ${message}`); };
  const positional = params.filter(p => p.kind === 'positional'), rest = params.find(p => p.kind === 'var_positional'), extras = params.find(p => p.kind === 'var_keyword');
  const bound = {}, given = new Set();
  args.slice(0, positional.length).forEach((value, i) => { bound[positional[i].name] = value; given.add(positional[i].name); });
  if (rest) bound[rest.name] = args.slice(positional.length);
  if (extras) bound[extras.name] = {};
  for (const [key, value] of Object.entries(kwargs)) {
    const param = params.find(p => p.name === key && ['positional', 'keyword_only'].includes(p.kind));
    if (!param) { if (extras) { Object.defineProperty(bound[extras.name], key, { value, enumerable: true, writable: true, configurable: true }); continue; } fail(`got an unexpected keyword argument '${key}'`); }
    if (given.has(key)) fail(`got multiple values for argument '${key}'`);
    bound[key] = value; given.add(key);
  }
  if (!rest && args.length > positional.length) {
    const self = qualname === 'connect' ? 0 : 1;
    const required = positional.filter(p => p.default?.__required__).length + self;
    const maximum = positional.length + self, count = args.length + self;
    const keywords = params.filter(p => p.kind === 'keyword_only' && given.has(p.name)).length;
    const received = keywords ? `${count} positional argument${count === 1 ? '' : 's'} (and ${keywords} keyword-only argument${keywords === 1 ? '' : 's'}) were` : `${count} ${count === 1 ? 'was' : 'were'}`;
    fail(`takes ${required === maximum ? `${maximum} positional argument${maximum === 1 ? '' : 's'}` : `from ${required} to ${maximum} positional arguments`} but ${received} given`);
  }
  for (const kind of ['positional', 'keyword_only']) {
    const missing = params.filter(p => p.kind === kind && p.default?.__required__ && !given.has(p.name)).map(p => p.name);
    if (missing.length) fail(`missing ${missing.length} required ${kind === 'positional' ? 'positional' : 'keyword-only'} argument${missing.length === 1 ? '' : 's'}: ${namesText(missing)}`);
  }
  for (const param of params) if (!(param.name in bound)) bound[param.name] = structuredClone(param.default);
  return bound;
}
