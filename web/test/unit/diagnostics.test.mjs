// Guard verbatim diagnostic text against drift in its Python semantic reference.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { ERROR_FIXES, STATUS_FIXES, describeErrorCode, describeStatus } from '../../src/api/diagnostics.js';

test('all diagnostics match pymoveit2/_diagnostics.py verbatim', () => {
  const source = readFileSync(new URL('../../../pymoveit2/_diagnostics.py', import.meta.url), 'utf8');
  for (const [name, actual] of Object.entries({ ERROR_FIXES, STATUS_FIXES })) {
    const body = source.match(new RegExp(`${name}: Dict\\[str, str\\] = \\{([\\s\\S]*?)\\n\\}`))[1];
    const expected = Object.fromEntries([...body.matchAll(/"([^"\\]+)":\s*("(?:[^"\\]|\\.)*")/g)].map(match => [match[1], JSON.parse(match[2])]));
    assert.deepEqual(actual, expected);
  }
});
test('diagnostic punctuation, optional detail and source match Python', () => {
  assert.equal(describeErrorCode('SUCCESS'), null); assert.equal(describeErrorCode('UNDEFINED'), null); assert.equal(describeStatus('STATUS_SUCCEEDED'), null);
  assert.equal(describeErrorCode('MYSTERY', '  extra...  ', ' node '), "MYSTERY. extra. Reported by 'node'.");
  assert.equal(describeStatus('STATUS_ABORTED'), `STATUS_ABORTED: ${STATUS_FIXES.STATUS_ABORTED}`);
});
