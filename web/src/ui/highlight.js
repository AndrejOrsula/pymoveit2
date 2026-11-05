// Python syntax highlighting into HTML spans (classes tok-*), for code blocks,
// the console and the script editor. Not a full lexer: enough for pymoveit2
// scripts.

const KEYWORDS = new Set([
  'False', 'None', 'True', 'and', 'as', 'assert', 'async', 'await', 'break', 'class', 'continue', 'def',
  'del', 'elif', 'else', 'except', 'finally', 'for', 'from', 'global', 'if', 'import', 'in', 'is', 'lambda',
  'nonlocal', 'not', 'or', 'pass', 'raise', 'return', 'try', 'while', 'with', 'yield',
]);
const BUILTINS = new Set([
  'print', 'range', 'len', 'abs', 'min', 'max', 'round', 'list', 'tuple', 'dict', 'float', 'int', 'str',
  'bool', 'enumerate', 'zip', 'sum', 'help', 'dir', 'isinstance', 'sorted', 'reversed', 'any', 'all', 'type',
]);

const TOKEN = new RegExp(
  [
    '(?<comment>#[^\\n]*)',
    '(?<string>[rRbBfFuU]{0,2}(?:"""[\\s\\S]*?(?:"""|$)|\'\'\'[\\s\\S]*?(?:\'\'\'|$)|"(?:\\\\.|[^"\\\\\\n])*"?|\'(?:\\\\.|[^\'\\\\\\n])*\'?))',
    '(?<number>\\b(?:0[xX][0-9a-fA-F_]+|\\d[\\d_]*\\.?\\d*(?:[eE][+-]?\\d+)?|\\.\\d+(?:[eE][+-]?\\d+)?)\\b)',
    '(?<name>[A-Za-z_][A-Za-z0-9_]*)',
    '(?<other>[\\s\\S])',
  ].join('|'),
  'g',
);

export function escapeHtml(text) {
  return text.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
}

/** Returns HTML for Python source text. */
export function highlightPython(source) {
  let html = '';
  let previous = '';
  let match;
  TOKEN.lastIndex = 0;
  while ((match = TOKEN.exec(source)) !== null) {
    const text = match[0];
    const groups = match.groups;
    let cls = null;
    if (groups.comment) cls = 'comment';
    else if (groups.string) cls = 'string';
    else if (groups.number) cls = 'number';
    else if (groups.name) {
      const rest = source.slice(TOKEN.lastIndex);
      if (KEYWORDS.has(text)) cls = 'keyword';
      else if (previous === 'def' || previous === 'class') cls = 'fn';
      else if (/^\s*\(/.test(rest)) cls = BUILTINS.has(text) && !previous.endsWith('.') ? 'builtin' : 'fn';
      else if (previous.endsWith('.')) cls = 'attr';
    }
    html += cls ? `<span class="tok-${cls}">${escapeHtml(text)}</span>` : escapeHtml(text);
    if (!/^\s+$/.test(text)) previous = groups.name ? text : text === '.' ? `${previous}.` : text;
  }
  return html;
}
