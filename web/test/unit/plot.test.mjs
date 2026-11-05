// Trajectory plot: each setter keeps its value and redraws with it, the pointer
// maps to a trajectory time and pose, and each draw paints grid, labels, joint
// lines and cursors.
import test from 'node:test';
import assert from 'node:assert/strict';
import { TrajectoryPlot } from '../../src/ui/plot.js';

// The theme the page's CSS reports; an empty theme leaves the plot's fallbacks.
let theme = {};
globalThis.window = { devicePixelRatio: 1 };
globalThis.getComputedStyle = () => ({ getPropertyValue: (name) => theme[name] ?? '', backgroundColor: 'rgb(255, 255, 255)' });
// A canvas (200x100 CSS px unless given, at viewport x `left`) whose 2D context
// records every call and property write, and whose backing store records each resize.
function fakeCanvas({ width = 200, height = 100, left = 0, backing = [0, 0] } = {}) {
  const ops = [];
  const target = { measureText: (text) => ({ width: text.length * 6 }) };
  const ctx = new Proxy(target, {
    get: (t, name) => t[name] ?? ((...args) => ops.push([name, ...args])),
    set: (t, name, value) => ops.push(['set', name, value]),
  });
  let [backingWidth, backingHeight] = backing;
  return {
    ops,
    clientWidth: width,
    clientHeight: height,
    get width() {
      return backingWidth;
    },
    set width(value) {
      backingWidth = value;
      ops.push(['resize', 'width', value]);
    },
    get height() {
      return backingHeight;
    },
    set height(value) {
      backingHeight = value;
      ops.push(['resize', 'height', value]);
    },
    parentElement: null,
    getContext: () => ctx,
    getBoundingClientRect: () => ({ left, width }),
  };
}
const trajectory = {
  points: [
    { positions: [0, 0], velocities: [0, 0], time_from_start: { sec: 0, nanosec: 0 } },
    { positions: [1, 0.5], velocities: [4, 2], time_from_start: { sec: 1, nanosec: 0 } },
  ],
};
// The y-axis labels are drawn right-aligned at x = PAD.left - 4.
const yLabels = (ops) => ops.filter(([name, , x]) => name === 'fillText' && x === 30).map(([, text]) => text);
const has = (ops, ...call) => ops.some((op) => JSON.stringify(op) === JSON.stringify(call));
// Plot x of t = 0.5 s on a 1 s trajectory: PAD.left + 0.5 * (200 - 34 - 8), snapped to a pixel centre.
const MID = 113.5;

function plotted(shown = trajectory, options = {}, limits = null) {
  const canvas = fakeCanvas(options);
  const plot = new TrajectoryPlot(canvas);
  plot.setTrajectory(shown, limits);
  canvas.ops.length = 0;
  return { canvas, plot };
}

// What the recorded calls paint: each stroked path, text and filled rectangle
// with the context state it used, from the canvas defaults, to 1e-6 px.
function painted(ops) {
  const px = (v) => Math.round(v * 1e6) / 1e6 + 0;
  const state = { strokeStyle: '#000', fillStyle: '#000', lineWidth: 1, lineJoin: 'miter', dash: [], font: '10px sans-serif', textAlign: 'start', textBaseline: 'alphabetic' };
  const strokes = [];
  const texts = [];
  const rects = [];
  let path = [];
  for (const [name, ...args] of ops) {
    if (name === 'set') state[args[0]] = args[1];
    else if (name === 'setLineDash') state.dash = args[0];
    else if (name === 'beginPath') path = [];
    else if (name === 'moveTo' || name === 'lineTo') path.push([name, px(args[0]), px(args[1])]);
    else if (name === 'stroke') strokes.push({ path, color: state.strokeStyle, width: state.lineWidth, join: state.lineJoin, dash: state.dash });
    else if (name === 'fillText') texts.push({ text: args[0], x: px(args[1]), y: px(args[2]), color: state.fillStyle, font: state.font, align: state.textAlign, baseline: state.textBaseline });
    else if (name === 'fillRect') rects.push({ rect: args.map(px), color: state.fillStyle });
  }
  return { strokes, texts, rects };
}
// Expected paint: a polyline through [x, y] points, and a text label.
const line = (style, ...xy) => ({ path: xy.map(([x, y], i) => [i ? 'lineTo' : 'moveTo', x, y]), dash: [], ...style });
const label = (text, x, y, align, baseline, color = '#888', font = '10px monospace') => ({ text, x, y, color, font, align, baseline });
const GRID = { color: '#ccc', width: 1, join: 'miter' };
const JOINT = { color: '#888', width: 1.6, join: 'round' };

// A 2 s, two-joint trajectory spanning 0..1 rad. On a 142x134 canvas the plot
// area is 100x112 px, so t maps to x = 34 + 50 t and a value v to y = 112 - 100 v.
const point = (seconds, positions, velocities) => ({ positions, velocities, time_from_start: { sec: Math.trunc(seconds), nanosec: Math.round((seconds % 1) * 1e9) } });
const wave = { points: [point(0, [0, 0.25]), point(1, [1, 0.75]), point(2, [0.5, 1])] };
const SIZED = { width: 142, height: 134 };

test('setTrajectory keeps a non-empty trajectory, clears the cursors and redraws', () => {
  const { canvas, plot } = plotted();
  plot.setPlayhead(0.5);
  plot.setHover(0.5);
  canvas.ops.length = 0;
  plot.setTrajectory(trajectory, [{ lower: -1, upper: 1, max_velocity: 2 }]);
  assert.equal(plot.trajectory, trajectory);
  assert.deepEqual(plot.limits, [{ lower: -1, upper: 1, max_velocity: 2 }]);
  assert.equal(plot.playhead, null);
  assert.equal(plot.hover, null);
  assert.deepEqual(yLabels(canvas.ops), ['0.0', '0.5', '1.0']);
  assert.ok(!has(canvas.ops, 'moveTo', MID, 6));
  canvas.ops.length = 0;
  plot.setTrajectory({ points: [] });
  assert.equal(plot.trajectory, null);
  assert.equal(plot.limits, null);
  assert.ok(has(canvas.ops, 'fillText', 'No trajectory yet.', 100, 50));
  // A cleared plan and a trajectory without points both leave the plot empty.
  for (const empty of [null, {}]) {
    plot.setTrajectory(trajectory);
    plot.setTrajectory(empty);
    assert.equal(plot.trajectory, null);
  }
});

test('setSeries redraws the axis for the chosen series', () => {
  const { canvas, plot } = plotted();
  plot.setSeries('velocities');
  assert.equal(plot.series, 'velocities');
  assert.deepEqual(yLabels(canvas.ops), ['0.0', '2.0', '4.0']);
  canvas.ops.length = 0;
  plot.setSeries('positions');
  assert.equal(plot.series, 'positions');
  assert.deepEqual(yLabels(canvas.ops), ['0.0', '0.5', '1.0']);
});

test('setPlayhead draws a solid line at that time and null removes it', () => {
  const { canvas, plot } = plotted();
  plot.setPlayhead(0.5);
  assert.equal(plot.playhead, 0.5);
  assert.ok(has(canvas.ops, 'moveTo', MID, 6));
  assert.ok(has(canvas.ops, 'set', 'strokeStyle', '#000'));
  canvas.ops.length = 0;
  plot.setPlayhead(null);
  assert.equal(plot.playhead, null);
  assert.ok(has(canvas.ops, 'clearRect', 0, 0, 200, 100));
  assert.ok(!has(canvas.ops, 'moveTo', MID, 6));
});

test('setHover draws a dashed, labelled cursor at that time and null removes it', () => {
  const { canvas, plot } = plotted();
  plot.setHover(0.5);
  assert.equal(plot.hover, 0.5);
  assert.ok(has(canvas.ops, 'setLineDash', [3, 3]));
  assert.ok(has(canvas.ops, 'moveTo', MID, 6));
  assert.ok(has(canvas.ops, 'fillText', 't = 0.50 s', MID + 4, 7));
  canvas.ops.length = 0;
  plot.setHover(null);
  assert.equal(plot.hover, null);
  assert.ok(has(canvas.ops, 'clearRect', 0, 0, 200, 100));
  assert.ok(!has(canvas.ops, 'setLineDash', [3, 3]));
});

test('timeAt maps a pointer x across the plot area to trajectory time, with a small margin', () => {
  // The canvas sits at viewport x 10, so its 100 px plot area spans x 44..144.
  const { plot } = plotted(wave, { ...SIZED, left: 10 });
  assert.equal(plot.timeAt(69), 0.5);
  assert.equal(plot.timeAt(119), 1.5);
  // Within 2 % of either edge the pointer still picks that end; beyond, nothing.
  assert.equal(plot.timeAt(42), 0);
  assert.equal(plot.timeAt(146), 2);
  assert.equal(plot.timeAt(41), null);
  assert.equal(plot.timeAt(147), null);
});

test('timeAt is null without a trajectory and stays finite on a canvas narrower than its padding', () => {
  const empty = new TrajectoryPlot(fakeCanvas(SIZED));
  assert.equal(empty.timeAt(60), null);
  const { plot } = plotted(wave, { width: 30, height: 20, left: 10 });
  assert.equal(plot.timeAt(44), 0);
  assert.equal(plot.timeAt(44.5), 1);
});

test('positionsAt interpolates joint positions between points and holds the ends', () => {
  const { plot } = plotted({ points: [point(0, [0, 2]), { positions: [1, 4], time_from_start: 1 }, point(3, [5, 0])] });
  assert.deepEqual(plot.positionsAt(0), [0, 2]);
  assert.deepEqual(plot.positionsAt(0.5), [0.5, 3]);
  assert.deepEqual(plot.positionsAt(1), [1, 4]);
  assert.deepEqual(plot.positionsAt(2), [3, 2]);
  assert.deepEqual(plot.positionsAt(2.5), [4, 1]);
  assert.deepEqual(plot.positionsAt(3), [5, 0]);
  assert.deepEqual(plot.positionsAt(-1), [0, 2]);
  assert.deepEqual(plot.positionsAt(4), [5, 0]);
});

test('positionsAt is null without a trajectory and holds a single-point trajectory', () => {
  const empty = new TrajectoryPlot(fakeCanvas());
  assert.equal(empty.positionsAt(0), null);
  const { plot } = plotted({ points: [point(0, [0.5, -0.25])] });
  assert.deepEqual(plot.positionsAt(0), [0.5, -0.25]);
  assert.deepEqual(plot.positionsAt(1), [0.5, -0.25]);
});

test('draw paints a value grid, time labels and one line per joint across the plot area', () => {
  const { canvas, plot } = plotted(wave, SIZED);
  plot.draw();
  assert.ok(has(canvas.ops, 'setTransform', 1, 0, 0, 1, 0, 0));
  assert.ok(has(canvas.ops, 'clearRect', 0, 0, 142, 134));
  assert.deepEqual(painted(canvas.ops), {
    strokes: [
      line(GRID, [34, 112.5], [134, 112.5]),
      line(GRID, [34, 62.5], [134, 62.5]),
      line(GRID, [34, 12.5], [134, 12.5]),
      line(JOINT, [34, 112], [84, 12], [134, 62]),
      line(JOINT, [34, 87], [84, 37], [134, 12]),
    ],
    texts: [
      label('0.0', 30, 112.5, 'right', 'middle'),
      label('0.5', 30, 62.5, 'right', 'middle'),
      label('1.0', 30, 12.5, 'right', 'middle'),
      label('0.0 s', 34, 121, 'center', 'top'),
      label('0.5 s', 59, 121, 'center', 'top'),
      label('1.0 s', 84, 121, 'center', 'top'),
      label('1.5 s', 109, 121, 'center', 'top'),
      label('2.0 s', 134, 121, 'center', 'top'),
    ],
    rects: [],
  });
});

test('draw paints in the theme colours and font, cycling the joint colours', () => {
  theme = { '--muted': 'gray', '--border': 'silver', '--mono': 'Mono', '--joint-1': 'red', '--joint-2': 'green', '--strong': 'black' };
  try {
    const { canvas, plot } = plotted(wave, SIZED);
    plot.setHover(0.5);
    canvas.ops.length = 0;
    plot.setPlayhead(1);
    const { strokes, texts, rects } = painted(canvas.ops);
    assert.deepEqual(
      strokes.map(({ color }) => color),
      ['silver', 'silver', 'silver', 'red', 'green', 'gray', 'black'],
    );
    assert.deepEqual(new Set(texts.map(({ color, font }) => `${color} ${font}`)), new Set(['gray 10px Mono']));
    assert.deepEqual(rects, [{ rect: [60.5, 6, 66, 12], color: 'rgb(255, 255, 255)' }]);
    // An eighth joint takes the first joint's colour again.
    theme = { '--joint-1': 'red', '--joint-7': 'violet' };
    const eight = (v) => new Array(8).fill(v);
    const { canvas: wide, plot: arm } = plotted({ points: [point(0, eight(0)), point(1, eight(1))] }, SIZED);
    arm.draw();
    const jointColors = painted(wide.ops).strokes.filter(({ width }) => width === 1.6).map(({ color }) => color);
    assert.deepEqual(jointColors, ['red', '#888', '#888', '#888', '#888', '#888', 'violet', 'red']);
  } finally {
    theme = {};
  }
});

test('draw sizes the backing store for the device pixel ratio only when it changed', () => {
  const resizes = (ops) => ops.filter(([name]) => name === 'resize');
  window.devicePixelRatio = 2;
  try {
    const canvas = fakeCanvas();
    const plot = new TrajectoryPlot(canvas);
    plot.setTrajectory(trajectory);
    assert.deepEqual(resizes(canvas.ops), [['resize', 'width', 400], ['resize', 'height', 200]]);
    assert.ok(has(canvas.ops, 'setTransform', 2, 0, 0, 2, 0, 0));
    assert.ok(has(canvas.ops, 'clearRect', 0, 0, 200, 100));
    canvas.ops.length = 0;
    plot.draw();
    assert.deepEqual(resizes(canvas.ops), []);
    // A store wrong in only one dimension is resized in both.
    for (const backing of [[400, 0], [0, 200]]) {
      const stale = fakeCanvas({ backing });
      new TrajectoryPlot(stale).draw();
      assert.deepEqual(resizes(stale.ops), [['resize', 'width', 400], ['resize', 'height', 200]], `${backing}`);
    }
    // A browser that reports no ratio draws at 1.
    window.devicePixelRatio = undefined;
    const plain = fakeCanvas();
    new TrajectoryPlot(plain).draw();
    assert.deepEqual(resizes(plain.ops), [['resize', 'width', 200], ['resize', 'height', 100]]);
    assert.ok(has(plain.ops, 'setTransform', 1, 0, 0, 1, 0, 0));
  } finally {
    window.devicePixelRatio = 1;
  }
});

test('draw paints nothing while the canvas has no width or height', () => {
  for (const size of [{ width: 0 }, { height: 0 }]) {
    const canvas = fakeCanvas(size);
    new TrajectoryPlot(canvas).setTrajectory(trajectory);
    assert.deepEqual(canvas.ops, [], JSON.stringify(size));
  }
});

test('the velocity series spans a symmetric band of at least 0.1, capped at the fastest joint limit', () => {
  const limits = (...max) => max.map((max_velocity) => ({ lower: -3, upper: 3, max_velocity }));
  const moving = (j0, j1) => ({ points: [point(0, [0, 0], [0, 0]), point(1, [0.5, 0.5], [j0, j1]), point(2, [1, 1], [0, 0])] });
  // The band follows the largest speed in either direction: here -2 rad/s.
  const { canvas, plot } = plotted(moving(-2, 1), SIZED, limits(3, 3));
  plot.setSeries('velocities');
  assert.deepEqual(yLabels(canvas.ops), ['-2.0', '0.0', '2.0']);
  assert.deepEqual(
    painted(canvas.ops).strokes.filter(({ width }) => width === 1.6).map(({ path }) => path),
    [line(JOINT, [34, 62], [84, 112], [134, 62]).path, line(JOINT, [34, 62], [84, 37], [134, 62]).path],
  );
  for (const [shown, limit, labels] of [
    [moving(-2, 1), limits(1.5, 0.5), ['-1.0', '0.0', '1.0']],
    [moving(0.02, -0.01), limits(1, 1), ['-0.1', '0.0', '0.1']],
  ]) {
    const drawn = plotted(shown, SIZED, limit);
    drawn.plot.setSeries('velocities');
    assert.deepEqual(yLabels(drawn.canvas.ops), labels);
  }
  // A trajectory without velocities plots them as zero.
  const still = plotted(wave, SIZED);
  still.plot.setSeries('velocities');
  assert.deepEqual(yLabels(still.canvas.ops), ['-0.5', '0.0', '0.5']);
  assert.deepEqual(painted(still.canvas.ops).strokes.at(-1).path, line(JOINT, [34, 62], [84, 62], [134, 62]).path);
});

test('a series that never changes is centred, while a change of 1e-6 is drawn to scale', () => {
  const held = plotted({ points: [point(0, [1]), point(2, [1])] }, SIZED);
  held.plot.draw();
  assert.deepEqual(yLabels(held.canvas.ops), ['0.5', '1.0', '1.5']);
  assert.deepEqual(painted(held.canvas.ops).strokes.at(-1), line(JOINT, [34, 62], [134, 62]));
  const nudged = plotted({ points: [point(0, [0]), point(2, [1e-6])] }, SIZED);
  nudged.plot.draw();
  assert.deepEqual(painted(nudged.canvas.ops).strokes.at(-1), line(JOINT, [34, 112], [134, 12]));
});

test('the hover label sits right of the cursor and flips left within 60 px of the plot end', () => {
  const HOVER = { color: '#888', width: 1, join: 'round', dash: [3, 3] };
  const hovered = (seconds) => {
    const { canvas, plot } = plotted(wave, SIZED);
    plot.setHover(seconds);
    const { strokes, texts, rects } = painted(canvas.ops);
    return { cursor: strokes.at(-1), text: texts.at(-1), rects };
  };
  // The flip point is x = 34 + 100 - 60 = 74; a cursor snaps to a pixel centre.
  for (const [seconds, x, text, rect, align] of [
    [0.5, 59.5, 63.5, [60.5, 6, 66, 12], 'left'],
    [0.788, 73.5, 77.5, [74.5, 6, 66, 12], 'left'],
    [0.808, 74.5, 70.5, [7.5, 6, 66, 12], 'right'],
    [1.9, 129.5, 125.5, [62.5, 6, 66, 12], 'right'],
  ]) {
    const shown = hovered(seconds);
    assert.deepEqual(shown.cursor, line(HOVER, [x, 6], [x, 118]), `${seconds}`);
    assert.deepEqual(shown.text, label(`t = ${seconds.toFixed(2)} s`, text, 7, align, 'top'), `${seconds}`);
    assert.deepEqual(shown.rects, [{ rect, color: 'rgb(255, 255, 255)' }], `${seconds}`);
  }
  // Past the end the cursor stays on the last moment but the label keeps the time.
  const late = hovered(5);
  assert.deepEqual(late.cursor, line(HOVER, [134.5, 6], [134.5, 118]));
  assert.equal(late.text.text, 't = 5.00 s');
});

test('the playhead is a solid line that stays at the end once execution runs past it', () => {
  const { canvas, plot } = plotted(wave, SIZED);
  plot.setHover(1);
  canvas.ops.length = 0;
  plot.setPlayhead(3);
  const PLAYHEAD = { color: '#000', width: 1, join: 'round' };
  assert.deepEqual(painted(canvas.ops).strokes.at(-1), line(PLAYHEAD, [134.5, 6], [134.5, 118]));
});

test('labels drop the decimal for trajectories of 3 s or more and for values of 10 or more', () => {
  const { canvas, plot } = plotted({ points: [point(0, [0]), point(4, [10])] }, SIZED);
  plot.draw();
  assert.deepEqual(yLabels(canvas.ops), ['0.0', '5.0', '10']);
  const times = painted(canvas.ops).texts.filter(({ baseline }) => baseline === 'top');
  assert.deepEqual(times, [0, 1, 2, 3, 4].map((t) => label(`${t} s`, 34 + 25 * t, 121, 'center', 'top')));
});

test('draw places a one-point trajectory at the start and a canvas smaller than its padding at its corner', () => {
  const joints = (ops) => painted(ops).strokes.filter(({ width }) => width === 1.6);
  const single = plotted({ points: [point(0, [0.5, 0.2])] }, SIZED);
  single.plot.draw();
  assert.deepEqual(joints(single.canvas.ops), [line(JOINT, [34, 12]), line(JOINT, [34, 112])]);
  // A 30x20 canvas still gets a 1x1 px plot area at the padding corner.
  const tiny = plotted(trajectory, { width: 30, height: 20 });
  tiny.plot.draw();
  assert.deepEqual(joints(tiny.canvas.ops)[0], line(JOINT, [34, 6.946429], [35, 6.053571]));
});

test('every axis label names its own tick, however fine the steps', () => {
  const line = (duration, from, to) => ({
    points: [
      { positions: [from], time_from_start: { sec: 0, nanosec: 0 } },
      { positions: [to], time_from_start: { sec: Math.floor(duration), nanosec: Math.round((duration % 1) * 1e9) } },
    ],
  });
  const timeLabels = (ops) => ops.filter(([name, text]) => name === 'fillText' && text.endsWith(' s')).map(([, text]) => text);
  for (const [duration, from, to] of [[3, 0, 1], [0.3, 0, 1], [1, 0, 0.1], [1, 10, 12], [1, 0, 2.5]]) {
    const { canvas, plot } = plotted(line(duration, from, to));
    plot.draw();
    for (const labels of [timeLabels(canvas.ops), yLabels(canvas.ops)]) {
      assert.ok(labels.length > 1, `${duration} s, ${from}..${to}: ${labels}`);
      assert.deepEqual(labels, [...new Set(labels)], `${duration} s, ${from}..${to}`);
    }
  }
  const { canvas, plot } = plotted(line(4, 0, 1));
  plot.draw();
  assert.deepEqual(timeLabels(canvas.ops), ['0 s', '1 s', '2 s', '3 s', '4 s']);
});
