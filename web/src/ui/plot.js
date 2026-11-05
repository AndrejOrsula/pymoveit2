// Joint positions or velocities of one trajectory over time, drawn on a
// canvas, with a playhead that follows the robot during execution and a hover
// cursor that picks a moment of the trajectory to preview.

const JOINT_COLORS = 7;
const PAD = { left: 34, right: 8, top: 6, bottom: 16 };

function cssVar(element, name) {
  return getComputedStyle(element).getPropertyValue(name).trim();
}

/** The first opaque background behind `element`, for labels drawn over lines. */
function backdrop(element) {
  for (let e = element; e; e = e.parentElement) {
    const color = getComputedStyle(e).backgroundColor;
    if (color && color !== 'transparent' && !/^rgba\(.*,\s*0\)$/.test(color)) return color;
  }
  return '#fff';
}

/** Seconds from a JointTrajectory point's time_from_start. */
export function pointTime(point) {
  const t = point.time_from_start;
  return typeof t === 'number' ? t : t.sec + t.nanosec * 1e-9;
}

export class TrajectoryPlot {
  constructor(canvas) {
    this.canvas = canvas;
    this.trajectory = null;
    this.series = 'positions';
    this.playhead = null;
    this.hover = null;
    this.limits = null;
  }

  /** `limits` are per-joint {lower, upper, max_velocity} for the bands. */
  setTrajectory(trajectory, limits = null) {
    const kept = trajectory?.points?.length ? trajectory : null;
    this.#update({ trajectory: kept, limits, playhead: null, hover: null });
  }

  setSeries(series) {
    this.#update({ series });
  }

  setPlayhead(seconds) {
    this.#update({ playhead: seconds });
  }

  setHover(seconds) {
    this.#update({ hover: seconds });
  }

  /** Every state change redraws the plot. */
  #update(state) {
    Object.assign(this, state);
    this.draw();
  }

  /** Trajectory time under a viewport x coordinate, or null outside the plot. */
  timeAt(clientX) {
    if (!this.trajectory) return null;
    const rect = this.canvas.getBoundingClientRect();
    const fraction = (clientX - rect.left - PAD.left) / Math.max(1, rect.width - PAD.left - PAD.right);
    if (fraction < -0.02 || fraction > 1.02) return null;
    return Math.min(Math.max(fraction, 0), 1) * this.duration;
  }

  /** Joint positions at `seconds`, interpolated linearly between points. */
  positionsAt(seconds) {
    const points = this.trajectory?.points;
    if (!points) return null;
    let i = 1;
    while (i < points.length - 1 && pointTime(points[i]) < seconds) i += 1;
    const a = points[i - 1];
    const b = points[i] ?? a;
    const ta = pointTime(a);
    const tb = pointTime(b);
    const s = tb > ta ? Math.min(Math.max((seconds - ta) / (tb - ta), 0), 1) : 1;
    return a.positions.map((value, j) => value + (b.positions[j] - value) * s);
  }

  get duration() {
    const points = this.trajectory?.points;
    return points ? pointTime(points[points.length - 1]) : 0;
  }

  draw() {
    const { canvas } = this;
    const view = clearedContext(canvas);
    if (!view) return;
    const { ctx, width, height } = view;
    const { muted, border, font } = palette(canvas);
    ctx.font = `10px ${font}`;
    ctx.lineWidth = 1;

    if (!this.trajectory) {
      ctx.fillStyle = muted;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('No trajectory yet.', width / 2, height / 2);
      return;
    }

    const points = this.trajectory.points;
    const joints = points[0].positions.length;
    const values = points.map((p) => p[this.series] ?? new Array(joints).fill(0));
    const [lo, hi] = valueRange(values, this.series, this.limits);
    const duration = Math.max(this.duration, 1e-3);
    const plotW = Math.max(1, width - PAD.left - PAD.right);
    const plotH = Math.max(1, height - PAD.top - PAD.bottom);
    const frame = {
      ctx,
      canvas,
      plotW,
      plotH,
      duration,
      x: (t) => PAD.left + (t / duration) * plotW,
      y: (v) => PAD.top + (1 - (v - lo) / (hi - lo)) * plotH,
    };
    drawAxes(frame, lo, hi, muted, border);
    drawJoints(frame, points, values, joints);
    if (this.hover !== null) drawHover(frame, this.hover, muted);
    if (this.playhead !== null) cursorLine(frame, this.playhead, cssVar(canvas, '--strong') || '#000');
  }
}

/** Sizes the canvas store to its CSS box at the device pixel ratio and clears it; null while it has no area. */
function clearedContext(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return null;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  return { ctx, width, height };
}

/** The theme's label, grid and font settings, with fallbacks outside the page's CSS. */
function palette(canvas) {
  return {
    muted: cssVar(canvas, '--muted') || '#888',
    border: cssVar(canvas, '--border') || '#ccc',
    font: cssVar(canvas, '--mono') || 'monospace',
  };
}

/**
 * The [lo, hi] values the plot spans, with a margin: velocities with limits get a
 * symmetric band of at least 0.1 capped at the fastest joint, flat series a unit span.
 */
function valueRange(values, series, limits) {
  let lo = Infinity;
  let hi = -Infinity;
  for (const row of values) {
    for (const v of row) {
      lo = Math.min(lo, v);
      hi = Math.max(hi, v);
    }
  }
  if (series === 'velocities' && limits) {
    const vmax = Math.max(...limits.map((l) => l.max_velocity));
    hi = Math.max(hi, 0.1);
    hi = Math.min(Math.max(hi, -lo), vmax);
    lo = -hi;
  }
  if (hi - lo < 1e-6) {
    hi += 0.5;
    lo -= 0.5;
  }
  const span = hi - lo;
  return [lo - span * 0.06, hi + span * 0.06];
}

/** Value grid lines with their labels, and time labels under the plot. */
function drawAxes(frame, lo, hi, muted, border) {
  const { ctx, plotW, plotH, duration, x, y } = frame;
  ctx.strokeStyle = border;
  ctx.fillStyle = muted;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  const values = niceTicks(lo, hi, 4);
  for (const v of values) {
    const yy = Math.round(y(v)) + 0.5;
    ctx.beginPath();
    ctx.moveTo(PAD.left, yy);
    ctx.lineTo(PAD.left + plotW, yy);
    ctx.stroke();
    ctx.fillText(v.toFixed(Math.max(Math.abs(v) < 10 ? 1 : 0, stepDecimals(values))), PAD.left - 4, yy);
  }
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  const times = niceTicks(0, duration, 6);
  for (const t of times) {
    ctx.fillText(`${t.toFixed(stepDecimals(times))} s`, x(t), PAD.top + plotH + 3);
  }
}

/** One line per joint through its value at each point, in the joint's theme colour. */
function drawJoints(frame, points, values, joints) {
  const { ctx, canvas, x, y } = frame;
  ctx.lineWidth = 1.6;
  ctx.lineJoin = 'round';
  for (let j = 0; j < joints; j += 1) {
    ctx.strokeStyle = cssVar(canvas, `--joint-${(j % JOINT_COLORS) + 1}`) || '#888';
    ctx.beginPath();
    points.forEach((p, i) => {
      const px = x(pointTime(p));
      const py = y(values[i][j]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }
}

/** The dashed hover cursor and its time label, which flips left near the plot's end. */
function drawHover(frame, seconds, muted) {
  const { ctx, canvas, plotW } = frame;
  ctx.setLineDash([3, 3]);
  const px = cursorLine(frame, seconds, muted);
  ctx.setLineDash([]);
  const label = `t = ${seconds.toFixed(2)} s`;
  const right = px > PAD.left + plotW - 60;
  const labelW = ctx.measureText(label).width + 6;
  ctx.fillStyle = backdrop(canvas);
  ctx.fillRect(right ? px - 1 - labelW : px + 1, PAD.top, labelW, 12);
  ctx.fillStyle = muted;
  ctx.textBaseline = 'top';
  ctx.textAlign = right ? 'right' : 'left';
  ctx.fillText(label, px + (right ? -4 : 4), PAD.top + 1);
}

/** A vertical line across the plot at `seconds`, held at the end past it; returns its x. */
function cursorLine(frame, seconds, color) {
  const { ctx, plotH, duration, x } = frame;
  const px = Math.round(x(Math.min(seconds, duration))) + 0.5;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(px, PAD.top);
  ctx.lineTo(px, PAD.top + plotH);
  ctx.stroke();
  return px;
}

/** The fewest decimals that write the step between `ticks` exactly, so no two labels read alike. */
function stepDecimals(ticks) {
  // A lone tick gives a NaN step, which needs no decimals.
  const step = ticks[1] - ticks[0];
  let decimals = 0;
  while (Math.abs(Math.round(step * 10 ** decimals) - step * 10 ** decimals) > 1e-6) decimals += 1;
  return decimals;
}

function niceTicks(lo, hi, count) {
  const span = hi - lo;
  if (!(span > 0)) return [lo];
  const raw = span / count;
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * magnitude).find((s) => s >= raw) ?? raw;
  const ticks = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) ticks.push(Math.abs(v) < 1e-9 ? 0 : v);
  return ticks;
}
