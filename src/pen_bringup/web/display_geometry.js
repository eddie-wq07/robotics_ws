// Simplify display geometry only. Executable poses and their timing stay intact.
export function cloneSurfaceVisual(source, visible = true) {
  const copy = source.clone(true);
  copy.position.set(0, 0, 0);
  copy.quaternion.identity();
  copy.scale.set(1, 1, 1);
  copy.visible = visible;
  copy.userData.sharedRenderResources = true;
  return copy;
}

export function simplifiedIndices(points, { tolerance = .000025, times = null, maxInterval = .05 } = {}) {
  if (points.length < 3) return points.map((_, i) => i);
  const keep = new Uint8Array(points.length);
  keep[0] = keep[points.length - 1] = 1;
  const stack = [[0, points.length - 1]], limit = tolerance * tolerance;
  while (stack.length) {
    const [start, end] = stack.pop(), a = points[start], b = points[end];
    const d = b.map((v, k) => v - a[k]);
    const length2 = d.reduce((sum, v) => sum + v * v, 0);
    let worst = limit, split = -1;
    for (let i = start + 1; i < end; i++) {
      const p = points[i];
      const u = length2 ? Math.max(0, Math.min(1, d.reduce((sum, v, k) => sum + (p[k] - a[k]) * v, 0) / length2)) : 0;
      const distance2 = p.reduce((sum, v, k) => sum + (v - a[k] - u * d[k]) ** 2, 0);
      if (distance2 > worst) { worst = distance2; split = i; }
    }
    if (split >= 0) {
      keep[split] = 1; stack.push([start, split], [split, end]);
    }
  }
  if (times) {
    let previous = 0;
    for (let i = 1; i < points.length; i++) {
      if (keep[i] || times[i] - times[previous] >= maxInterval) { keep[i] = 1; previous = i; }
    }
  }
  return [...keep.keys()].filter(i => keep[i]);
}

export function displayDrawRows(samples, worldTips, localTips) {
  const rows = [];
  for (let i = 1; i < samples.length; i++) {
    if (!samples[i - 1].extrude || !samples[i].extrude) continue;
    const start = i - 1;
    while (i + 1 < samples.length && samples[i + 1].extrude) i++;
    const points = worldTips.slice(start, i + 1).map(p => p.toArray());
    const times = samples.slice(start, i + 1).map(p => p.time_s);
    const indices = simplifiedIndices(points, { times });
    for (let j = 1; j < indices.length; j++) {
      const a = start + indices[j - 1], b = start + indices[j];
      rows.push({ a: samples[a], b: samples[b], worldA: worldTips[a], worldB: worldTips[b],
        localA: localTips[a], localB: localTips[b] });
    }
  }
  return rows;
}
