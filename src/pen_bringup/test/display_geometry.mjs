// Run with Node; no browser or GPU is opened by this check.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
const source = readFileSync(new URL('../web/display_geometry.js', import.meta.url), 'utf8');
const { simplifiedIndices, displayDrawRows, cloneSurfaceVisual } = await import('data:text/javascript,' + encodeURIComponent(source));
const THREE = await import(new URL('../web/vendor/three/build/three.module.js', import.meta.url));
const body = new THREE.Group(); body.position.set(1,2,3); body.rotation.z=.5;
const bottle = new THREE.Mesh(new THREE.BoxGeometry(), new THREE.MeshStandardMaterial());
bottle.position.set(.005,0,.03); body.add(bottle);
const copy = cloneSurfaceVisual(body);
assert.equal(copy.children[0].geometry, bottle.geometry);
assert.equal(copy.children[0].material, bottle.material);
assert.deepEqual(copy.children[0].position.toArray(), bottle.position.toArray());
assert.deepEqual(copy.position.toArray(), [0,0,0]);
assert.deepEqual(copy.quaternion.toArray(), [0,0,0,1]);
assert(copy.userData.sharedRenderResources);

const line = Array.from({ length: 10001 }, (_, i) => [i * .000001, 0, 0]);
assert.deepEqual(simplifiedIndices(line), [0, 10000]);
const ring = [[0,0,0], [.01,0,0], [.01,.01,0], [0,.01,0], [0,0,0]];
assert.deepEqual(simplifiedIndices(ring), [0,1,2,3,4]);

const curve = Array.from({ length: 2001 }, (_, i) => {
  const a = i / 2000 * Math.PI * 2;
  return [.009 * Math.cos(a), .009 * Math.sin(a), 0];
});
const keep = simplifiedIndices(curve);
assert(keep.length < 100);
for (let k = 1; k < keep.length; k++) {
  const a = curve[keep[k-1]], b = curve[keep[k]], d = b.map((v,i) => v-a[i]);
  const l2 = d.reduce((n,v) => n+v*v,0);
  for (let i = keep[k-1]; i <= keep[k]; i++) {
    const u = Math.max(0,Math.min(1,d.reduce((n,v,j) => n+(curve[i][j]-a[j])*v,0)/l2));
    assert(Math.hypot(...curve[i].map((v,j) => v-a[j]-u*d[j])) <= .000025 + 1e-12);
  }
}
const samples = line.map((_, i) => ({ time_s: i/1000, extrude: i!==5000 }));
const points = line.map(p => ({ toArray: () => p }));
const before = JSON.stringify(samples);
const rows = displayDrawRows(samples, points, points);
assert(rows.length < 250);
assert(rows.every(r => r.b.time_s-r.a.time_s <= .051+1e-9));
assert(!rows.some(r => r.a.time_s<5 && r.b.time_s>5));
assert.equal(JSON.stringify(samples), before);
console.log('Display geometry: shape tolerance, timing, extrusion gaps and immutable samples passed.');
