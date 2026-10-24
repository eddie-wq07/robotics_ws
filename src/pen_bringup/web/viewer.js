// One three.js view: an object, strokes, or the robot cell with its motion.
// Ported from gilwoolee/3d_art viewer/src/viewer.ts (MIT), without a build step.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { simplifiedIndices } from './display_geometry.js';

const matrix = values => new THREE.Matrix4().fromArray(values.flat()).transpose();
export const LAYER_COLOURS = [0x4f86df, 0xed846f, 0x6856c0, 0x91b4ed, 0xc783c5];
// A layer's colour: the five above, or a blue-coral-violet gradient for taller reliefs.
export function layerColour(layer, count) {
  if (count <= LAYER_COLOURS.length) return LAYER_COLOURS[(layer - 1) % LAYER_COLOURS.length];
  const anchors = [[0x4f, 0x86, 0xdf], [0xed, 0x84, 0x6f], [0x68, 0x56, 0xc0]];
  const where = (layer - 1) / Math.max(1, count - 1) * (anchors.length - 1);
  const low = Math.min(Math.floor(where), anchors.length - 2), u = where - low;
  const [r, g, b] = anchors[low].map((v, i) => Math.round(v + (anchors[low + 1][i] - v) * u));
  return (r << 16) | (g << 8) | b;
}

export class Viewer {
  constructor(container, { studio = false } = {}) {
    this.container = container;
    this.studio = studio;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    const theme = getComputedStyle(document.documentElement);
    this.renderer.setClearColor(theme.getPropertyValue('--viewport-bg').trim());
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = studio ? THREE.NeutralToneMapping : THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = studio ? 1.0 : 1.25;
    container.prepend(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(35, 1, 0.001, 10);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.dirty = true;
    this.controls.addEventListener('change', () => this.requestRender(false));
    const gl = this.renderer.getContext(), debug = gl.getExtension('WEBGL_debug_renderer_info');
    this.container.dataset.renderer = debug ? gl.getParameter(debug.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
    this.scene.add(new THREE.HemisphereLight(0xffffff, studio ? 0x888888 : 0x889bb0, studio ? .65 : 2.4));
    const light = new THREE.DirectionalLight(0xffffff, studio ? 2.2 : 2.8);
    light.position.set(1.2, -1.5, 2.5);
    const fill = new THREE.DirectionalLight(studio ? 0xffffff : 0xe4ecff, studio ? .8 : 1.3);
    fill.position.set(-0.4, 0.1, 0.3);
    this.scene.add(light, fill);
    if (studio) {
      const environment = new RoomEnvironment();
      const generator = new THREE.PMREMGenerator(this.renderer);
      this.environmentMap = generator.fromScene(environment);
      this.scene.environment = this.environmentMap.texture;
      environment.dispose(); generator.dispose();
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.autoUpdate = false;
      this.renderer.shadowMap.needsUpdate = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      light.castShadow = true;
      light.shadow.mapSize.set(2048, 2048);
      Object.assign(light.shadow.camera, {left: -1.2, right: 1.2, top: 1.2, bottom: -1.2, near: .1, far: 5});
      light.shadow.bias = -.0001; light.shadow.normalBias = .0005;
      const ground = new THREE.Mesh(new THREE.PlaneGeometry(4, 4),
        new THREE.ShadowMaterial({opacity: .32}));
      ground.position.z = -.0005; ground.receiveShadow = true;
      this.scene.add(ground);
    }
    this.grid = new THREE.GridHelper(studio ? 2 : .28, studio ? 20 : 14,
      new THREE.Color(theme.getPropertyValue('--grid-major').trim()),
      new THREE.Color(theme.getPropertyValue('--grid-minor').trim()));
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -0.0003;
    this.content = new THREE.Group();
    this.scene.add(this.grid, this.content);
    this.loader = new GLTFLoader();
    this.center = new THREE.Vector3();
    this.extent = 0.15;
    this.arm = null;
    this.beads = null;
    new ResizeObserver(() => this.resize()).observe(container);
    const draw = () => {
      requestAnimationFrame(draw);
      this.controls.update();
      if (this.dirty) { this.renderer.render(this.scene, this.camera); this.dirty = false; }
    };
    draw();
  }

  requestRender(shadows = true) {
    this.dirty = true;
    if (shadows) this.renderer.shadowMap.needsUpdate = true;
  }

  resize() {
    const { width, height } = this.container.getBoundingClientRect();
    if (!width || !height) return;
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  clear() {
    this.content.traverse(object => {
      for (let parent = object; parent; parent = parent.parent)
        if (parent.userData.sharedRenderResources) return;
      if (object.geometry) object.geometry.dispose();
      const materials = object.material ? [object.material].flat() : [];
      for (const material of materials) material.dispose();
    });
    this.content.clear();
    this.arm = null;
    this.beads = null;
    this.requestRender();
  }

  async glb(url, parent = this.content) {
    const gltf = await this.loader.loadAsync(url);
    parent.add(gltf.scene);
    this.requestRender();
    return gltf.scene;
  }

  // A mesh's materials: lighter, optionally see-through.
  static tint(object, { color, opacity = 1, vertexColors } = {}) {
    object.traverse(node => {
      if (!node.isMesh) return;
      const hasColours = !!node.geometry.attributes.color;
      node.material = new THREE.MeshStandardMaterial({
        color: color ?? 0xffffff, roughness: 0.75, metalness: 0,
        vertexColors: vertexColors ?? hasColours,
        transparent: opacity < 1, opacity, depthWrite: opacity >= 1, side: THREE.DoubleSide,
      });
    });
  }

  // A see-through ghost with its sharp edges drawn: an object's expected place
  // beside its measured one.
  static outline(object, hex) {
    Viewer.tint(object, { color: hex, opacity: 0.06, vertexColors: false });
    const meshes = [];
    object.traverse(node => { if (node.isMesh) meshes.push(node); });
    for (const node of meshes) {
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(node.geometry, 20),
        new THREE.LineBasicMaterial({ color: hex, transparent: true, opacity: 0.8 }));
      node.add(edges);
    }
  }

  // Strands as 1 mm tubes: one instanced cylinder per segment, coloured per segment.
  static tubes(segments, radius, capacity) {
    const mesh = new THREE.InstancedMesh(
      new THREE.CylinderGeometry(radius, radius, 1, 8, 1, false),
      new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.6 }),
      Math.max(1, capacity ?? segments.length));
    mesh.frustumCulled = false;
    Viewer.placeTubes(mesh, segments, radius);
    return mesh;
  }

  static placeTubes(mesh, segments, radius) {
    const up = new THREE.Vector3(0, 1, 0), a = new THREE.Vector3(), b = new THREE.Vector3();
    const direction = new THREE.Vector3(), turn = new THREE.Quaternion(), m = new THREE.Matrix4();
    const scale = new THREE.Vector3(1, 1, 1), colour = new THREE.Color();
    let index = 0;
    for (const [p, q, hex] of segments) {
      a.set(p[0], p[1], p[2]); b.set(q[0], q[1], q[2]);
      direction.subVectors(b, a);
      const length = direction.length();
      if (length < 1e-9) continue;
      turn.setFromUnitVectors(up, direction.divideScalar(length));
      scale.set(1, length + radius, 1);
      m.compose(a.add(b).multiplyScalar(0.5), turn, scale);
      mesh.setMatrixAt(index, m);
      mesh.setColorAt(index, colour.setHex(hex));
      index++;
    }
    mesh.count = index;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  }

  strokes(toolpath, { travels = true } = {}) {
    const radius = toolpath.strand_width_mm / 2000 * 0.9;
    const segments = [];
    for (const stroke of toolpath.strokes) {
      const hex = layerColour(stroke.layer, toolpath.layers);
      const points = simplifiedIndices(stroke.points_m).map(i => stroke.points_m[i]);
      for (let i = 1; i < points.length; i++) segments.push([points[i - 1], points[i], hex]);
    }
    const group = new THREE.Group();
    group.add(Viewer.tubes(segments, radius));
    if (travels) {
      const line = [];
      for (const travel of toolpath.travels) {
        for (let i = 1; i < travel.points_m.length; i++) line.push(...travel.points_m[i - 1], ...travel.points_m[i]);
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(line, 3));
      group.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: 0x9aa39a, transparent: true, opacity: 0.6 })));
    }
    this.content.add(group);
    this.requestRender();
    return group;
  }

  // Small balls at points: probe sites and touches.
  dots(points, hex, radius, parent = this.content) {
    const mesh = new THREE.InstancedMesh(new THREE.SphereGeometry(radius, 16, 10),
      new THREE.MeshStandardMaterial({ color: hex, roughness: 0.5 }), Math.max(1, points.length));
    const m = new THREE.Matrix4();
    points.forEach((p, i) => mesh.setMatrixAt(i, m.makeTranslation(p[0], p[1], p[2])));
    mesh.count = points.length;
    mesh.frustumCulled = false;
    parent.add(mesh);
    return mesh;
  }

  // The robot: its GLB posed from the manifest's kinematic chain, and the pen.
  async robot(rig, url, cell) {
    const source = await this.glb(url);
    const toolGeometry = new THREE.Group(), tip = new THREE.Group();
    this.content.add(toolGeometry, tip);
    for (const body of cell.arm.tool_model.visual_bodies) {
      let geometry;
      if (body.type === 'box') geometry = new THREE.BoxGeometry(...body.size_m);
      else if (body.type === 'sphere') geometry = new THREE.SphereGeometry(body.radius_m, 24, 16);
      else if (body.type === 'cylinder') {
        geometry = new THREE.CylinderGeometry(body.radius_m, body.radius_m, body.length_m, 32);
        geometry.rotateX(Math.PI / 2); // Collision cylinders use local z; three.js uses y.
      } else {
        geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(body.vertices.flat(), 3));
        geometry.setIndex(body.faces.flat());
        geometry.computeVertexNormals();
      }
      const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: body.status?.startsWith('placeholder') ? 0xe9a28c : 0x647b99, roughness: 0.65 }));
      mesh.name = body.name;
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(matrix(body.origin));
      mesh.matrixWorldNeedsUpdate = true;
      toolGeometry.add(mesh);
    }
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.0012, 12, 8), new THREE.MeshStandardMaterial({ color: 0x327aa0 }));
    tip.add(marker);
    this.arm = { rig, source, toolGeometry, marker, cell };
    this.pose(cell.arm.initial_positions);
  }

  // Pose the arm at joint angles q; returns the pen tip in the world.
  pose(q) {
    const { rig, source, toolGeometry, marker, cell } = this.arm;
    const frames = new Map([[rig.kinematic_root, matrix(cell.arm.world_from_base)]]);
    const axis = new THREE.Vector3();
    for (const joint of rig.kinematic_chain) {
      const frame = frames.get(joint.parent).clone().multiply(matrix(joint.origin));
      if (joint.index !== null && joint.axis) {
        frame.multiply(new THREE.Matrix4().makeRotationAxis(axis.set(...joint.axis).normalize(), q[joint.index]));
      }
      frames.set(joint.child, frame);
    }
    for (const visual of rig.visual_nodes) {
      const node = source.getObjectByName(visual.name);
      if (!node) continue;
      node.matrixAutoUpdate = false;
      node.matrix.copy(frames.get(visual.link)).multiply(matrix(visual.origin));
      node.matrixWorldNeedsUpdate = true;
    }
    const flange = frames.get(rig.kinematic_tip);
    toolGeometry.matrixAutoUpdate = false;
    toolGeometry.matrix.copy(flange);
    toolGeometry.matrixWorldNeedsUpdate = true;
    const tcp = flange.clone().multiply(matrix(cell.arm.flange_to_tcp));
    marker.position.setFromMatrixPosition(tcp);
    return marker.position.toArray();
  }

  // Deposited strand: grows as the plan plays.
  deposit(segments, radius, capacity) {
    if (!this.beads || this.beads.instanceMatrix.count < segments.length) {
      if (this.beads) { this.content.remove(this.beads); this.beads.geometry.dispose(); }
      this.beads = Viewer.tubes([], radius, Math.max(1024, capacity ?? segments.length));
      this.content.add(this.beads);
    }
    Viewer.placeTubes(this.beads, segments, radius);
  }

  platform(cell) {
    const env = cell.environment, origin = cell.world_from_build.map(row => row[3]);
    const height = Math.max(0.006, env.plate_top_z_m - env.table_z_m);
    const block = new THREE.Mesh(new THREE.BoxGeometry(env.plate_size_m[0], env.plate_size_m[1], height),
      new THREE.MeshStandardMaterial({ color: 0xb2c2d3, roughness: 0.9, transparent: true, opacity: 0.6 }));
    block.position.set(origin[0], origin[1], env.plate_top_z_m - height / 2);
    this.content.add(block);
    this.requestRender();
  }

  frame(object, { direction = [1, -1.4, 0.9], margin = 2.4 } = {}) {
    object.updateWorldMatrix(true, true); // Placed groups may not have rendered yet.
    const box = new THREE.Box3().setFromObject(object);
    if (box.isEmpty()) return;
    box.getCenter(this.center);
    const size = box.getSize(new THREE.Vector3());
    this.extent = Math.max(0.03, size.x, size.y, size.z);
    this.controls.minDistance = this.extent * 0.08;
    this.controls.maxDistance = this.extent * 12;
    if (!this.studio) {
      this.grid.scale.setScalar(Math.max(this.extent, 0.1) / 0.15);
      this.grid.position.set(this.center.x, this.center.y, this.grid.position.z);
    }
    const view = new THREE.Vector3(...direction).normalize();
    let distance = this.extent * margin;
    if (this.studio) {
      const right = new THREE.Vector3().crossVectors(this.camera.up, view).normalize();
      const up = new THREE.Vector3().crossVectors(view, right).normalize();
      const tanV = Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2));
      const tanH = tanV * this.camera.aspect;
      distance = 0;
      for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y]) for (const z of [box.min.z, box.max.z]) {
        const corner = new THREE.Vector3(x, y, z).sub(this.center);
        distance = Math.max(distance, corner.dot(view) + Math.max(Math.abs(corner.dot(right)) / tanH, Math.abs(corner.dot(up)) / tanV));
      }
      distance *= margin;
    }
    this.camera.far = Math.max(10, distance * 4); this.camera.updateProjectionMatrix();
    this.camera.position.copy(this.center).addScaledVector(view, distance);
    this.controls.target.copy(this.center);
    this.controls.update();
    this.requestRender();
  }
}

export { THREE, matrix };
