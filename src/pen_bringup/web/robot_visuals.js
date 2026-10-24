// CAD appearance and display-only registration. No motion commands.
import * as THREE from 'three';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { mergeVertices } from 'three/addons/utils/BufferGeometryUtils.js';

const neutralColors = {pen_holder: 0x424b55, pen_body: 0xd3d6da,
  pen_jig: 0x535e69, pen_surface: 0xe8e5de};

export function applyJointPositions(joints, positions) {
  for (const {joint, group} of joints) {
    const angle = positions?.[joint.name];
    if (joint.type === 'revolute' && Number.isFinite(angle)) {
      group.quaternion.setFromAxisAngle(new THREE.Vector3(...joint.axis).normalize(), angle);
    }
  }
}

export async function loadVisual(visual, linkName, materialGroups = null) {
  const base = new THREE.MeshStandardMaterial({
    color: neutralColors[linkName] ?? 0xb8bdc6, roughness: .55, metalness: .08,
  });
  if (visual.rgba) {
    base.color.setRGB(...visual.rgba.slice(0, 3), THREE.SRGBColorSpace);
    base.opacity = visual.rgba[3]; base.transparent = base.opacity < 1;
  }
  let shape;
  if (visual.kind === 'mesh' && /\.(glb|gltf)$/i.test(visual.url)) {
    // Keep the scan's node transforms and embedded PBR materials intact.
    shape = new THREE.Group();
    shape.add((await new GLTFLoader().loadAsync(visual.url)).scene);
  } else if (visual.kind === 'mesh' && visual.url.endsWith('.obj')) {
    const response = await fetch(visual.url);
    if (!response.ok) throw new Error(`Cannot load ${visual.url}`);
    const text = await response.text();
    const loader = new OBJLoader();
    const library = text.match(/^mtllib\s+(.+?)\s*$/m);
    if (library) {
      const url = new URL(library[1].trim(), new URL(visual.url, location.href));
      const materials = await new MTLLoader().loadAsync(url.href);
      materials.preload(); loader.setMaterials(materials);
    }
    shape = loader.parse(text);
    shape.traverse(mesh => {
      if (!mesh.isMesh) return;
      const convert = source => new THREE.MeshStandardMaterial({
        name: source.name, color: source.color, map: source.map,
        normalMap: source.normalMap, opacity: source.opacity,
        transparent: source.transparent, side: source.side,
        roughness: source.name === 'cover' ? .52 : .46,
        metalness: 0,
        envMapIntensity: .25,
      });
      mesh.material = library ? (Array.isArray(mesh.material)
        ? mesh.material.map(convert) : convert(mesh.material)) : base;
    });
  } else if (visual.kind === 'mesh') {
    let geometry = await new STLLoader().loadAsync(visual.url);
    if (linkName === 'pen_body') {
      geometry.deleteAttribute('normal');
      const smooth = mergeVertices(geometry, .000001); geometry.dispose();
      smooth.computeVertexNormals(); geometry = smooth;
    }
    if (materialGroups) {
      geometry.clearGroups();
      const materials = materialGroups.map((part, index) => {
        if (part.start_vertex < 0 || part.vertex_count <= 0 ||
            part.start_vertex + part.vertex_count > geometry.attributes.position.count)
          throw new Error('Combined CAD material group exceeds mesh bounds');
        geometry.addGroup(part.start_vertex, part.vertex_count, index);
        return new THREE.MeshStandardMaterial({color: neutralColors[part.component] ?? base.color,
          roughness: .55, metalness: .08});
      });
      shape = new THREE.Mesh(geometry, materials);
    } else shape = new THREE.Mesh(geometry, base);
  } else if (visual.kind === 'box') {
    shape = new THREE.Mesh(new THREE.BoxGeometry(...visual.size.split(/\s+/).map(Number)), base);
  } else if (visual.kind === 'sphere') {
    shape = new THREE.Mesh(new THREE.SphereGeometry(Number(visual.radius), 48, 32), base);
  } else if (visual.kind === 'cylinder') {
    const geometry = new THREE.CylinderGeometry(Number(visual.radius), Number(visual.radius), Number(visual.length), 48);
    geometry.rotateX(Math.PI / 2); shape = new THREE.Mesh(geometry, base);
  }
  if (!shape) return null;
  if (visual.scale) shape.scale.set(...visual.scale.split(/\s+/).map(Number));
  shape.position.set(...visual.xyz);
  shape.quaternion.setFromEuler(new THREE.Euler(...visual.rpy, 'ZYX'));
  shape.traverse(mesh => {
    if (mesh.isMesh) { mesh.castShadow = true; mesh.receiveShadow = true; }
  });
  return shape;
}

export function registerDisplayModel(links, model) {
  const display = model.visualization || {};
  const mount = display.holder_mount;
  const jigMount = display.jig_mount;
  if (jigMount?.xyz_m?.length === 3 && jigMount.xyz_m.every(Number.isFinite) &&
      jigMount?.quaternion_xyzw?.length === 4 && jigMount.quaternion_xyzw.every(Number.isFinite) &&
      links.has('pen_jig')) {
    const origin = links.get('pen_jig').parent;
    origin.position.set(...jigMount.xyz_m);
    origin.quaternion.set(...jigMount.quaternion_xyzw);
  }
  const surfaceMount = display.flat_mount || display.sphere_mount;
  if (surfaceMount && links.has('pen_surface')) {
    const origin = links.get('pen_surface').parent;
    origin.position.set(...surfaceMount.xyz_m);
    origin.quaternion.set(...surfaceMount.quaternion_xyzw);
  }
  if (mount && links.has('pen_holder')) {
    const origin = links.get('pen_holder').parent;
    origin.position.set(...mount.xyz_m);
    origin.quaternion.set(...mount.quaternion_xyzw);

  }
}
