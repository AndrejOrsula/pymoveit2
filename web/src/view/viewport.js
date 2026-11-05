// The 3D scene: the robot at its current state, a translucent "ghost" for a
// goal or a planned trajectory, the planned end-effector path, the planner's
// search tree, planning-scene objects, the collision shapes the planner
// checks, and an interactive goal marker. The world is the planning frame:
// z up, meters. Sizes scale with the robot's `view.scale`.

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

import { linkTransforms } from '../core/kinematics.js';
import { resolvedObject } from '../core/collision.js';

const CREASE = THREE.MathUtils.degToRad(32);

/** Colors that follow RViz's planning-scene conventions, per scheme. */
const PALETTE = {
  light: { object: 0x3f9b57, attached: 0x8b5cc7, allowed: 0x7a8aa6, contact: 0xd9363e, body: 0xf3f4f6, dark: 0x2b2d31, grey: 0x9aa1aa, grid: 0xc9ccd3, grid2: 0xe3e5ea, sky: 0xffffff, ground: 0xb8bcc6 },
  dark: { object: 0x56c271, attached: 0xb38cf0, allowed: 0x8b9bb8, contact: 0xff5b61, body: 0xe9eaed, dark: 0x4a4e56, grey: 0x6b717a, grid: 0x4a4d55, grid2: 0x34363c, sky: 0xd9def0, ground: 0x2a2c33 },
};

function cssColor(name) {
  return new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888');
}

/** Primitive geometry in shape_msgs/SolidPrimitive conventions (z-axis shapes). */
function primitiveGeometry(object, suzanne) {
  const d = object.dimensions;
  switch (object.type) {
    case 'box':
      return new THREE.BoxGeometry(d[0], d[1], d[2]);
    case 'sphere':
      return new THREE.SphereGeometry(d[0], 32, 20);
    case 'cylinder': {
      const geometry = new THREE.CylinderGeometry(d[1], d[1], d[0], 40);
      geometry.rotateX(Math.PI / 2);
      return geometry;
    }
    case 'cone': {
      const geometry = new THREE.ConeGeometry(d[1], d[0], 40);
      geometry.rotateX(Math.PI / 2);
      return geometry;
    }
    case 'mesh': {
      const geometry = suzanne ? suzanne.clone() : new THREE.IcosahedronGeometry(0.1, 1);
      const s = object.scale ?? [1, 1, 1];
      geometry.scale(s[0], s[1], s[2]);
      return geometry;
    }
    default:
      return new THREE.BoxGeometry(0.05, 0.05, 0.05);
  }
}

function poseToObject3D(target, pose) {
  target.position.set(pose.position[0], pose.position[1], pose.position[2]);
  target.quaternion.set(pose.quat_xyzw[0], pose.quat_xyzw[1], pose.quat_xyzw[2], pose.quat_xyzw[3]);
}

export class Viewport extends EventTarget {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {HTMLElement} container  Sized element around the canvas; labels go here.
   * @param {{model: import('../core/types.js').RobotModel, assetBase?: string}} options
   */
  constructor(canvas, container, { model, assetBase = 'assets/' }) {
    super();
    this.canvas = canvas;
    this.container = container;
    this.model = model;
    this.assetBase = assetBase;
    // Frames the whole reach, up to the arm pointing straight up.
    this.home = { target: new THREE.Vector3(...model.view.target), camera: new THREE.Vector3(...model.view.camera) };
    this.size = model.view.scale;
    this.layers = { goal: true, path: true, tree: false, spheres: false };
    this.scheme = window.demoTheme ? window.demoTheme() : 'dark';
    this.state = { q: model.named_states[model.group_name][model.default_state].slice(), fingers: model.gripper.open_joint_positions[0] };
    this.ghostState = null;
    this.objects = new Map();
    this.contacts = new Set();
    this.selectedId = null;
    this.goalEnabled = false;
    this.disposed = false;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.NeutralToneMapping;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.02 * this.size, 30 * this.size);
    this.camera.up.set(0, 0, 1);
    this.camera.position.copy(this.home.camera);

    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.55;
    pmrem.dispose();

    this.hemi = new THREE.HemisphereLight(0xffffff, 0x888888, 1.1);
    this.hemi.position.set(0, 0, 1);
    this.scene.add(this.hemi);
    this.key = new THREE.DirectionalLight(0xffffff, 2.2);
    this.key.position.set(1.2, -0.9, 2.2);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(2048, 2048);
    this.key.shadow.camera.left = -1.2 * this.size;
    this.key.shadow.camera.right = 1.2 * this.size;
    this.key.shadow.camera.top = 1.2 * this.size;
    this.key.shadow.camera.bottom = -1.2 * this.size;
    this.key.shadow.camera.near = 0.5;
    this.key.shadow.camera.far = 5;
    this.key.shadow.bias = -0.0004;
    this.key.shadow.normalBias = 0.01;
    this.scene.add(this.key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.6);
    fill.position.set(-1.5, 1.2, 0.8);
    this.scene.add(fill);

    this.#buildFloor();

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.copy(this.home.target);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.minDistance = 0.4 * this.size;
    this.controls.maxDistance = 6 * this.size;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.update();

    this.robot = this.#makeRobotGroup();
    this.scene.add(this.robot.root);
    this.ghost = null;

    this.pathLine = this.#makeLine(0.9);
    this.executedLine = this.#makeLine(0.5);
    this.treeLines = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ transparent: true, opacity: 0.35, depthWrite: false }),
    );
    this.treeLines.visible = false;
    this.scene.add(this.pathLine, this.executedLine, this.treeLines);

    this.sphereMesh = null;
    this.objectRoot = new THREE.Group();
    this.scene.add(this.objectRoot);

    this.#buildGoalMarker();
    this.#buildPointer();
    this.#buildTouch();

    this.label = document.createElement('div');
    this.label.className = 'scene-label';
    this.label.hidden = true;
    container.appendChild(this.label);

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();

    this.onTheme = () => this.applyTheme();
    window.addEventListener('demo-theme', this.onTheme);
    this.applyTheme();

    canvas.addEventListener('webglcontextlost', (event) => {
      event.preventDefault();
      this.dispatchEvent(new CustomEvent('contextlost'));
    });
  }

  /** Whether this browser can draw the scene at all. */
  static supported() {
    try {
      const canvas = document.createElement('canvas');
      return Boolean(canvas.getContext('webgl2') || canvas.getContext('webgl'));
    } catch {
      return false;
    }
  }

  #buildFloor() {
    this.floor = new THREE.Group();
    this.floor.position.z = this.model.view.ground;
    const shadow = new THREE.Mesh(new THREE.CircleGeometry(1.6 * this.size, 64), new THREE.ShadowMaterial({ opacity: 0.22 }));
    shadow.receiveShadow = true;
    shadow.position.z = -0.0005;
    this.floor.add(shadow);
    this.grid = new THREE.GridHelper(2.4 * this.size, 24);
    this.grid.rotateX(Math.PI / 2);
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.55;
    this.grid.material.depthWrite = false;
    this.floor.add(this.grid);
    // The planning frame's axes at the base: x red, y green, z blue.
    const axes = new THREE.AxesHelper(0.12 * this.size);
    axes.position.z = 0.001 - this.model.view.ground;
    axes.material.depthTest = false;
    axes.renderOrder = 2;
    this.floor.add(axes);
    this.scene.add(this.floor);
  }

  #makeRobotGroup() {
    const root = new THREE.Group();
    const links = {};
    for (const name of Object.keys(this.model.meshes)) {
      const group = new THREE.Group();
      group.matrixAutoUpdate = false;
      group.name = name;
      links[name] = group;
      root.add(group);
    }
    return { root, links };
  }

  #makeLine(opacity) {
    const line = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ transparent: true, opacity, depthWrite: false }),
    );
    line.visible = false;
    line.frustumCulled = false;
    return line;
  }

  #buildGoalMarker() {
    this.goal = new THREE.Group();
    this.goal.name = 'goal';
    const axes = new THREE.AxesHelper(0.09 * this.size);
    axes.material.depthTest = false;
    axes.renderOrder = 3;
    this.goal.add(axes);
    this.goalBall = new THREE.Mesh(
      new THREE.SphereGeometry(0.014 * Math.sqrt(this.size), 24, 16),
      new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.9, depthTest: false }),
    );
    this.goalBall.renderOrder = 3;
    this.goal.add(this.goalBall);
    this.goal.visible = false;
    this.scene.add(this.goal);

    this.transform = new TransformControls(this.camera, this.canvas);
    this.transform.setSize(0.75);
    this.transform.setSpace('local');
    this.transformHelper = this.transform.getHelper();
    this.scene.add(this.transformHelper);
    this.transform.addEventListener('dragging-changed', (event) => {
      this.controls.enabled = !event.value;
      if (!event.value && !this.cancelTransformCommit) this.#commitTransform();
    });
    this.transform.addEventListener('objectChange', () => {
      const object = this.transform.object;
      if (object === this.goal) {
        this.dispatchEvent(new CustomEvent('goal-change', { detail: this.getGoalPose() }));
      } else if (object) {
        this.dispatchEvent(new CustomEvent('object-change', { detail: { id: object.userData.id, pose: this.#poseOf(object) } }));
      }
    });
  }

  #commitTransform() {
    const object = this.transform.object;
    if (object === this.goal) {
      this.dispatchEvent(new CustomEvent('goal-commit', { detail: this.getGoalPose() }));
    } else if (object?.userData.id) {
      this.dispatchEvent(new CustomEvent('object-moved', { detail: { id: object.userData.id, pose: this.#poseOf(object) } }));
    }
  }

  #poseOf(object) {
    const p = object.position;
    const q = object.quaternion;
    return { position: [p.x, p.y, p.z], quat_xyzw: [q.x, q.y, q.z, q.w] };
  }

  /**
   * Two fingers pinch or pan the view, even when the first one landed on the
   * gizmo: its drag is undone, the orbit controls take both fingers, and the
   * gizmo waits until they lift. Listens on the container, in the capture
   * phase, to act before the controls' own listeners on the canvas.
   */
  #buildTouch() {
    const touches = new Map();
    this.container.addEventListener(
      'pointerdown',
      (event) => {
        if (event.pointerType !== 'touch' || event.target !== this.canvas) return;
        touches.set(event.pointerId, event);
        if (touches.size < 2 || !this.transform.enabled) return;
        if (this.transform.dragging) {
          this.transform.reset();
          this.transform.pointerUp(null);
          // The orbit controls ignored the first finger while the gizmo had it
          // (OrbitControls' bound handler, three 0.186).
          for (const [id, touch] of touches) if (id !== event.pointerId) this.controls._onPointerDown(touch);
        }
        this.transform.enabled = false;
      },
      true,
    );
    this.container.addEventListener(
      'pointermove',
      (event) => {
        if (touches.has(event.pointerId)) touches.set(event.pointerId, event);
      },
      true,
    );
    const lift = (event) => {
      if (!touches.delete(event.pointerId) || touches.size) return;
      this.transform.enabled = true;
    };
    window.addEventListener('pointerup', lift, true);
    window.addEventListener('pointercancel', lift, true);
  }

  #buildPointer() {
    const raycaster = new THREE.Raycaster();
    const ndc = new THREE.Vector2();
    let down = null;
    // Only a tap with one pointer selects: a second finger makes it a pinch.
    const pointers = new Set();
    this.canvas.addEventListener('pointerdown', (event) => {
      pointers.add(event.pointerId);
      down = pointers.size === 1 ? { x: event.clientX, y: event.clientY } : null;
    });
    window.addEventListener('pointercancel', (event) => pointers.delete(event.pointerId), true);
    window.addEventListener('pointerup', (event) => pointers.delete(event.pointerId), true);
    this.canvas.addEventListener('pointerup', (event) => {
      if (!down || this.transform.dragging) return;
      const moved = Math.hypot(event.clientX - down.x, event.clientY - down.y);
      down = null;
      if (moved > 4) return;
      const rect = this.canvas.getBoundingClientRect();
      ndc.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
      raycaster.setFromCamera(ndc, this.camera);
      const pickable = [...this.objects.values()].map((entry) => entry.mesh);
      if (this.goal.visible) pickable.push(this.goalBall);
      const hit = raycaster.intersectObjects(pickable, false)[0];
      if (hit?.object === this.goalBall) {
        this.dispatchEvent(new CustomEvent('select', { detail: { kind: 'goal' } }));
      } else if (hit) {
        this.dispatchEvent(new CustomEvent('select', { detail: { kind: 'object', id: hit.object.userData.id } }));
      } else {
        this.dispatchEvent(new CustomEvent('select', { detail: { kind: 'none' } }));
      }
    });
  }

  /** Loads the robot's meshes; resolves once the robot can be drawn. */
  async load() {
    const loader = new GLTFLoader();
    const names = [...new Set(Object.values(this.model.meshes))];
    const loaded = await Promise.all(
      names.map((name) => loader.loadAsync(`${this.assetBase}${this.model.view.assets}/${name}.glb`).then((gltf) => [name, gltf.scene])),
    );
    const suzanne = await loader.loadAsync(`${this.assetBase}suzanne.glb`).catch(() => null);
    this.suzanneGeometry = null;
    suzanne?.scene.traverse((node) => {
      if (node.isMesh && !this.suzanneGeometry) this.suzanneGeometry = toCreasedNormals(node.geometry, CREASE);
    });

    this.parts = new Map();
    for (const [name, scene] of loaded) {
      const parts = [];
      scene.updateMatrixWorld(true);
      scene.traverse((node) => {
        if (!node.isMesh) return;
        const geometry = toCreasedNormals(node.geometry.clone().applyMatrix4(node.matrixWorld), CREASE);
        parts.push({ material: node.name.split('_')[0] === 'accent' ? node.name : node.name.replace(/_\d+$/, ''), geometry });
      });
      this.parts.set(name, parts);
    }

    this.materials = this.#robotMaterials();
    for (const [link, meshName] of Object.entries(this.model.meshes)) {
      for (const part of this.parts.get(meshName)) {
        const mesh = new THREE.Mesh(part.geometry, this.#materialFor(part.material));
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.link = link;
        this.robot.links[link].add(mesh);
      }
    }
    this.ghost = this.#makeGhost();
    this.scene.add(this.ghost.root);
    this.applyTheme();
    this.setRobotState(this.state.q, this.state.fingers);
  }

  #robotMaterials() {
    return {
      body: new THREE.MeshPhysicalMaterial({ color: 0xf3f4f6, roughness: 0.38, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.25 }),
      dark: new THREE.MeshStandardMaterial({ color: 0x2b2d31, roughness: 0.55, metalness: 0.1 }),
      grey: new THREE.MeshStandardMaterial({ color: 0x9aa1aa, roughness: 0.6, metalness: 0.0 }),
      contact: new THREE.MeshStandardMaterial({ color: 0xd9363e, roughness: 0.5, emissive: 0x5a0c10 }),
      accent: new Map(),
    };
  }

  #materialFor(name) {
    if (name === 'body' || name === 'dark' || name === 'grey') return this.materials[name];
    if (!this.materials.accent.has(name)) {
      const color = new THREE.Color(`#${name.split('_')[1] ?? 'ffffff'}`);
      this.materials.accent.set(name, new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.6, roughness: 0.4 }));
    }
    return this.materials.accent.get(name);
  }

  #makeGhost() {
    const root = new THREE.Group();
    const links = {};
    const material = new THREE.MeshStandardMaterial({ transparent: true, opacity: 0.3, depthWrite: false, roughness: 0.6 });
    // Links no joint moves, such as the base, would only tint the robot's own.
    const low = linkTransforms(this.model, this.model.joints.map((j) => j.lower));
    const high = linkTransforms(this.model, this.model.joints.map((j) => j.upper));
    for (const [link, meshName] of Object.entries(this.model.meshes)) {
      if (low[link].every((v, i) => Math.abs(v - high[link][i]) < 1e-9)) continue;
      const group = new THREE.Group();
      group.matrixAutoUpdate = false;
      for (const part of this.parts.get(meshName)) group.add(new THREE.Mesh(part.geometry, material));
      links[link] = group;
      root.add(group);
    }
    root.visible = false;
    return { root, links, material };
  }

  applyTheme() {
    this.scheme = window.demoTheme ? window.demoTheme() : 'dark';
    const palette = PALETTE[this.scheme] ?? PALETTE.dark;
    this.palette = palette;
    const background = cssColor('--demo-surface');
    this.scene.background = background;
    this.hemi.color.set(palette.sky);
    this.hemi.groundColor.set(palette.ground);
    const gridMaterials = Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material];
    this.grid.geometry.dispose();
    this.floor.remove(this.grid);
    this.grid = new THREE.GridHelper(2.4 * this.size, 24, palette.grid, palette.grid2);
    this.grid.rotateX(Math.PI / 2);
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.7;
    this.grid.material.depthWrite = false;
    this.floor.add(this.grid);
    for (const material of gridMaterials) material.dispose();
    const accent = cssColor('--demo-accent');
    this.accent = accent;
    this.pathLine.material.color.copy(accent);
    this.executedLine.material.color.copy(accent);
    this.treeLines.material.color.set(this.scheme === 'dark' ? 0x9aa4bf : 0x6b7590);
    this.goalBall.material.color.copy(accent);
    if (this.materials) {
      this.materials.body.color.set(palette.body);
      this.materials.dark.color.set(palette.dark);
      this.materials.grey.color.set(palette.grey);
    }
    if (this.ghost) this.#toneGhost(this.ghostTone ?? 'goal');
    for (const entry of this.objects.values()) this.#styleObject(entry);
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  resetView() {
    this.camera.position.copy(this.home.camera);
    this.controls.target.copy(this.home.target);
    this.controls.update();
  }

  /** Shows the robot at arm joint positions q and finger position fingers. */
  setRobotState(q, fingers) {
    this.state = { q: q.slice(), fingers };
    const transforms = linkTransforms(this.model, q, fingers);
    for (const [link, group] of Object.entries(this.robot.links)) {
      const m = transforms[link];
      if (m) group.matrix.fromArray(m);
      group.matrixWorldNeedsUpdate = true;
    }
    this.transforms = transforms;
    for (const entry of this.objects.values()) {
      if (!entry.object.attached) continue;
      poseToObject3D(entry.mesh, resolvedObject(entry.object, transforms).pose);
    }
    if (this.layers.spheres) this.#updateSpheres(transforms);
    if (this.ghost) this.#showGhost();
  }

  /**
   * Shows a translucent copy of the robot, or hides it for null.
   * @param {number[]|null} q
   * @param {number} [fingers]
   * @param {'goal'|'preview'|'bad'} [tone]
   */
  setGhost(q, fingers = this.state.fingers, tone = 'goal') {
    if (!this.ghost) return;
    this.ghostState = q ? { q: q.slice(), fingers } : null;
    if (!q) {
      this.ghost.root.visible = false;
      return;
    }
    const transforms = linkTransforms(this.model, q, fingers);
    for (const [link, group] of Object.entries(this.ghost.links)) {
      group.matrix.fromArray(transforms[link]);
      group.matrixWorldNeedsUpdate = true;
    }
    this.#toneGhost(tone);
    this.#showGhost();
  }

  /** Shows the ghost with the Goal layer, unless it covers the robot, which would only tint it. */
  #showGhost() {
    const ghost = this.ghostState?.q;
    const covers = ghost && ghost.every((v, i) => Math.abs(v - this.state.q[i]) < 1e-3);
    this.ghost.root.visible = this.layers.goal && Boolean(ghost) && !covers;
  }

  #toneGhost(tone) {
    this.ghostTone = tone;
    const color = tone === 'bad' ? new THREE.Color(this.palette.contact) : this.accent ?? new THREE.Color(0x7c91d9);
    this.ghost.material.color.copy(color);
    this.ghost.material.emissive.copy(color).multiplyScalar(0.25);
    this.ghost.material.opacity = tone === 'preview' ? 0.22 : 0.32;
  }

  /** The end-effector path of a plan (kind "plan") or of the motion so far ("executed"). */
  setPath(points, kind = 'plan') {
    const line = kind === 'plan' ? this.pathLine : this.executedLine;
    if (!points || points.length < 2) {
      line.visible = false;
      return;
    }
    line.geometry.dispose();
    line.geometry = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(p[0], p[1], p[2])));
    line.visible = this.layers.path;
  }

  /** The planner's search tree as end-effector segments, or null. */
  setTree(edges) {
    if (!edges || edges.length === 0) {
      this.treeLines.visible = false;
      return;
    }
    const positions = new Float32Array(edges.length * 6);
    edges.forEach(([a, b], i) => positions.set([a[0], a[1], a[2], b[0], b[1], b[2]], i * 6));
    this.treeLines.geometry.dispose();
    this.treeLines.geometry = new THREE.BufferGeometry();
    this.treeLines.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.treeLines.visible = this.layers.tree;
  }

  /** @param {import('../core/types.js').CollisionObject[]} objects */
  setObjects(objects) {
    const seen = new Set();
    for (const object of objects) {
      seen.add(object.id);
      const displayed = this.transforms ? resolvedObject(object, this.transforms) : object;
      let entry = this.objects.get(object.id);
      const shapeKey = `${object.type}:${object.dimensions.join(',')}:${(object.scale ?? []).join(',')}`;
      if (!entry || entry.shapeKey !== shapeKey) {
        if (entry) {
          if (this.transform.object === entry.mesh) this.#detachTransform();
          this.#removeObject(entry);
        }
        const material = new THREE.MeshStandardMaterial({ transparent: true, roughness: 0.55, metalness: 0.0 });
        const mesh = new THREE.Mesh(primitiveGeometry(object, this.suzanneGeometry), material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.userData.id = object.id;
        const edges = new THREE.LineSegments(
          new THREE.EdgesGeometry(mesh.geometry, 30),
          new THREE.LineBasicMaterial({ transparent: true, opacity: 0.35 }),
        );
        mesh.add(edges);
        entry = { mesh, edges, shapeKey, object };
        this.objects.set(object.id, entry);
        this.objectRoot.add(mesh);
      }
      entry.object = object;
      if (object.attached && this.transform.object === entry.mesh) this.#detachTransform();
      if (this.transform.object !== entry.mesh || !this.transform.dragging) poseToObject3D(entry.mesh, displayed.pose);
      if (this.selectedId === object.id) {
        if (!object.attached && this.transform.object !== entry.mesh) this.transform.attach(entry.mesh);
      }
      this.#styleObject(entry);
    }
    for (const [id, entry] of this.objects) {
      if (!seen.has(id)) {
        if (this.transform.object === entry.mesh) this.#detachTransform();
        this.#removeObject(entry);
        this.objects.delete(id);
      }
    }
  }

  #detachTransform() {
    if (this.transform.dragging) {
      this.cancelTransformCommit = true;
      try {
        this.transform.pointerUp(null);
      } finally {
        this.cancelTransformCommit = false;
      }
    }
    this.transform.detach();
  }

  #removeObject(entry) {
    this.objectRoot.remove(entry.mesh);
    entry.mesh.geometry.dispose();
    entry.mesh.material.dispose();
    entry.edges.geometry.dispose();
    entry.edges.material.dispose();
  }

  #styleObject(entry) {
    const palette = this.palette ?? PALETTE.dark;
    const { object } = entry;
    const inContact = this.contacts.has(object.id);
    const color = inContact ? palette.contact : object.attached ? palette.attached : object.allowed ? palette.allowed : palette.object;
    entry.mesh.material.color.set(color);
    entry.mesh.material.opacity = object.allowed ? 0.45 : 0.88;
    entry.mesh.material.emissive.set(inContact ? palette.contact : 0x000000).multiplyScalar(inContact ? 0.35 : 0);
    entry.edges.material.color.set(this.selectedId === object.id ? this.accent ?? 0x7c91d9 : color);
    entry.edges.material.opacity = this.selectedId === object.id ? 1 : 0.35;
  }

  /** Highlights the links and objects in each colliding pair. */
  setContacts(pairs) {
    const next = new Set(pairs.flat());
    const same = next.size === this.contacts.size && [...next].every((name) => this.contacts.has(name));
    if (same) return;
    this.contacts = next;
    if (this.materials) {
      for (const [link, group] of Object.entries(this.robot.links)) {
        const hit = next.has(link);
        for (const mesh of group.children) {
          if (!mesh.userData.original) mesh.userData.original = mesh.material;
          mesh.material = hit ? this.materials.contact : mesh.userData.original;
        }
      }
    }
    for (const entry of this.objects.values()) this.#styleObject(entry);
  }

  /** Shows the goal marker at a pose (of the end effector), or hides it for null. */
  setGoalMarker(pose, { editable = true } = {}) {
    this.goalEnabled = Boolean(pose);
    if (!pose) {
      this.goal.visible = false;
      if (this.transform.object === this.goal) this.transform.detach();
      this.label.hidden = true;
      return;
    }
    if (!this.transform.dragging || this.transform.object !== this.goal) poseToObject3D(this.goal, pose);
    this.goal.visible = this.layers.goal;
    if (editable && this.layers.goal && this.transform.object !== this.goal && this.selectedId === null) {
      this.transform.attach(this.goal);
    }
  }

  getGoalPose() {
    return this.#poseOf(this.goal);
  }

  /** Attaches the gizmo to the goal ("goal"), an object id, or nothing (null). */
  select(target) {
    this.selectedId = target && target !== 'goal' ? target : null;
    if (target === 'goal' && this.goal.visible) this.transform.attach(this.goal);
    else if (this.selectedId && this.objects.has(this.selectedId)) {
      const entry = this.objects.get(this.selectedId);
      if (entry.object.attached) this.transform.detach();
      else this.transform.attach(entry.mesh);
    } else this.transform.detach();
    for (const entry of this.objects.values()) this.#styleObject(entry);
  }

  setGizmoMode(mode) {
    this.transform.setMode(mode);
  }

  get gizmoMode() {
    return this.transform.mode;
  }

  setLayer(name, visible) {
    this.layers[name] = visible;
    if (name === 'goal') {
      this.goal.visible = visible && this.goalEnabled;
      this.transformHelper.visible = visible || this.selectedId !== null;
      if (this.ghost) this.#showGhost();
    }
    if (name === 'path') {
      this.pathLine.visible = visible && this.pathLine.geometry.attributes.position?.count > 1;
      this.executedLine.visible = visible && this.executedLine.geometry.attributes.position?.count > 1;
    }
    if (name === 'tree') this.treeLines.visible = visible && this.treeLines.geometry.attributes.position?.count > 1;
    if (name === 'spheres') {
      if (visible && this.transforms) this.#updateSpheres(this.transforms);
      if (this.sphereMesh) this.sphereMesh.visible = visible;
      if (this.boxMeshes) this.boxMeshes.visible = visible;
    }
  }

  #updateSpheres(transforms) {
    const entries = Object.entries(this.model.collision_spheres);
    const count = entries.reduce((n, [, spheres]) => n + spheres.length, 0);
    if (!this.sphereMesh) {
      const material = new THREE.MeshBasicMaterial({ color: 0xff9f43, transparent: true, opacity: 0.18, depthWrite: false });
      this.sphereMesh = new THREE.InstancedMesh(new THREE.SphereGeometry(1, 16, 10), material, count);
      this.sphereMesh.frustumCulled = false;
      this.scene.add(this.sphereMesh);
    }
    const m = new THREE.Matrix4();
    const link = new THREE.Matrix4();
    const v = new THREE.Vector3();
    let i = 0;
    for (const [name, spheres] of entries) {
      link.fromArray(transforms[name]);
      for (const [x, y, z, r] of spheres) {
        v.set(x, y, z).applyMatrix4(link);
        m.makeScale(r, r, r).setPosition(v);
        this.sphereMesh.setMatrixAt(i++, m);
      }
    }
    this.sphereMesh.instanceMatrix.needsUpdate = true;
    this.sphereMesh.visible = this.layers.spheres;
    // Boxes, such as a floor slab, are fixed: place them once, as outlines,
    // since a filled slab would tint the whole floor.
    if (!this.boxMeshes) {
      this.boxMeshes = new THREE.Group();
      const outline = new THREE.LineBasicMaterial({ color: this.sphereMesh.material.color });
      for (const [name, boxes] of Object.entries(this.model.collision_boxes ?? {})) {
        for (const { xyz, size } of boxes) {
          const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(...size)), outline);
          box.matrixAutoUpdate = false;
          box.matrix.fromArray(transforms[name]).multiply(new THREE.Matrix4().makeTranslation(...xyz));
          this.boxMeshes.add(box);
        }
      }
      this.scene.add(this.boxMeshes);
    }
    this.boxMeshes.visible = this.layers.spheres;
  }

  /** A short label above the goal marker, or none for null. */
  setGoalLabel(text, tone = 'ok') {
    this.labelText = text;
    this.label.dataset.tone = tone;
    if (text) this.label.textContent = text;
    this.label.hidden = !text;
  }

  #placeLabel() {
    if (this.label.hidden || !this.goal.visible) {
      if (!this.label.hidden && !this.goal.visible) this.label.style.visibility = 'hidden';
      return;
    }
    const v = this.goal.getWorldPosition(new THREE.Vector3()).project(this.camera);
    const x = (v.x * 0.5 + 0.5) * this.container.clientWidth;
    const y = (-v.y * 0.5 + 0.5) * this.container.clientHeight;
    this.label.style.visibility = v.z < 1 ? 'visible' : 'hidden';
    this.label.style.transform = `translate(${x}px, ${y}px) translate(-50%, -170%)`;
  }

  /** Draws one frame. */
  render() {
    if (this.disposed) return;
    this.controls.update();
    this.#placeLabel();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    this.disposed = true;
    window.removeEventListener('demo-theme', this.onTheme);
    this.resizeObserver.disconnect();
    this.transform.dispose();
    this.controls.dispose();
    this.renderer.dispose();
  }
}
