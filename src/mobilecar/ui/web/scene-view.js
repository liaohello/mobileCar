import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"];

function quaternionFromXyzw(values) {
  const [x = 0, y = 0, z = 0, w = 1] = values || [];
  return new THREE.Quaternion(x, y, z, w).normalize();
}

function materialFor(rgba, { ghost = false, mesh = false } = {}) {
  const color = new THREE.Color(rgba?.[0] ?? 0.65, rgba?.[1] ?? 0.68, rgba?.[2] ?? 0.72);
  return new THREE.MeshStandardMaterial({
    color,
    transparent: ghost,
    opacity: ghost ? 0.26 : (rgba?.[3] ?? 1),
    roughness: ghost ? 0.8 : (mesh ? 0.55 : 0.68),
    metalness: ghost ? 0 : (mesh ? 0.18 : 0.04),
    depthWrite: !ghost,
    side: THREE.DoubleSide,
  });
}

function widgetMaterial(color, opacity = 1) {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    depthTest: false,
  });
}

export class SceneView {
  constructor(container, { onTargetDrag = () => {}, onNotice = () => {} } = {}) {
    this.container = container;
    this.onTargetDrag = onTargetDrag;
    this.onNotice = onNotice;
    this.manifest = null;
    this.snapshot = null;
    this.worldGroups = new Map();
    this.linkGroups = new Map();
    this.ghostGroups = new Map();
    this.meshLoader = new STLLoader();
    this.dragPlane = new THREE.Plane();
    this.raycaster = new THREE.Raycaster();
    this.raycaster.params.Line.threshold = 0.002;
    this.pointer = new THREE.Vector2();
    this.draggingTarget = false;
    this.planObject = null;
    this.disposables = [];
    this.renderCount = 0;
    this.lastRenderAt = 0;
    this.lastFrameCostMs = null;
    this.frameRequest = null;
    this.renderTimer = null;
    this.needsRender = true;
    this.interacting = false;
    this.adaptiveRendering = true;
    this.preferredPixelRatio = 1.0;
    this.maxPixelRatio = 1.5;
    this.staticRenderIntervalMs = 100;
    this.shadowMapSize = 512;
    this.frameCosts = [];

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1118);
    this.scene.fog = new THREE.Fog(0x0b1118, 2.2, 5.5);

    this.camera = new THREE.PerspectiveCamera(46, 1, 0.01, 30);
    this.camera.position.set(1.25, -1.35, 1.0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(
      Math.min(window.devicePixelRatio, this.preferredPixelRatio),
    );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.075;
    this.controls.minDistance = 0.18;
    this.controls.maxDistance = 5.5;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.target.set(0.08, 0, 0.12);
    this.controls.addEventListener("start", () => {
      this.interacting = true;
      this._requestRender(true);
    });
    this.controls.addEventListener("end", () => {
      this.interacting = false;
      this._requestRender();
    });
    this.controls.addEventListener("change", () => this._requestRender(true));

    // MuJoCo is Z-up; Three.js is Y-up. Keep all model-derived objects under
    // one transform so geometry, TCP and target coordinates cannot diverge.
    this.mujocoRoot = new THREE.Group();
    this.mujocoRoot.rotation.x = -Math.PI / 2;
    this.scene.add(this.mujocoRoot);
    this.worldRoot = new THREE.Group();
    this.actualRoot = new THREE.Group();
    this.ghostRoot = new THREE.Group();
    this.mujocoRoot.add(this.worldRoot, this.actualRoot, this.ghostRoot);
    this._addEnvironment();
    this._addTargetWidget();
    this._addActualTcpWidget();
    this._bindPointer();

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this._requestRender(true);
  }

  _addEnvironment() {
    const hemi = new THREE.HemisphereLight(0xbcd9ff, 0x19212c, 2.1);
    this.scene.add(hemi);
    this.keyLight = new THREE.DirectionalLight(0xffffff, 3.0);
    this.keyLight.position.set(1.4, -0.7, 2.2);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.set(this.shadowMapSize, this.shadowMapSize);
    this.keyLight.shadow.camera.near = 0.05;
    this.keyLight.shadow.camera.far = 6;
    this.scene.add(this.keyLight);
    const rim = new THREE.DirectionalLight(0x5de4ff, 1.4);
    rim.position.set(-1.5, 1.2, 1.3);
    this.scene.add(rim);

    const grid = new THREE.GridHelper(4, 40, 0x29445a, 0x172733);
    grid.position.y = -0.001;
    grid.material.transparent = true;
    grid.material.opacity = 0.68;
    this.scene.add(grid);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(5, 5),
      new THREE.MeshStandardMaterial({
        color: 0x101b24,
        roughness: 0.95,
        metalness: 0.02,
        transparent: true,
        opacity: 0.82,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.003;
    floor.receiveShadow = true;
    this.scene.add(floor);
  }

  _addTargetWidget() {
    this.targetGroup = new THREE.Group();
    const core = new THREE.Mesh(new THREE.SphereGeometry(0.012, 20, 14), widgetMaterial(0xffc857));
    core.userData.targetHandle = true;
    this.targetCore = core;
    const halo = new THREE.Mesh(
      new THREE.TorusGeometry(0.026, 0.002, 10, 32),
      widgetMaterial(0xffc857, 0.78),
    );
    halo.rotation.x = Math.PI / 2;
    const cross = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.038, 0.038, 0.038)),
      new THREE.LineBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.48, depthTest: false }),
    );
    this.targetGroup.add(core, halo, cross);
    this.targetGroup.visible = false;
    this.mujocoRoot.add(this.targetGroup);
  }

  _addActualTcpWidget() {
    this.actualTcp = new THREE.Mesh(
      new THREE.SphereGeometry(0.0075, 16, 12),
      widgetMaterial(0x40e0a3),
    );
    this.actualTcp.visible = false;
    this.mujocoRoot.add(this.actualTcp);
  }

  _bindPointer() {
    const canvas = this.renderer.domElement;
    canvas.addEventListener("pointerdown", (event) => {
      if (!this.targetGroup.visible) return;
      this._setPointer(event);
      this.raycaster.setFromCamera(this.pointer, this.camera);
      // Only the compact center handle initiates dragging. Raycasting the
      // whole target group makes the wide line/torus widgets steal orbit input.
      const hits = this.raycaster.intersectObject(this.targetCore, false);
      if (!hits.length) return;
      this.draggingTarget = true;
      this.controls.enabled = false;
      const normal = this.camera.getWorldDirection(new THREE.Vector3()).negate();
      const targetWorld = this.targetGroup.getWorldPosition(new THREE.Vector3());
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, targetWorld);
      canvas.setPointerCapture(event.pointerId);
      event.preventDefault();
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!this.draggingTarget) return;
      this._setPointer(event);
      this.raycaster.setFromCamera(this.pointer, this.camera);
      const hit = new THREE.Vector3();
      if (!this.raycaster.ray.intersectPlane(this.dragPlane, hit)) return;
      const localHit = this.mujocoRoot.worldToLocal(hit.clone());
      this.targetGroup.position.copy(localHit);
      this.onTargetDrag([localHit.x, localHit.y, localHit.z]);
      this._requestRender(true);
    });
    const finish = (event) => {
      if (!this.draggingTarget) return;
      this.draggingTarget = false;
      this.controls.enabled = true;
      if (event?.pointerId !== undefined && canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
    };
    canvas.addEventListener("pointerup", finish);
    canvas.addEventListener("pointercancel", finish);
  }

  _setPointer(event) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  }

  _clearGroup(root, map) {
    for (const group of map.values()) root.remove(group);
    map.clear();
    while (root.children.length) {
      const child = root.children.pop();
      child.traverse?.((node) => {
        if (node.geometry) node.geometry.dispose();
        if (Array.isArray(node.material)) node.material.forEach((m) => m.dispose());
        else node.material?.dispose();
      });
    }
  }

  clearManifest() {
    this._clearGroup(this.worldRoot, this.worldGroups);
    this._clearGroup(this.actualRoot, this.linkGroups);
    this._clearGroup(this.ghostRoot, this.ghostGroups);
    this._clearPlan();
  }

  setManifest(manifest) {
    this.manifest = manifest;
    this.clearManifest();
    for (const link of manifest.world_links || []) {
      const group = this._buildLink(link, { ghost: false, staticWorld: true });
      this.worldRoot.add(group);
      this.worldGroups.set(link.name, { group, link });
    }
    this._buildRobotTree(this.actualRoot, this.linkGroups, { ghost: false });
    this._buildRobotTree(this.ghostRoot, this.ghostGroups, { ghost: true });
    this.ghostRoot.visible = false;
    this.fitView();
  }

  _buildRobotTree(root, map, options) {
    const links = this.manifest?.links || [];
    const byName = new Map(links.map((link) => [link.name, link]));
    const children = new Map();
    for (const link of links) {
      const key = link.parent_name && byName.has(link.parent_name) ? link.parent_name : "__root__";
      if (!children.has(key)) children.set(key, []);
      children.get(key).push(link);
    }
    const addChildren = (parentName, parentGroup) => {
      for (const link of children.get(parentName) || []) {
        const group = this._buildLink(link, options);
        parentGroup.add(group);
        map.set(link.name, { group, link });
        addChildren(link.name, group);
      }
    };
    addChildren("__root__", root);
  }

  _buildLink(link, { ghost = false, staticWorld = false } = {}) {
    const group = new THREE.Group();
    group.name = link.name;
    group.userData.sceneLink = link;
    const transform = link.local_transform || {};
    if (!staticWorld || link.joint_type === "fixed") {
      group.position.fromArray(transform.position_m || [0, 0, 0]);
      group.quaternion.copy(quaternionFromXyzw(transform.orientation_xyzw));
    }
    for (const geom of link.geoms || []) {
      const node = this._buildGeom(geom, ghost);
      if (node) group.add(node);
    }
    return group;
  }

  _buildGeom(geom, ghost) {
    const transform = geom.local_transform || {};
    const parent = new THREE.Group();
    parent.position.fromArray(transform.position_m || [0, 0, 0]);
    parent.quaternion.copy(quaternionFromXyzw(transform.orientation_xyzw));
    const size = geom.size_m || [];
    let mesh = null;
    if (geom.geom_type === "box") {
      mesh = new THREE.Mesh(
        new THREE.BoxGeometry(2 * (size[0] ?? 0.01), 2 * (size[1] ?? 0.01), 2 * (size[2] ?? 0.01)),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
    } else if (geom.geom_type === "sphere") {
      mesh = new THREE.Mesh(
        new THREE.SphereGeometry(size[0] ?? 0.01, 24, 16),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
    } else if (geom.geom_type === "ellipsoid") {
      mesh = new THREE.Mesh(
        new THREE.SphereGeometry(1, 24, 16),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
      mesh.scale.set(size[0] ?? 0.01, size[1] ?? 0.01, size[2] ?? 0.01);
    } else if (geom.geom_type === "cylinder") {
      mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(size[0] ?? 0.01, size[0] ?? 0.01, 2 * (size[1] ?? 0.01), 24),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
      mesh.rotation.x = Math.PI / 2;
    } else if (geom.geom_type === "capsule") {
      mesh = new THREE.Mesh(
        new THREE.CapsuleGeometry(size[0] ?? 0.01, 2 * (size[1] ?? 0.01), 6, 16),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
      mesh.rotation.x = Math.PI / 2;
    } else if (geom.geom_type === "plane") {
      mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(4, 4),
        materialFor(geom.rgba, { ghost, mesh: true }),
      );
      // MuJoCo planes lie in local XY; mujocoRoot supplies the global Z-up
      // to Y-up conversion. No additional rotation belongs here.
    } else if (geom.geom_type === "mesh") {
      mesh = this._loadMesh(geom, ghost);
    }
    if (!mesh) return null;
    mesh.castShadow = !ghost;
    mesh.receiveShadow = !ghost;
    mesh.userData.geom = geom;
    parent.add(mesh);
    return parent;
  }

  _loadMesh(geom, ghost) {
    const asset = (this.manifest?.mesh_assets || []).find((item) => item.asset_id === geom.mesh_asset_id);
    if (!asset) return null;
    const material = materialFor(geom.rgba, { ghost, mesh: true });
    const fallback = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.025, 0.025), material);
    fallback.userData.meshPending = true;
    this.meshLoader.load(
      asset.uri,
      (geometry) => {
        geometry.computeVertexNormals();
        if (geom.size_m?.length === 3) geometry.scale(...geom.size_m);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = !ghost;
        mesh.receiveShadow = !ghost;
        mesh.userData.geom = geom;
        fallback.parent?.add(mesh);
        fallback.visible = false;
        this._scheduleFit();
        this._requestRender(true);
      },
      undefined,
      () => {
        fallback.visible = false;
        this.onNotice(`网格加载失败：${asset.asset_id}`);
        this._requestRender(true);
      },
    );
    return fallback;
  }

  _applyJoint(group, jointName, modelRad) {
    if (!group) return;
    const link = group.userData.sceneLink;
    if (!link || link.joint_type !== "hinge") return;
    if (!JOINT_NAMES.includes(jointName) && jointName !== "gripper_base_joint") return;
    const base = quaternionFromXyzw(link.local_transform?.orientation_xyzw);
    const axis = new THREE.Vector3(...(link.joint_axis || [0, 0, 1])).normalize();
    group.quaternion.copy(base).multiply(new THREE.Quaternion().setFromAxisAngle(axis, modelRad));
  }

  updateSnapshot(snapshot) {
    if (!snapshot || !this.manifest) return;
    this.snapshot = snapshot;
    const baseGroup = this.linkGroups.get("base_link")?.group;
    if (baseGroup && snapshot.base) {
      baseGroup.position.fromArray(snapshot.base.position_world_sim_m);
      baseGroup.quaternion.copy(quaternionFromXyzw(snapshot.base.orientation_world_sim_xyzw));
    }
    this._applyJointValues(snapshot.arm?.model_joint_positions_rad || [], snapshot.gripper?.model_drive_position_rad);
    if (snapshot.tcp?.position_m) {
      this.actualTcp.position.fromArray(snapshot.tcp.position_m);
      this.actualTcp.visible = true;
    }
    const objectPose = snapshot.diagnostics?.mode === "GT_DEBUG"
      ? snapshot.diagnostics?.object_pose_world_sim
      : null;
    const objectGroup = this.worldGroups.get("target_cube")?.group;
    if (objectPose && objectGroup) {
      objectGroup.position.fromArray(objectPose.position_m || [0, 0, 0]);
      objectGroup.quaternion.copy(quaternionFromXyzw(objectPose.orientation_xyzw));
    }
    this._requestRender();
  }

  _applyJointValues(values, gripperRad, ghost = false) {
    const map = ghost ? this.ghostGroups : this.linkGroups;
    for (const [name, entry] of map) {
      const jointIndex = JOINT_NAMES.indexOf(entry.link.joint_name);
      if (jointIndex >= 0) {
        this._applyJoint(entry.group, entry.link.joint_name, values[jointIndex] ?? 0);
      } else if (entry.link.joint_name === "gripper_base_joint" && Number.isFinite(gripperRad)) {
        this._applyJoint(entry.group, entry.link.joint_name, gripperRad);
      }
    }
  }

  showPlan(plan) {
    this._clearPlan();
    if (!plan) {
      this.ghostRoot.visible = false;
      return;
    }
    const endpoint = plan.waypoints_model_rad?.at(-1);
    if (endpoint?.length) {
      this.ghostRoot.visible = true;
      const baseGroup = this.ghostGroups.get("base_link")?.group;
      const actualBase = this.linkGroups.get("base_link")?.group;
      if (baseGroup && actualBase) {
        baseGroup.position.copy(actualBase.position);
        baseGroup.quaternion.copy(actualBase.quaternion);
      }
      this._applyJointValues(endpoint, this.snapshot?.gripper?.model_drive_position_rad, true);
    }
    if (plan.position_m) {
      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...(this.snapshot?.tcp?.position_m || plan.position_m)),
        new THREE.Vector3(...plan.position_m),
      ]);
      const line = new THREE.Line(
        geometry,
        new THREE.LineDashedMaterial({
          color: 0x58d8ff,
          dashSize: 0.012,
          gapSize: 0.008,
          transparent: true,
          opacity: 0.8,
        }),
      );
      line.computeLineDistances();
      this.planObject = line;
      this.mujocoRoot.add(line);
    }
    this._requestRender(true);
  }

  _clearPlan() {
    this.ghostRoot.visible = false;
    if (this.planObject) {
      this.mujocoRoot.remove(this.planObject);
      this.planObject.geometry?.dispose();
      this.planObject.material?.dispose();
      this.planObject = null;
    }
  }

  setTarget(position, visible = true) {
    this.targetGroup.position.fromArray(position);
    this.targetGroup.visible = visible;
    this._requestRender(true);
  }

  fitView() {
    const box = new THREE.Box3();
    if (this.actualRoot.children.length) box.expandByObject(this.actualRoot);
    if (!box.isEmpty()) {
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const distance = Math.max(size.length() * 1.25, 0.65);
      this.controls.target.copy(center).add(new THREE.Vector3(0.03, 0, 0.03));
      this.camera.position.copy(center).add(new THREE.Vector3(distance * 0.8, -distance, distance * 0.72));
      this.camera.near = Math.max(0.005, distance / 200);
      this.camera.far = distance * 20;
      this.camera.updateProjectionMatrix();
    }
    this.controls.update();
    this._requestRender(true);
  }

  _scheduleFit() {
    window.clearTimeout(this.fitTimer);
    this.fitTimer = window.setTimeout(() => this.fitView(), 120);
  }

  resize() {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this._requestRender(true);
  }

  configureRendering(options = {}) {
    this.adaptiveRendering = options.adaptive_rendering !== false;
    this.preferredPixelRatio = Math.max(
      0.5,
      Number(options.preferred_pixel_ratio ?? this.preferredPixelRatio),
    );
    this.maxPixelRatio = Math.max(
      this.preferredPixelRatio,
      Number(options.max_pixel_ratio ?? this.maxPixelRatio),
    );
    this.staticRenderIntervalMs =
      Math.max(16, Number(options.static_render_interval_s ?? 0.1) * 1000);
    const shadowSize = Math.max(128, Number(options.shadow_map_size ?? 512));
    if (shadowSize !== this.shadowMapSize) {
      this.shadowMapSize = shadowSize;
      this.keyLight.shadow.mapSize.set(shadowSize, shadowSize);
      this.keyLight.shadow.map?.dispose();
      this.keyLight.shadow.map = null;
    }
    this.renderer.setPixelRatio(
      Math.min(window.devicePixelRatio, this.maxPixelRatio),
    );
    this._requestRender(true);
  }

  _requestRender(immediate = false) {
    this.needsRender = true;
    if (immediate && this.renderTimer !== null) {
      window.clearTimeout(this.renderTimer);
      this.renderTimer = null;
    }
    if (this.frameRequest !== null || this.renderTimer !== null) return;
    const delay =
      immediate || this.interacting
        ? 0
        : Math.max(0, this.staticRenderIntervalMs - (performance.now() - this.lastRenderAt));
    if (delay > 1) {
      this.renderTimer = window.setTimeout(() => {
        this.renderTimer = null;
        this._scheduleFrame();
      }, delay);
    } else {
      this._scheduleFrame();
    }
  }

  _scheduleFrame() {
    if (this.frameRequest !== null) return;
    this.frameRequest = requestAnimationFrame(() => this._renderFrame());
  }

  _renderFrame() {
    this.frameRequest = null;
    const controlsChanged = this.controls.update();
    const started = performance.now();
    this.renderer.render(this.scene, this.camera);
    this.lastFrameCostMs = performance.now() - started;
    this.lastRenderAt = performance.now();
    this.renderCount += 1;
    this.needsRender = false;
    this._adaptQuality(this.lastFrameCostMs);
    if (this.interacting || controlsChanged) this._scheduleFrame();
  }

  _adaptQuality(frameCostMs) {
    if (!this.adaptiveRendering) return;
    this.frameCosts.push(frameCostMs);
    if (this.frameCosts.length < 12) return;
    const sorted = [...this.frameCosts].sort((a, b) => a - b);
    const p90 = sorted[Math.floor((sorted.length - 1) * 0.9)];
    this.frameCosts = [];
    const currentRatio = this.renderer.getPixelRatio();
    if (p90 > 24 && currentRatio > this.preferredPixelRatio) {
      this.renderer.setPixelRatio(this.preferredPixelRatio);
      this.onNotice(`渲染负载较高，像素比降至 ${this.preferredPixelRatio.toFixed(2)}。`);
    } else if (p90 > 36 && this.renderer.shadowMap.enabled) {
      this.renderer.shadowMap.enabled = false;
      this.scene.traverse((node) => {
        if (node.material) node.material.needsUpdate = true;
      });
      this.onNotice("渲染负载较高，已关闭实时阴影。");
    }
  }

  dispose() {
    if (this.frameRequest !== null) cancelAnimationFrame(this.frameRequest);
    if (this.renderTimer !== null) window.clearTimeout(this.renderTimer);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearManifest();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
