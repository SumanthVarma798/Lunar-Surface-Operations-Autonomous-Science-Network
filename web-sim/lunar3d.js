/* ═══════════════════════════════════════════════════════
   LSOAS — 3D Visualization System
   Two camera modes + orbital physics + collision handling
   ═══════════════════════════════════════════════════════ */

class VisualizationController {
  constructor(bus, containerId) {
    this.bus = bus;
    this.container = document.getElementById(containerId);
    this.width = this.container.clientWidth;
    this.height = this.container.clientHeight;

    this.MOON_RADIUS = 1.0;
    this.ROVER_SURFACE_OFFSET = 0.01;
    this.SATELLITE_COUNT = 3;
    this.GRAVITY_MU = 1.22;
    this.DEFAULT_RESTITUTION = 0.68;

    this.scene = null;
    this.rootGroup = null;
    this.camera = null;
    this.renderer = null;
    this.moon = null;
    this.stars = null;
    this.earthBase = null;

    this.viewMode = "orbital";
    this.orbitalDistance = 5.35;
    this.astronautPitch = 1.08;
    this.lastFrameTs = performance.now();

    this.dragState = {
      active: false,
      button: 0,
      x: 0,
      y: 0,
    };

    this.worldBodies = {
      dynamic: [],
      static: [],
    };
    this.satellites = [];
    this.interSatelliteLinks = [];
    this.rovers = new Map();
    this.selectedRoverId = null;
    this.collisionCounter = 0;

    this.tmpVecA = new THREE.Vector3();
    this.tmpVecB = new THREE.Vector3();
    this.tmpVecC = new THREE.Vector3();
    this.tmpVecD = new THREE.Vector3();
    this.earthAnchor = new THREE.Vector3(7.2, 2.8, 5.9);

    this.init();
    this.bindBus();
    this.animate();
  }

  init() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x02040c);

    this.rootGroup = new THREE.Group();
    this.scene.add(this.rootGroup);

    this.camera = new THREE.PerspectiveCamera(
      42,
      this.width / this.height,
      0.02,
      100,
    );

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    this.renderer.setSize(this.width, this.height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.container.appendChild(this.renderer.domElement);

    this.createLighting();
    this.createMoon();
    this.createStars();
    this.createEarthBase();
    this.createSatellites();
    this.setupInteraction();
    this.setViewMode("orbital", false);

    window.addEventListener("resize", () => this.onResize());
  }

  bindBus() {
    this.bus.on("earth:telemetry", (data) => this.upsertRoverFromTelemetry(data));
    this.bus.on("fleet:update", (fleet) => this.upsertFleetSnapshot(fleet));
    this.bus.on("earth:selected-rover", (data) => {
      if (data?.rover_id) this.setSelectedRover(data.rover_id);
    });
  }

  createLighting() {
    const ambient = new THREE.AmbientLight(0xffffff, 0.14);
    this.scene.add(ambient);

    const hemi = new THREE.HemisphereLight(0xa8d6ff, 0x060708, 0.52);
    hemi.position.set(0, 6, 0);
    this.scene.add(hemi);

    const sun = new THREE.DirectionalLight(0xf8fcff, 2.25);
    sun.position.set(6.2, 4.4, 5.1);
    this.scene.add(sun);

    const fill = new THREE.PointLight(0x7cc8ff, 0.55, 24, 2);
    fill.position.set(-3.4, 2.1, 2.8);
    this.scene.add(fill);

    const rim = new THREE.PointLight(0x3b82f6, 0.95, 18, 2);
    rim.position.set(-2.8, 2.1, -3.6);
    this.scene.add(rim);
  }

  createMoon() {
    const geometry = new THREE.SphereGeometry(this.MOON_RADIUS, 96, 96);
    const textureLoader = new THREE.TextureLoader();
    const colorMap = textureLoader.load(
      "https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/moon_1024.jpg",
    );
    colorMap.anisotropy = 12;

    const material = new THREE.MeshStandardMaterial({
      map: colorMap,
      bumpMap: colorMap,
      bumpScale: 0.048,
      roughness: 0.92,
      metalness: 0.01,
    });

    this.moon = new THREE.Mesh(geometry, material);
    this.rootGroup.add(this.moon);

    const glow = new THREE.Mesh(
      new THREE.SphereGeometry(this.MOON_RADIUS * 1.045, 64, 64),
      new THREE.MeshBasicMaterial({
        color: 0x5fa8ff,
        transparent: true,
        opacity: 0.065,
        blending: THREE.AdditiveBlending,
        side: THREE.BackSide,
      }),
    );
    this.rootGroup.add(glow);

    this.worldBodies.static.push({
      id: "moon",
      type: "moon",
      radius: this.MOON_RADIUS,
      position: new THREE.Vector3(0, 0, 0),
      mesh: this.moon,
      mass: Infinity,
    });
  }

  createStars() {
    const geometry = new THREE.BufferGeometry();
    const vertices = [];
    const colors = [];

    for (let i = 0; i < 1400; i++) {
      const radius = THREE.MathUtils.randFloat(13, 36);
      const theta = THREE.MathUtils.randFloat(0, Math.PI * 2);
      const phi = THREE.MathUtils.randFloat(0, Math.PI);
      const sinPhi = Math.sin(phi);

      vertices.push(
        radius * sinPhi * Math.cos(theta),
        radius * Math.cos(phi),
        radius * sinPhi * Math.sin(theta),
      );

      const v = THREE.MathUtils.randFloat(0.58, 0.95);
      colors.push(v, v, THREE.MathUtils.randFloat(0.72, 1.0));
    }

    geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(vertices, 3),
    );
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));

    this.stars = new THREE.Points(
      geometry,
      new THREE.PointsMaterial({
        size: 0.04,
        transparent: true,
        opacity: 0.75,
        vertexColors: true,
      }),
    );
    this.scene.add(this.stars);
  }

  createEarthBase() {
    const baseGroup = new THREE.Group();
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 24, 24),
      new THREE.MeshStandardMaterial({
        color: 0x7dd3fc,
        emissive: 0x0b2f40,
        emissiveIntensity: 0.6,
        roughness: 0.4,
        metalness: 0.2,
      }),
    );
    baseGroup.add(core);

    const halo = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 18, 18),
      new THREE.MeshBasicMaterial({
        color: 0x22d3ee,
        transparent: true,
        opacity: 0.24,
        blending: THREE.AdditiveBlending,
      }),
    );
    baseGroup.add(halo);

    baseGroup.position.copy(this.earthAnchor);
    this.rootGroup.add(baseGroup);
    this.earthBase = baseGroup;
  }

  createSatellites() {
    const orbitDefs = [
      { radius: 2.65, tiltX: 0.24, tiltZ: 0.0, phase: 0.0, size: 0.038, color: 0x5dd6ff },
      { radius: 3.25, tiltX: 0.88, tiltZ: 0.51, phase: 2.1, size: 0.042, color: 0xf0abfc },
      { radius: 3.95, tiltX: 0.56, tiltZ: -0.78, phase: 4.35, size: 0.046, color: 0x93c5fd },
    ];

    orbitDefs.forEach((def, index) => {
      const basis = this.computeOrbitBasis(def.tiltX, def.tiltZ);
      const sat = this.buildSatellite(index + 1, def, basis);
      this.satellites.push(sat);
      this.rootGroup.add(sat.group);
      this.rootGroup.add(sat.linkToEarth);
      this.rootGroup.add(sat.linkToRover);
      this.rootGroup.add(sat.orbitPath);
      this.worldBodies.dynamic.push(sat.body);
    });

    this.createInterSatelliteLinks();
  }

  computeOrbitBasis(tiltX, tiltZ) {
    const normal = new THREE.Vector3(0, 1, 0);
    normal.applyAxisAngle(new THREE.Vector3(1, 0, 0), tiltX);
    normal.applyAxisAngle(new THREE.Vector3(0, 0, 1), tiltZ);
    normal.normalize();

    const ref = Math.abs(normal.y) > 0.92
      ? new THREE.Vector3(1, 0, 0)
      : new THREE.Vector3(0, 1, 0);

    const tangentA = new THREE.Vector3().crossVectors(ref, normal).normalize();
    const tangentB = new THREE.Vector3().crossVectors(normal, tangentA).normalize();
    return { normal, tangentA, tangentB };
  }

  buildSatellite(index, def, basis) {
    const speed = Math.sqrt(this.GRAVITY_MU / def.radius);
    const pos = new THREE.Vector3()
      .copy(basis.tangentA)
      .multiplyScalar(Math.cos(def.phase) * def.radius)
      .addScaledVector(basis.tangentB, Math.sin(def.phase) * def.radius);

    const vel = new THREE.Vector3()
      .copy(basis.tangentA)
      .multiplyScalar(-Math.sin(def.phase))
      .addScaledVector(basis.tangentB, Math.cos(def.phase))
      .multiplyScalar(speed);

    const group = new THREE.Group();
    const bus = new THREE.Mesh(
      new THREE.BoxGeometry(def.size * 1.15, def.size * 0.55, def.size * 0.5),
      new THREE.MeshStandardMaterial({
        color: 0xb4bdca,
        metalness: 0.55,
        roughness: 0.29,
        emissive: 0x151f2f,
      }),
    );
    group.add(bus);

    const panelMat = new THREE.MeshStandardMaterial({
      color: def.color,
      emissive: new THREE.Color(def.color).multiplyScalar(0.45),
      metalness: 0.65,
      roughness: 0.35,
    });
    const panelGeo = new THREE.BoxGeometry(def.size * 1.8, def.size * 0.08, def.size * 0.7);
    const panelLeft = new THREE.Mesh(panelGeo, panelMat);
    panelLeft.position.x = -def.size * 1.2;
    const panelRight = panelLeft.clone();
    panelRight.position.x = def.size * 1.2;
    group.add(panelLeft, panelRight);

    const dish = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0, def.size * 0.28, def.size * 0.36, 14, 1),
      new THREE.MeshStandardMaterial({
        color: 0xe5e7eb,
        metalness: 0.5,
        roughness: 0.25,
      }),
    );
    dish.rotation.z = Math.PI / 2;
    dish.position.z = def.size * 0.46;
    group.add(dish);

    group.position.copy(pos);

    const body = {
      id: `sat-${index}`,
      type: "satellite",
      radius: def.size * 0.58,
      mass: 8.6 + index * 1.4,
      restitution: this.DEFAULT_RESTITUTION,
      dynamic: true,
      position: pos.clone(),
      velocity: vel.clone(),
      group,
      collisionCooldown: 0,
    };

    const orbitPath = this.createOrbitPath(def.radius, basis, index, def.color);
    const linkToEarth = this.createBeam({
      color: 0x8be9fd,
      opacity: 0.46,
      thickness: 0.0095,
    });
    const linkToRover = this.createBeam({
      color: 0xfbbf24,
      opacity: 0.42,
      thickness: 0.0075,
    });

    return {
      body,
      group,
      orbitPath,
      linkToEarth,
      linkToRover,
    };
  }

  createOrbitPath(radius, basis, index, orbitColor) {
    const points = [];
    const segments = 180;
    for (let i = 0; i <= segments; i++) {
      const a = (i / segments) * Math.PI * 2;
      const p = new THREE.Vector3()
        .copy(basis.tangentA)
        .multiplyScalar(Math.cos(a) * radius)
        .addScaledVector(basis.tangentB, Math.sin(a) * radius);
      points.push(p);
    }

    const group = new THREE.Group();
    const curve = new THREE.CatmullRomCurve3(points, true);
    const orbitTube = new THREE.Mesh(
      new THREE.TubeGeometry(curve, segments, 0.0045 + index * 0.0009, 10, true),
      new THREE.MeshStandardMaterial({
        color: orbitColor,
        emissive: new THREE.Color(orbitColor).multiplyScalar(0.22),
        transparent: true,
        opacity: 0.34,
        roughness: 0.52,
        metalness: 0.12,
      }),
    );
    group.add(orbitTube);

    const dashed = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineDashedMaterial({
        color: 0xe2e8f0,
        transparent: true,
        opacity: 0.42,
        dashSize: 0.23 + index * 0.03,
        gapSize: 0.11 + index * 0.02,
      }),
    );
    dashed.computeLineDistances();
    group.add(dashed);

    return group;
  }

  createBeam({ color, opacity, thickness }) {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(1, 1, 1, 12, 1, true),
      new THREE.MeshStandardMaterial({
        color,
        emissive: new THREE.Color(color).multiplyScalar(0.3),
        emissiveIntensity: 0.95,
        transparent: true,
        opacity,
        roughness: 0.26,
        metalness: 0.06,
      }),
    );
    mesh.userData.thickness = thickness;
    mesh.visible = false;
    return mesh;
  }

  createInterSatelliteLinks() {
    this.interSatelliteLinks = [];
    for (let i = 0; i < this.satellites.length; i += 1) {
      const next = (i + 1) % this.satellites.length;
      const link = {
        a: i,
        b: next,
        beam: this.createBeam({
          color: 0x38bdf8,
          opacity: 0.26,
          thickness: 0.0052,
        }),
      };
      this.interSatelliteLinks.push(link);
      this.rootGroup.add(link.beam);
    }
  }

  updateBeam(beam, start, end) {
    this.tmpVecA.copy(end).sub(start);
    const length = this.tmpVecA.length();
    if (length < 0.001) {
      beam.visible = false;
      return;
    }

    beam.visible = true;
    const radius = beam.userData.thickness || 0.005;
    beam.position.copy(start).addScaledVector(this.tmpVecA, 0.5);
    beam.scale.set(radius, length, radius);
    beam.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      this.tmpVecA.normalize(),
    );
  }

  upsertFleetSnapshot(fleet) {
    Object.values(fleet || {}).forEach((entry) => this.upsertRoverFromTelemetry(entry));
  }

  upsertRoverFromTelemetry(data) {
    const roverId = data?.rover_id;
    if (!roverId) return;

    let rover = this.rovers.get(roverId);
    if (!rover) {
      rover = this.createRover(roverId);
      this.rovers.set(roverId, rover);
      this.rootGroup.add(rover.group);
      this.worldBodies.static.push(rover.staticCollider);
      if (!this.selectedRoverId) this.setSelectedRover(roverId);
    }

    rover.state = String(data.state || rover.state || "IDLE").toUpperCase();
    rover.battery = Number.isFinite(Number(data.battery))
      ? Number(data.battery)
      : rover.battery;

    if (data.position && Number.isFinite(Number(data.position.lat)) && Number.isFinite(Number(data.position.lon))) {
      rover.lat = Number(data.position.lat);
      rover.lon = Number(data.position.lon);
      this.placeRover(rover);
    }

    this.updateRoverStateStyle(rover);
    this.updateMetaText();
  }

  createRover(roverId) {
    const group = new THREE.Group();
    const materials = [];

    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x34d399,
      emissive: 0x0a1f16,
      roughness: 0.63,
      metalness: 0.18,
    });
    materials.push(bodyMat);

    const chassis = new THREE.Mesh(
      new THREE.BoxGeometry(0.022, 0.010, 0.014),
      bodyMat,
    );
    group.add(chassis);

    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0016, 0.0016, 0.014, 9),
      new THREE.MeshStandardMaterial({
        color: 0xd1d5db,
        roughness: 0.44,
        metalness: 0.48,
      }),
    );
    mast.position.set(0, 0.012, 0.0);
    group.add(mast);

    const cameraHead = new THREE.Mesh(
      new THREE.BoxGeometry(0.007, 0.0048, 0.0048),
      new THREE.MeshStandardMaterial({
        color: 0xe5e7eb,
        roughness: 0.35,
        metalness: 0.56,
      }),
    );
    cameraHead.position.set(0, 0.02, 0.0);
    group.add(cameraHead);

    const wheelMat = new THREE.MeshStandardMaterial({
      color: 0x111827,
      roughness: 0.82,
      metalness: 0.07,
    });
    for (let side = -1; side <= 1; side += 2) {
      for (let i = -1; i <= 1; i += 1) {
        const wheel = new THREE.Mesh(
          new THREE.CylinderGeometry(0.0028, 0.0028, 0.0024, 12),
          wheelMat,
        );
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(i * 0.0092, -0.005, side * 0.0076);
        group.add(wheel);
      }
    }

    const antenna = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0008, 0.0008, 0.011, 8),
      new THREE.MeshStandardMaterial({
        color: 0x9ca3af,
        roughness: 0.5,
        metalness: 0.6,
      }),
    );
    antenna.position.set(-0.0066, 0.012, -0.003);
    group.add(antenna);

    const dish = new THREE.Mesh(
      new THREE.ConeGeometry(0.0024, 0.0048, 14),
      new THREE.MeshStandardMaterial({
        color: 0xf3f4f6,
        roughness: 0.25,
        metalness: 0.65,
      }),
    );
    dish.rotation.z = Math.PI / 2;
    dish.position.set(-0.009, 0.0172, -0.003);
    group.add(dish);

    const rover = {
      id: roverId,
      group,
      materials,
      lat: -43.3,
      lon: -11.2,
      battery: 1.0,
      state: "IDLE",
      staticCollider: {
        id: roverId,
        type: "rover",
        radius: 0.018,
        mass: Infinity,
        restitution: 0.5,
        position: new THREE.Vector3(),
      },
    };

    this.placeRover(rover);
    return rover;
  }

  placeRover(rover) {
    const normal = this.latLonToUnit(rover.lat, rover.lon);
    const radius = this.MOON_RADIUS + this.ROVER_SURFACE_OFFSET;
    rover.group.position.copy(normal).multiplyScalar(radius);
    rover.staticCollider.position.copy(rover.group.position);

    const up = new THREE.Vector3(0, 1, 0);
    const orient = new THREE.Quaternion().setFromUnitVectors(up, normal);
    rover.group.quaternion.copy(orient);
  }

  latLonToUnit(lat, lon) {
    const phi = (90 - lat) * (Math.PI / 180);
    const theta = (lon + 180) * (Math.PI / 180);
    const x = -(Math.sin(phi) * Math.cos(theta));
    const z = Math.sin(phi) * Math.sin(theta);
    const y = Math.cos(phi);
    return new THREE.Vector3(x, y, z).normalize();
  }

  setSelectedRover(roverId) {
    if (!roverId || !this.rovers.has(roverId)) return;
    this.selectedRoverId = roverId;
    this.rovers.forEach((rover, id) => {
      rover.group.scale.setScalar(id === roverId ? 1.2 : 1.0);
      rover.group.traverse((obj) => {
        if (!obj.isMesh || !obj.material) return;
        if (!Object.prototype.hasOwnProperty.call(obj.material, "emissive")) return;
        if (id === roverId) obj.material.emissiveIntensity = 0.6;
        else obj.material.emissiveIntensity = 0.24;
      });
    });
    this.updateMetaText();
  }

  updateRoverStateStyle(rover) {
    const state = rover.state;
    let color = 0x34d399;
    let emissive = 0x0a1f16;

    if (state === "EXECUTING") {
      color = 0x3b82f6;
      emissive = 0x11244d;
    } else if (state === "SAFE_MODE") {
      color = 0xf59e0b;
      emissive = 0x3a2a09;
    } else if (state === "ERROR") {
      color = 0xef4444;
      emissive = 0x450f0f;
    }

    rover.materials.forEach((mat) => {
      mat.color.setHex(color);
      mat.emissive.setHex(emissive);
    });
  }

  setViewMode(mode, emitEvent = true) {
    const next = mode === "astronaut" ? "astronaut" : "orbital";
    this.viewMode = next;
    this.container.classList.toggle("view-astronaut", next === "astronaut");
    this.updateCamera(true);
    this.updateMetaText();
    if (emitEvent) this.bus.emit("viz:view-mode", { mode: next });
  }

  updateCamera(force = false) {
    if (!this.camera) return;

    if (this.viewMode === "orbital") {
      this.camera.fov = 38;
      this.camera.position.set(0, 0, this.orbitalDistance);
      this.camera.lookAt(0, 0, 0);
    } else {
      this.camera.fov = 44;
      this.camera.position.set(0, 2.6, 3.45);
      this.camera.lookAt(0, -this.astronautPitch, 0);
    }

    if (force) this.camera.updateProjectionMatrix();
  }

  updateMetaText() {
    const metaEl = document.getElementById("lunar-meta");
    if (!metaEl) return;

    const modeLabel = this.viewMode === "astronaut" ? "Astronaut" : "Orbital";
    if (!this.selectedRoverId || !this.rovers.has(this.selectedRoverId)) {
      metaEl.textContent = `${modeLabel} view | Rover: --`;
      return;
    }

    const rover = this.rovers.get(this.selectedRoverId);
    const lat = Number(rover.lat).toFixed(2);
    const lon = Number(rover.lon).toFixed(2);
    metaEl.textContent =
      `${modeLabel} | ${rover.id.toUpperCase()} | Lat ${lat}, Lon ${lon} | Collisions ${this.collisionCounter}`;
  }

  setupInteraction() {
    this.container.addEventListener("contextmenu", (e) => e.preventDefault());

    this.container.addEventListener("mousedown", (e) => {
      this.dragState.active = true;
      this.dragState.button = e.button;
      this.dragState.x = e.clientX;
      this.dragState.y = e.clientY;
    });

    window.addEventListener("mouseup", () => {
      this.dragState.active = false;
    });

    window.addEventListener("mousemove", (e) => {
      if (!this.dragState.active || !this.rootGroup) return;
      const dx = e.clientX - this.dragState.x;
      const dy = e.clientY - this.dragState.y;
      this.dragState.x = e.clientX;
      this.dragState.y = e.clientY;

      const rollGesture = this.dragState.button === 2 || e.shiftKey;
      if (rollGesture) {
        this.rootGroup.rotateZ(dx * 0.0065);
      } else {
        this.rootGroup.rotateY(dx * 0.0065);
        this.rootGroup.rotateX(dy * 0.0065);
      }
    });

    this.container.addEventListener(
      "wheel",
      (e) => {
        e.preventDefault();
        if (this.viewMode === "orbital") {
          this.orbitalDistance = THREE.MathUtils.clamp(
            this.orbitalDistance + e.deltaY * 0.0012,
            3.2,
            11.5,
          );
        } else {
          this.astronautPitch = THREE.MathUtils.clamp(
            this.astronautPitch + e.deltaY * 0.0009,
            0.9,
            2.1,
          );
        }
        this.updateCamera();
      },
      { passive: false },
    );
  }

  physicsStep(dt) {
    const clampedDt = Math.min(dt, 0.034);

    this.worldBodies.dynamic.forEach((body) => {
      const r = body.position.length();
      if (r > 0.0001) {
        const accelScale = -this.GRAVITY_MU / (r * r * r);
        this.tmpVecA.copy(body.position).multiplyScalar(accelScale);
        body.velocity.addScaledVector(this.tmpVecA, clampedDt);
      }

      body.position.addScaledVector(body.velocity, clampedDt);
      body.group.position.copy(body.position);
      body.group.lookAt(0, 0, 0);

      if (body.collisionCooldown > 0) {
        body.collisionCooldown = Math.max(0, body.collisionCooldown - clampedDt);
      }

      this.resolveMoonCollision(body);
      this.resolveRoverCollisions(body);
    });

    this.resolveSatelliteCollisions();
    this.updateSatelliteLinks();
  }

  resolveMoonCollision(body) {
    const minDist = this.MOON_RADIUS + body.radius;
    const dist = body.position.length();
    if (dist >= minDist) return;

    if (dist < 0.00001) {
      body.position.set(minDist, 0, 0);
    } else {
      body.position.multiplyScalar(minDist / dist);
    }

    this.tmpVecA.copy(body.position).normalize();
    const vN = body.velocity.dot(this.tmpVecA);
    if (vN < 0) {
      body.velocity.addScaledVector(
        this.tmpVecA,
        -(1 + body.restitution) * vN,
      );
    }

    if (body.collisionCooldown <= 0) {
      this.emitCollisionEvent(`${body.id} hit moon`);
      body.collisionCooldown = 0.35;
    }
  }

  resolveRoverCollisions(body) {
    this.rovers.forEach((rover) => {
      const roverBody = rover.staticCollider;
      this.tmpVecA.copy(body.position).sub(roverBody.position);
      const distSq = this.tmpVecA.lengthSq();
      const minDist = body.radius + roverBody.radius;

      if (distSq >= minDist * minDist) return;

      const dist = Math.sqrt(Math.max(distSq, 0.0000001));
      this.tmpVecA.multiplyScalar(1 / dist);
      const penetration = minDist - dist;
      body.position.addScaledVector(this.tmpVecA, penetration);

      const velAlongNormal = body.velocity.dot(this.tmpVecA);
      if (velAlongNormal < 0) {
        body.velocity.addScaledVector(
          this.tmpVecA,
          -(1 + 0.58) * velAlongNormal,
        );
      }

      if (body.collisionCooldown <= 0) {
        this.emitCollisionEvent(`${body.id} brushed ${rover.id}`);
        body.collisionCooldown = 0.35;
      }
    });
  }

  resolveSatelliteCollisions() {
    const sats = this.worldBodies.dynamic;
    for (let i = 0; i < sats.length; i += 1) {
      for (let j = i + 1; j < sats.length; j += 1) {
        const a = sats[i];
        const b = sats[j];

        this.tmpVecA.copy(b.position).sub(a.position);
        const distSq = this.tmpVecA.lengthSq();
        const minDist = a.radius + b.radius;
        if (distSq >= minDist * minDist) continue;

        const dist = Math.sqrt(Math.max(distSq, 0.0000001));
        const normal = this.tmpVecA.multiplyScalar(1 / dist);
        const overlap = minDist - dist;

        a.position.addScaledVector(normal, -overlap * 0.5);
        b.position.addScaledVector(normal, overlap * 0.5);

        const relVel = this.tmpVecB.copy(b.velocity).sub(a.velocity);
        const velAlongNormal = relVel.dot(normal);
        if (velAlongNormal > 0) continue;

        const restitution = Math.min(a.restitution, b.restitution);
        const impulseMag =
          (-(1 + restitution) * velAlongNormal) /
          (1 / a.mass + 1 / b.mass);

        this.tmpVecC.copy(normal).multiplyScalar(impulseMag);
        a.velocity.addScaledVector(this.tmpVecC, -1 / a.mass);
        b.velocity.addScaledVector(this.tmpVecC, 1 / b.mass);

        if (a.collisionCooldown <= 0 && b.collisionCooldown <= 0) {
          this.emitCollisionEvent(`${a.id} collided with ${b.id}`);
          a.collisionCooldown = 0.35;
          b.collisionCooldown = 0.35;
        }
      }
    }
  }

  emitCollisionEvent(message) {
    this.collisionCounter += 1;
    this.updateMetaText();
    this.bus.emit("log", {
      tag: "system",
      text: `🛰 Physics collision: ${message}`,
    });
  }

  updateSatelliteLinks() {
    let selectedPos = null;
    if (this.selectedRoverId && this.rovers.has(this.selectedRoverId)) {
      selectedPos = this.rovers.get(this.selectedRoverId).group.position;
    } else if (this.rovers.size > 0) {
      selectedPos = this.rovers.values().next().value.group.position;
    }

    this.satellites.forEach((sat) => {
      const satPos = sat.body.position;
      this.updateBeam(sat.linkToEarth, satPos, this.earthAnchor);

      const roverTarget = selectedPos || this.tmpVecD.set(0, 0, 0);
      this.updateBeam(sat.linkToRover, satPos, roverTarget);
    });

    this.interSatelliteLinks.forEach((link) => {
      const satA = this.satellites[link.a];
      const satB = this.satellites[link.b];
      if (!satA || !satB) return;
      this.updateBeam(link.beam, satA.body.position, satB.body.position);
    });
  }

  onResize() {
    this.width = this.container.clientWidth;
    this.height = this.container.clientHeight;
    if (!this.camera || !this.renderer) return;

    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(this.width, this.height);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    const now = performance.now();
    const dt = (now - this.lastFrameTs) / 1000;
    this.lastFrameTs = now;

    this.physicsStep(dt);
    this.updateCamera();
    this.renderer.render(this.scene, this.camera);
  }
}

window.VisualizationController = VisualizationController;
