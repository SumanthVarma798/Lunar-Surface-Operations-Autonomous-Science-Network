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
    this.ROVER_SURFACE_OFFSET = 0.005;
    this.LOS_MARGIN = 0.012;
    this.LOS_ENDPOINT_TOLERANCE = 0.04;
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
    this.orbitalDistance = 8.4;
    this.astronautPitch = 0.09;
    this.astronautHeight = 0.018;
    this.astronautShoulderOffset = 0.009;
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
    this.linkStatus = { total: 0, clear: 0, blocked: 0 };
    this.curiosityGeometry = null;
    this.curiosityModelReady = false;
    this.curiosityModelFailed = false;
    this.curiosityAssetUrl = "assets/nasa/curiosity-nasa.stl";

    this.tmpVecA = new THREE.Vector3();
    this.tmpVecB = new THREE.Vector3();
    this.tmpVecC = new THREE.Vector3();
    this.tmpVecD = new THREE.Vector3();
    this.tmpVecE = new THREE.Vector3();
    this.tmpQuatA = new THREE.Quaternion();
    this.tmpColor = new THREE.Color();
    this.worldUp = new THREE.Vector3(0, 1, 0);
    this.earthAnchorOrbital = new THREE.Vector3(8.4, 3.2, 6.8);
    this.earthAnchor = this.earthAnchorOrbital.clone();

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
      0.003,
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
    this.loadCuriosityModel();
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
    const colorMap = this.createProceduralMoonTexture();

    const material = new THREE.MeshStandardMaterial({
      map: colorMap,
      bumpMap: colorMap,
      bumpScale: 0.03,
      roughness: 0.94,
      metalness: 0.01,
    });

    this.moon = new THREE.Mesh(geometry, material);
    this.rootGroup.add(this.moon);
    this.upgradeMoonTexture(material);

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

  createProceduralMoonTexture() {
    const width = 2048;
    const height = 1024;
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");

    const image = ctx.createImageData(width, height);
    const data = image.data;
    for (let y = 0; y < height; y += 1) {
      const v = y / height;
      for (let x = 0; x < width; x += 1) {
        const i = (y * width + x) * 4;
        const u = x / width;
        const grain = 10 * (Math.sin(u * 85) + Math.cos(v * 51)) + 12 * Math.sin((u + v) * 41);
        const shade = Math.round(112 + grain + Math.random() * 18);
        data[i] = shade;
        data[i + 1] = shade;
        data[i + 2] = shade + 3;
        data[i + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);

    for (let c = 0; c < 520; c += 1) {
      const x = Math.random() * width;
      const y = Math.random() * height;
      const r = 3 + Math.random() * 30;
      const ring = ctx.createRadialGradient(x, y, 0, x, y, r);
      ring.addColorStop(0, "rgba(235,235,235,0.16)");
      ring.addColorStop(0.55, "rgba(170,170,170,0.14)");
      ring.addColorStop(1, "rgba(70,70,70,0)");
      ctx.fillStyle = ring;
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fill();
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = Math.min(
      14,
      this.renderer.capabilities.getMaxAnisotropy ? this.renderer.capabilities.getMaxAnisotropy() : 14,
    );
    return texture;
  }

  upgradeMoonTexture(material) {
    const textureLoader = new THREE.TextureLoader();
    textureLoader.load(
      "https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/moon_1024.jpg",
      (colorMap) => {
        colorMap.colorSpace = THREE.SRGBColorSpace;
        colorMap.anisotropy = Math.min(
          14,
          this.renderer.capabilities.getMaxAnisotropy ? this.renderer.capabilities.getMaxAnisotropy() : 14,
        );
        material.map = colorMap;
        material.bumpMap = colorMap;
        material.bumpScale = 0.022;
        material.needsUpdate = true;
      },
      undefined,
      () => {
        this.bus.emit("log", {
          tag: "system",
          text: "⚠️ External moon texture unavailable, using procedural lunar albedo",
        });
      },
    );
  }

  loadCuriosityModel() {
    if (!THREE.STLLoader) {
      this.curiosityModelFailed = true;
      this.bus.emit("log", {
        tag: "system",
        text: "⚠️ STL loader unavailable, using fallback rover mesh",
      });
      return;
    }

    const loader = new THREE.STLLoader();
    loader.load(
      this.curiosityAssetUrl,
      (geometry) => {
        this.curiosityGeometry = this.normalizeCuriosityGeometry(geometry);
        this.curiosityModelReady = true;
        this.rovers.forEach((rover) => this.attachCuriosityModel(rover));
      },
      undefined,
      () => {
        this.curiosityModelFailed = true;
        this.bus.emit("log", {
          tag: "system",
          text: "⚠️ Curiosity model failed to load, using fallback rover mesh",
        });
      },
    );
  }

  normalizeCuriosityGeometry(geometry) {
    geometry.computeBoundingBox();
    geometry.computeVertexNormals();
    geometry.center();

    const size = new THREE.Vector3();
    geometry.boundingBox.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z, 0.0001);
    const targetSize = 0.036;
    const scale = targetSize / maxDim;

    geometry.scale(scale, scale, scale);
    geometry.rotateX(-Math.PI / 2);
    geometry.translate(0, 0.004, 0);
    geometry.computeBoundingSphere();
    return geometry;
  }

  attachCuriosityModel(rover) {
    if (!rover || !this.curiosityModelReady || rover.modelMesh) return;

    const modelMaterial = new THREE.MeshStandardMaterial({
      color: 0xc6d0dc,
      roughness: 0.55,
      metalness: 0.52,
      emissive: 0x0b1220,
      emissiveIntensity: 0.24,
    });
    const model = new THREE.Mesh(this.curiosityGeometry, modelMaterial);
    model.castShadow = false;
    model.receiveShadow = false;
    rover.group.add(model);
    rover.modelMesh = model;
    rover.modelMaterial = modelMaterial;

    if (rover.fallbackMesh) {
      rover.fallbackMesh.visible = false;
    }

    this.updateRoverStateStyle(rover);
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
      new THREE.SphereGeometry(0.18, 26, 26),
      new THREE.MeshStandardMaterial({
        color: 0x93c5fd,
        emissive: 0x1d4e89,
        emissiveIntensity: 0.55,
        roughness: 0.36,
        metalness: 0.08,
      }),
    );
    baseGroup.add(core);

    const cloudBand = new THREE.Mesh(
      new THREE.SphereGeometry(0.186, 20, 20),
      new THREE.MeshBasicMaterial({
        color: 0xe0f2fe,
        transparent: true,
        opacity: 0.17,
      }),
    );
    baseGroup.add(cloudBand);

    const halo = new THREE.Mesh(
      new THREE.SphereGeometry(0.26, 18, 18),
      new THREE.MeshBasicMaterial({
        color: 0x67e8f9,
        transparent: true,
        opacity: 0.22,
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
      { radius: 3.4, tiltX: 0.16, tiltZ: -0.06, phase: 0.0, size: 0.048, color: 0x4fd1f9 },
      { radius: 4.55, tiltX: 0.78, tiltZ: 0.46, phase: 2.1, size: 0.054, color: 0xa78bfa },
      { radius: 5.95, tiltX: 0.52, tiltZ: -0.86, phase: 4.35, size: 0.061, color: 0x93c5fd },
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
      color: 0x67e8f9,
      emissive: 0x1c90a8,
      blockedColor: 0xfb7185,
      blockedEmissive: 0x7f1d1d,
      opacity: 0.48,
      blockedOpacity: 0.7,
      thickness: 0.012,
    });
    const linkToRover = this.createBeam({
      color: 0xfacc15,
      emissive: 0x7c5d0b,
      blockedColor: 0xf97316,
      blockedEmissive: 0x7c2d12,
      opacity: 0.42,
      blockedOpacity: 0.66,
      thickness: 0.009,
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
      new THREE.TubeGeometry(curve, segments, 0.004 + index * 0.001, 10, true),
      new THREE.MeshStandardMaterial({
        color: orbitColor,
        emissive: new THREE.Color(orbitColor).multiplyScalar(0.2),
        transparent: true,
        opacity: 0.3,
        roughness: 0.56,
        metalness: 0.12,
      }),
    );
    group.add(orbitTube);

    const dashed = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineDashedMaterial({
        color: 0x93c5fd,
        transparent: true,
        opacity: 0.34,
        dashSize: 0.2 + index * 0.03,
        gapSize: 0.12 + index * 0.02,
      }),
    );
    dashed.computeLineDistances();
    group.add(dashed);

    return group;
  }

  createBeam({
    color,
    emissive = 0x223449,
    blockedColor = 0xfb7185,
    blockedEmissive = 0x7f1d1d,
    opacity,
    blockedOpacity = 0.7,
    thickness,
  }) {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(1, 1, 1, 12, 1, true),
      new THREE.MeshStandardMaterial({
        color,
        emissive: new THREE.Color(emissive),
        emissiveIntensity: 0.95,
        transparent: true,
        opacity,
        roughness: 0.26,
        metalness: 0.06,
      }),
    );
    mesh.userData = {
      thickness,
      style: {
        color,
        emissive,
        blockedColor,
        blockedEmissive,
        opacity,
        blockedOpacity,
      },
      blocked: false,
    };
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
          color: 0x818cf8,
          emissive: 0x3730a3,
          blockedColor: 0xf87171,
          blockedEmissive: 0x7f1d1d,
          opacity: 0.24,
          blockedOpacity: 0.58,
          thickness: 0.0062,
        }),
      };
      this.interSatelliteLinks.push(link);
      this.rootGroup.add(link.beam);
    }
  }

  setBeamBlockedState(beam, blocked) {
    if (!beam || !beam.material || !beam.userData?.style) return;
    if (beam.userData.blocked === blocked) return;
    beam.userData.blocked = blocked;

    const style = beam.userData.style;
    const material = beam.material;
    const color = blocked ? style.blockedColor : style.color;
    const emissive = blocked ? style.blockedEmissive : style.emissive;
    const opacity = blocked ? style.blockedOpacity : style.opacity;

    material.color.setHex(color);
    material.emissive.setHex(emissive);
    material.opacity = opacity;
  }

  isLineOfSightBlocked(
    start,
    end,
    { allowStartSurfaceTouch = false, allowEndSurfaceTouch = false } = {},
  ) {
    this.tmpVecA.copy(end).sub(start);
    const lenSq = this.tmpVecA.lengthSq();
    if (lenSq < 0.0000001) return false;

    let t = -start.dot(this.tmpVecA) / lenSq;
    if (t <= 0 || t >= 1) return false;

    if (allowStartSurfaceTouch && t < this.LOS_ENDPOINT_TOLERANCE) return false;
    if (allowEndSurfaceTouch && t > 1 - this.LOS_ENDPOINT_TOLERANCE) return false;

    this.tmpVecB.copy(start).addScaledVector(this.tmpVecA, t);
    return this.tmpVecB.length() < this.MOON_RADIUS + this.LOS_MARGIN;
  }

  updateBeam(beam, start, end, blocked = false) {
    this.tmpVecA.copy(end).sub(start);
    const length = this.tmpVecA.length();
    if (length < 0.001) {
      beam.visible = false;
      return;
    }

    beam.visible = true;
    const radius = beam.userData?.thickness || 0.005;
    beam.position.copy(start).addScaledVector(this.tmpVecA, 0.5);
    beam.scale.set(radius, length, radius);
    beam.quaternion.setFromUnitVectors(
      this.worldUp,
      this.tmpVecA.normalize(),
    );
    this.setBeamBlockedState(beam, blocked);
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
    const fallbackMaterial = new THREE.MeshStandardMaterial({
      color: 0x34d399,
      emissive: 0x0a1f16,
      roughness: 0.48,
      metalness: 0.26,
    });
    const fallbackMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.018, 0.01, 0.012),
      fallbackMaterial,
    );
    group.add(fallbackMesh);

    const beaconMaterial = new THREE.MeshBasicMaterial({ color: 0x34d399 });
    const beacon = new THREE.Mesh(
      new THREE.SphereGeometry(0.0028, 10, 10),
      beaconMaterial,
    );
    beacon.position.set(0, 0.014, 0);
    group.add(beacon);

    const rover = {
      id: roverId,
      group,
      fallbackMesh,
      fallbackMaterial,
      beaconMaterial,
      modelMesh: null,
      modelMaterial: null,
      lat: -43.3,
      lon: -11.2,
      battery: 1.0,
      state: "IDLE",
      staticCollider: {
        id: roverId,
        type: "rover",
        radius: 0.015,
        mass: Infinity,
        restitution: 0.5,
        position: new THREE.Vector3(),
      },
    };

    this.attachCuriosityModel(rover);
    this.placeRover(rover);
    this.updateRoverStateStyle(rover);
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
      const selected = id === roverId;
      rover.group.scale.setScalar(selected ? 1.12 : 1.0);
      rover.group.traverse((obj) => {
        if (!obj.isMesh || !obj.material) return;
        if (!Object.prototype.hasOwnProperty.call(obj.material, "emissive")) return;
        obj.material.emissiveIntensity = selected ? 0.5 : 0.24;
      });
    });
    this.updateMetaText();
  }

  updateRoverStateStyle(rover) {
    const state = rover.state;
    let accent = 0x34d399;
    let fallbackEmissive = 0x0a1f16;

    if (state === "EXECUTING") {
      accent = 0x3b82f6;
      fallbackEmissive = 0x11244d;
    } else if (state === "SAFE_MODE") {
      accent = 0xf59e0b;
      fallbackEmissive = 0x3a2a09;
    } else if (state === "ERROR") {
      accent = 0xef4444;
      fallbackEmissive = 0x450f0f;
    }

    if (rover.fallbackMaterial) {
      rover.fallbackMaterial.color.setHex(accent);
      rover.fallbackMaterial.emissive.setHex(fallbackEmissive);
    }

    if (rover.beaconMaterial) {
      rover.beaconMaterial.color.setHex(accent);
    }

    if (rover.modelMaterial) {
      rover.modelMaterial.color.setHex(0xc6d0dc);
      this.tmpColor.setHex(accent).multiplyScalar(0.26);
      rover.modelMaterial.emissive.copy(this.tmpColor);
    }
  }

  setViewMode(mode, emitEvent = true) {
    const next = mode === "astronaut" ? "astronaut" : "orbital";
    this.viewMode = next;
    this.container.classList.toggle("view-astronaut", next === "astronaut");
    this.satellites.forEach((sat) => {
      sat.orbitPath.visible = next === "orbital";
    });
    this.updateCamera(true);
    this.updateMetaText();
    if (emitEvent) this.bus.emit("viz:view-mode", { mode: next });
  }

  getViewAnchorLocal() {
    if (this.selectedRoverId && this.rovers.has(this.selectedRoverId)) {
      return this.rovers.get(this.selectedRoverId).group.position;
    }
    if (this.rovers.size > 0) {
      return this.rovers.values().next().value.group.position;
    }
    return null;
  }

  updateOrbitalCamera() {
    this.camera.fov = 35;
    this.camera.position.set(0, 0, this.orbitalDistance);
    this.camera.lookAt(0, 0, 0);

    if (!this.earthAnchor.equals(this.earthAnchorOrbital)) {
      this.earthAnchor.copy(this.earthAnchorOrbital);
      if (this.earthBase) this.earthBase.position.copy(this.earthAnchor);
    }
    if (this.earthBase) this.earthBase.scale.setScalar(1);
  }

  updateAstronautCamera() {
    const anchorLocal = this.getViewAnchorLocal();
    if (!anchorLocal) {
      this.camera.fov = 40;
      this.camera.position.set(0, 2.2, 4.2);
      this.camera.lookAt(0, 0, 0);
      return;
    }

    const rootQuat = this.rootGroup.quaternion;
    const normalWorld = this.tmpVecA.copy(anchorLocal).applyQuaternion(rootQuat).normalize();
    const surfaceWorld = this.tmpVecB.copy(normalWorld).multiplyScalar(this.MOON_RADIUS + this.ROVER_SURFACE_OFFSET);

    const horizonRef = Math.abs(normalWorld.dot(this.worldUp)) > 0.94
      ? this.tmpVecC.set(1, 0, 0)
      : this.tmpVecC.copy(this.worldUp);
    const rightWorld = this.tmpVecD.crossVectors(horizonRef, normalWorld).normalize();
    const forwardWorld = this.tmpVecE.crossVectors(normalWorld, rightWorld).normalize();

    const lookDir = forwardWorld.clone().applyAxisAngle(rightWorld, this.astronautPitch).normalize();
    const eye = surfaceWorld
      .clone()
      .addScaledVector(normalWorld, this.astronautHeight)
      .addScaledVector(rightWorld, this.astronautShoulderOffset);

    this.camera.fov = 52;
    this.camera.position.copy(eye);
    this.camera.lookAt(eye.clone().addScaledVector(lookDir, 3.6));

    this.updateEarthForAstronautView(normalWorld, rightWorld, forwardWorld);
  }

  updateEarthForAstronautView(normalWorld, rightWorld, forwardWorld) {
    if (!this.earthBase) return;

    const earthDirectionWorld = this.tmpVecA
      .copy(forwardWorld)
      .applyAxisAngle(rightWorld, 0.065)
      .applyAxisAngle(normalWorld, -0.34)
      .normalize();

    this.tmpQuatA.copy(this.rootGroup.quaternion).invert();
    this.earthAnchor.copy(earthDirectionWorld).applyQuaternion(this.tmpQuatA).multiplyScalar(10.4);
    this.earthBase.position.copy(this.earthAnchor);
    this.earthBase.scale.setScalar(1.22);
  }

  updateCamera(force = false) {
    if (!this.camera) return;

    if (this.viewMode === "orbital") {
      this.updateOrbitalCamera();
    } else {
      this.updateAstronautCamera();
    }

    if (force) this.camera.updateProjectionMatrix();
  }

  updateMetaText() {
    const metaEl = document.getElementById("lunar-meta");
    if (!metaEl) return;

    const modeLabel = this.viewMode === "astronaut" ? "Astronaut" : "Orbital";
    const selectedLabel = this.selectedRoverId && this.rovers.has(this.selectedRoverId)
      ? this.selectedRoverId.toUpperCase()
      : "ROVER --";

    if (this.linkStatus.total <= 0) {
      metaEl.classList.remove("warning");
      metaEl.textContent = `${modeLabel} | ${selectedLabel} | LOS pending`;
      return;
    }

    const blocked = this.linkStatus.blocked;
    const clear = this.linkStatus.clear;
    metaEl.classList.toggle("warning", blocked > 0);
    metaEl.textContent = `${modeLabel} | ${selectedLabel} | LOS ${clear}/${this.linkStatus.total} clear${blocked > 0 ? ` (${blocked} blocked)` : ""}`;
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
            5.2,
            16.0,
          );
        } else {
          this.astronautPitch = THREE.MathUtils.clamp(
            this.astronautPitch + e.deltaY * 0.0009,
            -0.12,
            0.38,
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

    const linkStatus = { total: 0, clear: 0, blocked: 0 };
    const hideVisualLinks = this.viewMode === "astronaut";

    this.satellites.forEach((sat) => {
      const satPos = sat.body.position;
      const earthBlocked = this.isLineOfSightBlocked(satPos, this.earthAnchor);
      this.updateBeam(sat.linkToEarth, satPos, this.earthAnchor, earthBlocked);
      if (hideVisualLinks) sat.linkToEarth.visible = false;
      linkStatus.total += 1;
      if (earthBlocked) linkStatus.blocked += 1;
      else linkStatus.clear += 1;

      if (selectedPos) {
        const roverBlocked = this.isLineOfSightBlocked(satPos, selectedPos, {
          allowEndSurfaceTouch: true,
        });
        this.updateBeam(sat.linkToRover, satPos, selectedPos, roverBlocked);
        if (hideVisualLinks) sat.linkToRover.visible = false;
        linkStatus.total += 1;
        if (roverBlocked) linkStatus.blocked += 1;
        else linkStatus.clear += 1;
      } else {
        sat.linkToRover.visible = false;
      }
    });

    this.interSatelliteLinks.forEach((link) => {
      const satA = this.satellites[link.a];
      const satB = this.satellites[link.b];
      if (!satA || !satB) return;
      const blocked = this.isLineOfSightBlocked(satA.body.position, satB.body.position);
      this.updateBeam(link.beam, satA.body.position, satB.body.position, blocked);
      if (hideVisualLinks) link.beam.visible = false;
      linkStatus.total += 1;
      if (blocked) linkStatus.blocked += 1;
      else linkStatus.clear += 1;
    });

    if (
      linkStatus.total !== this.linkStatus.total
      || linkStatus.clear !== this.linkStatus.clear
      || linkStatus.blocked !== this.linkStatus.blocked
    ) {
      this.linkStatus = linkStatus;
      this.updateMetaText();
    } else {
      this.linkStatus = linkStatus;
    }
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
