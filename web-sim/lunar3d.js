/* ═══════════════════════════════════════════════════════
   LSOAS — Orbital Visualization System
   Orbital visualization + physics + collision handling
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
    this.GRAVITY_MU = 1.22;
    this.DEFAULT_RESTITUTION = 0.68;

    this.scene = null;
    this.rootGroup = null;
    this.camera = null;
    this.renderer = null;
    this.moon = null;
    this.stars = null;
    this.earthBase = null;
    this.earthCloudLayer = null;
    this.sunSurface = null;
    this.sunGlow = null;
    this.sunGroup = null;
    this.sunLight = null;
    this.latestCelestialFrame = null;
    this.displayEarthMoonDistanceLu = 10.8;
    this.displaySunDistanceClampLu = { min: 88, max: 320 };

    this.viewMode = "orbital";
    this.defaultOrbitalDistance = 7.6;
    this.defaultCameraAzimuth = -2.22;
    this.defaultCameraPolar = 1.08;
    this.defaultEarthFrameFactor = 0.22;
    this.orbitalDistance = this.defaultOrbitalDistance;
    this.cameraDistance = this.defaultOrbitalDistance;
    this.cameraMinDistance = 1.35;
    this.cameraMaxDistance = 16.0;
    this.cameraAzimuth = 0.54;
    this.cameraPolar = 1.08;
    this.cameraTarget = new THREE.Vector3(0, 0, 0);
    this.cameraTargetDesired = new THREE.Vector3(0, 0, 0);
    this.lastFrameTs = performance.now();

    this.dragState = {
      active: false,
      button: 0,
      mode: "orbit",
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
    this.linkStatus = {
      total: 0,
      clear: 0,
      blocked: 0,
      earthTotal: 0,
      earthClear: 0,
      earthBlocked: 0,
      roverTotal: 0,
      roverClear: 0,
      roverBlocked: 0,
      interTotal: 0,
      interClear: 0,
      interBlocked: 0,
      hasIssue: false,
      issueReason: "",
    };
    this.signalLossState = {
      active: false,
      startedAtMs: 0,
      currentDurationMs: 0,
      lastDurationMs: null,
      outageCount: 0,
    };
    this.curiosityGeometry = null;
    this.curiosityModelReady = false;
    this.curiosityModelFailed = false;
    this.curiosityAssetUrl = "assets/nasa/curiosity-nasa.stl";

    this.tmpVecA = new THREE.Vector3();
    this.tmpVecB = new THREE.Vector3();
    this.tmpVecC = new THREE.Vector3();
    this.tmpVecD = new THREE.Vector3();
    this.tmpColor = new THREE.Color();
    this.earthAnchorOrbital = new THREE.Vector3(8.2, 1.0, 6.6);
    this.earthAnchor = this.earthAnchorOrbital.clone();
    this.sunAnchorOrbital = new THREE.Vector3(20.6, 2.1, 15.8);
    this.earthTextureUrls = {
      color: "assets/nasa/earth-blue-marble-2048.png",
      clouds: "assets/nasa/earth-clouds-2048.jpg",
    };
    this.sunTextureUrl = "assets/nasa/sun-sdo-2048-0171.jpg";

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
      420,
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
    this.createSun();
    this.createSatellites();
    this.setupInteraction();
    this.setViewMode("orbital", false);

    window.addEventListener("resize", () => this.onResize());
  }

  bindBus() {
    this.bus.on("earth:telemetry", (data) =>
      this.upsertRoverFromTelemetry(data),
    );
    this.bus.on("fleet:update", (fleet) => this.upsertFleetSnapshot(fleet));
    this.bus.on("celestial:ephemeris", (frame) =>
      this.setCelestialFrame(frame),
    );
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

    this.sunLight = new THREE.DirectionalLight(0xf8fcff, 2.35);
    this.sunLight.position.copy(this.sunAnchorOrbital);
    this.scene.add(this.sunLight);
    this.scene.add(this.sunLight.target);
    this.sunLight.target.position.set(0, 0, 0);

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
        const grain =
          10 * (Math.sin(u * 85) + Math.cos(v * 51)) +
          12 * Math.sin((u + v) * 41);
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
      this.renderer.capabilities.getMaxAnisotropy
        ? this.renderer.capabilities.getMaxAnisotropy()
        : 14,
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
          this.renderer.capabilities.getMaxAnisotropy
            ? this.renderer.capabilities.getMaxAnisotropy()
            : 14,
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
          text: "WARN: External moon texture unavailable, using procedural lunar albedo",
        });
      },
    );
  }

  loadCuriosityModel() {
    if (!THREE.STLLoader) {
      this.curiosityModelFailed = true;
      this.bus.emit("log", {
        tag: "system",
        text: "WARN: STL loader unavailable, using fallback rover mesh",
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
          text: "WARN: Curiosity model failed to load, using fallback rover mesh",
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
    const targetSize = 0.065;
    const scale = targetSize / maxDim;

    geometry.scale(scale, scale, scale);
    geometry.rotateX(-Math.PI / 2);
    geometry.translate(0, 0.0072, 0);
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

    if (rover.fallbackGroup) {
      rover.fallbackGroup.visible = false;
    } else if (rover.fallbackMesh) {
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
    const earthRadius = 0.34;
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(earthRadius, 64, 64),
      new THREE.MeshStandardMaterial({
        color: 0x9fc4eb,
        emissive: 0x0f2742,
        emissiveIntensity: 0.46,
        roughness: 0.82,
        metalness: 0.02,
      }),
    );
    core.rotation.y = Math.PI * 0.88;
    baseGroup.add(core);

    const cloudBandMaterial = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.32,
      depthWrite: false,
      roughness: 1.0,
      metalness: 0.0,
    });
    const cloudBand = new THREE.Mesh(
      new THREE.SphereGeometry(earthRadius * 1.02, 52, 52),
      cloudBandMaterial,
    );
    cloudBand.rotation.y = Math.PI * 0.92;
    baseGroup.add(cloudBand);

    const atmosphere = new THREE.Mesh(
      new THREE.SphereGeometry(earthRadius * 1.08, 42, 42),
      new THREE.MeshBasicMaterial({
        color: 0xe0f2fe,
        transparent: true,
        opacity: 0.12,
        blending: THREE.AdditiveBlending,
      }),
    );
    baseGroup.add(atmosphere);

    const halo = new THREE.Mesh(
      new THREE.SphereGeometry(earthRadius * 1.34, 26, 26),
      new THREE.MeshBasicMaterial({
        color: 0x8ad8ff,
        transparent: true,
        opacity: 0.17,
        blending: THREE.AdditiveBlending,
      }),
    );
    baseGroup.add(halo);

    baseGroup.position.copy(this.earthAnchor);
    this.rootGroup.add(baseGroup);
    this.earthBase = baseGroup;
    this.earthCloudLayer = cloudBand;

    this.upgradeEarthTextures(core.material, cloudBandMaterial);
  }

  upgradeEarthTextures(coreMaterial, cloudMaterial) {
    const textureLoader = new THREE.TextureLoader();
    textureLoader.setCrossOrigin("anonymous");
    const maxAnisotropy = Math.min(
      14,
      this.renderer?.capabilities?.getMaxAnisotropy
        ? this.renderer.capabilities.getMaxAnisotropy()
        : 14,
    );
    const applyMapSettings = (texture, isColor = false) => {
      if (!texture) return;
      texture.wrapS = THREE.RepeatWrapping;
      texture.wrapT = THREE.ClampToEdgeWrapping;
      texture.anisotropy = maxAnisotropy;
      if (isColor) texture.colorSpace = THREE.SRGBColorSpace;
    };

    textureLoader.load(
      this.earthTextureUrls.color,
      (colorMap) => {
        applyMapSettings(colorMap, true);
        coreMaterial.map = colorMap;
        coreMaterial.needsUpdate = true;
      },
      undefined,
      () => {
        this.bus.emit("log", {
          tag: "system",
          text: "WARN: Earth color texture unavailable, using procedural fallback",
        });
      },
    );

    if (this.earthTextureUrls.normal) {
      textureLoader.load(
        this.earthTextureUrls.normal,
        (normalMap) => {
          applyMapSettings(normalMap, false);
          coreMaterial.normalMap = normalMap;
          coreMaterial.normalScale = new THREE.Vector2(0.55, 0.55);
          coreMaterial.needsUpdate = true;
        },
        undefined,
        () => {},
      );
    }

    if (this.earthTextureUrls.specular) {
      textureLoader.load(
        this.earthTextureUrls.specular,
        (specMap) => {
          applyMapSettings(specMap, false);
          coreMaterial.roughnessMap = specMap;
          coreMaterial.metalness = 0.0;
          coreMaterial.roughness = 0.78;
          coreMaterial.needsUpdate = true;
        },
        undefined,
        () => {},
      );
    }

    textureLoader.load(
      this.earthTextureUrls.clouds,
      (cloudMap) => {
        applyMapSettings(cloudMap, false);
        cloudMaterial.map = cloudMap;
        cloudMaterial.alphaMap = cloudMap;
        cloudMaterial.opacity = 0.42;
        cloudMaterial.needsUpdate = true;
      },
      undefined,
      () => {
        this.bus.emit("log", {
          tag: "system",
          text: "WARN: Earth cloud texture unavailable, using atmospheric fallback",
        });
      },
    );
  }

  createSun() {
    const sunGroup = new THREE.Group();
    const sunRadius = 2.35;
    const sunSurfaceMaterial = new THREE.MeshBasicMaterial({
      color: 0xffcc7a,
    });
    const sunSurface = new THREE.Mesh(
      new THREE.SphereGeometry(sunRadius, 72, 72),
      sunSurfaceMaterial,
    );
    sunGroup.add(sunSurface);

    const sunCorona = new THREE.Mesh(
      new THREE.SphereGeometry(sunRadius * 1.2, 40, 40),
      new THREE.MeshBasicMaterial({
        color: 0xffa155,
        transparent: true,
        opacity: 0.2,
        blending: THREE.AdditiveBlending,
      }),
    );
    sunGroup.add(sunCorona);

    const sunHalo = new THREE.Mesh(
      new THREE.SphereGeometry(sunRadius * 1.55, 32, 32),
      new THREE.MeshBasicMaterial({
        color: 0xffbe74,
        transparent: true,
        opacity: 0.13,
        blending: THREE.AdditiveBlending,
        side: THREE.BackSide,
      }),
    );
    sunGroup.add(sunHalo);

    sunGroup.position.copy(this.sunAnchorOrbital);
    this.scene.add(sunGroup);
    this.sunGroup = sunGroup;
    this.sunSurface = sunSurface;
    this.sunGlow = sunHalo;

    if (this.sunLight) {
      this.sunLight.position.copy(sunGroup.position);
      this.sunLight.target.position.set(0, 0, 0);
    }

    this.upgradeSunTexture(sunSurfaceMaterial);
  }

  upgradeSunTexture(sunSurfaceMaterial) {
    const textureLoader = new THREE.TextureLoader();
    textureLoader.setCrossOrigin("anonymous");
    textureLoader.load(
      this.sunTextureUrl,
      (sunMap) => {
        sunMap.colorSpace = THREE.SRGBColorSpace;
        sunMap.wrapS = THREE.ClampToEdgeWrapping;
        sunMap.wrapT = THREE.ClampToEdgeWrapping;
        sunSurfaceMaterial.map = sunMap;
        sunSurfaceMaterial.alphaMap = sunMap;
        sunSurfaceMaterial.transparent = true;
        sunSurfaceMaterial.opacity = 0.96;
        sunSurfaceMaterial.blending = THREE.AdditiveBlending;
        sunSurfaceMaterial.depthWrite = false;
        sunSurfaceMaterial.needsUpdate = true;
      },
      undefined,
      () => {
        this.bus.emit("log", {
          tag: "system",
          text: "WARN: Sun texture unavailable, using emissive fallback",
        });
      },
    );
  }

  setCelestialFrame(frame) {
    if (!frame || !frame.earth_from_moon_km || !frame.sun_from_moon_km) return;
    this.latestCelestialFrame = frame;

    const earthKmVec = frame.earth_from_moon_km;
    const sunKmVec = frame.sun_from_moon_km;
    const earthDistKm = Math.max(
      Number(frame.earth_distance_km) ||
        this.tmpVecA.set(earthKmVec.x, earthKmVec.y, earthKmVec.z).length(),
      1,
    );
    const sunDistKmRaw = Math.max(
      Number(frame.sun_distance_km) ||
        this.tmpVecB.set(sunKmVec.x, sunKmVec.y, sunKmVec.z).length(),
      1,
    );

    const earthScale = this.displayEarthMoonDistanceLu / earthDistKm;
    this.earthAnchorOrbital.set(
      earthKmVec.x * earthScale,
      earthKmVec.y * earthScale,
      earthKmVec.z * earthScale,
    );

    const ratioRaw = sunDistKmRaw / earthDistKm;
    const compressedRatio = Math.max(9, Math.log10(ratioRaw + 1) * 9.2);
    const sunDistanceLu = THREE.MathUtils.clamp(
      this.displayEarthMoonDistanceLu * compressedRatio,
      this.displaySunDistanceClampLu.min,
      this.displaySunDistanceClampLu.max,
    );

    this.tmpVecC
      .set(sunKmVec.x, sunKmVec.y, sunKmVec.z)
      .normalize()
      .multiplyScalar(sunDistanceLu);
    this.sunAnchorOrbital.copy(this.tmpVecC);

    // Initial snap if they are uninitialized (at 0,0,0)
    if (this.earthBase && this.earthBase.position.lengthSq() < 0.1) {
      this.earthBase.position.copy(this.earthAnchorOrbital);
    }
    if (this.sunGroup && this.sunGroup.position.lengthSq() < 0.1) {
      this.sunGroup.position.copy(this.sunAnchorOrbital);
      if (this.sunLight) {
        this.sunLight.position.copy(this.sunAnchorOrbital);
      }
    }
  }

  createSatellites() {
    const orbitDefs = [
      {
        radius: 3.4,
        tiltX: 0.16,
        tiltZ: -0.06,
        phase: 0.0,
        size: 0.048,
        color: 0x4fd1f9,
      },
      {
        radius: 4.55,
        tiltX: 0.78,
        tiltZ: 0.46,
        phase: 2.1,
        size: 0.054,
        color: 0xa78bfa,
      },
      {
        radius: 5.95,
        tiltX: 0.52,
        tiltZ: -0.86,
        phase: 4.35,
        size: 0.061,
        color: 0x93c5fd,
      },
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

    const ref =
      Math.abs(normal.y) > 0.92
        ? new THREE.Vector3(1, 0, 0)
        : new THREE.Vector3(0, 1, 0);

    const tangentA = new THREE.Vector3().crossVectors(ref, normal).normalize();
    const tangentB = new THREE.Vector3()
      .crossVectors(normal, tangentA)
      .normalize();
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

    const busMaterial = new THREE.MeshStandardMaterial({
      color: 0xb8c3d4,
      metalness: 0.62,
      roughness: 0.28,
      emissive: 0x152030,
      emissiveIntensity: 0.35,
    });
    const bus = new THREE.Mesh(
      new THREE.BoxGeometry(def.size * 1.05, def.size * 0.46, def.size * 0.5),
      busMaterial,
    );
    group.add(bus);

    const serviceRing = new THREE.Mesh(
      new THREE.CylinderGeometry(
        def.size * 0.23,
        def.size * 0.23,
        def.size * 0.58,
        14,
      ),
      new THREE.MeshStandardMaterial({
        color: 0xd5dbe6,
        metalness: 0.54,
        roughness: 0.34,
      }),
    );
    serviceRing.rotation.x = Math.PI / 2;
    group.add(serviceRing);

    const panelMat = new THREE.MeshStandardMaterial({
      color: def.color,
      emissive: new THREE.Color(def.color).multiplyScalar(0.52),
      metalness: 0.74,
      roughness: 0.32,
    });
    const panelWing = new THREE.BoxGeometry(
      def.size * 1.85,
      def.size * 0.06,
      def.size * 0.64,
    );
    const panelFrameMat = new THREE.MeshStandardMaterial({
      color: 0x7f94ac,
      metalness: 0.5,
      roughness: 0.4,
    });
    const panelFrame = new THREE.BoxGeometry(
      def.size * 1.9,
      def.size * 0.02,
      def.size * 0.72,
    );
    const panelLeft = new THREE.Mesh(panelWing, panelMat);
    const panelLeftFrame = new THREE.Mesh(panelFrame, panelFrameMat);
    panelLeft.position.x = -def.size * 1.22;
    panelLeftFrame.position
      .copy(panelLeft.position)
      .setY(panelLeft.position.y + def.size * 0.02);
    const panelRight = panelLeft.clone();
    const panelRightFrame = panelLeftFrame.clone();
    panelRight.position.x = def.size * 1.22;
    panelRightFrame.position.x = def.size * 1.22;
    group.add(panelLeft, panelLeftFrame, panelRight, panelRightFrame);

    const antennaMast = new THREE.Mesh(
      new THREE.CylinderGeometry(
        def.size * 0.045,
        def.size * 0.045,
        def.size * 0.44,
        10,
      ),
      new THREE.MeshStandardMaterial({
        color: 0xd0d8e5,
        metalness: 0.48,
        roughness: 0.31,
      }),
    );
    antennaMast.position.set(0, def.size * 0.28, def.size * 0.08);
    group.add(antennaMast);

    const dish = new THREE.Mesh(
      new THREE.CylinderGeometry(
        def.size * 0.03,
        def.size * 0.28,
        def.size * 0.34,
        18,
        1,
      ),
      new THREE.MeshStandardMaterial({
        color: 0xe5e7eb,
        metalness: 0.56,
        roughness: 0.24,
      }),
    );
    dish.rotation.x = Math.PI / 2;
    dish.position.set(0, def.size * 0.28, def.size * 0.39);
    group.add(dish);

    const sensorPod = new THREE.Mesh(
      new THREE.SphereGeometry(def.size * 0.11, 10, 10),
      new THREE.MeshStandardMaterial({
        color: 0x9fb4d0,
        emissive: 0x1a3350,
        emissiveIntensity: 0.6,
        roughness: 0.3,
        metalness: 0.42,
      }),
    );
    sensorPod.position.set(0, 0, def.size * 0.35);
    group.add(sensorPod);

    const navBeacon = new THREE.Mesh(
      new THREE.SphereGeometry(def.size * 0.055, 8, 8),
      new THREE.MeshBasicMaterial({ color: 0x93c5fd }),
    );
    navBeacon.position.set(0, def.size * 0.32, -def.size * 0.28);
    group.add(navBeacon);

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
      clear: {
        color: 0x38bdf8,
        opacity: 0.58,
        dashed: false,
      },
      error: {
        color: 0xef4444,
        opacity: 0.76,
        dashed: false,
      },
      thickness: 0.012,
    });
    const linkToRover = this.createBeam({
      clear: {
        color: 0x22c55e,
        opacity: 0.64,
        dashed: false,
      },
      error: {
        color: 0xef4444,
        opacity: 0.82,
        dashed: true,
      },
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

    const dashed = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineDashedMaterial({
        color: orbitColor,
        transparent: true,
        opacity: 0.58,
        dashSize: 0.11 + index * 0.02,
        gapSize: 0.07 + index * 0.015,
      }),
    );
    dashed.computeLineDistances();
    return dashed;
  }

  createBeam({ clear, error, thickness }) {
    const clearStyle = {
      color: clear?.color ?? 0x38bdf8,
      opacity: clear?.opacity ?? 0.6,
      dashed: Boolean(clear?.dashed),
      dashSize: clear?.dashSize ?? 0.11 + thickness * 6.5,
      gapSize: clear?.gapSize ?? 0.06 + thickness * 4.5,
    };
    const errorStyle = {
      color: error?.color ?? 0xef4444,
      opacity: error?.opacity ?? 0.8,
      dashed: Boolean(error?.dashed),
      dashSize: error?.dashSize ?? 0.11 + thickness * 6.5,
      gapSize: error?.gapSize ?? 0.06 + thickness * 4.5,
    };

    const beam = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(),
        new THREE.Vector3(0, 0.001, 0),
      ]),
      this.createBeamMaterial(clearStyle),
    );
    if (clearStyle.dashed) beam.computeLineDistances();
    beam.userData = {
      style: {
        clear: clearStyle,
        error: errorStyle,
      },
      activeStyle: "clear",
      blocked: false,
    };
    beam.visible = false;
    return beam;
  }

  createInterSatelliteLinks() {
    this.interSatelliteLinks = [];
    for (let i = 0; i < this.satellites.length; i += 1) {
      const next = (i + 1) % this.satellites.length;
      const link = {
        a: i,
        b: next,
        beam: this.createBeam({
          clear: {
            color: 0x3b82f6,
            opacity: 0.42,
            dashed: true,
            dashSize: 0.13,
            gapSize: 0.09,
          },
          error: {
            color: 0xb45309,
            opacity: 0.54,
            dashed: true,
            dashSize: 0.13,
            gapSize: 0.09,
          },
          thickness: 0.0062,
        }),
      };
      this.interSatelliteLinks.push(link);
      this.rootGroup.add(link.beam);
    }
  }

  createBeamMaterial(style) {
    if (style.dashed) {
      return new THREE.LineDashedMaterial({
        color: style.color,
        transparent: true,
        opacity: style.opacity,
        dashSize: style.dashSize,
        gapSize: style.gapSize,
      });
    }

    return new THREE.LineBasicMaterial({
      color: style.color,
      transparent: true,
      opacity: style.opacity,
    });
  }

  applyBeamStyle(beam, styleKey) {
    if (!beam || !beam.userData?.style) return;
    const style = beam.userData.style[styleKey];
    if (!style) return;

    const existing = beam.material;
    const isDashed = Boolean(
      existing && existing.type === "LineDashedMaterial",
    );
    if (isDashed !== style.dashed) {
      if (existing && typeof existing.dispose === "function")
        existing.dispose();
      beam.material = this.createBeamMaterial(style);
    } else {
      beam.material.color.setHex(style.color);
      beam.material.opacity = style.opacity;
      if (style.dashed) {
        beam.material.dashSize = style.dashSize;
        beam.material.gapSize = style.gapSize;
      }
      beam.material.needsUpdate = true;
    }

    if (style.dashed) beam.computeLineDistances();
    beam.userData.activeStyle = styleKey;
  }

  setBeamBlockedState(beam, blocked) {
    if (!beam || !beam.material || !beam.userData?.style) return;
    const styleKey = blocked ? "error" : "clear";
    if (
      beam.userData.blocked === blocked &&
      beam.userData.activeStyle === styleKey
    )
      return;
    beam.userData.blocked = blocked;
    this.applyBeamStyle(beam, styleKey);
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
    if (allowEndSurfaceTouch && t > 1 - this.LOS_ENDPOINT_TOLERANCE)
      return false;

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
    beam.geometry.setFromPoints([start, end]);
    this.setBeamBlockedState(beam, blocked);
    if (beam.material && beam.material.type === "LineDashedMaterial") {
      beam.computeLineDistances();
    }
  }

  upsertFleetSnapshot(fleet) {
    Object.values(fleet || {}).forEach((entry) =>
      this.upsertRoverFromTelemetry(entry),
    );
  }

  upsertRoverFromTelemetry(data) {
    const roverId = data?.rover_id;
    if (!roverId) return;
    if (data?.celestial) this.setCelestialFrame(data.celestial);

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

    if (
      data.position &&
      Number.isFinite(Number(data.position.lat)) &&
      Number.isFinite(Number(data.position.lon))
    ) {
      if (rover.targetLat === undefined) {
        rover.lat = Number(data.position.lat);
        rover.lon = Number(data.position.lon);
        this.placeRover(rover);
      }
      rover.targetLat = Number(data.position.lat);
      rover.targetLon = Number(data.position.lon);
    }

    this.updateRoverStateStyle(rover);
    this.updateMetaText();
  }

  createRover(roverId) {
    const group = new THREE.Group();
    const fallbackGroup = new THREE.Group();
    group.add(fallbackGroup);

    const fallbackMaterial = new THREE.MeshStandardMaterial({
      color: 0x34d399,
      emissive: 0x0a1f16,
      roughness: 0.46,
      metalness: 0.28,
    });

    const chassisDeck = new THREE.Mesh(
      new THREE.BoxGeometry(0.033, 0.008, 0.022),
      fallbackMaterial,
    );
    chassisDeck.position.y = 0.005;
    fallbackGroup.add(chassisDeck);

    const fallbackMesh = new THREE.Mesh(
      new THREE.BoxGeometry(0.03, 0.008, 0.02),
      fallbackMaterial,
    );
    fallbackGroup.add(fallbackMesh);

    const panel = new THREE.Mesh(
      new THREE.BoxGeometry(0.024, 0.002, 0.026),
      new THREE.MeshStandardMaterial({
        color: 0x6aa8dc,
        emissive: 0x143150,
        emissiveIntensity: 0.38,
        roughness: 0.42,
        metalness: 0.62,
      }),
    );
    panel.position.y = 0.011;
    fallbackGroup.add(panel);

    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0016, 0.0016, 0.012, 8),
      new THREE.MeshStandardMaterial({
        color: 0xc8d2df,
        metalness: 0.46,
        roughness: 0.36,
      }),
    );
    mast.position.set(0.006, 0.016, 0.002);
    fallbackGroup.add(mast);

    const cameraHead = new THREE.Mesh(
      new THREE.BoxGeometry(0.008, 0.005, 0.006),
      new THREE.MeshStandardMaterial({
        color: 0xcdd9e8,
        metalness: 0.44,
        roughness: 0.33,
      }),
    );
    cameraHead.position.set(0.006, 0.022, 0.002);
    fallbackGroup.add(cameraHead);

    const wheelGeo = new THREE.CylinderGeometry(0.0048, 0.0048, 0.0034, 12);
    const wheelMat = new THREE.MeshStandardMaterial({
      color: 0x4a5868,
      roughness: 0.84,
      metalness: 0.1,
    });
    const wheelOffsets = [
      [-0.012, 0.0, -0.011],
      [0.0, 0.0, -0.011],
      [0.012, 0.0, -0.011],
      [-0.012, 0.0, 0.011],
      [0.0, 0.0, 0.011],
      [0.012, 0.0, 0.011],
    ];
    wheelOffsets.forEach(([x, y, z]) => {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat);
      wheel.rotation.z = Math.PI / 2;
      wheel.position.set(x, y, z);
      fallbackGroup.add(wheel);
    });

    const beaconMaterial = new THREE.MeshBasicMaterial({ color: 0x34d399 });
    const beacon = new THREE.Mesh(
      new THREE.SphereGeometry(0.0048, 10, 10),
      beaconMaterial,
    );
    beacon.position.set(0, 0.027, 0);
    fallbackGroup.add(beacon);

    const execRing = new THREE.Mesh(
      new THREE.TorusGeometry(0.018, 0.002, 8, 24),
      new THREE.MeshBasicMaterial({
        color: 0x3b82f6,
        transparent: true,
        opacity: 0,
      }),
    );
    execRing.rotation.x = Math.PI / 2;
    execRing.position.y = 0.005;
    group.add(execRing);

    const rover = {
      id: roverId,
      group,
      fallbackGroup,
      fallbackMesh,
      fallbackMaterial,
      beaconMaterial,
      execRing,
      execPhase: 0,
      modelMesh: null,
      modelMaterial: null,
      lat: -43.3,
      lon: -11.2,
      targetLat: undefined,
      targetLon: undefined,
      battery: 1.0,
      state: "IDLE",
      staticCollider: {
        id: roverId,
        type: "rover",
        radius: 0.028,
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
      rover.group.scale.setScalar(selected ? 1.25 : 1.12);
      rover.group.traverse((obj) => {
        if (!obj.isMesh || !obj.material) return;
        if (!Object.prototype.hasOwnProperty.call(obj.material, "emissive"))
          return;
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
    const next = "orbital";
    this.viewMode = next;
    this.resetView(false);
    this.updateCamera(true);
    this.updateMetaText();
    if (emitEvent) this.bus.emit("viz:view-mode", { mode: next });
  }

  resetView(emitEvent = true) {
    this.cameraDistance = this.defaultOrbitalDistance;
    this.orbitalDistance = this.cameraDistance;
    this.cameraAzimuth = this.defaultCameraAzimuth;
    this.cameraPolar = this.defaultCameraPolar;
    this.cameraTargetDesired
      .copy(this.earthAnchorOrbital)
      .multiplyScalar(this.defaultEarthFrameFactor);
    this.cameraTargetDesired.y *= 0.42;
    this.cameraTarget.copy(this.cameraTargetDesired);
    this.updateCamera(true);
    this.syncNavigationTelemetry();
    if (emitEvent) this.bus.emit("viz:navigation", { action: "reset" });
  }

  focusSelectedRover() {
    if (!this.selectedRoverId || !this.rovers.has(this.selectedRoverId)) return;
    const selected = this.rovers.get(this.selectedRoverId);
    this.cameraTargetDesired.copy(selected.group.position);
    this.cameraTarget.copy(this.cameraTargetDesired);
    this.cameraDistance = THREE.MathUtils.clamp(
      this.cameraDistance,
      this.cameraMinDistance,
      5.8,
    );
    this.cameraPolar = THREE.MathUtils.clamp(
      this.cameraPolar,
      0.2,
      Math.PI - 0.2,
    );
    this.orbitalDistance = this.cameraDistance;
    this.updateCamera(true);
    this.syncNavigationTelemetry();
    this.bus.emit("viz:navigation", {
      action: "focus-selected",
      rover_id: this.selectedRoverId,
    });
  }

  setTopView() {
    this.cameraPolar = 0.16;
    this.cameraAzimuth = 0.0;
    if (this.selectedRoverId && this.rovers.has(this.selectedRoverId)) {
      this.cameraTargetDesired.copy(
        this.rovers.get(this.selectedRoverId).group.position,
      );
      this.cameraTarget.copy(this.cameraTargetDesired);
    }
    this.updateCamera(true);
    this.syncNavigationTelemetry();
    this.bus.emit("viz:navigation", { action: "top-view" });
  }

  panCamera(dx, dy) {
    this.tmpVecA.copy(this.camera.position).sub(this.cameraTarget).normalize();
    this.tmpVecB.crossVectors(this.camera.up, this.tmpVecA).normalize();
    this.tmpVecC.crossVectors(this.tmpVecA, this.tmpVecB).normalize();

    const panScale = this.cameraDistance * 0.0019;
    this.cameraTargetDesired.addScaledVector(this.tmpVecB, -dx * panScale);
    this.cameraTargetDesired.addScaledVector(this.tmpVecC, dy * panScale);
    this.cameraTarget.copy(this.cameraTargetDesired);
  }

  formatDurationMs(durationMs) {
    const totalSeconds = Math.max(0, durationMs) / 1000;
    if (totalSeconds < 60) {
      const precision = totalSeconds < 10 ? 1 : 0;
      return `${totalSeconds.toFixed(precision)}s`;
    }

    const totalWholeSeconds = Math.floor(totalSeconds);
    const hours = Math.floor(totalWholeSeconds / 3600);
    const minutes = Math.floor((totalWholeSeconds % 3600) / 60);
    const seconds = totalWholeSeconds % 60;
    if (hours > 0) return `${hours}h ${minutes}m ${seconds}s`;
    return `${minutes}m ${seconds}s`;
  }

  updateSignalLossTracking(linkStatus, nowMs = performance.now()) {
    const hasNoLineOfSight = Boolean(linkStatus.hasIssue);
    const tracker = this.signalLossState;
    let transitioned = false;

    if (hasNoLineOfSight) {
      if (!tracker.active) {
        tracker.active = true;
        tracker.startedAtMs = nowMs;
        tracker.currentDurationMs = 0;
        tracker.outageCount += 1;
        transitioned = true;
        const reason =
          linkStatus.issueReason || "Issue: no line-of-sight contact";
        this.bus.emit("log", {
          tag: "fault",
          text: `LOS outage started: ${reason}`,
        });
      } else {
        tracker.currentDurationMs = Math.max(0, nowMs - tracker.startedAtMs);
      }
      return transitioned;
    }

    if (tracker.active) {
      const durationMs = Math.max(0, nowMs - tracker.startedAtMs);
      tracker.active = false;
      tracker.startedAtMs = 0;
      tracker.currentDurationMs = 0;
      tracker.lastDurationMs = durationMs;
      transitioned = true;
      this.bus.emit("log", {
        tag: "system",
        text: `LOS restored after ${this.formatDurationMs(durationMs)}`,
      });
    }

    return transitioned;
  }

  updateSignalLossUi(nowMs = performance.now()) {
    const tracker = this.signalLossState;
    const lossValueEl = document.getElementById("lunar-kpi-loss");
    const lossCardEl = document.getElementById("lunar-kpi-loss-card");
    const stripValueEl = document.getElementById("orbital-last-loss-duration");
    const metaEl = document.getElementById("lunar-meta");

    let activeDurationMs = tracker.currentDurationMs;
    if (tracker.active) {
      activeDurationMs = Math.max(0, nowMs - tracker.startedAtMs);
      tracker.currentDurationMs = activeDurationMs;
    }

    if (lossCardEl) {
      lossCardEl.classList.remove("is-good", "is-alert");
      if (tracker.active) lossCardEl.classList.add("is-alert");
      else if (tracker.lastDurationMs !== null)
        lossCardEl.classList.add("is-good");
    }

    if (tracker.active) {
      const activeText = `Active ${this.formatDurationMs(activeDurationMs)}`;
      if (lossValueEl) lossValueEl.textContent = activeText;
      if (stripValueEl) {
        stripValueEl.textContent = `LOS outage active: ${this.formatDurationMs(activeDurationMs)}`;
      }
      if (metaEl) {
        const reason =
          this.linkStatus.issueReason || "Issue: no line-of-sight contact";
        metaEl.classList.add("warning");
        metaEl.textContent = `${reason} · Outage ${this.formatDurationMs(activeDurationMs)}`;
      }
      return;
    }

    if (tracker.lastDurationMs !== null) {
      const lastText = `Last ${this.formatDurationMs(tracker.lastDurationMs)}`;
      if (lossValueEl) lossValueEl.textContent = lastText;
      if (stripValueEl) {
        stripValueEl.textContent = `Last LOS outage: ${this.formatDurationMs(tracker.lastDurationMs)}`;
      }
      return;
    }

    if (lossValueEl) lossValueEl.textContent = "--";
    if (stripValueEl) stripValueEl.textContent = "Last LOS outage: --";
  }

  updateCamera(force = false) {
    if (!this.camera) return;
    this.camera.fov = 35;
    this.cameraDistance = THREE.MathUtils.clamp(
      this.cameraDistance,
      this.cameraMinDistance,
      this.cameraMaxDistance,
    );
    this.orbitalDistance = this.cameraDistance;
    this.cameraPolar = THREE.MathUtils.clamp(
      this.cameraPolar,
      0.08,
      Math.PI - 0.08,
    );

    const sinPolar = Math.sin(this.cameraPolar);
    this.tmpVecA.set(
      this.cameraDistance * sinPolar * Math.sin(this.cameraAzimuth),
      this.cameraDistance * Math.cos(this.cameraPolar),
      this.cameraDistance * sinPolar * Math.cos(this.cameraAzimuth),
    );
    this.camera.position.copy(this.cameraTarget).add(this.tmpVecA);
    this.camera.lookAt(this.cameraTarget);

    if (!this.earthAnchor.equals(this.earthAnchorOrbital)) {
      this.earthAnchor.copy(this.earthAnchorOrbital);
      if (this.earthBase) this.earthBase.position.copy(this.earthAnchor);
    }
    if (this.earthBase) this.earthBase.scale.setScalar(1.28);

    if (force) this.camera.updateProjectionMatrix();
  }

  syncNavigationTelemetry() {
    if (!this.camera) return;

    const headingValueEl = document.getElementById("orbital-nav-heading");
    const pitchValueEl = document.getElementById("orbital-nav-pitch");
    const distanceValueEl = document.getElementById("orbital-nav-distance");
    const earthMoonDistanceEl = document.getElementById(
      "orbital-earth-moon-distance",
    );
    const roverEarthAngleEl = document.getElementById(
      "orbital-rover-earth-angle",
    );
    const roverSideEl = document.getElementById("orbital-rover-side");
    const roverCoordsEl = document.getElementById("orbital-rover-coords");
    const earthBearingEl = document.getElementById("orbital-earth-bearing");
    const compassNeedleEl = document.getElementById("orbital-compass-needle");

    this.camera.getWorldDirection(this.tmpVecA).normalize();
    const pitchDeg = Math.round(
      THREE.MathUtils.radToDeg(
        Math.asin(THREE.MathUtils.clamp(this.tmpVecA.y, -1, 1)),
      ),
    );
    this.tmpVecB.copy(this.tmpVecA);
    this.tmpVecB.y = 0;
    let headingDeg = 0;
    if (this.tmpVecB.lengthSq() > 0.0000001) {
      this.tmpVecB.normalize();
      headingDeg =
        (THREE.MathUtils.radToDeg(Math.atan2(this.tmpVecB.x, this.tmpVecB.z)) +
          360) %
        360;
    }

    if (headingValueEl)
      headingValueEl.textContent = `HDG ${String(Math.round(headingDeg)).padStart(3, "0")}°`;
    if (pitchValueEl)
      pitchValueEl.textContent = `PITCH ${pitchDeg >= 0 ? "+" : ""}${pitchDeg}°`;
    if (distanceValueEl)
      distanceValueEl.textContent = `RANGE ${this.cameraDistance.toFixed(2)} LU`;
    if (earthMoonDistanceEl)
      earthMoonDistanceEl.textContent = `Earth-Moon: ${this.earthAnchor.length().toFixed(2)} LU`;

    if (compassNeedleEl) {
      compassNeedleEl.style.transform = `rotate(${headingDeg}deg)`;
    }

    const selectedRover =
      this.selectedRoverId && this.rovers.has(this.selectedRoverId)
        ? this.rovers.get(this.selectedRoverId)
        : null;

    if (!selectedRover) {
      if (roverEarthAngleEl)
        roverEarthAngleEl.textContent = "Rover→Earth angle: --";
      if (roverSideEl) roverSideEl.textContent = "Lunar side: --";
      if (roverCoordsEl) roverCoordsEl.textContent = "Lat -- · Lon --";
      if (earthBearingEl) earthBearingEl.textContent = "--";
      return;
    }

    const roverPos = selectedRover.group.position;
    this.tmpVecC.copy(roverPos).normalize();
    this.tmpVecD.copy(this.earthAnchor).normalize();
    const roverEarthAngle = THREE.MathUtils.radToDeg(
      THREE.MathUtils.clamp(this.tmpVecC.angleTo(this.tmpVecD), 0, Math.PI),
    );
    const onNearSide = roverEarthAngle <= 90;

    if (roverEarthAngleEl) {
      roverEarthAngleEl.textContent = `Rover→Earth angle: ${roverEarthAngle.toFixed(1)}°`;
    }
    if (roverSideEl) {
      roverSideEl.textContent = `Lunar side: ${onNearSide ? "Nearside" : "Farside"}`;
    }
    if (roverCoordsEl) {
      roverCoordsEl.textContent = `Lat ${Number(selectedRover.lat).toFixed(2)}° · Lon ${Number(selectedRover.lon).toFixed(2)}°`;
    }

    const worldUp = new THREE.Vector3(0, 1, 0);
    this.tmpVecA.copy(worldUp).cross(this.tmpVecC);
    if (this.tmpVecA.lengthSq() < 0.000001) {
      this.tmpVecA.set(1, 0, 0);
    } else {
      this.tmpVecA.normalize();
    }
    this.tmpVecB.crossVectors(this.tmpVecC, this.tmpVecA).normalize();
    this.tmpVecD.copy(this.earthAnchor).sub(roverPos).normalize();
    const bearingRad = Math.atan2(
      this.tmpVecD.dot(this.tmpVecA),
      this.tmpVecD.dot(this.tmpVecB),
    );
    const bearingDeg = (THREE.MathUtils.radToDeg(bearingRad) + 360) % 360;
    if (earthBearingEl) {
      earthBearingEl.textContent = `${bearingDeg.toFixed(0)}° from north`;
    }
  }

  updateMetaText() {
    const metaEl = document.getElementById("lunar-meta");
    const roverValueEl = document.getElementById("lunar-kpi-rover");
    const losValueEl = document.getElementById("lunar-kpi-los");
    const linksValueEl = document.getElementById("lunar-kpi-links");
    const zoomValueEl = document.getElementById("lunar-kpi-zoom");
    const fleetValueEl = document.getElementById("lunar-kpi-fleet");
    const losCardEl = document.getElementById("lunar-kpi-los-card");
    const linksCardEl = document.getElementById("lunar-kpi-links-card");
    const roverCardEl = document.getElementById("lunar-kpi-rover-card");
    const lossCardEl = document.getElementById("lunar-kpi-loss-card");

    const setCardState = (cardEl, state) => {
      if (!cardEl) return;
      cardEl.classList.remove("is-good", "is-alert");
      if (state === "good") cardEl.classList.add("is-good");
      else if (state === "alert") cardEl.classList.add("is-alert");
    };

    const selectedRover =
      this.selectedRoverId && this.rovers.has(this.selectedRoverId)
        ? this.rovers.get(this.selectedRoverId)
        : null;
    const selectedLabel = selectedRover
      ? selectedRover.id.toUpperCase()
      : "ROVER --";
    const roverCount = this.rovers.size;
    const total = this.linkStatus.total;
    const earthTotal = this.linkStatus.earthTotal;
    const earthClear = this.linkStatus.earthClear;
    const roverTotal = this.linkStatus.roverTotal;
    const roverClear = this.linkStatus.roverClear;
    const hasIssue = Boolean(this.linkStatus.hasIssue);
    const zoomFactor = (
      this.defaultOrbitalDistance / Math.max(this.orbitalDistance, 0.2)
    ).toFixed(1);

    if (roverValueEl) roverValueEl.textContent = selectedLabel;
    if (linksValueEl) {
      linksValueEl.textContent =
        total > 0
          ? `E ${earthClear}/${earthTotal} · R ${roverClear}/${roverTotal}`
          : "E 0/0 · R 0/0";
    }
    if (zoomValueEl) zoomValueEl.textContent = `${zoomFactor}x`;
    if (fleetValueEl) fleetValueEl.textContent = String(roverCount);

    if (total <= 0) {
      if (losValueEl) losValueEl.textContent = "Pending";
      setCardState(losCardEl, null);
      setCardState(linksCardEl, null);
      if (lossCardEl) lossCardEl.classList.remove("is-good", "is-alert");
      if (metaEl) {
        metaEl.classList.remove("warning");
        metaEl.textContent =
          "Awaiting stable comm links · LMB Orbit · Shift+LMB Pan · Wheel Zoom";
      }
      this.syncNavigationTelemetry();
      this.updateSignalLossUi();
      return;
    }

    if (hasIssue) {
      if (losValueEl) losValueEl.textContent = "Issue";
      setCardState(losCardEl, "alert");
      setCardState(linksCardEl, "alert");
      if (metaEl) {
        metaEl.classList.add("warning");
        metaEl.textContent =
          this.linkStatus.issueReason ||
          "Issue detected · Check relay geometry";
      }
    } else {
      if (losValueEl) losValueEl.textContent = "Nominal";
      setCardState(losCardEl, "good");
      setCardState(linksCardEl, "good");
      if (metaEl) {
        metaEl.classList.remove("warning");
        metaEl.textContent = `Nominal network geometry · Earth ${earthClear}/${earthTotal} · Rover ${roverClear}/${roverTotal}`;
      }
    }

    if (selectedRover && metaEl) {
      const lat = Number(selectedRover.lat).toFixed(2);
      const lon = Number(selectedRover.lon).toFixed(2);
      metaEl.textContent += ` · Focus ${selectedLabel} (${lat}, ${lon})`;
      setCardState(roverCardEl, "good");
    } else {
      setCardState(roverCardEl, null);
    }

    this.syncNavigationTelemetry();
    this.updateSignalLossUi();
  }

  setupInteraction() {
    this.container.addEventListener("contextmenu", (e) => e.preventDefault());
    this.container.style.cursor = "grab";

    this.container.addEventListener("mousedown", (e) => {
      const panGesture =
        e.button === 1 || e.button === 2 || (e.button === 0 && e.shiftKey);
      const orbitGesture = e.button === 0 && !e.shiftKey;
      if (!panGesture && !orbitGesture) return;

      this.dragState.active = true;
      this.dragState.button = e.button;
      this.dragState.mode = panGesture ? "pan" : "orbit";
      this.dragState.x = e.clientX;
      this.dragState.y = e.clientY;
      this.container.style.cursor = panGesture ? "grabbing" : "move";
      e.preventDefault();
    });

    window.addEventListener("mouseup", () => {
      this.dragState.active = false;
      this.container.style.cursor = "grab";
    });

    window.addEventListener("mousemove", (e) => {
      if (!this.dragState.active || !this.camera) return;
      const dx = e.clientX - this.dragState.x;
      const dy = e.clientY - this.dragState.y;
      this.dragState.x = e.clientX;
      this.dragState.y = e.clientY;

      if (this.dragState.mode === "pan") {
        this.panCamera(dx, dy);
      } else {
        this.cameraAzimuth -= dx * 0.006;
        this.cameraPolar += dy * 0.006;
      }

      this.updateCamera();
      this.syncNavigationTelemetry();
    });

    this.container.addEventListener(
      "wheel",
      (e) => {
        e.preventDefault();
        const zoomScale = Math.exp(e.deltaY * 0.0011);
        this.cameraDistance = THREE.MathUtils.clamp(
          this.cameraDistance * zoomScale,
          this.cameraMinDistance,
          this.cameraMaxDistance,
        );
        this.updateCamera();
        this.syncNavigationTelemetry();
      },
      { passive: false },
    );

    this.container.addEventListener("dblclick", () => {
      this.focusSelectedRover();
    });
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
        body.collisionCooldown = Math.max(
          0,
          body.collisionCooldown - clampedDt,
        );
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
      body.velocity.addScaledVector(this.tmpVecA, -(1 + body.restitution) * vN);
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
          (-(1 + restitution) * velAlongNormal) / (1 / a.mass + 1 / b.mass);

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
      text: `Physics collision: ${message}`,
    });
  }

  updateSatelliteLinks() {
    let selectedPos = null;
    if (this.selectedRoverId && this.rovers.has(this.selectedRoverId)) {
      selectedPos = this.rovers.get(this.selectedRoverId).group.position;
    } else if (this.rovers.size > 0) {
      selectedPos = this.rovers.values().next().value.group.position;
    }

    const linkStatus = {
      total: 0,
      clear: 0,
      blocked: 0,
      earthTotal: 0,
      earthClear: 0,
      earthBlocked: 0,
      roverTotal: 0,
      roverClear: 0,
      roverBlocked: 0,
      interTotal: 0,
      interClear: 0,
      interBlocked: 0,
      hasIssue: false,
      issueReason: "",
    };

    this.satellites.forEach((sat) => {
      const satPos = sat.body.position;
      const earthBlocked = this.isLineOfSightBlocked(satPos, this.earthAnchor);
      this.updateBeam(sat.linkToEarth, satPos, this.earthAnchor, earthBlocked);
      linkStatus.total += 1;
      linkStatus.earthTotal += 1;
      if (earthBlocked) {
        linkStatus.blocked += 1;
        linkStatus.earthBlocked += 1;
      } else {
        linkStatus.clear += 1;
        linkStatus.earthClear += 1;
      }

      if (selectedPos) {
        const roverBlocked = this.isLineOfSightBlocked(satPos, selectedPos, {
          allowEndSurfaceTouch: true,
        });
        this.updateBeam(sat.linkToRover, satPos, selectedPos, roverBlocked);
        linkStatus.total += 1;
        linkStatus.roverTotal += 1;
        if (roverBlocked) {
          linkStatus.blocked += 1;
          linkStatus.roverBlocked += 1;
        } else {
          linkStatus.clear += 1;
          linkStatus.roverClear += 1;
        }
      } else {
        sat.linkToRover.visible = false;
      }
    });

    this.interSatelliteLinks.forEach((link) => {
      const satA = this.satellites[link.a];
      const satB = this.satellites[link.b];
      if (!satA || !satB) return;
      const blocked = this.isLineOfSightBlocked(
        satA.body.position,
        satB.body.position,
      );
      this.updateBeam(
        link.beam,
        satA.body.position,
        satB.body.position,
        blocked,
      );
      linkStatus.total += 1;
      linkStatus.interTotal += 1;
      if (blocked) {
        linkStatus.blocked += 1;
        linkStatus.interBlocked += 1;
      } else {
        linkStatus.clear += 1;
        linkStatus.interClear += 1;
      }
    });

    const earthIssue = linkStatus.earthBlocked > 2;
    const roverIssue = linkStatus.roverTotal > 0 && linkStatus.roverClear === 0;
    linkStatus.hasIssue = earthIssue || roverIssue;
    if (earthIssue && roverIssue) {
      linkStatus.issueReason =
        "Issue: Earth relays degraded (>2 lost) and rover unreachable";
    } else if (earthIssue) {
      linkStatus.issueReason = "Issue: More than 2 satellites lost Earth LOS";
    } else if (roverIssue) {
      linkStatus.issueReason = "Issue: Rover unreachable from all satellites";
    }

    const lossTransitioned = this.updateSignalLossTracking(
      linkStatus,
      performance.now(),
    );
    const prev = this.linkStatus;
    const changed = Object.keys(linkStatus).some(
      (k) => linkStatus[k] !== prev[k],
    );
    this.linkStatus = linkStatus;
    if (changed || lossTransitioned) {
      this.bus.emit("orbital:los-status", {
        hasIssue: linkStatus.hasIssue,
        issueReason: linkStatus.issueReason,
        earthClear: linkStatus.earthClear,
        earthTotal: linkStatus.earthTotal,
        roverClear: linkStatus.roverClear,
        roverTotal: linkStatus.roverTotal,
      });
      this.updateMetaText();
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
    const clampedDt = Math.min(Math.max(dt, 0), 0.06);

    this.physicsStep(dt);

    if (this.earthBase) {
      this.earthBase.rotation.y += clampedDt * 0.032;
      if (this.earthAnchorOrbital) {
        this.earthBase.position.lerp(this.earthAnchorOrbital, clampedDt * 2.0);
        this.earthAnchor.copy(this.earthBase.position);
      }
    }
    if (this.earthCloudLayer) {
      this.earthCloudLayer.rotation.y += clampedDt * 0.065;
    }
    if (this.sunSurface) {
      this.sunSurface.rotation.y += clampedDt * 0.018;
    }
    if (this.sunGroup && this.sunAnchorOrbital) {
      this.sunGroup.position.lerp(this.sunAnchorOrbital, clampedDt * 2.0);
      if (this.sunLight) {
        this.sunLight.position.copy(this.sunGroup.position);
      }
    }

    this.rovers.forEach((rover) => {
      // Smoothly interpolate rover positions
      if (rover.targetLat !== undefined && rover.targetLon !== undefined) {
        const dLat = rover.targetLat - rover.lat;
        const dLon = rover.targetLon - rover.lon;
        if (Math.abs(dLat) > 1e-6 || Math.abs(dLon) > 1e-6) {
          rover.lat += dLat * clampedDt * 3.0;
          rover.lon += dLon * clampedDt * 3.0;
          this.placeRover(rover);
        }
      }

      // Animate execution ring
      if (rover.state === "EXECUTING" && rover.execRing) {
        rover.execPhase += clampedDt * 1.5;
        const scale = 1 + (rover.execPhase % 1) * 2;
        rover.execRing.scale.set(scale, scale, scale);
        rover.execRing.material.opacity = 0.8 * (1.0 - (rover.execPhase % 1));
      } else if (rover.execRing) {
        rover.execRing.material.opacity = 0;
        rover.execPhase = 0;
      }
    });

    // Animate satellite beam dash offsets to simulate data flow
    if (this.rootGroup) {
      this.rootGroup.traverse((obj) => {
        if (
          obj.isLine &&
          obj.material &&
          obj.material.type === "LineDashedMaterial"
        ) {
          obj.material.dashOffset =
            (obj.material.dashOffset || 0) - clampedDt * 0.8;
        }
      });
    }

    this.updateCamera();
    this.updateSignalLossUi(now);
    this.renderer.render(this.scene, this.camera);
  }
}

window.VisualizationController = VisualizationController;
