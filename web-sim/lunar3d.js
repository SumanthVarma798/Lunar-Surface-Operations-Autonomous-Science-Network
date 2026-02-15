/* ═══════════════════════════════════════════════════════
   LSOAS — 3D Visualization System
   Renders the Moon and Rover using Three.js
   ═══════════════════════════════════════════════════════ */

class VisualizationController {
  constructor(bus, containerId) {
    this.bus = bus;
    this.container = document.getElementById(containerId);
    this.width = this.container.clientWidth;
    this.height = this.container.clientHeight;

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.moonGroup = null;
    this.moon = null;
    this.roverMarker = null;
    this.roverTrail = null;
    this.roverTrailPoints = [];
    this.maxTrailPoints = 140;

    this.init();

    // Listen for telemetry to update rover position
    this.bus.on("telemetry:display", (data) => this.updateRover(data));

    // Handle resize
    window.addEventListener("resize", () => this.onResize());
  }

  init() {
    // 1. Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000000); // Space black
    this.moonGroup = new THREE.Group();
    this.scene.add(this.moonGroup);

    // 2. Camera
    this.camera = new THREE.PerspectiveCamera(
      40,
      this.width / this.height,
      0.1,
      1000,
    );
    // Keep the globe centered while still slightly zoomed in.
    this.camera.position.set(0, 0, 2.65);

    // 3. Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setSize(this.width, this.height);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.container.appendChild(this.renderer.domElement);

    // 4. Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.2);
    this.scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xffffff, 2.0);
    sunLight.position.set(5, 3, 5);
    this.scene.add(sunLight);

    // 5. Moon Mesh
    this.createMoon();

    // 6. Rover Marker
    this.createRover();

    // 7. Stars/Background (Optional but nice)
    this.createStars();

    // 8. Orbit Controls (Manual implementation for simple rotation)
    // For full OrbitControls we'd need the addon, but we can do simple mouse drag.
    this.setupInteraction();

    // Start Loop
    this.animate();
  }

  createMoon() {
    const geometry = new THREE.SphereGeometry(1, 64, 64);

    const textureLoader = new THREE.TextureLoader();
    const colorMap = textureLoader.load(
      "https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/moon_1024.jpg",
    );
    const displacementMap = textureLoader.load(
      "https://raw.githubusercontent.com/mrdoob/three.js/master/examples/textures/planets/moon_1024.jpg", // Using same for displacement as placeholder if cleaner one not found
    );

    const material = new THREE.MeshStandardMaterial({
      map: colorMap,
      displacementMap: displacementMap,
      displacementScale: 0.05,
      roughness: 0.8,
    });

    this.moon = new THREE.Mesh(geometry, material);
    this.moonGroup.add(this.moon);
  }

  createRover() {
    // Marker is a small cone pointing out from surface
    const geometry = new THREE.ConeGeometry(0.02, 0.08, 16);
    geometry.rotateX(Math.PI / 2); // Point outward
    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 }); // Start green

    this.roverMarker = new THREE.Mesh(geometry, material);
    this.moonGroup.add(this.roverMarker);

    const trailMaterial = new THREE.LineBasicMaterial({
      color: 0x22d3ee,
      transparent: true,
      opacity: 0.75,
    });
    const trailGeometry = new THREE.BufferGeometry().setFromPoints([]);
    this.roverTrail = new THREE.Line(trailGeometry, trailMaterial);
    this.moonGroup.add(this.roverTrail);

    // Initial position update
    this.setLatLon(0, 0); // Default, will update on first telemetry
  }

  createStars() {
    const geometry = new THREE.BufferGeometry();
    const vertices = [];
    for (let i = 0; i < 1000; i++) {
      vertices.push(
        THREE.MathUtils.randFloatSpread(200),
        THREE.MathUtils.randFloatSpread(200),
        THREE.MathUtils.randFloatSpread(200),
      );
    }
    geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(vertices, 3),
    );
    const particles = new THREE.Points(
      geometry,
      new THREE.PointsMaterial({ color: 0x888888, size: 0.5 }),
    );
    this.scene.add(particles);
  }

  setLatLon(lat, lon) {
    if (!this.roverMarker) return;

    // Convert lat/lon to 3D position on sphere radius 1.
    const phi = (90 - lat) * (Math.PI / 180);
    const theta = (lon + 180) * (Math.PI / 180);

    const x = -(Math.sin(phi) * Math.cos(theta));
    const z = Math.sin(phi) * Math.sin(theta);
    const y = Math.cos(phi);

    // Position slightly above surface (radius 1 + displacement)
    const radius = 1.05;
    this.roverMarker.position.set(x * radius, y * radius, z * radius);

    // Look away from center
    this.roverMarker.lookAt(new THREE.Vector3(x * 2, y * 2, z * 2));

    this.appendTrailPoint(new THREE.Vector3(x * radius, y * radius, z * radius));
  }

  appendTrailPoint(point) {
    if (!this.roverTrail) return;

    const prev = this.roverTrailPoints[this.roverTrailPoints.length - 1];
    if (prev && prev.distanceTo(point) < 0.0035) return;

    this.roverTrailPoints.push(point.clone());
    if (this.roverTrailPoints.length > this.maxTrailPoints) {
      this.roverTrailPoints.shift();
    }

    this.roverTrail.geometry.setFromPoints(this.roverTrailPoints);
  }

  updateRover(data) {
    if (data.position) {
      this.setLatLon(data.position.lat, data.position.lon);
    }

    // Update color based on state
    if (this.roverMarker) {
      let color = 0x34d399; // IDLE - green
      if (data.state === "EXECUTING")
        color = 0x3b82f6; // Blue
      else if (data.state === "SAFE_MODE")
        color = 0xfbbf24; // Amber
      else if (data.state === "ERROR") color = 0xf87171; // Red

      this.roverMarker.material.color.setHex(color);
    }
  }

  setupInteraction() {
    let isDragging = false;
    let previousMousePosition = { x: 0, y: 0 };

    this.container.addEventListener("mousedown", (e) => {
      isDragging = true;
    });

    this.container.addEventListener("mousemove", (e) => {
      if (isDragging) {
        const deltaMove = {
          x: e.offsetX - previousMousePosition.x,
          y: e.offsetY - previousMousePosition.y,
        };

        // Rotate moon
        if (this.moonGroup) {
          this.moonGroup.rotation.y += deltaMove.x * 0.005;
          this.moonGroup.rotation.x += deltaMove.y * 0.005;
        }
      }
      previousMousePosition = {
        x: e.offsetX,
        y: e.offsetY,
      };
    });

    this.container.addEventListener("mouseup", () => {
      isDragging = false;
    });

    this.container.addEventListener("wheel", (e) => {
      // Simple zoom
      this.camera.position.z += e.deltaY * 0.001;
      this.camera.position.z = Math.max(
        1.6,
        Math.min(this.camera.position.z, 7.5),
      );
    });
  }

  onResize() {
    this.width = this.container.clientWidth;
    this.height = this.container.clientHeight;

    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(this.width, this.height);
  }

  animate() {
    requestAnimationFrame(() => this.animate());

    // Slow auto-rotation if not interactive (optional)
    // if (this.moon) this.moon.rotation.y += 0.0005;

    this.renderer.render(this.scene, this.camera);
  }
}

// Export
window.VisualizationController = VisualizationController;
