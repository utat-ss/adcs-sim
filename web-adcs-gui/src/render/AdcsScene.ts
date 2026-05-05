import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { MissionSample, MissionTimeline, Vec3 } from "../types";
import { EARTH_RADIUS_KM } from "../physics/constants";
import { makeOrbitPath } from "../physics/orbit";
import { moonDistanceKm, sunDistanceKm } from "../physics/environment";

const KM_TO_SCENE = 1 / 2500;
const EARTH_RADIUS_SCENE = EARTH_RADIUS_KM * KM_TO_SCENE;
const ATMOSPHERE_SCALE = 1.035;
const ORBIT_ALTITUDE_VISUAL_SCALE = 3.2;
const PHASE_RADIUS = 0.012;
const SATELLITE_VISUAL_SCALE = 0.12;
const SATELLITE_MAX_EXTENT_UNSCALED = 0.82;
const SATELLITE_MAX_EXTENT_SCENE = SATELLITE_VISUAL_SCALE * SATELLITE_MAX_EXTENT_UNSCALED;

export class AdcsScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene: THREE.Scene;
  readonly camera: THREE.PerspectiveCamera;
  readonly controls: OrbitControls;

  timeline: MissionTimeline | null = null;
  playing = true;
  timeScale = 55;
  onSample?: (sample: MissionSample) => void;

  private readonly canvas: HTMLCanvasElement;
  private readonly earthGroup = new THREE.Group();
  private readonly orbitGroup = new THREE.Group();
  private readonly satelliteGroup = new THREE.Group();
  private readonly targetGroup = new THREE.Group();
  private readonly sunGroup = new THREE.Group();
  private readonly moonGroup = new THREE.Group();
  private readonly shadowGroup = new THREE.Group();
  private readonly vectorGroup = new THREE.Group();
  private readonly clock = new THREE.Clock();
  private readonly sunLight = new THREE.DirectionalLight(0xffffff, 2.85);
  private readonly ambientLight = new THREE.AmbientLight(0x91a7d6, 0.16);
  private readonly cameraVelocity = new THREE.Vector3();

  private missionSeconds = 0;
  private viewMode: "orbit" | "chase" = "orbit";
  private lastSampleIndex = -1;
  private animationId = 0;
  private earthMesh: THREE.Mesh | null = null;
  private atmosphereMesh: THREE.Mesh | null = null;
  private boresightLine: THREE.Line | null = null;
  private targetLine: THREE.Line | null = null;
  private targetMarker: THREE.Mesh | null = null;
  private groundTrackLine: THREE.Line | null = null;
  private activeMarker: THREE.Mesh | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x020307);
    this.scene.fog = new THREE.FogExp2(0x020307, 0.024);

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false,
      preserveDrawingBuffer: true,
      powerPreference: "high-performance"
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.12;

    this.camera = new THREE.PerspectiveCamera(44, 1, 0.01, 500);
    this.camera.position.set(0, 8.6, 21.5);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.075;
    this.controls.minDistance = EARTH_RADIUS_SCENE * 1.45;
    this.controls.maxDistance = 80;
    this.controls.target.set(0, 0, 0);

    this.scene.add(this.earthGroup, this.orbitGroup, this.satelliteGroup, this.targetGroup, this.sunGroup, this.moonGroup, this.shadowGroup, this.vectorGroup);
    this.scene.add(this.ambientLight, this.sunLight);
    this.scene.add(this.sunLight.target);

    this.buildStaticScene();
    this.resize();
    window.addEventListener("resize", this.resize);
  }

  setTimeline(timeline: MissionTimeline): void {
    this.timeline = timeline;
    this.missionSeconds = 0;
    this.lastSampleIndex = -1;
    clearGroup(this.orbitGroup);
    clearGroup(this.targetGroup);
    clearGroup(this.vectorGroup);
    this.targetLine = null;
    this.targetMarker = null;
    this.groundTrackLine = null;

    const startPath = makeOrbitPath(timeline.config.startOrbit, 420).map(toSceneVec);
    const endPath = makeOrbitPath(timeline.config.endOrbit, 420).map(toSceneVec);
    this.orbitGroup.add(makeLineLoop(startPath, 0x2dd4bf, 0.18));
    this.orbitGroup.add(makeLineLoop(endPath, 0xfb7185, 0.18));
    this.buildPhaseTubes(timeline);
    this.buildGroundTrack(timeline);
    this.buildTargetMarker(timeline.samples[0]);
    this.updateFromSample(timeline.samples[0]);
  }

  setViewMode(mode: "orbit" | "chase"): void {
    this.viewMode = mode;
    this.controls.enabled = mode === "orbit";
    if (mode === "orbit") {
      this.controls.target.set(0, 0, 0);
      this.camera.position.set(0, 8.6, 21.5);
    }
  }

  togglePlaying(): boolean {
    this.playing = !this.playing;
    return this.playing;
  }

  start(): void {
    this.clock.start();
    const animate = () => {
      this.animationId = requestAnimationFrame(animate);
      this.tick(this.clock.getDelta());
    };
    animate();
  }

  stop(): void {
    cancelAnimationFrame(this.animationId);
    window.removeEventListener("resize", this.resize);
  }

  private readonly resize = (): void => {
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width));
    const height = Math.max(1, Math.floor(rect.height));
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  };

  private tick(deltaSeconds: number): void {
    if (this.timeline && this.playing) {
      const duration = this.timeline.config.durationHours * 3600;
      this.missionSeconds = (this.missionSeconds + deltaSeconds * this.timeScale) % duration;
    }

    if (this.timeline) {
      const sample = this.sampleAt(this.missionSeconds);
      this.updateFromSample(sample);
    }

    this.earthGroup.rotation.y += deltaSeconds * 0.02;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  private sampleAt(tSeconds: number): MissionSample {
    if (!this.timeline) throw new Error("Timeline not loaded");
    const samples = this.timeline.samples;
    const duration = this.timeline.config.durationHours * 3600;
    const alpha = Math.max(0, Math.min(1, tSeconds / duration));
    return samples[Math.min(samples.length - 1, Math.max(0, Math.round(alpha * (samples.length - 1))))];
  }

  private updateFromSample(sample: MissionSample): void {
    const position = toSceneVec(sample.state.positionKm);
    const q = sample.attitude.quaternion;
    this.satelliteGroup.position.copy(position);
    this.satelliteGroup.quaternion.set(q[1], q[2], q[3], q[0]);

    if (this.activeMarker) {
      this.activeMarker.position.copy(position);
      (this.activeMarker.material as THREE.MeshBasicMaterial).opacity = sample.environment.eclipseFactor < 0.2 ? 0.38 : 0.78;
    }

    this.updateLighting(sample);
    this.updateTargetLine(sample);

    if (this.viewMode === "chase") {
      const velocity = toSceneDirection(sample.state.velocityKmS);
      const radial = position.clone().normalize();
      const desired = position.clone().add(velocity.multiplyScalar(-1.2)).add(radial.multiplyScalar(0.75));
      this.camera.position.lerp(desired, 0.08);
      this.controls.target.lerp(position, 0.14);
      this.camera.lookAt(position);
    }

    if (sample.index !== this.lastSampleIndex) {
      this.onSample?.(sample);
      this.lastSampleIndex = sample.index;
    }
  }

  private updateLighting(sample: MissionSample): void {
    const sunDirection = toSceneDirection(sample.environment.sunEci);
    this.sunLight.position.copy(sunDirection.clone().multiplyScalar(50));
    this.sunLight.target.position.set(0, 0, 0);
    this.sunGroup.position.copy(sunDirection.clone().multiplyScalar(34));
    this.moonGroup.position.copy(toSceneDirection(sample.environment.moonEci).multiplyScalar(moonDistanceKm() * KM_TO_SCENE * 0.16));

    const antiSun = sunDirection.clone().multiplyScalar(-1);
    this.shadowGroup.position.copy(antiSun.clone().multiplyScalar(5.8));
    this.shadowGroup.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), antiSun);

    if (this.atmosphereMesh) {
      const material = this.atmosphereMesh.material as THREE.ShaderMaterial;
      material.uniforms.sunDirection.value.copy(sunDirection);
    }
  }

  private updateTargetLine(sample: MissionSample): void {
    const sat = toSceneVec(sample.state.positionKm);
    const target = toSceneVec(sample.environment.targetEciKm);
    if (!this.targetLine) {
      const geometry = new THREE.BufferGeometry().setFromPoints([sat, target]);
      this.targetLine = new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({
          color: 0x4ade80,
          transparent: true,
          opacity: 0.5
        })
      );
      this.vectorGroup.add(this.targetLine);
    } else {
      this.targetLine.geometry.setFromPoints([sat, target]);
    }
    const material = this.targetLine.material as THREE.LineBasicMaterial;
    material.opacity = sample.environment.targetVisible ? 0.62 : 0.16;

    if (this.targetMarker) {
      this.targetMarker.position.copy(target);
      this.targetMarker.lookAt(target.clone().multiplyScalar(1.5));
      (this.targetMarker.material as THREE.MeshBasicMaterial).opacity = sample.environment.targetVisible ? 0.78 : 0.34;
    }
  }

  private buildStaticScene(): void {
    this.buildStars();
    this.buildEarth();
    this.buildSatellite();
    this.buildSunMoon();
    this.buildShadow();
  }

  private buildStars(): void {
    const geometry = new THREE.BufferGeometry();
    const vertices: number[] = [];
    const colors: number[] = [];
    let seed = 7;
    const random = () => {
      seed = (seed * 16807) % 2147483647;
      return (seed - 1) / 2147483646;
    };
    for (let i = 0; i < 1900; i += 1) {
      const theta = random() * Math.PI * 2;
      const u = random() * 2 - 1;
      const radius = 95 + random() * 40;
      const f = Math.sqrt(1 - u * u);
      vertices.push(radius * f * Math.cos(theta), radius * u, radius * f * Math.sin(theta));
      const warmth = random();
      colors.push(0.55 + warmth * 0.45, 0.62 + warmth * 0.34, 0.78 + random() * 0.2);
    }
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    this.scene.add(
      new THREE.Points(
        geometry,
        new THREE.PointsMaterial({
          size: 0.035,
          vertexColors: true,
          transparent: true,
          opacity: 0.78,
          depthWrite: false
        })
      )
    );
  }

  private buildEarth(): void {
    const earthGeometry = new THREE.SphereGeometry(EARTH_RADIUS_SCENE, 128, 64);
    const earthMaterial = new THREE.MeshStandardMaterial({
      map: makeEarthTexture(),
      roughness: 0.72,
      metalness: 0.0
    });
    this.earthMesh = new THREE.Mesh(earthGeometry, earthMaterial);
    this.earthGroup.add(this.earthMesh);

    const grid = new THREE.Group();
    for (let lat = -60; lat <= 60; lat += 30) {
      grid.add(makeLatitudeLine(lat));
    }
    for (let lon = 0; lon < 180; lon += 30) {
      grid.add(makeLongitudeLine(lon));
    }
    this.earthGroup.add(grid);

    this.atmosphereMesh = new THREE.Mesh(
      new THREE.SphereGeometry(EARTH_RADIUS_SCENE * ATMOSPHERE_SCALE, 128, 64),
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        side: THREE.BackSide,
        uniforms: {
          sunDirection: { value: new THREE.Vector3(1, 0, 0) }
        },
        vertexShader: `
          varying vec3 vNormal;
          void main() {
            vNormal = normalize(normalMatrix * normal);
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
          }
        `,
        fragmentShader: `
          varying vec3 vNormal;
          uniform vec3 sunDirection;
          void main() {
            float rim = pow(1.0 - abs(vNormal.z), 2.3);
            float sun = clamp(dot(normalize(vNormal), normalize(sunDirection)) * 0.5 + 0.5, 0.15, 1.0);
            gl_FragColor = vec4(0.24, 0.68, 1.0, rim * sun * 0.46);
          }
        `
      })
    );
    this.earthGroup.add(this.atmosphereMesh);
  }

  private buildSatellite(): void {
    const bus = new THREE.Mesh(
      new THREE.BoxGeometry(0.11, 0.11, 0.34),
      new THREE.MeshStandardMaterial({
        color: 0xe5e7eb,
        roughness: 0.38,
        metalness: 0.42
      })
    );
    const panelMaterial = new THREE.MeshStandardMaterial({
      color: 0x1d4ed8,
      emissive: 0x0f172a,
      roughness: 0.52,
      metalness: 0.18
    });
    const panelLeft = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.012, 0.18), panelMaterial);
    panelLeft.position.x = -0.27;
    const panelRight = panelLeft.clone();
    panelRight.position.x = 0.27;
    const antenna = new THREE.Mesh(
      new THREE.CylinderGeometry(0.006, 0.006, 0.4, 16),
      new THREE.MeshStandardMaterial({ color: 0xf8fafc, metalness: 0.6, roughness: 0.3 })
    );
    antenna.rotation.x = Math.PI / 2;
    antenna.position.z = -0.29;

    this.satelliteGroup.add(bus, panelLeft, panelRight, antenna);
    this.boresightLine = makeAxisLine([0, 0, 0], [0, 0, 0.82], 0x4ade80, 0.8);
    this.satelliteGroup.add(this.boresightLine);
    this.satelliteGroup.add(makeAxisLine([0, 0, 0], [0.74, 0, 0], 0xfacc15, 0.72));
    this.satelliteGroup.scale.setScalar(SATELLITE_VISUAL_SCALE);

    this.activeMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.07, 24, 12),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.82 })
    );
    this.activeMarker.renderOrder = 4;
    this.scene.add(this.activeMarker);
  }

  private buildSunMoon(): void {
    const sunHalo = new THREE.Mesh(
      new THREE.SphereGeometry(0.42, 48, 24),
      new THREE.MeshBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.94 })
    );
    const sunGlow = new THREE.Mesh(
      new THREE.SphereGeometry(1.0, 48, 24),
      new THREE.MeshBasicMaterial({ color: 0xff9f1c, transparent: true, opacity: 0.14, blending: THREE.AdditiveBlending, depthWrite: false })
    );
    this.sunGroup.add(sunGlow, sunHalo);

    const moon = new THREE.Mesh(
      new THREE.SphereGeometry(0.22, 48, 24),
      new THREE.MeshStandardMaterial({ color: 0xbbb7ad, roughness: 0.86, metalness: 0.0 })
    );
    this.moonGroup.add(moon);
  }

  private buildShadow(): void {
    const cylinder = new THREE.Mesh(
      new THREE.CylinderGeometry(EARTH_RADIUS_SCENE * 0.995, EARTH_RADIUS_SCENE * 0.92, 13.5, 72, 1, true),
      new THREE.MeshBasicMaterial({
        color: 0x070b14,
        transparent: true,
        opacity: 0.26,
        side: THREE.DoubleSide,
        depthWrite: false
      })
    );
    cylinder.renderOrder = -1;
    this.shadowGroup.add(cylinder);
  }

  private buildPhaseTubes(timeline: MissionTimeline): void {
    for (const phase of timeline.phases) {
      const points = timeline.samples
        .filter((sample) => sample.tSeconds >= phase.startSeconds && sample.tSeconds <= phase.endSeconds)
        .map((sample) => toSceneVec(sample.state.positionKm));
      if (points.length > 2) {
        this.orbitGroup.add(makeTube(points, phase.color, PHASE_RADIUS, 0.78));
      }
    }
  }

  private buildGroundTrack(timeline: MissionTimeline): void {
    const points = timeline.samples.map((sample) => toSceneVec(surfacePoint(sample.state.positionKm))).filter((_, index) => index % 3 === 0);
    this.groundTrackLine = makeLine(points, 0x4ade80, 0.22);
    this.orbitGroup.add(this.groundTrackLine);
  }

  private buildTargetMarker(sample: MissionSample): void {
    const marker = new THREE.Mesh(
      new THREE.RingGeometry(0.08, 0.15, 48),
      new THREE.MeshBasicMaterial({
        color: 0x4ade80,
        transparent: true,
        opacity: 0.75,
        side: THREE.DoubleSide
      })
    );
    const pos = toSceneVec(sample.environment.targetEciKm);
    marker.position.copy(pos);
    marker.lookAt(pos.clone().multiplyScalar(1.5));
    this.targetMarker = marker;
    this.targetGroup.add(marker);
  }
}

export function toSceneVec(v: Vec3): THREE.Vector3 {
  const sceneDirection = new THREE.Vector3(v[0], v[2], -v[1]);
  const physicalRadiusKm = Math.hypot(v[0], v[1], v[2]);
  if (physicalRadiusKm < 1e-9) return sceneDirection;
  return sceneDirection.normalize().multiplyScalar(sceneRadiusForPhysicalRadiusKm(physicalRadiusKm));
}

function toSceneDirection(v: Vec3): THREE.Vector3 {
  return new THREE.Vector3(v[0], v[2], -v[1]).normalize();
}

function surfacePoint(positionKm: Vec3): Vec3 {
  const r = Math.hypot(positionKm[0], positionKm[1], positionKm[2]);
  if (r < 1e-9) return [EARTH_RADIUS_KM, 0, 0];
  const scale = EARTH_RADIUS_KM / r;
  return [positionKm[0] * scale, positionKm[1] * scale, positionKm[2] * scale];
}

function sceneRadiusForPhysicalRadiusKm(radiusKm: number): number {
  const altitudeKm = radiusKm - EARTH_RADIUS_KM;
  return EARTH_RADIUS_SCENE + altitudeKm * KM_TO_SCENE * ORBIT_ALTITUDE_VISUAL_SCALE;
}

function makeLine(points: THREE.Vector3[], color: THREE.ColorRepresentation, opacity: number): THREE.Line {
  return new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity })
  );
}

function makeLineLoop(points: THREE.Vector3[], color: THREE.ColorRepresentation, opacity: number): THREE.LineLoop {
  return new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity })
  );
}

function makeTube(points: THREE.Vector3[], color: string, radius: number, opacity: number): THREE.Mesh {
  const curve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.TubeGeometry(curve, Math.max(16, points.length * 2), radius, 8, false);
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthWrite: false
  });
  return new THREE.Mesh(geometry, material);
}

function makeAxisLine(from: Vec3, to: Vec3, color: THREE.ColorRepresentation, opacity: number): THREE.Line {
  return new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...from), new THREE.Vector3(...to)]),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity })
  );
}

function makeLatitudeLine(latDeg: number): THREE.Line {
  const lat = THREE.MathUtils.degToRad(latDeg);
  const radius = Math.cos(lat) * EARTH_RADIUS_SCENE * 1.002;
  const z = Math.sin(lat) * EARTH_RADIUS_SCENE * 1.002;
  const points: THREE.Vector3[] = [];
  for (let i = 0; i <= 180; i += 1) {
    const theta = (i / 180) * Math.PI * 2;
    points.push(new THREE.Vector3(radius * Math.cos(theta), z, radius * Math.sin(theta)));
  }
  return makeLine(points, 0xffffff, 0.065);
}

function makeLongitudeLine(lonDeg: number): THREE.Line {
  const lon = THREE.MathUtils.degToRad(lonDeg);
  const points: THREE.Vector3[] = [];
  for (let i = -90; i <= 90; i += 2) {
    const lat = THREE.MathUtils.degToRad(i);
    points.push(
      new THREE.Vector3(
        EARTH_RADIUS_SCENE * 1.002 * Math.cos(lat) * Math.cos(lon),
        EARTH_RADIUS_SCENE * 1.002 * Math.sin(lat),
        EARTH_RADIUS_SCENE * 1.002 * Math.cos(lat) * Math.sin(lon)
      )
    );
  }
  return makeLine(points, 0xffffff, 0.055);
}

function makeEarthTexture(): THREE.CanvasTexture {
  const canvas = document.createElement("canvas");
  canvas.width = 2048;
  canvas.height = 1024;
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("2D canvas unavailable");

  const ocean = ctx.createLinearGradient(0, 0, 0, canvas.height);
  ocean.addColorStop(0, "#092340");
  ocean.addColorStop(0.5, "#0d4f7c");
  ocean.addColorStop(1, "#061426");
  ctx.fillStyle = ocean;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.globalAlpha = 0.72;
  for (let i = 0; i < 110; i += 1) {
    const x = pseudoNoise(i, 0) * canvas.width;
    const y = pseudoNoise(i, 1) * canvas.height;
    const w = 70 + pseudoNoise(i, 2) * 170;
    const h = 28 + pseudoNoise(i, 3) * 110;
    ctx.fillStyle = i % 4 === 0 ? "#a59468" : i % 3 === 0 ? "#4d7d55" : "#27614e";
    ctx.beginPath();
    ctx.ellipse(x, y, w, h, pseudoNoise(i, 4) * Math.PI, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.globalAlpha = 0.18;
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 5;
  for (let i = 0; i < 70; i += 1) {
    const y = pseudoNoise(i, 8) * canvas.height;
    ctx.beginPath();
    ctx.moveTo(0, y);
    for (let x = 0; x <= canvas.width; x += 80) {
      ctx.lineTo(x, y + Math.sin(x * 0.014 + i) * 16);
    }
    ctx.stroke();
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 8;
  return texture;
}

function pseudoNoise(a: number, b: number): number {
  return fract(Math.sin(a * 91.345 + b * 27.119) * 43758.5453);
}

function fract(x: number): number {
  return x - Math.floor(x);
}

function clearGroup(group: THREE.Group): void {
  while (group.children.length) {
    const child = group.children.pop();
    if (!child) continue;
    child.traverse((object) => {
      const mesh = object as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const material = mesh.material;
      if (Array.isArray(material)) {
        material.forEach((item) => item.dispose());
      } else if (material) {
        material.dispose();
      }
    });
  }
}

export function sceneScaleInfo(): {
  kmToScene: number;
  earthRadiusScene: number;
  atmosphereRadiusScene: number;
  orbitAltitudeVisualScale: number;
  satelliteVisualScale: number;
  satelliteMaxExtentScene: number;
  sunSceneDistance: number;
  sceneRadiusForPhysicalRadiusKm: (radiusKm: number) => number;
} {
  return {
    kmToScene: KM_TO_SCENE,
    earthRadiusScene: EARTH_RADIUS_SCENE,
    atmosphereRadiusScene: EARTH_RADIUS_SCENE * ATMOSPHERE_SCALE,
    orbitAltitudeVisualScale: ORBIT_ALTITUDE_VISUAL_SCALE,
    satelliteVisualScale: SATELLITE_VISUAL_SCALE,
    satelliteMaxExtentScene: SATELLITE_MAX_EXTENT_SCENE,
    sunSceneDistance: sunDistanceKm() * KM_TO_SCENE,
    sceneRadiusForPhysicalRadiusKm
  };
}
