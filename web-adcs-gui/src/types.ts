export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];

export type TrajectoryMode = "natural" | "hohmann" | "plane-change" | "phasing";
export type PointingMode = "auto" | "sun-track" | "target-track" | "detumble" | "nadir" | "inertial";

export interface OrbitalElements {
  semiMajorAxisKm: number;
  eccentricity: number;
  inclinationDeg: number;
  raanDeg: number;
  argPerigeeDeg: number;
  trueAnomalyDeg: number;
  epochIso: string;
}

export interface StateVector {
  positionKm: Vec3;
  velocityKmS: Vec3;
}

export interface GroundPoint {
  latDeg: number;
  lonDeg: number;
}

export interface SpacecraftConfig {
  massKg: number;
  dimensionsM: Vec3;
  inertiaKgM2: Vec3;
  solarPanelAreaM2: number;
  solarPanelEfficiency: number;
  cameraHalfAngleDeg: number;
}

export interface MissionConfig {
  epochIso: string;
  durationHours: number;
  sampleCount: number;
  trajectoryMode: TrajectoryMode;
  pointingMode: PointingMode;
  startOrbit: OrbitalElements;
  endOrbit: OrbitalElements;
  target: GroundPoint;
  spacecraft: SpacecraftConfig;
}

export interface EphemerisSample {
  epochMs: number;
  sunEci: Vec3;
  moonEci: Vec3;
}

export interface EnvironmentSample {
  sunEci: Vec3;
  moonEci: Vec3;
  eclipseFactor: number;
  betaAngleDeg: number;
  magneticFieldUt: Vec3;
  groundPoint: GroundPoint;
  targetEciKm: Vec3;
  targetVisible: boolean;
}

export interface AttitudeSample {
  quaternion: Quat;
  bodyX: Vec3;
  bodyY: Vec3;
  bodyZ: Vec3;
  boresightEci: Vec3;
  mode: Exclude<PointingMode, "auto">;
  angularRateDegS: number;
}

export interface MissionSample {
  index: number;
  tSeconds: number;
  date: Date;
  phase: MissionPhaseName;
  state: StateVector;
  environment: EnvironmentSample;
  attitude: AttitudeSample;
  altitudeKm: number;
  speedKmS: number;
  solarPowerW: number;
}

export type MissionPhaseName =
  | "Detumble"
  | "Sun Acquire"
  | "Transfer Burn"
  | "Target Track"
  | "Downlink"
  | "Eclipse"
  | "Momentum Dump";

export interface MissionPhase {
  name: MissionPhaseName;
  startSeconds: number;
  endSeconds: number;
  color: string;
}

export interface MissionTimeline {
  config: MissionConfig;
  samples: MissionSample[];
  phases: MissionPhase[];
  transferDeltaVKmS: number;
  transferTimeSeconds: number;
  dataSource: string;
}

export interface ProjectDefaults {
  source: string;
  notes: string[];
  spacecraft: SpacecraftConfig;
  orbit: {
    startAltitudeKm: number;
    endAltitudeKm: number;
    eccentricity: number;
    inclinationDeg: number;
    raanDeg: number;
    argPerigeeDeg: number;
    trueAnomalyDeg: number;
  };
  target: GroundPoint;
}
