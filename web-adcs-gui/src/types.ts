export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];

export type TrajectoryMode = "natural" | "hohmann" | "plane-change" | "phasing";
export type PointingMode = "auto" | "sun-track" | "target-track" | "detumble" | "nadir" | "inertial";
export type TimelineSourceKind = "preview" | "replay";
export type SceneViewMode = "earth" | "orbit" | "spacecraft" | "chase";

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
  replay?: ReplayTelemetry;
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
  sourceKind?: TimelineSourceKind;
  provenance?: ReplayProvenance;
  frames?: ReplayFrames;
  constants?: ReplayConstants;
  events?: ReplayEvent[];
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

export interface ReplayProvenance {
  source?: string;
  model?: string;
  gitBranch?: string;
  gitCommit?: string;
  generatedAtUtc?: string;
  matlabVersion?: string;
  simStopTimeSec?: number;
  simFixedStepSec?: number;
  notes?: string[];
}

export interface ReplayFrames {
  position?: string;
  velocity?: string;
  attitudeQuaternion?: string;
  angularRate?: string;
  units?: Record<string, string>;
}

export interface ReplayConstants {
  earthRadiusKm?: number;
}

export interface ReplaySensor {
  id: string;
  kind?: "boresight" | "conic" | string;
  axisBody: Vec3;
  halfAngleDeg?: number;
  exclusionDeg?: Record<string, number>;
}

export interface ReplaySpacecraft {
  name?: string;
  dimensionsM?: Vec3;
  cameraHalfAngleDeg?: number;
  sensorBoresightsBody?: ReplaySensor[];
}

export interface ReplayQuality {
  attitudeValid?: boolean;
  starTrackerValid?: boolean;
  sampleSource?: string;
}

export interface ReplaySample {
  tSec: number;
  utc?: string;
  positionEciKm: Vec3;
  velocityEciKmS: Vec3;
  velocityEciKmps?: Vec3;
  qEciToBody: Quat;
  omegaBodyRadS?: Vec3;
  wheelMomentumNms?: number[];
  reactionWheelMomentumNms?: number[];
  rwTorqueBodyNm?: Vec3;
  reactionWheelCommandNm?: Vec3;
  reactionWheelTorqueBodyNm?: Vec3;
  reactionWheelTorqueWheelNm?: number[];
  rwTauWheelsNm?: number[];
  mtqDipoleAm2?: Vec3;
  magnetorquerCommandAm2?: Vec3;
  externalTorqueNm?: Vec3;
  sunEci?: Vec3;
  moonEci?: Vec3;
  bEciT?: Vec3;
  magneticFieldEciT?: Vec3;
  eclipse?: boolean;
  mode?: string;
  modeId?: number;
  trackerValid?: boolean | number;
  attitudeValid?: boolean | number;
  desiredAttitudeErrorDeg?: number;
  quality?: ReplayQuality;
}

export interface ReplayEvent {
  tSec: number;
  kind?: string;
  label: string;
}

export interface ReplayExport {
  schema: "utat.adcs.replay.v1";
  epochIso?: string;
  provenance?: ReplayProvenance;
  frames?: ReplayFrames;
  constants?: ReplayConstants;
  spacecraft?: ReplaySpacecraft;
  samples: ReplaySample[];
  events?: ReplayEvent[];
}

export interface ReplayTelemetry {
  qEciToBody: Quat;
  omegaBodyRadS?: Vec3;
  wheelMomentumNms?: number[];
  rwTorqueBodyNm?: Vec3;
  reactionWheelCommandNm?: Vec3;
  rwTauWheelsNm?: number[];
  mtqDipoleAm2?: Vec3;
  bEciT?: Vec3;
  attitudeValid?: boolean;
  starTrackerValid?: boolean;
  sampleSource?: string;
}

export interface LayerVisibility {
  orbit: boolean;
  groundTrack: boolean;
  target: boolean;
  sunMoon: boolean;
  shadow: boolean;
  bodyAxes: boolean;
  inertialAxes: boolean;
  lvlhAxes: boolean;
  sensorCone: boolean;
  footprint: boolean;
}
