import type { AttitudeSample, EnvironmentSample, PointingMode, StateVector, Vec3 } from "../types";
import { buildRightHandedBasis, degToRad, dot, normalize, quaternionFromBasis, scale, sub } from "./math";
import { velocityFrame } from "./orbit";

export const DETUMBLE_INITIAL_RATE_DEG_S = 1.8;
export const DETUMBLE_RESIDUAL_RATE_DEG_S = 0.03;
export const DETUMBLE_TIME_CONSTANT_SECONDS = 900;

export function attitudeForMode(
  requestedMode: PointingMode,
  state: StateVector,
  environment: EnvironmentSample,
  tSeconds: number,
  autoMode: Exclude<PointingMode, "auto">
): AttitudeSample {
  const mode = requestedMode === "auto" ? autoMode : requestedMode;
  const { radial, alongTrack } = velocityFrame(state.positionKm, state.velocityKmS);
  const nadir = scale(radial, -1);
  const toTarget = normalize(sub(environment.targetEciKm, state.positionKm), nadir);
  const sun = normalize(environment.sunEci);
  let bodyZ: Vec3;
  let preferredX: Vec3;
  let angularRateDegS = 0.015;

  if (mode === "sun-track") {
    bodyZ = projectAwayFrom(sun, radial, sun);
    preferredX = sun;
    angularRateDegS = 0.004;
  } else if (mode === "target-track") {
    bodyZ = toTarget;
    preferredX = projectAwayFrom(sun, bodyZ, alongTrack);
    angularRateDegS = 0.035;
  } else if (mode === "detumble") {
    angularRateDegS = detumbleAngularRateDegS(tSeconds);
    const spin = detumbleSpinAngleRad(tSeconds);
    bodyZ = normalize([Math.sin(spin), Math.cos(spin * 0.73), Math.sin(spin * 0.41)], nadir);
    preferredX = normalize([Math.cos(spin * 0.9), Math.sin(spin), 0.35], alongTrack);
  } else if (mode === "inertial") {
    bodyZ = [0, 0, 1];
    preferredX = [1, 0, 0];
    angularRateDegS = 0.001;
  } else {
    bodyZ = nadir;
    preferredX = alongTrack;
    angularRateDegS = 0.02;
  }

  const basis = buildRightHandedBasis(bodyZ, preferredX);
  return {
    quaternion: quaternionFromBasis(basis.x, basis.y, basis.z),
    bodyX: basis.x,
    bodyY: basis.y,
    bodyZ: basis.z,
    boresightEci: basis.z,
    mode,
    angularRateDegS
  };
}

export function autoPointingModeForPhase(phase: string, targetVisible: boolean): Exclude<PointingMode, "auto"> {
  if (phase === "Detumble") return "detumble";
  if (phase === "Sun Acquire" || phase === "Eclipse") return "sun-track";
  if (phase === "Target Track" && targetVisible) return "target-track";
  if (phase === "Downlink") return "nadir";
  if (phase === "Momentum Dump") return "sun-track";
  return "nadir";
}

export function detumbleAngularRateDegS(tSeconds: number): number {
  return Math.max(
    DETUMBLE_RESIDUAL_RATE_DEG_S,
    DETUMBLE_INITIAL_RATE_DEG_S * Math.exp(-Math.max(0, tSeconds) / DETUMBLE_TIME_CONSTANT_SECONDS)
  );
}

export function detumbleProgressForRate(rateDegS: number): number {
  const span = DETUMBLE_INITIAL_RATE_DEG_S - DETUMBLE_RESIDUAL_RATE_DEG_S;
  return Math.min(1, Math.max(0, (DETUMBLE_INITIAL_RATE_DEG_S - rateDegS) / span));
}

function detumbleSpinAngleRad(tSeconds: number): number {
  const t = Math.max(0, tSeconds);
  const decayRateAtResidual = DETUMBLE_RESIDUAL_RATE_DEG_S / DETUMBLE_INITIAL_RATE_DEG_S;
  const residualStartSeconds = DETUMBLE_TIME_CONSTANT_SECONDS * Math.log(1 / decayRateAtResidual);
  const decayingSeconds = Math.min(t, residualStartSeconds);
  const decayingAngleDeg =
    DETUMBLE_INITIAL_RATE_DEG_S *
    DETUMBLE_TIME_CONSTANT_SECONDS *
    (1 - Math.exp(-decayingSeconds / DETUMBLE_TIME_CONSTANT_SECONDS));
  const residualAngleDeg = t > residualStartSeconds ? DETUMBLE_RESIDUAL_RATE_DEG_S * (t - residualStartSeconds) : 0;
  return degToRad(decayingAngleDeg + residualAngleDeg);
}

function projectAwayFrom(vector: Vec3, normal: Vec3, fallback: Vec3): Vec3 {
  const projected = sub(vector, scale(normal, dot(vector, normal)));
  return normalize(projected, fallback);
}
