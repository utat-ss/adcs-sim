import type { OrbitalElements, StateVector, Vec3 } from "../types";
import { EARTH_J2, EARTH_RADIUS_KM, EARTH_ROTATION_RAD_S, J2000_MS, MU_EARTH_KM3_S2 } from "./constants";
import {
  add,
  cross,
  degToRad,
  dot,
  interpolateAngleDeg,
  lerp,
  norm,
  normalize,
  radToDeg,
  rotateX,
  rotateZ,
  scale,
  sub,
  wrapDegrees
} from "./math";

export function orbitalPeriodSeconds(semiMajorAxisKm: number): number {
  return 2 * Math.PI * Math.sqrt(Math.pow(semiMajorAxisKm, 3) / MU_EARTH_KM3_S2);
}

export function meanMotionRadS(semiMajorAxisKm: number): number {
  return Math.sqrt(MU_EARTH_KM3_S2 / Math.pow(semiMajorAxisKm, 3));
}

export function trueToEccentricAnomalyRad(trueAnomalyRad: number, eccentricity: number): number {
  const e = eccentricity;
  const sinE = (Math.sqrt(1 - e * e) * Math.sin(trueAnomalyRad)) / (1 + e * Math.cos(trueAnomalyRad));
  const cosE = (e + Math.cos(trueAnomalyRad)) / (1 + e * Math.cos(trueAnomalyRad));
  return Math.atan2(sinE, cosE);
}

export function eccentricToTrueAnomalyRad(eccentricAnomalyRad: number, eccentricity: number): number {
  const e = eccentricity;
  const sinNu = (Math.sqrt(1 - e * e) * Math.sin(eccentricAnomalyRad)) / (1 - e * Math.cos(eccentricAnomalyRad));
  const cosNu = (Math.cos(eccentricAnomalyRad) - e) / (1 - e * Math.cos(eccentricAnomalyRad));
  return Math.atan2(sinNu, cosNu);
}

export function solveKeplerRad(meanAnomalyRad: number, eccentricity: number): number {
  const e = eccentricity;
  let E = e < 0.8 ? meanAnomalyRad : Math.PI;
  for (let i = 0; i < 12; i += 1) {
    const f = E - e * Math.sin(E) - meanAnomalyRad;
    const fp = 1 - e * Math.cos(E);
    E -= f / fp;
  }
  return E;
}

export function elementsToState(elements: OrbitalElements, secondsFromEpoch = 0, includeJ2 = true): StateVector {
  const a = elements.semiMajorAxisKm;
  const e = Math.min(Math.max(elements.eccentricity, 0), 0.95);
  const i = degToRad(elements.inclinationDeg);
  let raan = degToRad(elements.raanDeg);
  let argPerigee = degToRad(elements.argPerigeeDeg);
  const nu0 = degToRad(elements.trueAnomalyDeg);
  const p = a * (1 - e * e);
  const n = meanMotionRadS(a);

  if (includeJ2) {
    const factor = Math.pow(EARTH_RADIUS_KM / p, 2);
    const raanDot = -1.5 * EARTH_J2 * n * factor * Math.cos(i);
    const argDot = 0.75 * EARTH_J2 * n * factor * (5 * Math.cos(i) * Math.cos(i) - 1);
    raan += raanDot * secondsFromEpoch;
    argPerigee += argDot * secondsFromEpoch;
  }

  const E0 = trueToEccentricAnomalyRad(nu0, e);
  const M0 = E0 - e * Math.sin(E0);
  const M = normalizeRadians(M0 + n * secondsFromEpoch);
  const E = solveKeplerRad(M, e);
  const nu = eccentricToTrueAnomalyRad(E, e);
  const radius = p / (1 + e * Math.cos(nu));

  const rPerifocal: Vec3 = [radius * Math.cos(nu), radius * Math.sin(nu), 0];
  const vScale = Math.sqrt(MU_EARTH_KM3_S2 / p);
  const vPerifocal: Vec3 = [-vScale * Math.sin(nu), vScale * (e + Math.cos(nu)), 0];

  const rotateToEci = (v: Vec3) => rotateZ(rotateX(rotateZ(v, argPerigee), i), raan);

  return {
    positionKm: rotateToEci(rPerifocal),
    velocityKmS: rotateToEci(vPerifocal)
  };
}

export function stateToElements(state: StateVector, epochIso: string): OrbitalElements {
  const r = state.positionKm;
  const v = state.velocityKmS;
  const rMag = norm(r);
  const vMag = norm(v);
  const h = cross(r, v);
  const hMag = norm(h);
  const node = cross([0, 0, 1], h);
  const nMag = norm(node);
  const eccentricityVector = sub(scale(cross(v, h), 1 / MU_EARTH_KM3_S2), scale(r, 1 / rMag));
  const e = norm(eccentricityVector);
  const energy = (vMag * vMag) / 2 - MU_EARTH_KM3_S2 / rMag;
  const a = -MU_EARTH_KM3_S2 / (2 * energy);
  const inclinationDeg = radToDeg(Math.acos(h[2] / hMag));
  const raanDeg = nMag < 1e-9 ? 0 : wrapDegrees(radToDeg(Math.atan2(node[1], node[0])));
  const argPerigeeDeg =
    nMag < 1e-9 || e < 1e-9
      ? 0
      : wrapDegrees(radToDeg(Math.atan2(dot(cross(node, eccentricityVector), h) / hMag, dot(node, eccentricityVector))));
  const trueAnomalyDeg =
    e < 1e-9 ? 0 : wrapDegrees(radToDeg(Math.atan2(dot(cross(eccentricityVector, r), h) / hMag, dot(eccentricityVector, r))));

  return {
    semiMajorAxisKm: a,
    eccentricity: e,
    inclinationDeg,
    raanDeg,
    argPerigeeDeg,
    trueAnomalyDeg,
    epochIso
  };
}

export function groundPointFromEci(positionKm: Vec3, date: Date): { latDeg: number; lonDeg: number } {
  const theta = gmstRad(date);
  const ecef = rotateZ(positionKm, -theta);
  const rxy = Math.hypot(ecef[0], ecef[1]);
  const latDeg = radToDeg(Math.atan2(ecef[2], rxy));
  const lonDeg = wrapLongitudeDeg(radToDeg(Math.atan2(ecef[1], ecef[0])));
  return { latDeg, lonDeg };
}

export function groundPointToEciKm(latDeg: number, lonDeg: number, date: Date, altitudeKm = 0): Vec3 {
  const lat = degToRad(latDeg);
  const lon = degToRad(lonDeg);
  const radius = EARTH_RADIUS_KM + altitudeKm;
  const ecef: Vec3 = [radius * Math.cos(lat) * Math.cos(lon), radius * Math.cos(lat) * Math.sin(lon), radius * Math.sin(lat)];
  return rotateZ(ecef, gmstRad(date));
}

export function gmstRad(date: Date): number {
  const d = (date.getTime() - J2000_MS) / 86400000;
  const gmstDeg = 280.46061837 + 360.98564736629 * d;
  return degToRad(wrapDegrees(gmstDeg));
}

export function interpolateOrbitElements(start: OrbitalElements, end: OrbitalElements, alpha: number): OrbitalElements {
  return {
    semiMajorAxisKm: lerp(start.semiMajorAxisKm, end.semiMajorAxisKm, alpha),
    eccentricity: lerp(start.eccentricity, end.eccentricity, alpha),
    inclinationDeg: lerp(start.inclinationDeg, end.inclinationDeg, alpha),
    raanDeg: interpolateAngleDeg(start.raanDeg, end.raanDeg, alpha),
    argPerigeeDeg: interpolateAngleDeg(start.argPerigeeDeg, end.argPerigeeDeg, alpha),
    trueAnomalyDeg: interpolateAngleDeg(start.trueAnomalyDeg, end.trueAnomalyDeg, alpha),
    epochIso: start.epochIso
  };
}

export function circularVelocityKmS(radiusKm: number): number {
  return Math.sqrt(MU_EARTH_KM3_S2 / radiusKm);
}

export function hohmannDeltaV(startRadiusKm: number, endRadiusKm: number): { deltaV1KmS: number; deltaV2KmS: number; totalKmS: number; transferTimeSeconds: number } {
  const r1 = startRadiusKm;
  const r2 = endRadiusKm;
  const transferA = (r1 + r2) / 2;
  const v1 = circularVelocityKmS(r1);
  const v2 = circularVelocityKmS(r2);
  const vt1 = Math.sqrt(MU_EARTH_KM3_S2 * (2 / r1 - 1 / transferA));
  const vt2 = Math.sqrt(MU_EARTH_KM3_S2 * (2 / r2 - 1 / transferA));
  const deltaV1KmS = Math.abs(vt1 - v1);
  const deltaV2KmS = Math.abs(v2 - vt2);
  const transferTimeSeconds = Math.PI * Math.sqrt(Math.pow(transferA, 3) / MU_EARTH_KM3_S2);
  return { deltaV1KmS, deltaV2KmS, totalKmS: deltaV1KmS + deltaV2KmS, transferTimeSeconds };
}

export function planeChangeDeltaV(radiusKm: number, inclinationDeltaDeg: number): number {
  return 2 * circularVelocityKmS(radiusKm) * Math.sin(Math.abs(degToRad(inclinationDeltaDeg)) / 2);
}

export function makeOrbitPath(elements: OrbitalElements, sampleCount = 360): Vec3[] {
  const period = orbitalPeriodSeconds(elements.semiMajorAxisKm);
  return Array.from({ length: sampleCount }, (_, index) => {
    const t = (index / sampleCount) * period;
    return elementsToState(elements, t, true).positionKm;
  });
}

export function betaAngleDeg(positionKm: Vec3, velocityKmS: Vec3, sunEci: Vec3): number {
  const h = normalize(cross(positionKm, velocityKmS));
  const sun = normalize(sunEci);
  return radToDeg(Math.asin(Math.max(-1, Math.min(1, dot(h, sun)))));
}

export function normalizeRadians(rad: number): number {
  return ((rad % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
}

export function wrapLongitudeDeg(lonDeg: number): number {
  const wrapped = ((lonDeg + 180) % 360) - 180;
  return wrapped < -180 ? wrapped + 360 : wrapped;
}

export function lineOfSightToGroundTarget(positionKm: Vec3, targetEciKm: Vec3): boolean {
  const targetNormal = normalize(targetEciKm);
  const targetToSat = normalize(sub(positionKm, targetEciKm));
  return dot(targetNormal, targetToSat) > 0;
}

export function velocityFrame(positionKm: Vec3, velocityKmS: Vec3): { radial: Vec3; alongTrack: Vec3; crossTrack: Vec3 } {
  const radial = normalize(positionKm);
  const crossTrack = normalize(cross(positionKm, velocityKmS));
  const alongTrack = normalize(cross(crossTrack, radial));
  return { radial, alongTrack, crossTrack };
}

export function earthRotationAngleSince(date: Date, start: Date): number {
  return EARTH_ROTATION_RAD_S * ((date.getTime() - start.getTime()) / 1000);
}
