import type { EnvironmentSample, EphemerisSample, StateVector, Vec3 } from "../types";
import {
  AU_KM,
  EARTH_MAG_DIPOLE_UT_EARTH3,
  EARTH_RADIUS_KM,
  J2000_MS,
  MOON_MEAN_DISTANCE_KM
} from "./constants";
import { angleBetweenDeg, clamp, cross, degToRad, dot, lerp, norm, normalize, radToDeg, scale, smoothstep, sub } from "./math";
import { betaAngleDeg, groundPointFromEci, groundPointToEciKm, lineOfSightToGroundTarget } from "./orbit";

export function sunMoonFromEphemeris(date: Date, samples: EphemerisSample[]): { sunEci: Vec3; moonEci: Vec3 } {
  if (samples.length < 2) {
    return { sunEci: analyticalSunEci(date), moonEci: analyticalMoonEci(date) };
  }

  const t = date.getTime();
  let upper = samples.findIndex((sample) => sample.epochMs >= t);
  if (upper <= 0) upper = 1;
  if (upper >= samples.length) upper = samples.length - 1;
  const a = samples[upper - 1];
  const b = samples[upper];
  const span = Math.max(1, b.epochMs - a.epochMs);
  const alpha = clamp((t - a.epochMs) / span, 0, 1);

  return {
    sunEci: normalize([
      lerp(a.sunEci[0], b.sunEci[0], alpha),
      lerp(a.sunEci[1], b.sunEci[1], alpha),
      lerp(a.sunEci[2], b.sunEci[2], alpha)
    ]),
    moonEci: normalize([
      lerp(a.moonEci[0], b.moonEci[0], alpha),
      lerp(a.moonEci[1], b.moonEci[1], alpha),
      lerp(a.moonEci[2], b.moonEci[2], alpha)
    ])
  };
}

export function analyticalSunEci(date: Date): Vec3 {
  const n = (date.getTime() - J2000_MS) / 86400000;
  const meanLongitude = degToRad((280.46 + 0.9856474 * n) % 360);
  const meanAnomaly = degToRad((357.528 + 0.9856003 * n) % 360);
  const eclipticLongitude =
    meanLongitude + degToRad(1.915) * Math.sin(meanAnomaly) + degToRad(0.02) * Math.sin(2 * meanAnomaly);
  const obliquity = degToRad(23.439 - 0.0000004 * n);
  return normalize([
    Math.cos(eclipticLongitude),
    Math.cos(obliquity) * Math.sin(eclipticLongitude),
    Math.sin(obliquity) * Math.sin(eclipticLongitude)
  ]);
}

export function analyticalMoonEci(date: Date): Vec3 {
  const d = (date.getTime() - J2000_MS) / 86400000;
  const L = degToRad((218.316 + 13.176396 * d) % 360);
  const M = degToRad((134.963 + 13.064993 * d) % 360);
  const F = degToRad((93.272 + 13.22935 * d) % 360);
  const lon = L + degToRad(6.289) * Math.sin(M);
  const lat = degToRad(5.128) * Math.sin(F);
  const obliquity = degToRad(23.439 - 0.0000004 * d);
  const xEcl = Math.cos(lat) * Math.cos(lon);
  const yEcl = Math.cos(lat) * Math.sin(lon);
  const zEcl = Math.sin(lat);
  return normalize([xEcl, yEcl * Math.cos(obliquity) - zEcl * Math.sin(obliquity), yEcl * Math.sin(obliquity) + zEcl * Math.cos(obliquity)]);
}

export function eclipseFactor(positionKm: Vec3, sunEci: Vec3): number {
  const sun = normalize(sunEci);
  const alongSun = dot(positionKm, sun);
  if (alongSun >= 0) return 1;
  const perpendicular = norm(sub(positionKm, scale(sun, alongSun)));
  const penumbraWidthKm = 90;
  return smoothstep(EARTH_RADIUS_KM - penumbraWidthKm, EARTH_RADIUS_KM + penumbraWidthKm, perpendicular);
}

export function magneticFieldDipoleUt(positionKm: Vec3): Vec3 {
  const magneticAxis = normalize([0.05, -0.18, 0.982]);
  const rEarth = norm(positionKm) / EARTH_RADIUS_KM;
  const rHat = normalize(positionKm);
  const mDotR = dot(magneticAxis, rHat);
  return scale(sub(scale(rHat, 3 * mDotR), magneticAxis), EARTH_MAG_DIPOLE_UT_EARTH3 / Math.pow(rEarth, 3));
}

export function buildEnvironmentSample(state: StateVector, date: Date, targetLatDeg: number, targetLonDeg: number, ephemeris: EphemerisSample[]): EnvironmentSample {
  const { sunEci, moonEci } = sunMoonFromEphemeris(date, ephemeris);
  const groundPoint = groundPointFromEci(state.positionKm, date);
  const targetEciKm = groundPointToEciKm(targetLatDeg, targetLonDeg, date, 0);
  const eclipse = eclipseFactor(state.positionKm, sunEci);
  return {
    sunEci,
    moonEci,
    eclipseFactor: eclipse,
    betaAngleDeg: betaAngleDeg(state.positionKm, state.velocityKmS, sunEci),
    magneticFieldUt: magneticFieldDipoleUt(state.positionKm),
    groundPoint,
    targetEciKm,
    targetVisible: lineOfSightToGroundTarget(state.positionKm, targetEciKm)
  };
}

export function sunDistanceKm(): number {
  return AU_KM;
}

export function moonDistanceKm(): number {
  return MOON_MEAN_DISTANCE_KM;
}

export function targetOffNadirDeg(positionKm: Vec3, targetEciKm: Vec3): number {
  const nadir = normalize(scale(positionKm, -1));
  const toTarget = normalize(sub(targetEciKm, positionKm));
  return angleBetweenDeg(nadir, toTarget);
}

export function orbitNormalToSunDeg(positionKm: Vec3, velocityKmS: Vec3, sunEci: Vec3): number {
  return 90 - Math.abs(radToDeg(Math.asin(clamp(dot(normalize(cross(positionKm, velocityKmS)), normalize(sunEci)), -1, 1))));
}
