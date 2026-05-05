import type {
  EphemerisSample,
  MissionConfig,
  MissionPhase,
  MissionPhaseName,
  MissionSample,
  MissionTimeline,
  OrbitalElements,
  StateVector
} from "../types";
import { EARTH_RADIUS_KM, SOLAR_CONSTANT_W_M2 } from "./constants";
import { attitudeForMode, autoPointingModeForPhase } from "./attitude";
import { buildEnvironmentSample } from "./environment";
import { clamp, dot, lerp, norm, normalize, scale } from "./math";
import {
  elementsToState,
  hohmannDeltaV,
  interpolateOrbitElements,
  orbitalPeriodSeconds,
  planeChangeDeltaV
} from "./orbit";

const PHASE_COLORS: Record<MissionPhaseName, string> = {
  Detumble: "#f97316",
  "Sun Acquire": "#facc15",
  "Transfer Burn": "#38bdf8",
  "Target Track": "#4ade80",
  Downlink: "#a78bfa",
  Eclipse: "#94a3b8",
  "Momentum Dump": "#fb7185"
};

export function buildMissionTimeline(config: MissionConfig, ephemeris: EphemerisSample[], dataSource: string): MissionTimeline {
  const durationSeconds = config.durationHours * 3600;
  const samples = Math.max(240, Math.floor(config.sampleCount));
  const startRadius = config.startOrbit.semiMajorAxisKm;
  const endRadius = config.endOrbit.semiMajorAxisKm;
  const h = hohmannDeltaV(startRadius, endRadius);
  const planeDv = planeChangeDeltaV(startRadius, config.endOrbit.inclinationDeg - config.startOrbit.inclinationDeg);
  const transferDeltaVKmS =
    config.trajectoryMode === "plane-change" ? planeDv : config.trajectoryMode === "natural" ? 0 : h.totalKmS;
  const transferTimeSeconds =
    config.trajectoryMode === "natural" ? orbitalPeriodSeconds(config.startOrbit.semiMajorAxisKm) : h.transferTimeSeconds;
  const missionSamples: MissionSample[] = [];
  const date0 = new Date(config.epochIso);

  for (let index = 0; index < samples; index += 1) {
    const alpha = index / (samples - 1);
    const tSeconds = alpha * durationSeconds;
    const date = new Date(date0.getTime() + tSeconds * 1000);
    const state = stateForMissionTime(config, tSeconds, durationSeconds);
    const environment = buildEnvironmentSample(state, date, config.target.latDeg, config.target.lonDeg, ephemeris);
    const nominalPhase = nominalPhaseForAlpha(alpha, environment.eclipseFactor);
    const autoMode = autoPointingModeForPhase(nominalPhase, environment.targetVisible);
    const attitude = attitudeForMode(config.pointingMode, state, environment, tSeconds, autoMode);
    const altitudeKm = norm(state.positionKm) - EARTH_RADIUS_KM;
    const speedKmS = norm(state.velocityKmS);
    const panelIncidence = Math.max(0, dot(normalize(attitude.bodyX), normalize(environment.sunEci)));
    const solarPowerW =
      SOLAR_CONSTANT_W_M2 *
      config.spacecraft.solarPanelAreaM2 *
      config.spacecraft.solarPanelEfficiency *
      panelIncidence *
      environment.eclipseFactor;

    missionSamples.push({
      index,
      tSeconds,
      date,
      phase: nominalPhase,
      state,
      environment,
      attitude,
      altitudeKm,
      speedKmS,
      solarPowerW
    });
  }

  return {
    config,
    samples: missionSamples,
    phases: consolidatePhases(missionSamples),
    transferDeltaVKmS,
    transferTimeSeconds,
    dataSource
  };
}

export function stateForMissionTime(config: MissionConfig, tSeconds: number, durationSeconds: number): StateVector {
  const alpha = clamp(tSeconds / durationSeconds, 0, 1);

  if (config.trajectoryMode === "natural") {
    return elementsToState(config.startOrbit, tSeconds, true);
  }

  if (config.trajectoryMode === "hohmann") {
    if (alpha < 0.28) return elementsToState(config.startOrbit, tSeconds, true);
    if (alpha > 0.68) return elementsToState(config.endOrbit, tSeconds - durationSeconds * 0.68, true);
    const transferAlpha = (alpha - 0.28) / 0.4;
    return elementsToState(makeTransferOrbit(config.startOrbit, config.endOrbit, transferAlpha), 0, false);
  }

  if (config.trajectoryMode === "plane-change") {
    const blend = smoothBlend(alpha, 0.32, 0.62);
    const elements = interpolateOrbitElements(config.startOrbit, config.endOrbit, blend);
    return elementsToState(elements, tSeconds, true);
  }

  const phasingAlpha = smoothBlend(alpha, 0.22, 0.78);
  const phaseOrbit = interpolateOrbitElements(config.startOrbit, config.endOrbit, phasingAlpha);
  const phasingBias = Math.sin(Math.PI * phasingAlpha) * 85;
  return elementsToState(
    {
      ...phaseOrbit,
      semiMajorAxisKm: phaseOrbit.semiMajorAxisKm + phasingBias
    },
    tSeconds,
    true
  );
}

export function nominalPhaseForAlpha(alpha: number, eclipseFactor: number): MissionPhaseName {
  if (eclipseFactor < 0.2) return "Eclipse";
  if (alpha < 0.11) return "Detumble";
  if (alpha < 0.25) return "Sun Acquire";
  if (alpha < 0.66) return "Transfer Burn";
  if (alpha < 0.84) return "Target Track";
  if (alpha < 0.94) return "Downlink";
  return "Momentum Dump";
}

export function missionConfigFromForm(form: HTMLFormElement, fallback: MissionConfig): MissionConfig {
  const data = new FormData(form);
  const numberValue = (name: string, fallbackValue: number) => {
    const raw = Number(data.get(name));
    return Number.isFinite(raw) ? raw : fallbackValue;
  };
  const epochIso = normalizeEpochIso(String(data.get("epochIso") ?? ""), fallback.epochIso);

  const startOrbit: OrbitalElements = {
    semiMajorAxisKm: EARTH_RADIUS_KM + numberValue("startAltitudeKm", fallback.startOrbit.semiMajorAxisKm - EARTH_RADIUS_KM),
    eccentricity: numberValue("startEccentricity", fallback.startOrbit.eccentricity),
    inclinationDeg: numberValue("startInclinationDeg", fallback.startOrbit.inclinationDeg),
    raanDeg: numberValue("startRaanDeg", fallback.startOrbit.raanDeg),
    argPerigeeDeg: numberValue("startArgPerigeeDeg", fallback.startOrbit.argPerigeeDeg),
    trueAnomalyDeg: numberValue("startTrueAnomalyDeg", fallback.startOrbit.trueAnomalyDeg),
    epochIso
  };

  const endOrbit: OrbitalElements = {
    semiMajorAxisKm: EARTH_RADIUS_KM + numberValue("endAltitudeKm", fallback.endOrbit.semiMajorAxisKm - EARTH_RADIUS_KM),
    eccentricity: numberValue("endEccentricity", fallback.endOrbit.eccentricity),
    inclinationDeg: numberValue("endInclinationDeg", fallback.endOrbit.inclinationDeg),
    raanDeg: numberValue("endRaanDeg", fallback.endOrbit.raanDeg),
    argPerigeeDeg: numberValue("endArgPerigeeDeg", fallback.endOrbit.argPerigeeDeg),
    trueAnomalyDeg: numberValue("endTrueAnomalyDeg", fallback.endOrbit.trueAnomalyDeg),
    epochIso
  };

  return {
    ...fallback,
    epochIso,
    durationHours: numberValue("durationHours", fallback.durationHours),
    trajectoryMode: String(data.get("trajectoryMode") ?? fallback.trajectoryMode) as MissionConfig["trajectoryMode"],
    pointingMode: String(data.get("pointingMode") ?? fallback.pointingMode) as MissionConfig["pointingMode"],
    startOrbit,
    endOrbit,
    target: {
      latDeg: numberValue("targetLatDeg", fallback.target.latDeg),
      lonDeg: numberValue("targetLonDeg", fallback.target.lonDeg)
    }
  };
}

export function defaultMissionConfig(): MissionConfig {
  const epochIso = "2026-06-01T12:00:00Z";
  return {
    epochIso,
    durationHours: 9,
    sampleCount: 720,
    trajectoryMode: "hohmann",
    pointingMode: "auto",
    startOrbit: {
      semiMajorAxisKm: EARTH_RADIUS_KM + 500,
      eccentricity: 0.001,
      inclinationDeg: 51.6,
      raanDeg: 28,
      argPerigeeDeg: 0,
      trueAnomalyDeg: 12,
      epochIso
    },
    endOrbit: {
      semiMajorAxisKm: EARTH_RADIUS_KM + 650,
      eccentricity: 0.001,
      inclinationDeg: 54,
      raanDeg: 32,
      argPerigeeDeg: 0,
      trueAnomalyDeg: 205,
      epochIso
    },
    target: {
      latDeg: 34.7,
      lonDeg: -86.6
    },
    spacecraft: {
      massKg: 4,
      dimensionsM: [0.1, 0.1, 0.3],
      inertiaKgM2: [0.033333, 0.033333, 0.006667],
      solarPanelAreaM2: 0.19,
      solarPanelEfficiency: 0.29,
      cameraHalfAngleDeg: 8
    }
  };
}

function makeTransferOrbit(start: OrbitalElements, end: OrbitalElements, transferAlpha: number): OrbitalElements {
  const r1 = start.semiMajorAxisKm;
  const r2 = end.semiMajorAxisKm;
  const lowToHigh = r2 >= r1;
  const perigee = Math.min(r1, r2);
  const apogee = Math.max(r1, r2);
  const a = (perigee + apogee) / 2;
  const e = (apogee - perigee) / (apogee + perigee);
  const trueAnomalyDeg = lowToHigh ? lerp(0, 180, transferAlpha) : lerp(180, 360, transferAlpha);
  return {
    semiMajorAxisKm: a,
    eccentricity: e,
    inclinationDeg: lerp(start.inclinationDeg, end.inclinationDeg, transferAlpha),
    raanDeg: lerp(start.raanDeg, end.raanDeg, transferAlpha),
    argPerigeeDeg: start.argPerigeeDeg,
    trueAnomalyDeg,
    epochIso: start.epochIso
  };
}

function consolidatePhases(samples: MissionSample[]): MissionPhase[] {
  const phases: MissionPhase[] = [];
  for (const sample of samples) {
    const current = phases[phases.length - 1];
    if (!current || current.name !== sample.phase) {
      phases.push({
        name: sample.phase,
        startSeconds: sample.tSeconds,
        endSeconds: sample.tSeconds,
        color: PHASE_COLORS[sample.phase]
      });
    } else {
      current.endSeconds = sample.tSeconds;
    }
  }
  return phases;
}

function smoothBlend(x: number, start: number, end: number): number {
  const t = clamp((x - start) / (end - start), 0, 1);
  return t * t * (3 - 2 * t);
}

function normalizeEpochIso(input: string, fallback: string): string {
  const trimmed = input.trim();
  if (!trimmed) return fallback;
  if (trimmed.endsWith("Z")) return trimmed;
  return `${trimmed.length === 16 ? `${trimmed}:00` : trimmed}Z`;
}
