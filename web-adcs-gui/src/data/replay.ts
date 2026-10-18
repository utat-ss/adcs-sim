import type {
  MissionConfig,
  MissionPhase,
  MissionPhaseName,
  MissionSample,
  MissionTimeline,
  PointingMode,
  Quat,
  ReplayExport,
  ReplaySample,
  Vec3
} from "../types";
import { EARTH_RADIUS_KM, SOLAR_CONSTANT_W_M2 } from "../physics/constants";
import { analyticalMoonEci, analyticalSunEci } from "../physics/environment";
import { betaAngleDeg, groundPointFromEci, groundPointToEciKm, lineOfSightToGroundTarget, stateToElements } from "../physics/orbit";
import { clamp, dot, norm, normalize, normalizeQuat, quaternionFromBasis, radToDeg, scale, sub } from "../physics/math";

const PHASE_COLORS: Record<MissionPhaseName, string> = {
  Detumble: "#f97316",
  "Sun Acquire": "#facc15",
  "Transfer Burn": "#38bdf8",
  "Target Track": "#4ade80",
  Downlink: "#a78bfa",
  Eclipse: "#94a3b8",
  "Momentum Dump": "#fb7185"
};

export function parseReplayJson(text: string): ReplayExport {
  const parsed = JSON.parse(text) as unknown;
  if (!isRecord(parsed)) throw new Error("Replay file must contain a JSON object.");
  if (isRecord(parsed.metadata) && parsed.metadata.schema === "adcs-simulink-replay-v1") {
    return legacyTimeseriesReplayToSamples(parsed);
  }
  if (parsed.schema !== "utat.adcs.replay.v1") throw new Error("Replay schema must be utat.adcs.replay.v1.");
  if (!Array.isArray(parsed.samples) || parsed.samples.length < 2) throw new Error("Replay file must contain at least two samples.");

  const samples = parsed.samples.map((sample, index) => normalizeReplaySample(sample, index));
  for (let index = 1; index < samples.length; index += 1) {
    if (samples[index].tSec < samples[index - 1].tSec) throw new Error("Replay samples must have nondecreasing tSec.");
  }

  return {
    ...(parsed as unknown as ReplayExport),
    schema: "utat.adcs.replay.v1",
    provenance: provenanceFromRootReplay(parsed),
    constants: constantsFromRootReplay(parsed),
    spacecraft: spacecraftFromRootReplay(parsed),
    frames: framesFromRootReplay(parsed),
    epochIso: epochFromRootReplay(parsed),
    events: eventsFromRootReplay(parsed),
    samples
  };
}

export function replayToMissionTimeline(replay: ReplayExport, fallbackConfig: MissionConfig): MissionTimeline {
  const samples = replay.samples;
  const first = samples[0];
  const last = samples[samples.length - 1];
  const epoch = replay.epochIso ?? fallbackConfig.epochIso;
  const firstDate = parseReplayDate(first, epoch, first.tSec);
  const lastDate = parseReplayDate(last, firstDate.toISOString(), last.tSec - first.tSec);
  const durationSeconds = Math.max(1, last.tSec - first.tSec);
  const spacecraft = {
    ...fallbackConfig.spacecraft,
    ...(replay.spacecraft?.dimensionsM ? { dimensionsM: replay.spacecraft.dimensionsM } : {}),
    ...(Number.isFinite(replay.spacecraft?.cameraHalfAngleDeg)
      ? { cameraHalfAngleDeg: Number(replay.spacecraft?.cameraHalfAngleDeg) }
      : {})
  };

  const config: MissionConfig = {
    ...fallbackConfig,
    epochIso: firstDate.toISOString(),
    durationHours: durationSeconds / 3600,
    sampleCount: samples.length,
    startOrbit: stateToElements({ positionKm: first.positionEciKm, velocityKmS: first.velocityEciKmS }, firstDate.toISOString()),
    endOrbit: stateToElements({ positionKm: last.positionEciKm, velocityKmS: last.velocityEciKmS }, lastDate.toISOString()),
    spacecraft
  };

  const missionSamples = samples.map((sample, index) => replaySampleToMissionSample(sample, index, firstDate, config, replay));

  return {
    config,
    samples: missionSamples,
    phases: consolidatePhases(missionSamples),
    transferDeltaVKmS: 0,
    transferTimeSeconds: durationSeconds,
    dataSource: replayDataSource(replay),
    sourceKind: "replay",
    provenance: replay.provenance,
    frames: replay.frames,
    constants: replay.constants,
    events: replay.events ?? []
  };
}

export function qEciToBodyToBodyInEci(qEciToBody: Quat): Quat {
  return normalizeQuat(qEciToBody);
}

export function rotateByScalarFirstQuat(q: Quat, v: Vec3): Vec3 {
  const nq = normalizeQuat(q);
  const [w, x, y, z] = nq;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)
  ];
}

function replaySampleToMissionSample(
  sample: ReplaySample,
  index: number,
  firstDate: Date,
  config: MissionConfig,
  replay: ReplayExport
): MissionSample {
  const date = sample.utc ? new Date(sample.utc) : new Date(firstDate.getTime() + (sample.tSec - replay.samples[0].tSec) * 1000);
  const state = { positionKm: sample.positionEciKm, velocityKmS: sample.velocityEciKmS };
  const qBodyToEci = qEciToBodyToBodyInEci(sample.qEciToBody);
  const bodyX = normalize(rotateByScalarFirstQuat(qBodyToEci, [1, 0, 0]), [1, 0, 0]);
  const bodyY = normalize(rotateByScalarFirstQuat(qBodyToEci, [0, 1, 0]), [0, 1, 0]);
  const bodyZ = normalize(rotateByScalarFirstQuat(qBodyToEci, [0, 0, 1]), [0, 0, 1]);
  const cameraAxis = cameraAxisBody(replay);
  const boresightEci = normalize(rotateByScalarFirstQuat(qBodyToEci, cameraAxis), bodyZ);
  const sunEci = normalize(sample.sunEci ?? analyticalSunEci(date));
  const moonEci = normalize(sample.moonEci ?? analyticalMoonEci(date));
  const targetEciKm = groundPointToEciKm(config.target.latDeg, config.target.lonDeg, date, 0);
  const omegaBodyRadS = sample.omegaBodyRadS;
  const bEciT = sample.bEciT;
  const phase = phaseFromReplay(sample);
  const panelIncidence = Math.max(0, dot(bodyX, sunEci));
  const eclipseFactor = sample.eclipse ? 0 : 1;
  const earthRadiusKm = earthRadiusKmForReplay(replay);
  const solarPowerW =
    SOLAR_CONSTANT_W_M2 * config.spacecraft.solarPanelAreaM2 * config.spacecraft.solarPanelEfficiency * panelIncidence * eclipseFactor;

  return {
    index,
    tSeconds: sample.tSec - replay.samples[0].tSec,
    date,
    phase,
    state,
    environment: {
      sunEci,
      moonEci,
      eclipseFactor,
      betaAngleDeg: betaAngleDeg(sample.positionEciKm, sample.velocityEciKmS, sunEci),
      magneticFieldUt: bEciT ? scale(bEciT, 1e6) : [0, 0, 0],
      groundPoint: groundPointFromEci(sample.positionEciKm, date),
      targetEciKm,
      targetVisible: lineOfSightToGroundTarget(sample.positionEciKm, targetEciKm)
    },
    attitude: {
      quaternion: quaternionFromBasis(bodyX, bodyY, bodyZ),
      bodyX,
      bodyY,
      bodyZ,
      boresightEci,
      mode: pointingModeFromReplay(sample),
      angularRateDegS: omegaBodyRadS ? radToDeg(norm(omegaBodyRadS)) : 0
    },
    altitudeKm: norm(sample.positionEciKm) - earthRadiusKm,
    speedKmS: norm(sample.velocityEciKmS),
    solarPowerW,
    replay: {
      qEciToBody: normalizeQuat(sample.qEciToBody),
      omegaBodyRadS,
      wheelMomentumNms: sample.wheelMomentumNms ?? sample.reactionWheelMomentumNms,
      rwTorqueBodyNm: sample.rwTorqueBodyNm,
      reactionWheelCommandNm: sample.reactionWheelCommandNm,
      rwTauWheelsNm: sample.rwTauWheelsNm,
      mtqDipoleAm2: sample.mtqDipoleAm2 ?? sample.magnetorquerCommandAm2,
      bEciT,
      attitudeValid: sample.quality?.attitudeValid,
      starTrackerValid: sample.quality?.starTrackerValid,
      sampleSource: sample.quality?.sampleSource
    }
  };
}

function normalizeReplaySample(raw: unknown, index: number): ReplaySample {
  if (!isRecord(raw)) throw new Error(`Replay sample ${index} must be an object.`);
  const sample = raw as Partial<ReplaySample>;
  const tSec = numberField(sample.tSec, `samples[${index}].tSec`);
  const velocityRaw = sample.velocityEciKmS ?? sample.velocityEciKmps;
  const bRaw = sample.bEciT ?? sample.magneticFieldEciT;
  const rwRaw = sample.rwTorqueBodyNm ?? sample.reactionWheelTorqueBodyNm;
  const mtqRaw = sample.mtqDipoleAm2 ?? sample.magnetorquerCommandAm2;
  const quality = sample.quality ?? {
    attitudeValid: flagOrUndefined(sample.attitudeValid),
    starTrackerValid: flagOrUndefined(sample.trackerValid),
    sampleSource: "logged"
  };
  return {
    ...sample,
    tSec,
    positionEciKm: vec3Field(sample.positionEciKm, `samples[${index}].positionEciKm`),
    velocityEciKmS: vec3Field(velocityRaw, `samples[${index}].velocityEciKmS or velocityEciKmps`),
    qEciToBody: quatField(sample.qEciToBody, `samples[${index}].qEciToBody`),
    omegaBodyRadS: sample.omegaBodyRadS ? vec3Field(sample.omegaBodyRadS, `samples[${index}].omegaBodyRadS`) : undefined,
    wheelMomentumNms: arrayField(sample.wheelMomentumNms ?? sample.reactionWheelMomentumNms, `samples[${index}].wheelMomentumNms or reactionWheelMomentumNms`),
    rwTorqueBodyNm: rwRaw ? vec3Field(rwRaw, `samples[${index}].rwTorqueBodyNm or reactionWheelTorqueBodyNm`) : undefined,
    reactionWheelCommandNm: sample.reactionWheelCommandNm ? vec3Field(sample.reactionWheelCommandNm, `samples[${index}].reactionWheelCommandNm`) : undefined,
    rwTauWheelsNm: arrayField(sample.rwTauWheelsNm ?? sample.reactionWheelTorqueWheelNm, `samples[${index}].rwTauWheelsNm or reactionWheelTorqueWheelNm`),
    mtqDipoleAm2: mtqRaw ? vec3Field(mtqRaw, `samples[${index}].mtqDipoleAm2 or magnetorquerCommandAm2`) : undefined,
    externalTorqueNm: sample.externalTorqueNm ? vec3Field(sample.externalTorqueNm, `samples[${index}].externalTorqueNm`) : undefined,
    sunEci: sample.sunEci ? vec3Field(sample.sunEci, `samples[${index}].sunEci`) : undefined,
    moonEci: sample.moonEci ? vec3Field(sample.moonEci, `samples[${index}].moonEci`) : undefined,
    bEciT: bRaw ? vec3Field(bRaw, `samples[${index}].bEciT or magneticFieldEciT`) : undefined,
    mode: sample.mode ?? modeNameFromId(sample.modeId),
    quality
  };
}

function cameraAxisBody(replay: ReplayExport): Vec3 {
  const sensors = replay.spacecraft?.sensorBoresightsBody ?? [];
  const camera = sensors.find((sensor) => sensor.id.toLowerCase().includes("camera")) ?? sensors.find((sensor) => sensor.kind === "conic");
  return normalize(camera?.axisBody ?? [0, 0, 1], [0, 0, 1]);
}

function phaseFromReplay(sample: ReplaySample): MissionPhaseName {
  if (sample.eclipse) return "Eclipse";
  const mode = (sample.mode ?? "").toLowerCase();
  if (mode.includes("detumble")) return "Detumble";
  if (mode.includes("sun")) return "Sun Acquire";
  if (mode.includes("target") || mode.includes("track")) return "Target Track";
  if (mode.includes("downlink")) return "Downlink";
  if (mode.includes("dump") || mode.includes("momentum")) return "Momentum Dump";
  return "Target Track";
}

function pointingModeFromReplay(sample: ReplaySample): Exclude<PointingMode, "auto"> {
  const mode = (sample.mode ?? "").toLowerCase();
  if (mode.includes("detumble")) return "detumble";
  if (mode.includes("sun")) return "sun-track";
  if (mode.includes("nadir")) return "nadir";
  if (mode.includes("target") || mode.includes("track")) return "target-track";
  return "inertial";
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

function replayDataSource(replay: ReplayExport): string {
  const source = replay.provenance?.source ?? "MATLAB/Simulink";
  const model = replay.provenance?.model ? ` ${replay.provenance.model}` : "";
  const commit = replay.provenance?.gitCommit ? ` @ ${replay.provenance.gitCommit.slice(0, 8)}` : "";
  return `${source}${model}${commit}`;
}

function parseReplayDate(sample: ReplaySample, fallbackEpochIso: string, fallbackOffsetSec: number): Date {
  if (sample.utc) return new Date(sample.utc);
  const base = new Date(fallbackEpochIso);
  return new Date(base.getTime() + fallbackOffsetSec * 1000);
}

function vec3Field(value: unknown, label: string): Vec3 {
  if (!Array.isArray(value) || value.length !== 3) throw new Error(`${label} must be a 3-element array.`);
  const out = value.map((item) => numberField(item, label)) as Vec3;
  return out;
}

function quatField(value: unknown, label: string): Quat {
  if (!Array.isArray(value) || value.length !== 4) throw new Error(`${label} must be a 4-element scalar-first quaternion.`);
  return normalizeQuat(value.map((item) => numberField(item, label)) as Quat);
}

function arrayField(value: unknown, label: string): number[] | undefined {
  if (value === undefined) return undefined;
  if (!Array.isArray(value)) throw new Error(`${label} must be an array.`);
  return value.map((item) => numberField(item, label));
}

function numberField(value: unknown, label: string): number {
  const numberValue = Number(value);
  if (!Number.isFinite(numberValue)) throw new Error(`${label} must be finite.`);
  return numberValue;
}

function epochFromRootReplay(raw: Record<string, unknown>): string | undefined {
  if (typeof raw.epochIso === "string") {
    const date = new Date(raw.epochIso);
    if (!Number.isFinite(date.getTime())) throw new Error("Replay epochIso must be a valid date.");
    return date.toISOString();
  }
  if (!isRecord(raw.simulation) || raw.simulation.epochJd === undefined) return undefined;
  const jd = numberField(raw.simulation.epochJd, "simulation.epochJd");
  const date = new Date((jd - 2440587.5) * 86400000);
  if (!Number.isFinite(date.getTime())) throw new Error("Replay epochJd is outside the supported date range.");
  return date.toISOString();
}

function eventsFromRootReplay(raw: Record<string, unknown>): ReplayExport["events"] {
  const events = raw.events;
  if (events === undefined) return [];
  if (isRecord(events) && (Array.isArray(events.modeChanges) || isRecord(events.modeChanges))) {
    // MATLAB jsonencode emits a scalar struct as an object, not a one-item array.
    const changes = Array.isArray(events.modeChanges) ? events.modeChanges : [events.modeChanges];
    return changes.map((event, index) => {
      if (!isRecord(event)) throw new Error(`Invalid mode event ${index}.`);
      return { tSec: numberField(event.tSec, `events.modeChanges[${index}].tSec`),
        kind: "mode", label: modeNameFromId(event.modeId) ?? `Mode ${event.modeId}` };
    });
  }
  if (!Array.isArray(events)) throw new Error("Replay events must be an array or a modeChanges object.");
  return events.map((event, index) => {
    if (!isRecord(event) || typeof event.label !== "string") throw new Error(`Invalid replay event ${index}.`);
    return { tSec: numberField(event.tSec, `events[${index}].tSec`), label: event.label,
      kind: typeof event.kind === "string" ? event.kind : undefined };
  });
}

function constantsFromRootReplay(raw: Record<string, unknown>): ReplayExport["constants"] {
  const explicit = isRecord(raw.constants) ? raw.constants : undefined;
  const centralBody = isRecord(raw.centralBody) ? raw.centralBody : undefined;
  const explicitRadius = Number(explicit?.earthRadiusKm);
  const centralRadius = Number(centralBody?.radiusKm);
  const earthRadiusKm = Number.isFinite(explicitRadius) ? explicitRadius : Number.isFinite(centralRadius) ? centralRadius : undefined;
  return earthRadiusKm ? { earthRadiusKm } : explicit;
}

function provenanceFromRootReplay(raw: Record<string, unknown>): ReplayExport["provenance"] {
  const provenance = isRecord(raw.provenance) ? raw.provenance : {};
  return {
    source: typeof provenance.source === "string" ? provenance.source : undefined,
    model: typeof provenance.model === "string" ? provenance.model : undefined,
    gitBranch: typeof provenance.gitBranch === "string" ? provenance.gitBranch : undefined,
    gitCommit: typeof provenance.gitCommit === "string" ? provenance.gitCommit : undefined,
    generatedAtUtc:
      typeof provenance.generatedAtUtc === "string"
        ? provenance.generatedAtUtc
        : typeof provenance.generatedUtc === "string"
          ? provenance.generatedUtc
          : undefined,
    matlabVersion: typeof provenance.matlabVersion === "string" ? provenance.matlabVersion : undefined,
    simStopTimeSec:
      typeof provenance.simStopTimeSec === "number"
        ? provenance.simStopTimeSec
        : typeof provenance.durationSec === "number"
          ? provenance.durationSec
          : undefined,
    simFixedStepSec:
      typeof provenance.simFixedStepSec === "number"
        ? provenance.simFixedStepSec
        : isRecord(raw.simulation) && typeof raw.simulation.dtSec === "number"
          ? raw.simulation.dtSec
          : undefined,
    notes: Array.isArray(provenance.notes) ? provenance.notes.map(String) : undefined
  };
}

function framesFromRootReplay(raw: Record<string, unknown>): ReplayExport["frames"] {
  if (isRecord(raw.frames)) return raw.frames as ReplayExport["frames"];
  const simulation = isRecord(raw.simulation) ? raw.simulation : {};
  return {
    position: typeof simulation.positionFrame === "string" ? simulation.positionFrame : "ECI",
    velocity: typeof simulation.positionFrame === "string" ? simulation.positionFrame : "ECI",
    attitudeQuaternion:
      typeof simulation.attitudeQuaternionConvention === "string"
        ? simulation.attitudeQuaternionConvention
        : "scalar-first ECI-to-body passive DCM convention; renderer maps body axes into Three.js scene coordinates",
    angularRate: "body",
    units: {
      position: typeof simulation.positionUnits === "string" ? simulation.positionUnits : "km",
      velocity: "km/s",
      angularRate: "rad/s",
      wheelMomentum: "N*m*s",
      torque: "N*m",
      magneticField: "T"
    }
  };
}

function spacecraftFromRootReplay(raw: Record<string, unknown>): ReplayExport["spacecraft"] {
  const spacecraft = isRecord(raw.spacecraft) ? ({ ...raw.spacecraft } as NonNullable<ReplayExport["spacecraft"]>) : {};
  const visualization = isRecord(raw.visualization) ? raw.visualization : {};
  const cameraAxis = arrayAsVec3(visualization.cameraBoresightBody) ?? [0, 0, 1];
  const starTrackerAxis = arrayAsVec3(visualization.starTrackerBoresightBody);
  const cameraHalfAngleDeg = Number(visualization.cameraFovHalfDeg);
  return {
    ...spacecraft,
    cameraHalfAngleDeg: Number.isFinite(cameraHalfAngleDeg) ? cameraHalfAngleDeg : spacecraft.cameraHalfAngleDeg,
    sensorBoresightsBody:
      spacecraft.sensorBoresightsBody ??
      [
        { id: "camera", kind: "conic", axisBody: cameraAxis, halfAngleDeg: Number.isFinite(cameraHalfAngleDeg) ? cameraHalfAngleDeg : 8 },
        ...(starTrackerAxis ? [{ id: "star_tracker", kind: "boresight", axisBody: starTrackerAxis }] : [])
      ]
  };
}

function arrayAsVec3(value: unknown): Vec3 | undefined {
  if (!Array.isArray(value) || value.length !== 3) return undefined;
  const out = value.map((item) => Number(item));
  return out.every((item) => Number.isFinite(item)) ? (out as Vec3) : undefined;
}

function modeNameFromId(modeId: unknown): string | undefined {
  const id = Number(modeId);
  if (id === 1) return "detumble";
  if (id === 2) return "coarse";
  if (id === 3) return "fine";
  return undefined;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function legacyTimeseriesReplayToSamples(raw: Record<string, unknown>): ReplayExport {
  const metadata = isRecord(raw.metadata) ? raw.metadata : {};
  const timeseries = isRecord(raw.timeseries) ? raw.timeseries : {};
  const qSeries = seriesField(timeseries.q, "timeseries.q");
  const posSeries = seriesField(timeseries.pos_eci_m, "timeseries.pos_eci_m");
  const velSeries = seriesField(timeseries.vel_eci_mps, "timeseries.vel_eci_mps");
  const sampleTimes = qSeries.time.length ? qSeries.time : posSeries.time;
  if (sampleTimes.length < 2) throw new Error("Legacy replay needs at least two attitude or position samples.");

  const omegaSeries = optionalSeries(timeseries.omega_rad_s);
  const hSeries = optionalSeries(timeseries.h_w_nms);
  const rwBodySeries = optionalSeries(timeseries.rw_torque_body_nm);
  const rwCommandSeries = optionalSeries(timeseries.rw_cmd_nm);
  const wheelTauSeries = optionalSeries(timeseries.rw_tau_wheels_nm);
  const mtqSeries = optionalSeries(timeseries.mtq_cmd_am2);
  const sunSeries = optionalSeries(timeseries.sun_eci);
  const moonSeries = optionalSeries(timeseries.moon_eci);
  const bSeries = optionalSeries(timeseries.B_eci_T);
  const eclipseSeries = optionalSeries(timeseries.eclipse_flag);
  const modeSeries = optionalSeries(timeseries.mode_id);
  const attValidSeries = optionalSeries(timeseries.att_valid);
  const trackerValidSeries = optionalSeries(timeseries.tracker_valid);
  const generated = typeof metadata.generated_utc === "string" ? metadata.generated_utc : undefined;

  const samples: ReplaySample[] = sampleTimes.map((tSec) => {
    const q = vectorAt(qSeries, tSec, 4, [1, 0, 0, 0]) as Quat;
    const posM = vectorAt(posSeries, tSec, 3, [EARTH_RADIUS_KM * 1000, 0, 0]) as Vec3;
    const velMps = vectorAt(velSeries, tSec, 3, [0, 0, 0]) as Vec3;
    const modeId = optionalScalarAt(modeSeries, tSec);
    return {
      tSec,
      positionEciKm: scale(posM, 1 / 1000),
      velocityEciKmS: scale(velMps, 1 / 1000),
      qEciToBody: normalizeQuat(q),
      omegaBodyRadS: optionalVectorAt(omegaSeries, tSec, 3),
      wheelMomentumNms: optionalArrayAt(hSeries, tSec),
      rwTorqueBodyNm: optionalVectorAt(rwBodySeries, tSec, 3),
      reactionWheelCommandNm: optionalVectorAt(rwCommandSeries, tSec, 3),
      rwTauWheelsNm: optionalArrayAt(wheelTauSeries, tSec),
      mtqDipoleAm2: optionalVectorAt(mtqSeries, tSec, 3),
      sunEci: optionalVectorAt(sunSeries, tSec, 3),
      moonEci: optionalVectorAt(moonSeries, tSec, 3),
      bEciT: optionalVectorAt(bSeries, tSec, 3),
      eclipse: (optionalScalarAt(eclipseSeries, tSec) ?? 0) > 0.5,
      mode: modeId === 1 ? "detumble" : modeId === 2 ? "coarse" : modeId === 3 ? "fine" : undefined,
      quality: {
        attitudeValid: flagOrUndefined(optionalScalarAt(attValidSeries, tSec)),
        starTrackerValid: flagOrUndefined(optionalScalarAt(trackerValidSeries, tSec)),
        sampleSource: "legacy-timeseries"
      }
    };
  });

  return {
    schema: "utat.adcs.replay.v1",
    provenance: {
      source: "MATLAB/Simulink",
      model: typeof metadata.model === "string" ? metadata.model : "adcs_sim",
      generatedAtUtc: generated,
      simStopTimeSec: typeof metadata.duration_s === "number" ? metadata.duration_s : undefined,
      notes: [
        "Converted in-browser from legacy metadata.schema=adcs-simulink-replay-v1 timeseries export.",
        "Integration should export root schema=utat.adcs.replay.v1 with samples[] for the final contract."
      ]
    },
    frames: {
      position: "ECI",
      velocity: "ECI",
      attitudeQuaternion: "scalar-first ECI-to-body passive DCM convention; renderer maps body axes into Three.js scene coordinates",
      angularRate: "body",
      units: {
        position: "km",
        velocity: "km/s",
        angularRate: "rad/s",
        wheelMomentum: "N*m*s",
        torque: "N*m",
        magneticField: "T"
      }
    },
    constants: {
      earthRadiusKm: earthRadiusKmFromLegacy(raw) ?? 6371
    },
    spacecraft: {
      name: "UTAT ADCS CubeSat",
      dimensionsM: [0.1, 0.1, 0.3],
      cameraHalfAngleDeg: 8,
      sensorBoresightsBody: [{ id: "camera", kind: "conic", axisBody: [0, 0, 1], halfAngleDeg: 8 }]
    },
    samples
  };
}

function earthRadiusKmForReplay(replay: ReplayExport): number {
  const radius = replay.constants?.earthRadiusKm;
  return Number.isFinite(radius) && radius && radius > 6000 && radius < 7000 ? radius : EARTH_RADIUS_KM;
}

function earthRadiusKmFromLegacy(raw: Record<string, unknown>): number | undefined {
  const parameters = isRecord(raw.parameters) ? raw.parameters : undefined;
  if (!parameters) return undefined;
  const candidates = ["earth_radius_km", "earthRadiusKm", "Re_km", "R_E_km", "Re"];
  for (const key of candidates) {
    const value = Number(parameters[key]);
    if (Number.isFinite(value)) return value > 10000 ? value / 1000 : value;
  }
  return undefined;
}

interface NumericSeries {
  time: number[];
  values: number[][];
}

function seriesField(raw: unknown, label: string): NumericSeries {
  const series = optionalSeries(raw);
  if (!series || series.time.length === 0 || series.values.length === 0) throw new Error(`${label} is required in legacy replay.`);
  return series;
}

function optionalSeries(raw: unknown): NumericSeries | undefined {
  if (!isRecord(raw) || !Array.isArray(raw.time_s) || !Array.isArray(raw.values)) return undefined;
  const time = raw.time_s.map((item) => Number(item)).filter((item) => Number.isFinite(item));
  const values = raw.values.map((row) => (Array.isArray(row) ? row : [row]).map((item) => Number(item))).filter((row) => row.every((item) => Number.isFinite(item)));
  return { time, values };
}

function nearestIndex(series: NumericSeries, tSec: number): number {
  let bestIndex = 0;
  let bestDelta = Number.POSITIVE_INFINITY;
  for (let index = 0; index < series.time.length; index += 1) {
    const delta = Math.abs(series.time[index] - tSec);
    if (delta < bestDelta) {
      bestDelta = delta;
      bestIndex = index;
    }
  }
  return Math.min(bestIndex, series.values.length - 1);
}

function vectorAt(series: NumericSeries, tSec: number, length: number, fallback: number[]): number[] {
  const row = series.values[nearestIndex(series, tSec)] ?? fallback;
  return Array.from({ length }, (_, index) => Number(row[index] ?? fallback[index] ?? 0));
}

function optionalVectorAt(series: NumericSeries | undefined, tSec: number, length: 3): Vec3 | undefined {
  if (!series) return undefined;
  return vectorAt(series, tSec, length, [0, 0, 0]) as Vec3;
}

function optionalArrayAt(series: NumericSeries | undefined, tSec: number): number[] | undefined {
  if (!series) return undefined;
  return series.values[nearestIndex(series, tSec)];
}

function optionalScalarAt(series: NumericSeries | undefined, tSec: number): number | undefined {
  if (!series) return undefined;
  return series.values[nearestIndex(series, tSec)]?.[0];
}

function flagOrUndefined(value: number | boolean | undefined): boolean | undefined {
  if (typeof value === "boolean") return value;
  return value === undefined || Number.isNaN(value) ? undefined : value > 0.5;
}
