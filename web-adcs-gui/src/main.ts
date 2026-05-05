import "./styles.css";
import { AdcsScene } from "./render/AdcsScene";
import { loadProjectData } from "./data/loaders";
import { DETUMBLE_RESIDUAL_RATE_DEG_S, detumbleProgressForRate } from "./physics/attitude";
import { buildMissionTimeline, defaultMissionConfig, missionConfigFromForm } from "./physics/mission";
import { hohmannDeltaV, orbitalPeriodSeconds } from "./physics/orbit";
import { EARTH_RADIUS_KM } from "./physics/constants";
import type { EphemerisSample, MissionConfig, MissionSample, MissionTimeline, Vec3 } from "./types";
import { clamp, dot, norm, normalize, scale, sub } from "./physics/math";

const canvas = query<HTMLCanvasElement>("#mission-canvas");
const form = query<HTMLFormElement>("#mission-form");
const playToggle = query<HTMLButtonElement>("#play-toggle");
const viewEarth = query<HTMLButtonElement>("#view-earth");
const viewSat = query<HTMLButtonElement>("#view-sat");
const ambientToggle = query<HTMLButtonElement>("#ambient-toggle");
const resetForm = query<HTMLButtonElement>("#reset-form");

const scene = new AdcsScene(canvas);
let missionConfig = defaultMissionConfig();
let timeline: MissionTimeline | null = null;
let ephemeris: EphemerisSample[] = [];
let dataSource = "Built-in ADCS defaults";

void initialize();

async function initialize(): Promise<void> {
  const loaded = await loadProjectData();
  missionConfig = loaded.config;
  ephemeris = loaded.ephemeris;
  dataSource = loaded.source;
  fillForm(missionConfig);
  applyMission(missionConfig);
  bindEvents();
  scene.start();
}

function bindEvents(): void {
  form.addEventListener("submit", (event) => {
    event.preventDefault();
    missionConfig = missionConfigFromForm(form, missionConfig);
    applyMission(missionConfig);
  });

  form.addEventListener("input", () => {
    const draft = missionConfigFromForm(form, missionConfig);
    updateDerivedOutputs(draft, timeline);
  });

  resetForm.addEventListener("click", () => {
    missionConfig = defaultMissionConfig();
    fillForm(missionConfig);
    applyMission(missionConfig);
  });

  playToggle.addEventListener("click", () => {
    const playing = scene.togglePlaying();
    playToggle.textContent = playing ? "II" : ">";
    playToggle.setAttribute("aria-label", playing ? "Pause propagation" : "Resume propagation");
  });

  viewEarth.addEventListener("click", () => {
    scene.setViewMode("orbit");
    setCameraReadout("Orbit camera");
  });

  viewSat.addEventListener("click", () => {
    scene.setViewMode("chase");
    setCameraReadout("Satellite chase");
  });

  ambientToggle.addEventListener("click", () => {
    document.body.classList.toggle("ambient");
  });
}

function applyMission(config: MissionConfig): void {
  timeline = buildMissionTimeline(config, ephemeris, dataSource);
  scene.setTimeline(timeline);
  scene.onSample = updateTelemetry;
  renderPhaseList(timeline);
  updateDerivedOutputs(config, timeline);
  setText("#data-source", `${timeline.dataSource} | ${ephemeris.length || "analytic"} ephemeris samples`);
  setText("#samples-output", `${timeline.samples.length} samples`);
}

function updateDerivedOutputs(config: MissionConfig, currentTimeline: MissionTimeline | null): void {
  const periodMin = orbitalPeriodSeconds(config.startOrbit.semiMajorAxisKm) / 60;
  const dv = hohmannDeltaV(config.startOrbit.semiMajorAxisKm, config.endOrbit.semiMajorAxisKm);
  setText("#period-output", `${periodMin.toFixed(1)} min`);
  setText("#dv-output", `${(currentTimeline?.transferDeltaVKmS ?? dv.totalKmS).toFixed(3)} km/s`);
}

function updateTelemetry(sample: MissionSample): void {
  setText("#phase-label", sample.phase);
  setText("#mission-time", `T+${formatDuration(sample.tSeconds)}`);
  setText("#mission-date", sample.date.toISOString().replace("T", " ").slice(0, 16) + " UTC");
  setText("#speed-output", `${sample.speedKmS.toFixed(2)} km/s`);
  setText("#radius-output", `${norm(sample.state.positionKm).toFixed(0)} km`);
  setText("#altitude-output", `${sample.altitudeKm.toFixed(0)} km`);
  setText("#lat-output", `${sample.environment.groundPoint.latDeg.toFixed(1)} deg`);
  setText("#lon-output", `${sample.environment.groundPoint.lonDeg.toFixed(1)} deg`);
  setText("#beta-output", `${sample.environment.betaAngleDeg.toFixed(1)} deg`);
  setText("#power-output", `${sample.solarPowerW.toFixed(0)} W`);
  setText("#eclipse-output", sample.environment.eclipseFactor < 0.2 ? "Eclipse" : sample.environment.eclipseFactor < 0.98 ? "Penumbra" : "Sunlit");
  setText("#mode-output", labelMode(sample.attitude.mode));
  updateOperationProgress(sample);
  setText("#boresight-output", formatVec(sample.attitude.boresightEci));
  setText("#sun-output", formatVec(sample.environment.sunEci));
  setText("#mag-output", `${norm(sample.environment.magneticFieldUt).toFixed(1)} uT`);
  setTimelineProgress(sample.tSeconds);
}

function renderPhaseList(currentTimeline: MissionTimeline): void {
  const list = query<HTMLOListElement>("#phase-list");
  list.replaceChildren();
  for (const phase of currentTimeline.phases) {
    const item = document.createElement("li");
    item.style.setProperty("--phase-color", phase.color);
    const label = document.createElement("span");
    label.textContent = phase.name;
    const time = document.createElement("time");
    time.textContent = `${formatDuration(phase.startSeconds)} - ${formatDuration(phase.endSeconds)}`;
    item.append(label, time);
    list.append(item);
  }
}

function fillForm(config: MissionConfig): void {
  setInput("startAltitudeKm", config.startOrbit.semiMajorAxisKm - EARTH_RADIUS_KM);
  setInput("startEccentricity", config.startOrbit.eccentricity);
  setInput("startInclinationDeg", config.startOrbit.inclinationDeg);
  setInput("startRaanDeg", config.startOrbit.raanDeg);
  setInput("startArgPerigeeDeg", config.startOrbit.argPerigeeDeg);
  setInput("startTrueAnomalyDeg", config.startOrbit.trueAnomalyDeg);
  setInput("endAltitudeKm", config.endOrbit.semiMajorAxisKm - EARTH_RADIUS_KM);
  setInput("endEccentricity", config.endOrbit.eccentricity);
  setInput("endInclinationDeg", config.endOrbit.inclinationDeg);
  setInput("endRaanDeg", config.endOrbit.raanDeg);
  setInput("endArgPerigeeDeg", config.endOrbit.argPerigeeDeg);
  setInput("endTrueAnomalyDeg", config.endOrbit.trueAnomalyDeg);
  setInput("durationHours", config.durationHours);
  setInput("targetLatDeg", config.target.latDeg);
  setInput("targetLonDeg", config.target.lonDeg);
  setInput("trajectoryMode", config.trajectoryMode);
  setInput("pointingMode", config.pointingMode);
  setInput("epochIso", config.epochIso.replace("Z", "").slice(0, 16));
}

function setInput(name: string, value: string | number): void {
  const input = form.elements.namedItem(name) as HTMLInputElement | HTMLSelectElement | null;
  if (!input) return;
  input.value = typeof value === "number" ? formatInputNumber(value) : value;
}

function formatInputNumber(value: number): string {
  return Number.isInteger(value) ? String(value) : String(Number(value.toFixed(4)));
}

function setText(selector: string, text: string): void {
  query<HTMLElement>(selector).textContent = text;
}

function setCameraReadout(text: string): void {
  setText("#camera-readout", text);
}

function setTimelineProgress(tSeconds: number): void {
  const durationSeconds = (timeline?.config.durationHours ?? missionConfig.durationHours) * 3600;
  const percent = durationSeconds > 0 ? Math.min(100, Math.max(0, (tSeconds / durationSeconds) * 100)) : 0;
  query<HTMLElement>("#timeline-progress").style.width = `${percent.toFixed(2)}%`;
}

function updateOperationProgress(sample: MissionSample): void {
  const status = operationStatus(sample);
  const progress = clamp(status.progressPercent, 0, 100);
  const rail = query<HTMLElement>(".operation-rail");

  setText("#operation-label", status.label);
  setText("#operation-progress-output", `${progress.toFixed(0)}%`);
  setText("#operation-detail-output", status.detail);
  setText("#operation-phase-output", status.phase);
  query<HTMLElement>("#operation-progress").style.width = `${progress.toFixed(2)}%`;
  rail.setAttribute("aria-valuenow", progress.toFixed(0));
}

function operationStatus(sample: MissionSample): { label: string; progressPercent: number; detail: string; phase: string } {
  const phasePercent = phaseProgressPercent(sample);
  const phase = `${sample.phase} ${phasePercent.toFixed(0)}%`;

  if (sample.attitude.mode === "detumble") {
    const progressPercent = detumbleProgressForRate(sample.attitude.angularRateDegS) * 100;
    return {
      label: "Detumble",
      progressPercent,
      detail: `Rate ${sample.attitude.angularRateDegS.toFixed(3)} deg/s, goal ${DETUMBLE_RESIDUAL_RATE_DEG_S.toFixed(2)} deg/s`,
      phase
    };
  }

  if (sample.attitude.mode === "target-track") {
    const target = normalize(sub(sample.environment.targetEciKm, sample.state.positionKm), sample.attitude.boresightEci);
    const progressPercent = alignmentPercent(sample.attitude.boresightEci, target);
    return {
      label: "Target Track",
      progressPercent,
      detail: sample.environment.targetVisible ? "Target visible" : "Target below horizon",
      phase
    };
  }

  if (sample.attitude.mode === "sun-track") {
    const progressPercent = alignmentPercent(sample.attitude.boresightEci, sample.environment.sunEci);
    return {
      label: "Sun Track",
      progressPercent,
      detail: `Power ${sample.solarPowerW.toFixed(0)} W`,
      phase
    };
  }

  if (sample.attitude.mode === "nadir") {
    const nadir = normalize(scale(sample.state.positionKm, -1), sample.attitude.boresightEci);
    return {
      label: "Nadir Pointing",
      progressPercent: alignmentPercent(sample.attitude.boresightEci, nadir),
      detail: `Altitude ${sample.altitudeKm.toFixed(0)} km`,
      phase
    };
  }

  return {
    label: "Inertial Hold",
    progressPercent: clamp((1 - sample.attitude.angularRateDegS / 0.05) * 100, 0, 100),
    detail: `Rate ${sample.attitude.angularRateDegS.toFixed(3)} deg/s`,
    phase
  };
}

function alignmentPercent(a: Vec3, b: Vec3): number {
  return clamp(dot(normalize(a), normalize(b)), 0, 1) * 100;
}

function phaseProgressPercent(sample: MissionSample): number {
  const currentPhase = timeline?.phases.find((phase) => sample.tSeconds >= phase.startSeconds && sample.tSeconds <= phase.endSeconds);
  if (!currentPhase) return 0;
  const span = currentPhase.endSeconds - currentPhase.startSeconds;
  if (span <= 0) return 100;
  return clamp(((sample.tSeconds - currentPhase.startSeconds) / span) * 100, 0, 100);
}

function formatDuration(seconds: number): string {
  const total = Math.max(0, Math.floor(seconds));
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return [h, m, s].map((value) => String(value).padStart(2, "0")).join(":");
}

function formatVec(v: Vec3): string {
  return `[${v.map((value) => value.toFixed(2)).join(", ")}]`;
}

function labelMode(mode: string): string {
  return mode
    .split("-")
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

function query<T extends Element>(selector: string): T {
  const element = document.querySelector<T>(selector);
  if (!element) throw new Error(`Missing element: ${selector}`);
  return element;
}
