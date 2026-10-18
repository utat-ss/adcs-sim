import "./styles.css";
import { AdcsScene } from "./render/AdcsScene";
import { loadProjectData } from "./data/loaders";
import { parseReplayJson, replayToMissionTimeline } from "./data/replay";
import { DETUMBLE_RESIDUAL_RATE_DEG_S, detumbleProgressForRate } from "./physics/attitude";
import { buildMissionTimeline, defaultMissionConfig, missionConfigFromForm } from "./physics/mission";
import { hohmannDeltaV, orbitalPeriodSeconds } from "./physics/orbit";
import { EARTH_RADIUS_KM } from "./physics/constants";
import type { EphemerisSample, LayerVisibility, MissionConfig, MissionSample, MissionTimeline, SceneViewMode, Vec3 } from "./types";
import { clamp, dot, norm, normalize, scale, sub } from "./physics/math";

const canvas = query<HTMLCanvasElement>("#mission-canvas");
const form = query<HTMLFormElement>("#mission-form");
const playToggle = query<HTMLButtonElement>("#play-toggle");
const viewEarth = query<HTMLButtonElement>("#view-earth");
const viewOrbit = query<HTMLButtonElement>("#view-orbit");
const viewSat = query<HTMLButtonElement>("#view-sat");
const ambientToggle = query<HTMLButtonElement>("#ambient-toggle");
const resetForm = query<HTMLButtonElement>("#reset-form");
const replayFile = query<HTMLInputElement>("#replay-file");
const loadDemoReplay = query<HTMLButtonElement>("#load-demo-replay");
const usePreview = query<HTMLButtonElement>("#use-preview");
const timeScrubber = query<HTMLInputElement>("#time-scrubber");
const speedSelect = query<HTMLSelectElement>("#speed-select");
const cameraSelect = query<HTMLSelectElement>("#camera-select");
const telemetryPlot = query<HTMLCanvasElement>("#telemetry-plot");

const scene = new AdcsScene(canvas);
let missionConfig = defaultMissionConfig();
let timeline: MissionTimeline | null = null;
let ephemeris: EphemerisSample[] = [];
let dataSource = "Built-in ADCS defaults";
let isScrubbing = false;

void initialize();

async function initialize(): Promise<void> {
  const loaded = await loadProjectData();
  missionConfig = loaded.config;
  ephemeris = loaded.ephemeris;
  dataSource = loaded.source;
  fillForm(missionConfig);
  bindEvents();
  applyMission(missionConfig);
  scene.setTimeScale(Number(speedSelect.value));
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

  usePreview.addEventListener("click", () => applyMission(missionConfigFromForm(form, missionConfig)));

  loadDemoReplay.addEventListener("click", async () => {
    const response = await fetch("/data/matlab-replay-demo.json");
    if (!response.ok) throw new Error("Demo replay JSON is unavailable.");
    applyReplayText(await response.text(), "Demo replay");
  });

  replayFile.addEventListener("change", async () => {
    const file = replayFile.files?.[0];
    if (!file) return;
    applyReplayText(await file.text(), file.name);
  });

  playToggle.addEventListener("click", () => {
    const playing = scene.togglePlaying();
    if (playing && Number(speedSelect.value) === 0) {
      speedSelect.value = "1";
      scene.setTimeScale(1);
    }
    updatePlayButton(playing);
  });

  speedSelect.addEventListener("change", () => {
    const speed = Number(speedSelect.value);
    scene.setTimeScale(speed);
    scene.setPlaying(speed > 0);
    updatePlayButton(speed > 0);
  });

  timeScrubber.addEventListener("input", () => {
    isScrubbing = true;
    scene.setMissionSeconds(Number(timeScrubber.value));
    setText("#scrub-time-output", formatDuration(Number(timeScrubber.value)));
  });

  timeScrubber.addEventListener("change", () => {
    scene.setMissionSeconds(Number(timeScrubber.value));
    isScrubbing = false;
  });

  viewEarth.addEventListener("click", () => setView("earth"));
  viewOrbit.addEventListener("click", () => setView("orbit"));
  viewSat.addEventListener("click", () => setView("spacecraft"));
  cameraSelect.addEventListener("change", () => setView(cameraSelect.value as Exclude<SceneViewMode, "chase">));

  query<HTMLElement>("#layer-controls").addEventListener("change", () => {
    const layers: Partial<LayerVisibility> = {};
    const controls = Array.from(document.querySelectorAll<HTMLInputElement>("#layer-controls input[data-layer]"));
    for (const control of controls) {
      layers[control.dataset.layer as keyof LayerVisibility] = control.checked;
    }
    scene.setLayerVisibility(layers);
    setText("#layer-count-output", `${controls.filter((control) => control.checked).length} on`);
  });

  ambientToggle.addEventListener("click", () => {
    document.body.classList.toggle("ambient");
  });
}

function applyMission(config: MissionConfig): void {
  timeline = buildMissionTimeline(config, ephemeris, dataSource);
  timeline.sourceKind = "preview";
  applyTimeline(timeline);
  setText("#source-mode-output", "Preview");
  setText("#schema-output", "preview");
  setText("#provenance-output", "Built-in analytic preview");
  setText("#mission-subtitle", "Preview mode · synthetic orbit and attitude");
  setText("#assumption-note", "Preview mode is illustrative web-only trajectory and attitude logic. It is useful for UI checks, not as validated Simulink output.");
}

function applyReplayText(text: string, label: string): void {
  try {
    const replay = parseReplayJson(text);
    timeline = replayToMissionTimeline(replay, missionConfigFromForm(form, missionConfig));
    applyTimeline(timeline);
    const sourceLabel = replay.provenance?.model ? `${label} · ${replay.provenance.model}` : label;
    setText("#source-mode-output", "MATLAB replay");
    setText("#schema-output", replay.schema);
    setText("#provenance-output", formatProvenance(timeline));
    setText("#mission-subtitle", `${sourceLabel} · logged simulator replay`);
    setText(
      "#assumption-note",
      "Replay mode draws logged MATLAB/Simulink states. The enlarged spacecraft, sensor cone, and footprint are visualization geometry; numerical parity comes from the exported telemetry, not this renderer."
    );
  } catch (error) {
    setText("#assumption-note", error instanceof Error ? error.message : "Replay import failed.");
  }
}

function applyTimeline(currentTimeline: MissionTimeline): void {
  timeline = currentTimeline;
  scene.onSample = updateTelemetry;
  scene.setTimeline(currentTimeline);
  renderPhaseList(currentTimeline);
  updateDerivedOutputs(currentTimeline.config, currentTimeline);
  setText("#data-source", dataSourceLabel(currentTimeline));
  setText("#samples-output", `${currentTimeline.samples.length} samples`);
  timeScrubber.min = "0";
  timeScrubber.max = String(Math.max(1, Math.round(currentTimeline.config.durationHours * 3600)));
  timeScrubber.step = currentTimeline.samples.length > 1 ? String(Math.max(0.001, currentTimeline.samples[1].tSeconds - currentTimeline.samples[0].tSeconds)) : "1";
  timeScrubber.value = "0";
  drawTelemetryPlot(currentTimeline, 0);
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
  updateReplayTelemetry(sample);
  setTimelineProgress(sample.tSeconds);
  if (!isScrubbing) {
    timeScrubber.value = String(sample.tSeconds);
    setText("#scrub-time-output", formatDuration(sample.tSeconds));
  }
  if (timeline) drawTelemetryPlot(timeline, sample.tSeconds);
}

function updateReplayTelemetry(sample: MissionSample): void {
  const replay = sample.replay;
  if (!replay) {
    setText("#quality-output", "preview");
    setText("#omega-output", "not logged");
    setText("#wheel-output", "not logged");
    setText("#rw-output", "not logged");
    setText("#mtq-output", "not logged");
    return;
  }

  const attitude = replay.attitudeValid === false ? "attitude invalid" : "attitude valid";
  const tracker = replay.starTrackerValid === false ? "tracker invalid" : replay.starTrackerValid === true ? "tracker valid" : "tracker n/a";
  setText("#quality-output", `${attitude} · ${tracker}`);
  setText("#omega-output", replay.omegaBodyRadS ? `${formatVec(replay.omegaBodyRadS)} rad/s` : "not logged");
  setText("#wheel-output", replay.wheelMomentumNms ? `[${replay.wheelMomentumNms.map((value) => value.toFixed(4)).join(", ")}] Nms` : "not logged");
  setText("#rw-output", replay.rwTorqueBodyNm ? `${formatVec(replay.rwTorqueBodyNm)} Nm` : "not logged");
  setText("#mtq-output", replay.mtqDipoleAm2 ? `${formatVec(replay.mtqDipoleAm2)} A·m²` : "not logged");
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

function setView(mode: Exclude<SceneViewMode, "chase">): void {
  scene.setViewMode(mode);
  cameraSelect.value = mode;
  setCameraReadout(mode === "earth" ? "Earth camera" : mode === "spacecraft" ? "Spacecraft camera" : "Orbit camera");
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
    return {
      label: "Target Track",
      progressPercent: alignmentPercent(sample.attitude.boresightEci, target),
      detail: sample.environment.targetVisible ? "Target visible" : "Target below horizon",
      phase
    };
  }

  if (sample.attitude.mode === "sun-track") {
    return {
      label: "Sun Track",
      progressPercent: alignmentPercent(sample.attitude.boresightEci, sample.environment.sunEci),
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

function drawTelemetryPlot(currentTimeline: MissionTimeline, activeT: number): void {
  const ctx = telemetryPlot.getContext("2d");
  if (!ctx) return;
  const width = telemetryPlot.width;
  const height = telemetryPlot.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "rgba(2, 6, 23, 0.96)";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(148, 163, 184, 0.22)";
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i += 1) {
    const y = (height / 4) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  const duration = Math.max(1, currentTimeline.config.durationHours * 3600);
  const rates = currentTimeline.samples.map((sample) => sample.attitude.angularRateDegS);
  const momenta = currentTimeline.samples.map((sample) => normArray(sample.replay?.wheelMomentumNms));
  drawSeries(ctx, currentTimeline.samples, rates, Math.max(0.01, ...rates), duration, width, height, "#7dd3fc");
  if (momenta.some((value) => value > 0)) drawSeries(ctx, currentTimeline.samples, momenta, Math.max(0.001, ...momenta), duration, width, height, "#4ade80");

  const x = clamp(activeT / duration, 0, 1) * width;
  ctx.strokeStyle = "rgba(248, 250, 252, 0.72)";
  ctx.beginPath();
  ctx.moveTo(x, 0);
  ctx.lineTo(x, height);
  ctx.stroke();
  ctx.fillStyle = "#94a3b8";
  ctx.font = "22px system-ui";
  ctx.fillText("rate deg/s", 12, 28);
  ctx.fillStyle = "#4ade80";
  ctx.fillText("|hW| Nms", 12, 55);
}

function drawSeries(
  ctx: CanvasRenderingContext2D,
  samples: MissionSample[],
  values: number[],
  maxValue: number,
  duration: number,
  width: number,
  height: number,
  color: string
): void {
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  values.forEach((value, index) => {
    const x = clamp(samples[index].tSeconds / duration, 0, 1) * width;
    const y = height - clamp(value / maxValue, 0, 1) * (height - 18) - 8;
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function normArray(values: number[] | undefined): number {
  if (!values?.length) return 0;
  return Math.hypot(...values);
}

function dataSourceLabel(currentTimeline: MissionTimeline): string {
  if (currentTimeline.sourceKind === "replay") {
    const frames = currentTimeline.frames?.attitudeQuaternion ?? "scalar-first ECI-to-body";
    return `${currentTimeline.dataSource} | ${currentTimeline.samples.length} logged samples | ${frames}`;
  }
  return `${currentTimeline.dataSource} | ${ephemeris.length || "analytic"} ephemeris samples | Preview`;
}

function formatProvenance(currentTimeline: MissionTimeline): string {
  const p = currentTimeline.provenance;
  if (!p) return "No provenance block";
  const parts = [p.source, p.model, p.matlabVersion, p.gitCommit ? p.gitCommit.slice(0, 8) : undefined, p.generatedAtUtc].filter(Boolean);
  return parts.join(" · ") || "Provenance present";
}

function updatePlayButton(playing: boolean): void {
  playToggle.textContent = playing ? "II" : ">";
  playToggle.setAttribute("aria-label", playing ? "Pause propagation" : "Resume propagation");
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
