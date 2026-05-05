import type { EphemerisSample, MissionConfig, ProjectDefaults, Vec3 } from "../types";
import { EARTH_RADIUS_KM } from "../physics/constants";
import { defaultMissionConfig } from "../physics/mission";
import { normalize } from "../physics/math";

export async function loadProjectData(): Promise<{ config: MissionConfig; ephemeris: EphemerisSample[]; source: string }> {
  const [defaults, ephemeris] = await Promise.all([loadDefaults(), loadEphemeris()]);
  const base = defaultMissionConfig();
  if (!defaults) {
    return { config: base, ephemeris, source: "Built-in ADCS defaults" };
  }

  const config: MissionConfig = {
    ...base,
    spacecraft: defaults.spacecraft,
    target: defaults.target,
    startOrbit: {
      ...base.startOrbit,
      semiMajorAxisKm: EARTH_RADIUS_KM + defaults.orbit.startAltitudeKm,
      eccentricity: defaults.orbit.eccentricity,
      inclinationDeg: defaults.orbit.inclinationDeg,
      raanDeg: defaults.orbit.raanDeg,
      argPerigeeDeg: defaults.orbit.argPerigeeDeg,
      trueAnomalyDeg: defaults.orbit.trueAnomalyDeg
    },
    endOrbit: {
      ...base.endOrbit,
      semiMajorAxisKm: EARTH_RADIUS_KM + defaults.orbit.endAltitudeKm,
      eccentricity: defaults.orbit.eccentricity,
      inclinationDeg: defaults.orbit.inclinationDeg + 2.4,
      raanDeg: defaults.orbit.raanDeg + 4,
      argPerigeeDeg: defaults.orbit.argPerigeeDeg,
      trueAnomalyDeg: 205
    }
  };

  return { config, ephemeris, source: defaults.source };
}

async function loadDefaults(): Promise<ProjectDefaults | null> {
  try {
    const response = await fetch("./data/adcs-project-defaults.json", { cache: "no-store" });
    if (!response.ok) return null;
    return (await response.json()) as ProjectDefaults;
  } catch {
    return null;
  }
}

async function loadEphemeris(): Promise<EphemerisSample[]> {
  try {
    const response = await fetch("./data/ephemeris_2026_weekly.csv", { cache: "no-store" });
    if (!response.ok) return [];
    const text = await response.text();
    return parseEphemerisCsv(text);
  } catch {
    return [];
  }
}

export function parseEphemerisCsv(text: string): EphemerisSample[] {
  const lines = text
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean);
  if (lines.length < 2) return [];

  const header = lines[0].split(",").map((part) => part.trim().toLowerCase());
  const indexOf = (names: string[]) => names.map((name) => header.indexOf(name)).find((index) => index !== -1) ?? -1;
  const dateIndex = indexOf(["date", "epoch", "datetime", "utc", "time"]);
  const sunIdx = [
    indexOf(["sun_x", "sun_eci_x", "sunx", "sun_dir_x"]),
    indexOf(["sun_y", "sun_eci_y", "suny", "sun_dir_y"]),
    indexOf(["sun_z", "sun_eci_z", "sunz", "sun_dir_z"])
  ];
  const moonIdx = [
    indexOf(["moon_x", "moon_eci_x", "moonx", "moon_dir_x"]),
    indexOf(["moon_y", "moon_eci_y", "moony", "moon_dir_y"]),
    indexOf(["moon_z", "moon_eci_z", "moonz", "moon_dir_z"])
  ];
  if (dateIndex < 0 || sunIdx.some((idx) => idx < 0)) return [];

  return lines
    .slice(1)
    .map((line) => line.split(",").map((part) => part.trim()))
    .map((cols) => {
      const date = new Date(cols[dateIndex]);
      const sun = normalize([toNumber(cols[sunIdx[0]]), toNumber(cols[sunIdx[1]]), toNumber(cols[sunIdx[2]])] as Vec3);
      const moon: Vec3 =
        moonIdx.some((idx) => idx < 0)
          ? [0.2, 0.8, 0.1]
          : normalize([toNumber(cols[moonIdx[0]]), toNumber(cols[moonIdx[1]]), toNumber(cols[moonIdx[2]])] as Vec3);
      return {
        epochMs: date.getTime(),
        sunEci: sun,
        moonEci: moon
      };
    })
    .filter((sample) => Number.isFinite(sample.epochMs))
    .sort((a, b) => a.epochMs - b.epochMs);
}

function toNumber(value: string | undefined): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
}
