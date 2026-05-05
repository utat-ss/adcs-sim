import { copyFileSync, existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const webRoot = path.resolve(__dirname, "..");
const repoRoot = path.resolve(webRoot, "..");
const publicData = path.join(webRoot, "public", "data");
const defaultsPath = path.join(publicData, "adcs-project-defaults.json");
const initPath = path.join(repoRoot, "init_adcs_params.m");
const ephemerisSource = path.join(repoRoot, "ephemeris_2026_weekly.csv");
const ephemerisTarget = path.join(publicData, "ephemeris_2026_weekly.csv");

mkdirSync(publicData, { recursive: true });

const defaults = JSON.parse(readFileSync(defaultsPath, "utf8"));
const changes = [];

if (existsSync(initPath)) {
  const source = stripMatlabComments(readFileSync(initPath, "utf8"));
  const massKg = findFirstNumber(source, ["mass_sc", "spacecraft_mass", "massKg", "mass"]);
  const dimX = findFirstNumber(source, ["sc_dim_x", "bus_dim_x", "dim_x"]);
  const dimY = findFirstNumber(source, ["sc_dim_y", "bus_dim_y", "dim_y"]);
  const dimZ = findFirstNumber(source, ["sc_dim_z", "bus_dim_z", "dim_z"]);
  const solarArea = findFirstNumber(source, ["solar_panel_area", "panel_area", "solarArea"]);
  const solarEfficiency = findFirstNumber(source, ["solar_panel_efficiency", "solar_efficiency", "eta_solar"]);

  if (massKg) {
    defaults.spacecraft.massKg = massKg;
    changes.push(`mass=${massKg} kg`);
  }

  if (dimX && dimY && dimZ) {
    defaults.spacecraft.dimensionsM = [dimX, dimY, dimZ];
    const mass = defaults.spacecraft.massKg;
    defaults.spacecraft.inertiaKgM2 = [
      (mass / 12) * (dimY * dimY + dimZ * dimZ),
      (mass / 12) * (dimX * dimX + dimZ * dimZ),
      (mass / 12) * (dimX * dimX + dimY * dimY)
    ].map((value) => Number(value.toFixed(6)));
    changes.push(`dimensions=${dimX}x${dimY}x${dimZ} m`);
  }

  if (solarArea) {
    defaults.spacecraft.solarPanelAreaM2 = solarArea;
    changes.push(`solarArea=${solarArea} m2`);
  }

  if (solarEfficiency) {
    defaults.spacecraft.solarPanelEfficiency = solarEfficiency;
    changes.push(`solarEfficiency=${solarEfficiency}`);
  }

  const earthRadius = findFirstNumber(source, ["R_earth", "earth_radius_m", "earth_radius"]) ?? 6378137;
  const altitude = findFirstNumber(source, ["altitude_km", "orbit_altitude_km", "h0_km", "alt0_km", "orbit_alt"]);
  const semiMajorAxis = findFirstNumber(source, ["a0", "semi_major_axis_km", "sma_km", "orbit_a"]);
  const eccentricity = findFirstNumber(source, ["eccentricity", "e0", "e_orbit", "orbit_ecc"]);
  const inclination = findFirstNumber(source, ["inclination_deg", "inc_deg", "i_deg", "orbit_inc"]);
  const raan = findFirstNumber(source, ["raan_deg", "Omega_deg", "raan0_deg", "orbit_RAAN"]);
  const argPerigee = findFirstNumber(source, ["arg_perigee_deg", "omega_deg", "argp_deg", "orbit_AOP"]);
  const trueAnomaly = findFirstNumber(source, ["true_anomaly_deg", "nu_deg", "ta_deg", "orbit_TA"]);

  if (altitude !== null) defaults.orbit.startAltitudeKm = lengthToKm(altitude);
  if (altitude === null && semiMajorAxis !== null) {
    defaults.orbit.startAltitudeKm = lengthToKm(semiMajorAxis) - lengthToKm(earthRadius);
  }
  if (eccentricity !== null) defaults.orbit.eccentricity = eccentricity;
  if (inclination !== null) defaults.orbit.inclinationDeg = inclination;
  if (raan !== null) defaults.orbit.raanDeg = raan;
  if (argPerigee !== null) defaults.orbit.argPerigeeDeg = argPerigee;
  if (trueAnomaly !== null) defaults.orbit.trueAnomalyDeg = trueAnomaly;

  defaults.source = "Synced from init_adcs_params.m and ephemeris_2026_weekly.csv";
}

if (existsSync(ephemerisSource)) {
  copyFileSync(ephemerisSource, ephemerisTarget);
  changes.push("ephemeris_2026_weekly.csv copied");
}

writeFileSync(defaultsPath, `${JSON.stringify(defaults, null, 2)}\n`);
console.log(changes.length ? `ADCS data sync: ${changes.join(", ")}` : "ADCS data sync: using bundled defaults");

function stripMatlabComments(source) {
  return source
    .split(/\r?\n/)
    .map((line) => line.replace(/%.*/, ""))
    .join("\n");
}

function findFirstNumber(source, names) {
  for (const name of names) {
    const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
    const match = source.match(new RegExp(`(?:^|\\n)\\s*${escaped}\\s*=\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)`, "i"));
    if (match) {
      const value = Number(match[1]);
      if (Number.isFinite(value)) return value;
    }
  }
  return null;
}

function lengthToKm(value) {
  return Math.abs(value) > 10000 ? value / 1000 : value;
}
