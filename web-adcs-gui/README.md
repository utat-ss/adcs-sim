# ADCS Web Mission GUI

This Vite/WebGL app is a browser-side mission interface for the ADCS simulator. It renders a scaled Earth-orbit scene with the spacecraft, Sun/Moon context vectors, eclipse volume, ground target, phase-colored trajectory tubes, sensor geometry, footprint overlays, and interactive Earth/orbit/spacecraft cameras.

## Run

```powershell
cd web-adcs-gui
npm install
npm run dev
```

The dev/build scripts run `scripts/sync-adcs-data.mjs` first. That script copies `../ephemeris_2026_weekly.csv` into `public/data/` and derives available spacecraft/orbit defaults from `../init_adcs_params.m`. If a variable is not recognizable, the app keeps the bundled documented baseline: 4 kg 3U bus, geometry-derived inertia, 500 km LEO start orbit, and 2026 Sun/Moon fallback ephemeris.

## Validation

```powershell
npm run build
npm run test
npm run test:e2e
```

`vitest` covers orbital propagation, Hohmann transfer estimates, eclipse geometry, mission phase generation, and attitude quaternion normalization. Playwright checks desktop/mobile rendering, verifies the WebGL canvas is nonblank, exercises orbit-camera dragging, updates mission parameters, and checks ambient mode.

If local command execution fails before PowerShell or `cmd` starts with `CreateProcessAsUserW failed: 1920`, restart the local shell before validating. After shell access is restored, run:

```powershell
cd web-adcs-gui
npm install
npm run build
npm run test
npm run test:e2e
```

If a local reference recording is available, extract frames outside source control and tune against them:

```powershell
ffmpeg -i "path/to/local-reference-video.mp4" -vf fps=1 "reference-frame-%03d.png"
```

## Implemented Physics

- Earth-centered inertial propagation from classical orbital elements.
- Kepler equation solve for elliptical two-body motion.
- J2 secular RAAN and argument-of-perigee drift during propagation.
- Hohmann transfer delta-v and transfer-time estimates for altitude changes.
- Plane-change delta-v estimate for inclination retargeting.
- Earth rotation conversion for target and ground-track coordinates.
- Cylindrical Earth eclipse and penumbra approximation.
- Low-precision analytical Sun/Moon fallback when CSV data is absent.
- Simple dipole magnetic field magnitude for ADCS context.
- Scalar-first quaternion convention `[q0, q1, q2, q3]` internally.
- Pointing modes for detumbling, sun tracking, target tracking, nadir imaging, and inertial hold.

## MATLAB Replay Contract

The browser has two clearly labeled data modes:

- **Preview** builds a synthetic browser-only orbit and attitude timeline from the form fields. It is useful for UI review and rough mission visualization, but it is not validated Simulink output.
- **MATLAB replay** consumes logged simulator states exported as JSON and draws those states with provenance in the UI.

The target replay schema is `utat.adcs.replay.v1`. It uses a root `samples[]` array so browser and MATLAB viewers can consume the same file without guessing how to join separate time series:

```json
{
  "schema": "utat.adcs.replay.v1",
  "provenance": {
    "source": "MATLAB/Simulink",
    "model": "adcs_sim",
    "gitBranch": "simulink",
    "gitCommit": "optional",
    "generatedAtUtc": "2026-09-20T14:40:00Z",
    "matlabVersion": "R2026a",
    "simStopTimeSec": 960,
    "simFixedStepSec": 0.1,
    "notes": ["Visualization consumes logged states; it does not recompute ADCS dynamics."]
  },
  "frames": {
    "position": "ECI",
    "velocity": "ECI",
    "attitudeQuaternion": "scalar-first ECI-to-body passive DCM convention; renderer maps body axes into Three.js scene coordinates",
    "angularRate": "body",
    "units": {
      "position": "km",
      "velocity": "km/s",
      "angularRate": "rad/s",
      "wheelMomentum": "N*m*s",
      "torque": "N*m",
      "magneticField": "T"
    }
  },
  "constants": {
    "earthRadiusKm": 6371
  },
  "spacecraft": {
    "name": "UTAT ADCS CubeSat",
    "dimensionsM": [0.1, 0.1, 0.3],
    "cameraHalfAngleDeg": 8,
    "sensorBoresightsBody": [
      { "id": "camera", "kind": "conic", "axisBody": [0, 0, 1], "halfAngleDeg": 8 },
      { "id": "star_tracker", "kind": "boresight", "axisBody": [0, 0, 1] }
    ]
  },
  "samples": [
    {
      "tSec": 0,
      "utc": "2026-06-01T12:00:00.000Z",
      "positionEciKm": [6871, 0, 0],
      "velocityEciKmS": [0, 7.61, 0],
      "qEciToBody": [1, 0, 0, 0],
      "omegaBodyRadS": [0.01, 0, 0],
      "wheelMomentumNms": [0, 0, 0, 0],
      "rwTorqueBodyNm": [0, 0, 0],
      "rwTauWheelsNm": [0, 0, 0, 0],
      "mtqDipoleAm2": [0, 0, 0],
      "sunEci": [1, 0, 0],
      "moonEci": [0, 1, 0],
      "bEciT": [0, 0, 0.00003],
      "eclipse": false,
      "mode": "detumble",
      "quality": { "attitudeValid": true, "starTrackerValid": true, "sampleSource": "logged" }
    }
  ]
}
```

The browser also accepts the current draft exporter shape with `metadata.schema = "adcs-simulink-replay-v1"` and a `timeseries` object, converting it in-browser for review. That compatibility path is transitional; the final exporter should write `utat.adcs.replay.v1` directly.

Quaternion handling is deliberate: the renderer does not pass `qEciToBody` straight to Three.js. It derives body axes in ECI from the scalar-first passive convention and then maps those axes into the scene coordinate system. This avoids a sign/frame error caused by Three.js using active local-to-world object rotations and the scene using `[x, z, -y]` for ECI-to-render coordinates.

## Visualization Limits

The 22 September replay checks use actual MATLAB output. Root exports accept
`simulation.epochJd`, `centralBody.radiusKm`, `events.modeChanges` (including a
single MATLAB struct), and the delivered `reactionWheelTorqueBodyNm` /
`reactionWheelTorqueWheelNm` fields. `reactionWheelCommandNm` remains separate;
it is never substituted for missing delivered torque. Mode events are retained
even when no displayed sample lands on their timestamps.

- The spacecraft mesh is an illustrative 3U-style bus enlarged for visibility; it is not CAD-derived hardware.
- The default camera cone is an illustrative +Z body-axis cone with an 8 degree half angle unless the replay file supplies a different `spacecraft.sensorBoresightsBody` contract.
- The sensor footprint is a geometric ray/sphere intersection drawn from the logged/rendered attitude. It is a visualization overlay, not an STK numerical clone.
- The replay path reads `constants.earthRadiusKm` for altitude and footprint display. Export this field from MATLAB, or align constants in both viewers to avoid a 7 km class altitude offset.
- Star tracker validity is shown from `quality.starTrackerValid`; the browser does not run an estimator, MEKF, exclusion-angle solver, or hardware validation.
