# ADCS Web Mission GUI

This Vite/WebGL app is a browser-side mission interface for the ADCS simulator. It renders a scaled Earth-orbit scene with the spacecraft, Sun/Moon context vectors, eclipse volume, ground target, phase-colored trajectory tubes, and interactive orbit/chase cameras.

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

If local command execution fails before PowerShell or `cmd` starts with `CreateProcessAsUserW failed: 1920`, restart the Codex/local runner before validating. After shell access is restored, run:

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
