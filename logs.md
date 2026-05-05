# Change Log

Use this file to record notable repository changes that affect model structure, simulation behavior, validation flow, configuration, or documentation.

## Logging guidelines

- Add newest entries at the top of the table.
- Prefer concrete, repository-level changes over minor edits.
- If an item is not committed yet, mark it as `Working tree`.
- Cross-reference matching completed items from `todo.md` when the change closes or documents tracked work.

## Recent changes

| Date | Status | Area | Change | Related completed tasks in `todo.md` |
|---|---|---|---|---|
| 2026-05-05 | Working tree | Web GUI operations | Reduced the Three.js spacecraft visual scale, added visual-only radial altitude exaggeration and wider camera framing so the default LEO satellite stays clear of the Earth/atmosphere shell, tightened panel density, changed the mobile status strip to avoid covering form fields, changed the browser detumble proxy to integrate a decaying body-rate model, added a live ADCS operation-progress rail for detumble/pointing modes, and expanded unit/e2e checks around detumble progress and render clearance. | — |
| 2026-04-23 | Working tree | Web GUI validation | Resumed post-runner validation for `web-adcs-gui/`: installed npm dependencies and Playwright Chromium, added Three.js type declarations, excluded Playwright specs from Vitest collection, corrected the near-circular orbit unit-test tolerance, changed eccentricity inputs to accept the synced `0.0001` MATLAB default, and made a first visual tuning pass with slimmer panels, wider camera framing, clearer lighting, and a bottom mission-progress rail. `npm run validate` now passes. | `Post-Restart Validation Checklist` |
| 2026-04-23 | Working tree | Repository hygiene | Added a root `.gitignore` to keep MATLAB/Simulink caches (`slprj/`, `*.slxc`, autosaves), LaTeX handbook intermediates, and local visual-reference captures out of normal source pushes; also documented the push-readiness guidance and recorded that a plaintext secret scan of the active source tree found no matches. | `Completed > Infrastructure > Repository push hygiene` |
| 2026-04-23 | Working tree | Physics and validation | Corrected the default inertia baseline to match the declared 4 kg, `0.10 x 0.10 x 0.30 m` bus geometry, aligned the live gravity-gradient DCM with the active sensor-frame convention, and expanded `tests\run_adcs_tests.m` with parameter-consistency plus frame-convention regression checks. | `Completed > Dynamics Subsystem > Geometry-consistent default inertia baseline`; `Completed > Infrastructure > Math regression checks for inertia and frame conventions` |
| 2026-04-23 | Working tree | Documentation | Updated `README.md`, `agents.md`, `reference.md`, `skills.md`, and the handbook source to document the geometry-derived inertia baseline and the shared ECI→body frame convention. | Documents the completed work recorded in `Completed > Dynamics Subsystem > Geometry-consistent default inertia baseline`; `Completed > Infrastructure > Math regression checks for inertia and frame conventions` |
| 2026-04-20 | Working tree | Build flow | Updated `build_adcs_model.m` to rebuild `adcs_sim.slx` in place by loading the existing generated model when present, clearing its contents, dropping stale `adcs_sim.slxc` cache artifacts, and saving back to the repository root so rebuilds avoid same-name model collisions. | `Completed > Infrastructure > Rebuild-safe model generation` |
| 2026-04-20 | Working tree | GUI and telemetry | Added `launch_adcs_gui.m` for interactive runs plus `simulate_adcs_case.m` and `summarize_adcs_simulation.m` to quantify attitude excursion, rate behavior, momentum, energy, and actuator utilization from the existing logged outputs. | `Completed > Infrastructure > Interactive GUI dashboard`; `Completed > Infrastructure > Quantified post-simulation metrics` |
| 2026-04-20 | Working tree | Structure | Refactored the generated-model source layout around `build_adcs_model.m`, subsystem builders in `builders\`, and shared helpers in `utils\`, keeping `adcs_sim.slx` as generated output instead of the source of truth. | `Completed > Model Architecture > Full Simulink model architecture`; `Completed > Model Architecture > Multi-file refactor` |
| 2026-04-20 | Working tree | Validation | Added `tests\run_adcs_tests.m` to rebuild the model, run a short simulation, and sanity-check the logged `q_log`, `omega_log`, and `h_W_log` outputs. | `Completed > Infrastructure > MATLAB smoke tests`; `Completed > Infrastructure > To Workspace logging` |
| 2026-04-20 | Working tree | Config and docs | Added `reference.md`, refreshed `README.md`, and introduced `sim_config.m` plus the `sim_config\` hierarchy as a compatibility view derived from `init_adcs_params.m`. | `Completed > Infrastructure > sim-finch structure integration` |
| 2026-04-20 | Committed (`8994999`) | Documentation | Added `agents.md`, `skills.md`, and `todo.md` to capture the project map, ADCS domain knowledge, and active task tracking. | Documents the completed work recorded throughout `Completed > Model Architecture`, `Environment Subsystem`, `Sensors Subsystem`, `Control Subsystem`, `Actuators Subsystem`, `Dynamics Subsystem`, and `Infrastructure` |
| 2026-02-13 | Committed (`068841a`) | Core setup | Added the initial ADCS model build flow with `build_adcs_model.m`, `init_adcs_params.m`, `ephemeris_2026_weekly.csv`, and `adcs_sim.slx`. | `Completed > Model Architecture > Parameter initialization script`; foundational setup for the completed architecture and subsystem work listed in `todo.md` |
| 2026-02-12 | Committed (`4832b58`) | Repository | Initialized the repository with the MIT `LICENSE`. | — |
## 2026-04-23 - WebGL ADCS Mission GUI

- Added `web-adcs-gui/`, a Vite/Three.js browser interface for the ADCS simulator.
- Implemented an interactive Earth-orbit mission scene with spacecraft body axes, solar panel axis, boresight vector, Sun/Moon context, eclipse volume, target line, ground track, and phase-colored trajectory tubes.
- Added mission controls for start orbit, end orbit, transfer mode, pointing mode, target coordinates, epoch, and duration.
- Added a data sync script that copies `ephemeris_2026_weekly.csv` and derives recognizable spacecraft/orbit defaults from `init_adcs_params.m` before dev/build.
- Added Vitest physics checks and Playwright browser checks for nonblank WebGL rendering, interaction, parameter updates, and ambient mode.
- Validation not yet executed in this session because PowerShell launch is currently failing with `CreateProcessAsUserW failed: 1920`.
## 2026-04-23 - Local Shell Runner Blocker

- Local shell execution failed before PowerShell or `cmd` could start, including trivial commands such as `Get-Location` and `cmd /c cd`.
- The repeated runner error was `CreateProcessAsUserW failed: 1920`.
- Because process launch failed, the session could not run `npm install`, `npm run build`, `npm run test`, `npm run test:e2e`, `ffmpeg`, or any file-listing commands.
- The issue appears to be outside the repository permissions layer: it occurs before a command begins executing.
- Next validation after restarting the Codex/local runner:
  - `cd web-adcs-gui`
  - `npm install`
  - `npm run build`
  - `npm run test`
  - `npm run test:e2e`
  - If a local reference recording is available, extract representative frames outside source control and tune the UI/rendering against those frames.
## 2026-04-23 - Web GUI Defaults Sync Alignment

- Aligned the web GUI fallback spacecraft dimensions and inertia with the active MATLAB baseline: 4 kg, `0.10 x 0.10 x 0.30 m`, and `diag([0.033333, 0.033333, 0.006667]) kg m^2`.
- Updated `web-adcs-gui/scripts/sync-adcs-data.mjs` to recognize the current `init_adcs_params.m` orbit variable names (`orbit_alt`, `orbit_ecc`, `orbit_inc`, `orbit_RAAN`, `orbit_AOP`, `orbit_TA`, and `orbit_a`) and convert SI lengths to kilometers for the browser mission model.
- Validation remains blocked by the local shell runner `CreateProcessAsUserW failed: 1920` error, so `npm install`, `npm run build`, `npm run test`, and `npm run test:e2e` still need to run after runner recovery.
