# ADCS Simulator — Task Tracker

> **Branch:** `simulink`
> **Purpose:** Track implementation status of the ADCS simulation built in MATLAB/Simulink.
> Core model is defined in `build_adcs_model.m`; parameters live in `init_adcs_params.m`.

---

## Status Legend

| Icon | Meaning |
|------|---------|
| ✅ | Done — implemented and functional |
| 🔲 | Incomplete — needs implementation or fix |
| 📋 | Future / nice-to-have |

---

## Completed (✅)

### Model Architecture
- ✅ **Full Simulink model architecture** — ENVIRONMENT → SENSORS → CONTROL → ACTUATORS → DYNAMICS pipeline (`build_adcs_model.m` orchestrator + `builders/`)
- ✅ **Signal routing and feedback loops** — quaternion/rate feedback from DYNAMICS back to SENSORS and CONTROL (`build_adcs_model.m`)
- ✅ **Parameter initialization script** — all physical constants, orbit elements, sensor specs, actuator specs (`init_adcs_params.m`)
- ✅ **Multi-file refactor** — monolithic `build_adcs_model.m` split into `builders/` (5 files) + `utils/` (5 files) + slim orchestrator

### Environment Subsystem
- ✅ **Ephemeris truth** — CSV interpolation for Sun/Moon vectors (`builders/build_environment.m`)
- ✅ **Compile-safe orbit fallback** — fixed finite LEO state used by the validated MATLAB smoke-test path (`builders/build_environment.m`)
- ✅ **Compile-safe magnetic field fallback** — fixed nonzero `B_ECI` output for the validated path (`builders/build_environment.m`)
- ✅ **Compile-safe eclipse fallback** — constant sunlit flag for the validated path (`builders/build_environment.m`)

### Sensors Subsystem
- ✅ **Deterministic sun-vector output** — eclipse-gated body-frame Sun direction (`builders/build_sensors.m`)
- ✅ **Star tracker fallback** — quaternion passthrough in the validated path (`builders/build_sensors.m`)
- ✅ **IMU fallback** — angular-rate passthrough in the validated path (`builders/build_sensors.m`)
- ✅ **Magnetometer fallback** — deterministic body-frame magnetic-field rotation (`builders/build_sensors.m`)
- ✅ **GNSS fallback** — position/velocity passthrough in the validated path (`builders/build_sensors.m`)

### Control Subsystem
- ✅ **Onboard ephemeris fallback** — constant reference vectors in the validated path (`builders/build_control.m`)
- ✅ **Attitude estimator fallback** — normalized quaternion + rate passthrough in the validated path (`builders/build_control.m`)
- ✅ **Reference generator fallback** — identity quaternion and zero-rate reference in the validated path (`builders/build_control.m`)
- ✅ **Mode-manager fallback** — constant coarse mode in the validated path (`builders/build_control.m`)
- ✅ **Control-law fallback** — zero commanded torque outputs in the validated path (`builders/build_control.m`)

### Actuators Subsystem
- ✅ **Actuator fallbacks** — RW, MTQ, and CMG blocks currently output zero torque in the validated path (`builders/build_actuators.m`)

### Dynamics Subsystem
- ✅ **Coupled rigid body dynamics** — Euler's equation with RW momentum coupling: `ḣ_tot = τ_ext − ω×h_tot` (`builders/build_dynamics.m`)
- ✅ **Gravity gradient torque** — `τ_gg = (3μ/R³)(r̂_body × J·r̂_body)` (`builders/build_dynamics.m`)
- ✅ **Geometry-consistent default inertia baseline** — `J` is now derived from `mass_sc` and the declared `sc_dim_*` bus dimensions so the default rigid-body parameters remain self-consistent (`init_adcs_params.m`)
- ✅ **Disturbance fallbacks** — aerodynamic drag, SRP, and residual magnetic torque blocks currently output zero disturbance in the validated path (`builders/build_dynamics.m`)
- ✅ **Quaternion kinematics** — quaternion derivative integration (`builders/build_dynamics.m`)
- ✅ **Quaternion renormalization** — unit-norm enforcement in the validated path (`builders/build_dynamics.m`)
- ✅ **RW momentum integrator** — tracks wheel angular momentum `h_W` (`builders/build_dynamics.m`)

### Infrastructure
- ✅ **To Workspace logging** — `q_log`, `omega_log`, `h_W_log` as Timeseries (`build_adcs_model.m`)
- ✅ **Simulation timing from init script** — solver fixed step and stop time use `dt` and `t_end` from `init_adcs_params.m` (`build_adcs_model.m`)
- ✅ **Rebuild-safe model generation** — `build_adcs_model.m` deletes stale generated `adcs_sim.slx` / `adcs_sim.slxc` artifacts and saves back to the repository root before each rebuild, avoiding same-name Simulink model collisions
- ✅ **MATLAB smoke tests** — automated build + short-run regression script in `tests/run_adcs_tests.m`, executed successfully in MATLAB R2026a
- ✅ **Math regression checks for inertia and frame conventions** — `tests/run_adcs_tests.m` now verifies the geometry-derived default inertia and enforces agreement between the live gravity-gradient and sensor ECI→body DCM conventions before simulation
- ✅ **Repository push hygiene** — root `.gitignore` now excludes MATLAB/Simulink caches, autosaves, handbook build auxiliaries, and local visual-reference captures; a plaintext secret scan of the active source tree found no matches during the 2026-04-23 audit
- ✅ **sim-finch structure integration** — added a root `README.md`, minimal `adcs-sim.prj`, and a derived `sim_config` compatibility hierarchy without splitting parameter truth away from `init_adcs_params.m`
- ✅ **Interactive GUI dashboard** — `launch_adcs_gui.m` lets users override core inputs, run the standard init/build/sim flow, and visualize plots plus summary tables
- ✅ **Quantified post-simulation metrics** — `simulate_adcs_case.m` and `summarize_adcs_simulation.m` derive attitude, rate, momentum, energy, and utilization metrics from the logged outputs
- ✅ **Web GUI defaults sync alignment** — browser fallback spacecraft values now match the active 4 kg, `0.10 x 0.10 x 0.30 m` MATLAB baseline, and the sync script recognizes the current `orbit_*` names from `init_adcs_params.m` with SI-to-kilometer conversion
- ✅ **Web GUI operation and orbit readability pass** — browser detumble now exposes decaying-rate progress, pointing modes report live operation progress, and the Three.js scene uses documented visual-only altitude exaggeration plus clearance checks so default LEO orbit geometry remains readable

---

## Open / Pending (🔲)

- 🔲 **Run 100+ simulation cases** — execute more than 100 simulations across different parameter dispersions and mission scenarios, then summarize trends, sensitivities, and failure cases
- 🔲 **Compare simulator results to other tools** — benchmark outputs against external analysis/simulation tools and record agreement gaps or model discrepancies
- 🔲 **Expand GUI functionality** — add richer controls, workflows, and result-inspection features beyond the current dashboard
- 🔲 **Verify implemented math against online research** — cross-check ADCS equations and assumptions against reputable external references and document any discrepancies

---

## Future / Nice-to-Have (📋)

- 📋 **Restore high-fidelity environment models** — reintroduce J2 orbit propagation, magnetic field variation, and eclipse geometry with MATLAB Function implementations that pass parser and sample-time constraints
- 📋 **Restore sampled/noisy sensor models** — reintroduce FSS, star tracker, IMU, magnetometer, and GNSS fidelity using discrete sample-time blocks or other compile-stable implementations
- 📋 **Restore advanced control/actuator logic** — reintroduce onboard ephemeris, MEKF, reference generation, mode logic, PD/B-dot/desaturation control, and nonzero actuator models with compile-stable implementations
- 📋 **Restore environmental disturbance models** — reintroduce drag, SRP, residual magnetic torque, and quaternion continuity logic without continuous-time persistent-state violations
- 📋 **Expanded GUI / dashboard** — advanced parameter entry, real-time 3D visualization, and richer telemetry panes beyond the current basic dashboard
- 📋 **Higher-fidelity magnetic field** — full IGRF spherical harmonic expansion instead of tilted dipole
- 📋 **Atmospheric density model** — NRLMSISE-00 or JB2008 for drag computation
- 📋 **Expanded regression coverage** — disturbance-specific tests, longer closed-loop cases, and Monte Carlo validation beyond the current smoke test
- 📋 **Ground station contact windows** — visibility analysis for downlink scheduling
- 📋 **Power / thermal coupling** — battery state-of-charge and thermal effects on actuator performance
- 📋 **Monte Carlo simulation framework** — parameter dispersion and sensitivity analysis
- 📋 **Code generation for flight software** — MATLAB Coder / Simulink Coder for embedded deployment
- 📋 **CI/CD pipeline** — automated model build, test, and regression via GitHub Actions
## Web ADCS GUI Follow-ups

- Replace fallback/default parsed values with a stricter export from MATLAB once the desired web data contract is settled.
- Add import support for `simulate_adcs_case.m` or `summarize_adcs_simulation.m` logged outputs so the web GUI can replay actual Simulink runs instead of only interactive browser-side propagation.
- Compare browser Hohmann/J2 propagation against a MATLAB-generated reference case after shell/MATLAB validation is available.
- Continue finer visual tuning against `.codex-video-reference/contact-sheet.jpg`, focusing on side-panel density, lighting polish, richer mission overlays, and ambient mode.
- Consider adding sensor-specific overlays for sun sensor FOV, star tracker keep-out zones, magnetometer vectors, reaction wheel momentum, and magnetorquer dipole commands.
## Post-Restart Validation Checklist

- Done 2026-04-23: Confirmed shell access works inside the Codex runner with `Get-Location` and `cmd /c cd`.
- Done 2026-04-23: From `web-adcs-gui/`, ran `npm install`.
- Done 2026-04-23: Ran `npm run build`; fixed missing Three.js type declarations.
- Done 2026-04-23: Ran `npm run test`; fixed Vitest collection and near-circular orbit radius assumptions.
- Done 2026-04-23: Ran `npm run test:e2e`; installed the Playwright Chromium runtime and fixed the synced eccentricity form-step validation bug.
- Done 2026-04-23: Extracted key frames from a local reference recording into an ignored visual-reference directory.
- Done 2026-04-23: Reviewed the extracted contact sheet against the WebGL scene; remaining visual matching is tracked as a follow-up rather than a validation blocker.
- Done 2026-04-23: Re-ran `npm run validate`; build, Vitest, and Playwright all passed.
