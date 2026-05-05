# ADCS Simulator

Programmatic MATLAB/Simulink attitude determination and control system (ADCS) simulator for the UTAT-SS stack.

The `simulink` branch builds the model from source code instead of treating the `.slx` file as the source of truth:

```
build_adcs_model.m  ->  builders\build_*.m  ->  adcs_sim.slx
```

## Quick start

```matlab
init_adcs_params
build_adcs_model
open_system('adcs_sim')
sim('adcs_sim')
```

For an interactive run with plots and derived metrics:

```matlab
launch_adcs_gui
```

For a short regression run:

```matlab
cd('tests')
results = run_adcs_tests();
```

## Repository layout

| Path | Purpose |
|---|---|
| `build_adcs_model.m` | Top-level model orchestrator and wiring |
| `launch_adcs_gui.m` | Interactive dashboard for setting core initial conditions, running the model, and visualizing outputs |
| `simulate_adcs_case.m` | Scriptable init/build/sim entrypoint with optional parameter overrides |
| `summarize_adcs_simulation.m` | Post-processing helper that derives quantitative behavior metrics from logged outputs |
| `builders\` | Source-of-truth subsystem builders for environment, sensors, control, actuators, and dynamics |
| `utils\` | Shared helper utilities for programmatic Simulink construction |
| `init_adcs_params.m` | Base-workspace parameter initialization used by the model build and simulation |
| `tests\run_adcs_tests.m` | Build + short closed-loop smoke test |
| `sim_config.m` | Structured compatibility view of the current simulator configuration |
| `sim_config\` | Lightweight config hierarchy derived from `init_adcs_params.m` |
| `reference.md` | Architecture, ports, and parameter reference |
| `skills.md` | Physics, estimation, and control notes |
| `agents.md` | Project map and workflow rules |
| `logs.md` | Repository change log for notable recent updates |
| `todo.md` | Task/status tracker |

## Notes

- `adcs_sim.slx` is generated output. Do not hand-edit it; regenerate it from the builders.
- `build_adcs_model` rebuilds `adcs_sim.slx` in the repository root, reuses that generated file in place when present, and clears stale `adcs_sim.slxc` cache artifacts so Simulink rebuilds do not hit same-name collisions.
- Quaternion convention is scalar-first: `[q0, q1, q2, q3]`.
- The default inertia tensor `J` is derived from `mass_sc` and `sc_dim_*` as a uniform-density 4 kg, `0.10 x 0.10 x 0.30 m` 3U cuboid baseline.
- `adcs-sim.prj` is a minimal MATLAB project entrypoint carried over from the older project structure.
- The `sim_config` hierarchy is compatibility infrastructure only. The live source of truth remains `init_adcs_params.m`.
- `launch_adcs_gui.m` overrides a small set of base-workspace parameters for a run, but `init_adcs_params.m` remains the canonical parameter source.
- The current MATLAB-validated build uses several compile-safe fallback MATLAB Function implementations where higher-fidelity versions were hitting parser or sample-time limitations.
## Browser ADCS GUI

An interactive WebGL mission interface lives in `web-adcs-gui/`. It is a Vite/Three.js app that visualizes Earth-orbit trajectory phases, ADCS pointing modes, eclipse state, Sun/Moon context, target tracking, and telemetry using project-derived defaults.

```matlab
% MATLAB/Simulink model flow remains unchanged:
init_adcs_params; build_adcs_model; sim('adcs_sim');
```

```powershell
# Browser GUI:
cd web-adcs-gui
npm install
npm run dev
```

The web app copies `ephemeris_2026_weekly.csv` and derives recognizable defaults from `init_adcs_params.m` during `npm run dev` and `npm run build`.
