# ADCS Simulator — Project Directory for AI Agents

> **Purpose:** This file is an index/map of the repository so that AI agents (and developers) can quickly locate code, parameters, signals, and documentation without scanning the entire codebase.
>
> **Branch:** `simulink` — the primary development branch for the Simulink-based ADCS simulation.

---

## 1. File Index

| File / Directory | Description |
|---|---|
| `build_adcs_model.m` | **Start here.** Slim orchestrator (~175 lines) that creates the Simulink model, calls builders, wires top-level signals, and saves. |
| `launch_adcs_gui.m` | Interactive dashboard entrypoint for running the simulator and visualizing derived results. |
| `simulate_adcs_case.m` | Scriptable init/build/sim wrapper with optional parameter overrides. |
| `summarize_adcs_simulation.m` | Post-processing helper that turns logged signals into quantitative behavior metrics. |
| `builders/` | Subsystem builder functions — one file per subsystem. Each contains the builder + all embedded code generators. |
| `builders/build_environment.m` | ENVIRONMENT subsystem: Ephemeris_Truth, Orbit_Propagator, Magnetic_Field_Model, Eclipse_Model |
| `builders/build_sensors.m` | SENSORS subsystem: Sun_Sensor_Suite, Star_Tracker, IMU, Magnetometer, GNSS |
| `builders/build_control.m` | CONTROL subsystem: Onboard_Ephemeris, Attitude_Estimator, Reference_Generator, Mode_Manager, Control_Law |
| `builders/build_actuators.m` | ACTUATORS subsystem: RW_Assembly, MTQ_Assembly, CMG |
| `builders/build_dynamics.m` | DYNAMICS subsystem: Euler_RHS, Gravity_Gradient, Aero_Drag_Torque, SRP_Torque, Residual_Mag_Torque, Quat_RHS, Quat_Norm, Integrators |
| `utils/` | Shared helper functions used by all builders. |
| `utils/csub.m` | Clear default In1→Out1 wiring in fresh subsystems |
| `utils/add_inports.m` | Add a column of Inport blocks |
| `utils/add_outports.m` | Add a column of Outport blocks |
| `utils/set_mfb_script.m` | Set embedded MATLAB code in MATLAB Function blocks |
| `utils/leaf.m` | Create stub subsystem with named ports |
| `tests/run_adcs_tests.m` | MATLAB regression entrypoint that rebuilds the model, runs a short simulation, and checks logged outputs. |
| `init_adcs_params.m` | Parameter initialization script. Defines spacecraft properties, orbital elements, sensor noise levels, actuator limits, and simulation settings. **Must be run before simulation.** |
| `sim_config.m` | Structured compatibility entrypoint that derives grouped config data from `init_adcs_params.m`. |
| `sim_config/` | Compatibility config hierarchy carried over from `sim-finch`, now populated with derived spacecraft/sensor/actuator catalog scripts. |
| `adcs-sim.prj` | Minimal MATLAB project entrypoint for opening the repo as a MATLAB project. |
| `README.md` | Human-facing quick-start and repository overview. |
| `logs.md` | Repository change log for notable recent updates. |
| `adcs_sim.slx` | Simulink model binary — generated output of `build_adcs_model.m`. Do not hand-edit; regenerate by running the build script. |
| `ephemeris_2026_weekly.csv` | Sun and Moon direction vectors in ECI for 2026 (53 weekly data points). Consumed by the `Ephemeris_Truth` subsystem via CSV interpolation. |
| `LICENSE` | MIT license. |
| `resources/` | Simulink project resources (contains `project/` subdirectory). |

---

## 2. Detailed Reference

For model architecture map, full model hierarchy, signal/port mappings, and parameter tables, see **[`reference.md`](reference.md)**.

---

## 3. Branch Overview

| Branch | Purpose |
|---|---|
| `simulink` | **Active development.** Full Simulink ADCS sim with environment, sensors, control, actuators, and dynamics. |
| `main` | Base / default branch. |
| `sim-finch` | Older branch with basic attitude-only simulation and Simulink project structure. |
| `mtm` | Magnetorquer-related development. |
| `sim-config` | Configuration infrastructure work. |
| `utils` | Utility functions. |

---

## 4. How to Run the Simulation

### Prerequisites
- MATLAB R2023b or later (with Simulink)
- Aerospace Blockset (recommended but not strictly required)

### Steps

```matlab
% 1. Navigate to the repo root in MATLAB
cd('path/to/adcs-sim')

% 2. Initialize all parameters in the workspace
init_adcs_params

% 3. (Re)build the Simulink model from source
build_adcs_model

% 4. Open and run the simulation
open_system('adcs_sim')
sim('adcs_sim')
```

> **Note:** `init_adcs_params` must be run before `build_adcs_model` or `sim()` because the build script and model reference workspace variables defined by the init script.

### Quick one-liner

```matlab
init_adcs_params; build_adcs_model; sim('adcs_sim');
```

### Interactive dashboard

```matlab
launch_adcs_gui
```

---

## 5. Conventions & Notes for AI Agents

- **Do not edit `adcs_sim.slx` directly.** It is a generated binary. All model changes should be made in the builder files or the orchestrator.
- **Quaternion convention is scalar-first** `[q0, q1, q2, q3]` throughout the codebase.
- **The default inertia tensor is geometry-derived.** `init_adcs_params.m` computes `J` from `mass_sc` and `sc_dim_*` as the uniform-density baseline for the declared 4 kg, 3U bus. If a different inertia model is introduced, document where it came from.
- **The orchestrator (`build_adcs_model.m`) + builders (`builders/`) are the source of truth** for model structure — block names, positions, port connections, gain values, and MATLAB Function blocks.
- **To modify a subsystem,** edit the corresponding file in `builders/`. Shared helpers live in `utils/`. Top-level wiring lives in `build_adcs_model.m`.
- **Parameters flow from `init_adcs_params.m` → MATLAB base workspace → Simulink model.** If a simulation fails with "undefined variable" errors, run `init_adcs_params` first.
- **`sim_config.m` and the `sim_config/` hierarchy are compatibility views, not independent parameter sources.** Keep them derived from `init_adcs_params.m` so configuration data does not drift from the model.
- **MATLAB Function blocks that use `persistent` state cannot run at continuous sample time.** If MATLAB reports this error, either give the block a discrete sample time or remove the persistent-state logic.
- **Ephemeris data** is loaded from `ephemeris_2026_weekly.csv` at simulation start. The CSV has 53 rows (weekly intervals for 2026).
- **Push hygiene matters.** Keep MATLAB/Simulink cache outputs such as `slprj/`, `*.slxc`, autosave files, and documentation build auxiliaries like `docs/*.aux`, `docs/*.log`, `docs/*.out`, and `docs/*.toc` out of Git; they are local/generated artifacts rather than source inputs.
- **No plaintext secrets were found in the active source tree during the 2026-04-23 push-readiness audit.** Re-run a text search for keys/tokens before future pushes if new config, workflow, or integration files are added.
- **Always generate a concise summary of what changed after completing work.** Include major code changes, documentation updates, validation performed, and any unresolved issues.
- **Document as you go.** Any new knowledge, conventions, physics, or lessons learned must be written to the appropriate markdown file automatically:
  - **`agents.md`** — workflow rules, conventions, agent instructions
  - **`reference.md`** — architecture, port mappings, signal flow, parameter tables
  - **`skills.md`** — physics, math, domain knowledge, equations
  - **`logs.md`** — notable repository changes and recent updates
  - **`todo.md`** — task tracking, future work items

---

## 6. ⛔ NEVER Commit or Push to GitHub

**This is a hard rule — no exceptions.**

- Do **not** run `git add`, `git commit`, `git push`, `git merge`, or any other git write operation.
- All git operations are the user's responsibility. If you think something should be committed, tell the user and let them do it.
- This applies to all models, agents, and sub-agents — including background tasks.

---

## 7. Agent Workflow — GPT-5.5 Only

When the user asks you to perform a task that involves codebase changes, **use GPT-5.5 for the entire workflow**:

### Single-Model Workflow
- Use model **`gpt-5.5`** for planning, research, implementation, and verification.
- Analyze the codebase, identify affected files, consider edge cases, and validate assumptions against `skills.md`.
- Write or update `plan.md` in the session workspace when a detailed plan is useful.
- Implement changes directly once the task is clear.
- Run any applicable validation after changes are made.

> **Rationale:** Use one model consistently for planning and execution to simplify the workflow and avoid unsupported-model issues.

---

## 8. How to Launch a Background Agent

Use the **`task` tool** when a separate context window is useful, but keep the model on **`gpt-5.5`**.

### Steps

1. **Finish planning** in the current conversation. Write a comprehensive prompt that includes all context the agent will need — it runs in a separate, stateless context window.

2. **Launch the agent** using the `task` tool:
   ```
   tool: task
   agent_type: general-purpose
   model: gpt-5.5              ← use the same model
   mode: background             ← runs async, you get notified on completion
   name: descriptive-name
   prompt: <full task description with all file paths, code context, and instructions>
   ```

3. **Wait for completion notification.** Continue other work in the meantime.

4. **Read results** with `read_agent(agent_id)` once notified.

5. **Verify the output** — review changed files, run tests if applicable.

### Key Notes

- The background agent is **stateless** — it cannot see your conversation history. Include everything it needs in the prompt: file paths, current code snippets, exact instructions, constraints (like "do not commit").
- **To verify which model ran**, include this line in the prompt: *"State which model you are (model name and ID) at the very start of your response."* The agent will self-identify, giving you confirmation in the output.
- Use **`gpt-5.5`** for background agents as well unless the user explicitly asks for something else.
- Use `mode: background` for long tasks so you can work in parallel.
- Use `mode: sync` only for quick tasks where you need the result immediately.
- Multiple `explore` agents can run in parallel; `general-purpose` and `task` agents should run one at a time (they have side effects).

### When to Parallelize vs. Serialize

**Parallelize** (multiple background agents at once) when:
- Changes target **different files** (e.g., new plotting script + new utility function)
- Tasks are **research/exploration** (e.g., "analyze 5 subsystems" → 5 explore agents)
- Tasks are **independent validations** (e.g., run different test suites)

**Serialize** (one agent at a time) when:
- Multiple changes target the **same file** — parallel agents will overwrite each other
- Changes have **dependencies** (e.g., task B needs the output of task A)
- Changes are **structurally coupled** (e.g., modifying a function signature requires updating all callers)

> **Example — this project:** After refactoring, each subsystem is in its own file (`builders/build_*.m`). Improvements to different subsystems (e.g., sensors + actuators + dynamics) can now be parallelized across separate background agents. Only changes to the same builder file must be serialized.
