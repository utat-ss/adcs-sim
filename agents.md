# ADCS Simulator — Project Directory for AI Agents

> **Purpose:** This file is an index/map of the repository so that AI agents (and developers) can quickly locate code, parameters, signals, and documentation without scanning the entire codebase.
>
> **Branch:** `simulink` — the primary development branch for the Simulink-based ADCS simulation.

---

## 1. File Index

| File / Directory | Description |
|---|---|
| `build_adcs_model.m` | **Start here.** ~66 KB MATLAB script that programmatically constructs the entire Simulink model (`adcs_sim.slx`). Every block, port, signal, and parameter is defined in this file. |
| `init_adcs_params.m` | Parameter initialization script. Defines spacecraft properties, orbital elements, sensor noise levels, actuator limits, and simulation settings. **Must be run before simulation.** |
| `adcs_sim.slx` | Simulink model binary — generated output of `build_adcs_model.m`. Do not hand-edit; regenerate by running the build script. |
| `ephemeris_2026_weekly.csv` | Sun and Moon direction vectors in ECI for 2026 (53 weekly data points). Consumed by the `Ephemeris_Truth` subsystem via CSV interpolation. |
| `LICENSE` | MIT license. |
| `resources/` | Simulink project resources (contains `project/` subdirectory). |
| `sim_config/` | Empty on this branch. Used on the `sim-finch` branch for configuration infrastructure. |

---

## 2. Model Architecture Map

The Simulink model follows a **feedforward + feedback** architecture:

```
ENVIRONMENT → SENSORS → CONTROL → ACTUATORS → DYNAMICS
                 ↑                                 │
                 └──────── feedback ───────────────┘
```

### Subsystem Lookup Table

| If you need to work on… | Look in subsystem… | Key outputs |
|---|---|---|
| Sun/Moon positions | `ENVIRONMENT/Ephemeris_Truth` | `Sun_vec_ECI`, `Moon_vec_ECI` |
| Orbit propagation | `ENVIRONMENT/Orbit_Propagator` | `pos_ECI`, `vel_ECI` |
| Magnetic field | `ENVIRONMENT/Magnetic_Field_Model` | `B_ECI` |
| Eclipse detection | `ENVIRONMENT/Eclipse_Model` | `eclipse_flag` |
| Sun sensors | `SENSORS/Sun_Sensor_Suite` | `sun_meas` (6 fine sun sensors) |
| Star tracker | `SENSORS/Star_Tracker` | `q_meas` (quaternion measurement) |
| Gyroscope / IMU | `SENSORS/IMU` | `omega_meas` |
| Magnetometer | `SENSORS/Magnetometer` | `B_body_meas` |
| GPS | `SENSORS/GNSS` | `pos_meas`, `vel_meas` |
| Onboard ephemeris prediction | `CONTROL/Onboard_Ephemeris` | `Sun_predicted`, `Mag_predicted` |
| Attitude estimation | `CONTROL/Attitude_Estimator` | `q_est`, `omega_est` (TRIAD) |
| Control torque commands | `CONTROL/Control_Law` | `rw_torque_cmd`, `mtq_dipole_cmd`, `cmg_gimbal_cmd` (PD + B-dot) |
| Reaction wheels | `ACTUATORS/RW_Assembly` | `torque` (4-wheel pyramid, pseudoinverse) |
| Magnetorquers | `ACTUATORS/MTQ_Assembly` | `torque` (3 rods, dipole saturation) |
| CMG | `ACTUATORS/CMG` | Zero torque (placeholder) |
| Torque summation | `ACTUATORS/Torque_Sum` | Total actuator torque |
| Rotational dynamics | `DYNAMICS/Euler_RHS` | `omega_dot` (Euler's equation) |
| Quaternion kinematics | `DYNAMICS/Quat_RHS` | `q_dot` |
| Angular velocity integration | `DYNAMICS/Integ_omega` | `omega` (IC = `omega0`) |
| Quaternion integration | `DYNAMICS/Integ_q` | `q` (IC = `q0`) |
| Quaternion normalization | `DYNAMICS/Quat_Norm` | `q_out` |
| Attitude plots | `Attitude_Scope` | — |
| Rate plots | `Rates_Scope` | — |

---

## 3. Full Model Hierarchy

```
adcs_sim  (TOP LEVEL)
├── ENVIRONMENT
│   ├── Ephemeris_Truth         → Sun_vec_ECI, Moon_vec_ECI  (CSV interpolation)
│   ├── Orbit_Propagator        → pos_ECI, vel_ECI           (Keplerian 2-body)
│   ├── Magnetic_Field_Model    → B_ECI                      (tilted dipole IGRF approx)
│   └── Eclipse_Model           → eclipse_flag                (cylindrical shadow)
│
├── SENSORS
│   ├── Sun_Sensor_Suite (6 FSS) → sun_meas       (FOV check + noise)
│   ├── Star_Tracker             → q_meas         (Sun/Moon exclusion + noise)
│   ├── IMU                      → omega_meas     (bias drift + ARW)
│   ├── Magnetometer             → B_body_meas    (hard-iron bias + noise)
│   └── GNSS                     → pos_meas, vel_meas  (white noise)
│
├── CONTROL
│   ├── Onboard_Ephemeris       → Sun_predicted, Mag_predicted  (Meeus model)
│   ├── Attitude_Estimator      → q_est, omega_est              (TRIAD method)
│   └── Control_Law             → rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd  (PD + B-dot)
│
├── ACTUATORS
│   ├── RW_Assembly (4 wheels)   → torque  (pyramid config, pseudoinverse distribution)
│   ├── MTQ_Assembly (3 rods)    → torque  (dipole saturation, τ = m × B)
│   ├── CMG (placeholder)        → zero torque
│   └── Torque_Sum
│
├── DYNAMICS
│   ├── Euler_RHS               → omega_dot  (Euler's equation)
│   ├── Quat_RHS                → q_dot      (quaternion kinematics)
│   ├── Integ_omega             → omega       (integrator, IC = omega0)
│   ├── Integ_q                 → q           (integrator, IC = q0)
│   └── Quat_Norm               → q_out       (renormalization)
│
├── Attitude_Scope
└── Rates_Scope
```

---

## 4. Signal / Port Mapping Reference

### Signal Flow (top level)

| From | To | Signals |
|---|---|---|
| `ENVIRONMENT` | `SENSORS` | `Sun_vec_ECI`, `Moon_vec_ECI`, `pos_ECI`, `vel_ECI`, `B_ECI`, `eclipse_flag` |
| `SENSORS` | `CONTROL` | `sun_meas`, `q_meas`, `omega_meas`, `B_body_meas`, `pos_meas`, `vel_meas` |
| `CONTROL` | `ACTUATORS` | `rw_torque_cmd`, `mtq_dipole_cmd`, `cmg_gimbal_cmd` |
| `ACTUATORS` | `DYNAMICS` | Total actuator torque (summed) |
| `DYNAMICS` | `SENSORS` | **Feedback:** `q_out` (attitude quaternion), `omega` (angular velocity) |

### Feedback Connections

| Signal | Source | Destination | Purpose |
|---|---|---|---|
| `q_out` | `DYNAMICS/Quat_Norm` | `SENSORS` (q_in) | True attitude for sensor models |
| `omega` | `DYNAMICS/Integ_omega` | `SENSORS` (omega_in) | True angular velocity for sensor models |

---

## 5. Parameter Quick-Reference

All parameters are defined in **`init_adcs_params.m`**. Key values:

### Spacecraft

| Parameter | Value | Notes |
|---|---|---|
| Form factor | 3U CubeSat | — |
| Mass | 4 kg | — |
| Inertia tensor `J` | `diag([0.05, 0.05, 0.08])` kg·m² | Principal axes |

### Orbit

| Parameter | Value | Notes |
|---|---|---|
| Altitude | 500 km | ISS-like |
| Inclination | 51.6° | ISS-like |
| Eccentricity | ~0 (near-circular) | — |

### Sensors

| Sensor | Key Noise Parameter |
|---|---|
| Fine Sun Sensors (×6) | 0.5° noise |
| Star Tracker | 10 arcsec (boresight) / 40 arcsec (cross-axis) |
| IMU | 0.01 deg/√s ARW |
| Magnetometer | 100 nT noise |
| GNSS | 10 m position noise |

### Actuators

| Actuator | Spec |
|---|---|
| Reaction Wheels (×4) | 5 mN·m max torque, pyramid config |
| Magnetorquers (×3) | 0.2 A·m² max dipole |
| CMG (×1) | Placeholder (outputs zero) |

### Simulation

| Parameter | Value |
|---|---|
| Time step `dt` | 0.1 s |
| Duration `t_end` | 6000 s (~1 orbit) |
| Solver | `ode4` (fixed-step Runge-Kutta) |
| Quaternion convention | Scalar-first `[q0, q1, q2, q3]` |

---

## 6. Branch Overview

| Branch | Purpose |
|---|---|
| `simulink` | **Active development.** Full Simulink ADCS sim with environment, sensors, control, actuators, and dynamics. |
| `main` | Base / default branch. |
| `sim-finch` | Older branch with basic attitude-only simulation and Simulink project structure. |
| `mtm` | Magnetorquer-related development. |
| `sim-config` | Configuration infrastructure work. |
| `utils` | Utility functions. |

---

## 7. How to Run the Simulation

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

---

## 8. Conventions & Notes for AI Agents

- **Do not edit `adcs_sim.slx` directly.** It is a generated binary. All model changes should be made in `build_adcs_model.m`.
- **Quaternion convention is scalar-first** `[q0, q1, q2, q3]` throughout the codebase.
- **The build script (`build_adcs_model.m`) is the single source of truth** for model structure — block names, positions, port connections, gain values, and MATLAB Function blocks are all defined there.
- **Parameters flow from `init_adcs_params.m` → MATLAB base workspace → Simulink model.** If a simulation fails with "undefined variable" errors, run `init_adcs_params` first.
- **Ephemeris data** is loaded from `ephemeris_2026_weekly.csv` at simulation start. The CSV has 53 rows (weekly intervals for 2026).

---

## 9. Agent Workflow — Plan Then Execute

When the user asks you to perform a task, **always follow this two-phase workflow**:

### Phase 1: Plan (Claude Opus 4.6 — high thinking)
- Use model **`claude-opus-4.6`** with high thinking to create a detailed implementation plan.
- Analyze the codebase, identify affected files, consider edge cases, and validate assumptions against `skills.md`.
- Write the plan to `plan.md` in the session workspace or present it to the user for review.
- Do **not** make code changes during this phase.

### Phase 2: Execute (GPT-5.3-Codex — high thinking)
- Switch to model **`gpt-5.3-codex`** with high thinking to implement the plan.
- Follow the plan step-by-step, making precise code changes.
- Run any applicable tests or validation after changes are made.

> **Rationale:** Opus 4.6 excels at reasoning, architecture, and planning. Codex 5.3 excels at fast, accurate code generation and execution. Separating the phases plays to each model's strengths.
