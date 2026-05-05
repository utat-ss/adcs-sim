# ADCS Simulator — Technical Reference

> **Purpose:** Model architecture details, signal/port mappings, and parameter tables. Extracted from `agents.md` to keep that file concise. See `agents.md` for the high-level project directory.

> **Current validation note:** the MATLAB-validated build currently runs with several compile-safe fallback MATLAB Function implementations in ENVIRONMENT, SENSORS, CONTROL, ACTUATORS, and disturbance-torque blocks. The table below reflects the currently running behavior, not the higher-fidelity designs previously prototyped in these builders.

---

## 1. Model Architecture Map

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
| Orbit propagation | `ENVIRONMENT/Orbit_Propagator` | `pos_ECI`, `vel_ECI` (fixed finite LEO state fallback) |
| Magnetic field | `ENVIRONMENT/Magnetic_Field_Model` | `B_ECI` (fixed field fallback) |
| Eclipse detection | `ENVIRONMENT/Eclipse_Model` | `eclipse_flag` (constant sunlit fallback) |
| Sun sensors | `SENSORS/Sun_Sensor_Suite` | `sun_meas` (deterministic body-frame Sun direction) |
| Star tracker | `SENSORS/Star_Tracker` | `q_meas` (quaternion passthrough fallback) |
| Gyroscope / IMU | `SENSORS/IMU` | `omega_meas` (rate passthrough fallback) |
| Magnetometer | `SENSORS/Magnetometer` | `B_body_meas` (deterministic body-frame rotation) |
| GPS | `SENSORS/GNSS` | `pos_meas`, `vel_meas` (state passthrough fallback) |
| Onboard ephemeris prediction | `CONTROL/Onboard_Ephemeris` | `Sun_predicted`, `Mag_predicted` (constant-vector fallback) |
| Attitude estimation | `CONTROL/Attitude_Estimator` | `q_est`, `omega_est` (normalized measurement passthrough) |
| Mode logic | `CONTROL/Mode_Manager` | `mode_id` (constant coarse-mode fallback) |
| Control torque commands | `CONTROL/Control_Law` | `rw_torque_cmd`, `mtq_dipole_cmd`, `cmg_gimbal_cmd` (zero-torque fallback) |
| Reaction wheels | `ACTUATORS/RW_Assembly` | `torque` (zero-torque fallback) |
| Magnetorquers | `ACTUATORS/MTQ_Assembly` | `torque` (zero-torque fallback) |
| CMG | `ACTUATORS/CMG` | Zero-torque fallback |
| Torque summation | `ACTUATORS/Ext_Sum` | Total external actuator torque |
| Rotational dynamics | `DYNAMICS/Euler_RHS` | `omega_dot` (Euler's equation) |
| Drag disturbance | `DYNAMICS/Aero_Drag_Torque` | Zero disturbance fallback |
| SRP disturbance | `DYNAMICS/SRP_Torque` | Zero disturbance fallback |
| Magnetic disturbance | `DYNAMICS/Residual_Mag_Torque` | Zero disturbance fallback |
| Quaternion kinematics | `DYNAMICS/Quat_RHS` | `q_dot` |
| Angular velocity integration | `DYNAMICS/Integ_omega` | `omega` (IC = `omega0`) |
| Quaternion integration | `DYNAMICS/Integ_q` | `q` (IC = `q0`) |
| Quaternion normalization | `DYNAMICS/Quat_Norm` | `q_out` |
| Attitude plots | `Attitude_Scope` | — |
| Rate plots | `Rates_Scope` | — |
| Regression entrypoint | `tests/run_adcs_tests.m` | Build + short simulation smoke test |

---

## 2. Full Model Hierarchy

```
adcs_sim  (TOP LEVEL)
├── ENVIRONMENT
│   ├── Ephemeris_Truth         → Sun_vec_ECI, Moon_vec_ECI  (CSV interpolation)
│   ├── Orbit_Propagator        → pos_ECI, vel_ECI           (fixed finite LEO state fallback)
│   ├── Magnetic_Field_Model    → B_ECI                      (constant field fallback)
│   └── Eclipse_Model           → eclipse_flag               (constant sunlit fallback)
│
├── SENSORS
│   ├── Sun_Sensor_Suite (6 FSS) → sun_meas       (deterministic body-frame Sun direction)
│   ├── Star_Tracker             → q_meas         (quaternion passthrough fallback)
│   ├── IMU                      → omega_meas     (rate passthrough fallback)
│   ├── Magnetometer             → B_body_meas    (deterministic body-frame rotation)
│   └── GNSS                     → pos_meas, vel_meas  (state passthrough fallback)
│
├── CONTROL
│   ├── Onboard_Ephemeris       → Sun_predicted, Mag_predicted  (constant-vector fallback)
│   ├── Attitude_Estimator      → q_est, omega_est              (normalized measurement passthrough)
│   ├── Reference_Generator     → q_des, omega_des              (identity quaternion, zero rate)
│   ├── Mode_Manager            → mode_id                       (constant coarse-mode fallback)
│   └── Control_Law             → rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd  (zero-torque fallback)
│
├── ACTUATORS
│   ├── RW_Assembly (4 wheels)   → torque  (zero-torque fallback)
│   ├── MTQ_Assembly (3 rods)    → torque  (zero-torque fallback)
│   ├── CMG (representative)     → torque  (zero-torque fallback)
│   └── Ext_Sum
│
├── DYNAMICS
│   ├── Euler_RHS               → omega_dot  (coupled Euler equation)
│   ├── Gravity_Gradient        → τ_gg       (gravity-gradient disturbance)
│   ├── Aero_Drag_Torque        → τ_drag     (zero disturbance fallback)
│   ├── SRP_Torque              → τ_srp      (zero disturbance fallback)
│   ├── Residual_Mag_Torque     → τ_mag      (zero disturbance fallback)
│   ├── Quat_RHS                → q_dot      (quaternion kinematics)
│   ├── Integ_omega             → omega       (integrator, IC = omega0)
│   ├── Integ_hW                → h_W         (wheel momentum integrator)
│   ├── Integ_q                 → q           (integrator, IC = q0)
│   └── Quat_Norm               → q_out       (renormalization without persistent continuity state)
│
├── Attitude_Scope
└── Rates_Scope
```

---

## 3. Signal / Port Mapping Reference

### Signal Flow (top level)

| From | To | Signals |
|---|---|---|
| `ENVIRONMENT` | `SENSORS` | `Sun_vec_ECI`, `Moon_vec_ECI`, `pos_ECI`, `vel_ECI`, `B_ECI`, `eclipse_flag` |
| `ENVIRONMENT` | `DYNAMICS` | `pos_ECI`, `vel_ECI`, `Sun_vec_ECI`, `B_ECI`, `eclipse_flag` |
| `SENSORS` | `CONTROL` | `sun_meas`, `q_meas`, `omega_meas`, `B_body_meas`, `pos_meas`, `vel_meas` |
| `DYNAMICS` | `CONTROL` | `h_W_out` |
| `CONTROL` | `ACTUATORS` | `rw_torque_cmd`, `mtq_dipole_cmd`, `cmg_gimbal_cmd` |
| `ACTUATORS` | `DYNAMICS` | `ext_torque`, `rw_torque_body`, `rw_tau_wheels` |
| `DYNAMICS` | `SENSORS` | **Feedback:** `q_out` (attitude quaternion), `omega_out` (angular velocity) |

### Feedback Connections

| Signal | Source | Destination | Purpose |
|---|---|---|---|
| `q_out` | `DYNAMICS/Quat_Norm` | `SENSORS` (q_in) | True attitude for sensor models |
| `omega_out` | `DYNAMICS/Integ_omega` | `SENSORS` (omega_in) | True angular velocity for sensor models |
| `h_W_out` | `DYNAMICS/Integ_hW` | `CONTROL` (h_W_in) | Wheel momentum state, currently unused by the validated fallback control path |

---

## 4. Parameter Quick-Reference

All parameters are defined in **`init_adcs_params.m`**. Key values:

### Spacecraft

| Parameter | Value | Notes |
|---|---|---|
| Form factor | 3U CubeSat | — |
| Mass | 4 kg | — |
| Inertia tensor `J` | `diag([0.0333, 0.0333, 0.0067])` kg·m² | Uniform-density 4 kg, `0.10 x 0.10 x 0.30 m` cuboid baseline |

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
| CMG (×1) | `0.01 N·m` max torque, `0.5 N·m·s` stored momentum |

### Disturbance / Estimation

Many of the higher-fidelity disturbance and estimation parameters remain defined in `init_adcs_params.m`, but the current MATLAB-validated build uses compile-safe fallback implementations instead of consuming all of them.

| Parameter | Value | Notes |
|---|---|---|
| `J2_earth` | `1.08262668e-3` | Defined in params; not used by the current fixed-state orbit fallback |
| Drag density anchor | `6.967e-13 kg/m³` at 500 km | Defined in params; current validated disturbance path outputs zero drag torque |
| SRP coefficient | `1.3` | Defined in params; current validated disturbance path outputs zero SRP torque |
| Residual dipole | `[5, 5, 5] mA·m²` | Defined in params; current validated disturbance path outputs zero magnetic disturbance torque |
| Estimator | Measurement passthrough fallback | Current validated path normalizes `q_meas` and passes through `omega_meas` |

### Simulation

| Parameter | Value |
|---|---|
| Time step `dt` | 0.01 s |
| Duration `t_end` | 6000 s (~1 orbit) |
| Solver | `ode4` (fixed-step Runge-Kutta) |
| Quaternion convention | Scalar-first `[q0, q1, q2, q3]` |

---

## 5. Validation

Basic regression coverage now lives in **`tests/run_adcs_tests.m`**. The script:

- Initializes parameters
- Verifies that the default inertia tensor stays consistent with the declared bus mass and geometry
- Verifies that the live gravity-gradient and sensor blocks use the same ECI→body DCM convention
- Rebuilds `adcs_sim`
- Verifies key top-level subsystem interfaces
- Runs a short 60 s simulation
- Checks that `q_log`, `omega_log`, and `h_W_log` are finite, quaternion norm stays bounded, and derived summary metrics are produced

### Interactive dashboard and derived metrics

Use **`launch_adcs_gui.m`** for interactive runs. The dashboard keeps `init_adcs_params.m` as the source of truth, lets the user override the current validated high-impact inputs (`dt`, `t_end`, `q0`, `omega0`, `h_W0`, and the diagonal of `J`), then runs the standard `init_adcs_params` -> `build_adcs_model` -> `sim('adcs_sim')` chain.

The GUI visualizes the existing logged outputs:

- `q_log`
- `omega_log`
- `h_W_log`

It also computes quantitative post-simulation metrics through **`summarize_adcs_simulation.m`**, including:

- attitude excursion from the initial quaternion
- quaternion norm error
- body-rate peak and RMS values
- angular-acceleration peak
- rotational kinetic energy
- reaction-wheel momentum and estimated wheel-speed utilization
- total angular momentum norm
## Web ADCS GUI

The `web-adcs-gui/` directory contains a browser-side Vite/Three.js interface for the Simulink-generated ADCS model. It is source code only and does not replace `adcs_sim.slx` or any builder file.

### Data Flow

`web-adcs-gui/scripts/sync-adcs-data.mjs` runs before `npm run dev` and `npm run build`. It copies the root `ephemeris_2026_weekly.csv` into `web-adcs-gui/public/data/` and updates `web-adcs-gui/public/data/adcs-project-defaults.json` from recognizable assignments in `init_adcs_params.m`. The parser recognizes the active MATLAB orbit names (`orbit_alt`, `orbit_ecc`, `orbit_inc`, `orbit_RAAN`, `orbit_AOP`, `orbit_TA`, and `orbit_a`) and converts SI lengths to kilometers for the browser model. If a value cannot be recognized, the app keeps its documented fallback defaults.

The synced MATLAB baseline currently uses `orbit_ecc = 0.0001`, so browser eccentricity inputs must accept at least four decimal places. Otherwise native HTML form validation can block mission updates even though the synced value is physically valid.

### Runtime Model

- Orbit state is propagated in ECI coordinates from classical orbital elements.
- The visible trajectory is display-scaled from kilometers to Three.js scene units, while telemetry remains in physical units. The render transform applies a visual-only radial altitude exaggeration factor of `3.2` so LEO trajectories and spacecraft clearance remain readable against an Earth-scale sphere.
- Mission phases are rendered as colored trajectory tubes: detumble, sun acquire, transfer burn, target track, downlink, eclipse, and momentum dump.
- ADCS pointing modes include detumbling, sun tracking, target tracking, nadir imaging, and inertial hold.
- The ADCS telemetry panel includes an operation-progress rail for the active pointing mode. Detumble progress is driven by the decaying body-rate estimate; target, Sun, and nadir modes report pointing-alignment progress; inertial hold reports low-rate hold progress.
- The bottom status strip includes the active data source, a mission-progress rail driven by the current sample time, the camera mode, and WebGL status.
- The spacecraft render is intentionally much smaller than the Earth display scale. At the default 500 km start altitude, its maximum visual extent is kept below the visually scaled clearance between the orbit radius and the atmospheric shell so the model does not clip into Earth.
- The app keeps the project quaternion convention internally as scalar-first `[q0, q1, q2, q3]`; Three.js receives reordered `[x, y, z, w]` quaternions only at the render boundary.

### Important Files

| File | Purpose |
|---|---|
| `web-adcs-gui/src/physics/orbit.ts` | Kepler propagation, J2 drift, Hohmann and plane-change estimates, ECI/ECEF ground geometry. |
| `web-adcs-gui/src/physics/environment.ts` | Sun/Moon interpolation or analytical fallback, eclipse, magnetic dipole, target visibility. |
| `web-adcs-gui/src/physics/attitude.ts` | ADCS pointing-frame construction and scalar-first quaternions. |
| `web-adcs-gui/src/physics/mission.ts` | Mission timeline, phase schedule, transfer state blending, solar power estimate. |
| `web-adcs-gui/src/render/AdcsScene.ts` | Three.js scene graph, Earth/Sun/Moon rendering, spacecraft model, fly paths, camera modes. |
| `web-adcs-gui/src/__tests__/physics.test.ts` | Vitest physics and mission-timeline checks. |
| `web-adcs-gui/tests/e2e/adcs-gui.spec.ts` | Browser rendering and interaction checks. |
