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
- ✅ **Full Simulink model architecture** — ENVIRONMENT → SENSORS → CONTROL → ACTUATORS → DYNAMICS pipeline (`build_adcs_model.m`)
- ✅ **Signal routing and feedback loops** — quaternion/rate feedback from DYNAMICS back to SENSORS and CONTROL (`build_adcs_model.m`)
- ✅ **Parameter initialization script** — all physical constants, orbit elements, sensor specs, actuator specs (`init_adcs_params.m`)

### Environment Subsystem
- ✅ **Ephemeris truth** — CSV interpolation for Sun/Moon vectors (`build_adcs_model.m`, environment subsystem)
- ✅ **Keplerian orbit propagator** — 2-body propagation, Newton iteration for Kepler's equation (`build_adcs_model.m`)
- ✅ **Tilted dipole magnetic field model** — dipole approximation in body frame (`build_adcs_model.m`)
- ✅ **Cylindrical eclipse model** — shadow flag from Sun/satellite geometry (`build_adcs_model.m`)

### Sensors Subsystem
- ✅ **6 Fine Sun Sensors** — FOV check, cosine response, noise (`build_adcs_model.m`, sensors subsystem)
- ✅ **Star tracker** — Sun/Moon exclusion angles, quaternion noise (`build_adcs_model.m`)
- ✅ **IMU** — bias drift (random walk) and angular random walk noise (`build_adcs_model.m`)
- ✅ **Magnetometer** — hard-iron bias offset and white noise (`build_adcs_model.m`)
- ✅ **GNSS receiver** — position/velocity with additive noise (`build_adcs_model.m`)

### Control Subsystem
- ✅ **TRIAD attitude estimator** — batch two-vector attitude determination (`build_adcs_model.m`, control subsystem)
- ✅ **PD controller for reaction wheels** — proportional-derivative torque command (`build_adcs_model.m`)
- ✅ **B-dot law for magnetorquers** — rate-based detumble control (`build_adcs_model.m`)

### Actuators Subsystem
- ✅ **Reaction wheel assembly** — 4-wheel pyramid config, pseudoinverse torque distribution, torque saturation (`build_adcs_model.m`)
- ✅ **Magnetorquer assembly** — dipole saturation, τ = m × B (`build_adcs_model.m`)

### Dynamics Subsystem
- ✅ **Rigid body dynamics** — Euler's equation for rotational motion (`build_adcs_model.m`, dynamics subsystem)
- ✅ **Quaternion kinematics** — quaternion derivative integration (`build_adcs_model.m`)
- ✅ **Quaternion renormalization** — unit-norm enforcement after integration (`build_adcs_model.m`)

---

## Incomplete / Needs Work (🔲)

### Actuators
- 🔲 **CMG model** — currently outputs zero torque (placeholder). Needs gimbal dynamics: τ = dH/dt (`build_adcs_model.m`, actuators subsystem)
- 🔲 **MTQ Assembly B-field input** — uses hardcoded `B_body = [2e-5; -1e-5; 4e-5]`. Should receive actual `B_body_meas` from SENSORS via an additional inport (`build_adcs_model.m`, actuators subsystem)

### Sensors
- 🔲 **Star tracker Earth-limb exclusion** — parameter `st_exclusion_earth = 25°` is defined in `init_adcs_params.m` but not implemented in the sensor MATLAB Function block (`build_adcs_model.m`, sensors subsystem)
- 🔲 **Sample-and-hold for sensors** — sample rates defined (FSS 10 Hz, ST 2 Hz, IMU 100 Hz, MAG 10 Hz, GNSS 1 Hz) in `init_adcs_params.m` but no zero-order hold or discrete sample logic is applied; all sensors run at simulation rate (`build_adcs_model.m`)

### Environment / Orbit
- 🔲 **J2 orbital perturbation** — orbit propagator is pure Keplerian. J2 secular effects (RAAN drift, argument-of-perigee drift) are significant for LEO accuracy over multiple orbits (`build_adcs_model.m`, environment subsystem)
- 🔲 **Orbit elements from init_adcs_params** — orbit propagator has hardcoded orbital elements instead of reading from workspace parameters set in `init_adcs_params.m` (`build_adcs_model.m`)

### Environmental Torques
- 🔲 **Gravity gradient torque** — not modeled. Significant for CubeSats; torque ≈ 3μ/R³ (I_z − I_x) (`build_adcs_model.m`, dynamics subsystem)
- 🔲 **Aerodynamic drag torque** — relevant at ~500 km altitude but not modeled (`build_adcs_model.m`)
- 🔲 **Solar radiation pressure torque** — not modeled; depends on surface area and reflectivity (`build_adcs_model.m`)
- 🔲 **Residual magnetic dipole torque** — not modeled; τ = m_residual × B (`build_adcs_model.m`)

### Control / Estimation
- 🔲 **Extended Kalman Filter (EKF)** — TRIAD is a batch estimator. A Multiplicative EKF (MEKF) would give better filtering and gyro-bias estimation (`build_adcs_model.m`, control subsystem)
- 🔲 **Mode manager / flight state machine** — no detumble → coarse-pointing → fine-pointing mode transitions; controller is always active (`build_adcs_model.m`, control subsystem)
- 🔲 **Momentum desaturation** — no logic to dump accumulated RW momentum using magnetorquers (`build_adcs_model.m`, control subsystem)
- 🔲 **Attitude reference generation** — target is always identity quaternion [0;0;0;1]. Need nadir-pointing, Sun-pointing, and other reference profiles (`build_adcs_model.m`, control subsystem)

### Reaction Wheels
- 🔲 **Reaction wheel momentum tracking** — saturation is applied per-torque only; no wheel momentum accumulation or speed tracking for desaturation triggers (`build_adcs_model.m`, actuators subsystem)

### Infrastructure
- 🔲 **Logging / telemetry** — only `Attitude_Scope` and `Rates_Scope` exist. Need `To Workspace` blocks or structured data logging (`build_adcs_model.m`)
- 🔲 **Unit tests** — no automated test scripts for model validation
- 🔲 **Integration with sim-finch branch** — `sim-finch` branch has good project structure (README, `.prj`, `sim_config` hierarchy) that should be merged into `simulink` branch

---

## Future / Nice-to-Have (📋)

- 📋 **Higher-fidelity magnetic field** — full IGRF spherical harmonic expansion instead of tilted dipole
- 📋 **Atmospheric density model** — NRLMSISE-00 or JB2008 for drag computation
- 📋 **Ground station contact windows** — visibility analysis for downlink scheduling
- 📋 **Power / thermal coupling** — battery state-of-charge and thermal effects on actuator performance
- 📋 **Monte Carlo simulation framework** — parameter dispersion and sensitivity analysis
- 📋 **Code generation for flight software** — MATLAB Coder / Simulink Coder for embedded deployment
- 📋 **CI/CD pipeline** — automated model build, test, and regression via GitHub Actions
