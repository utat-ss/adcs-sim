# ADCS Simulation — Domain Knowledge & Skills

This document contains the **domain knowledge, mathematical models, physics, and references** used in the UTAT-SS ADCS closed-loop simulation for a 3U CubeSat. It mixes the math that is active in the current validated branch with higher-fidelity models that are architecturally planned but presently running compile-safe fallbacks.

Use `builders/`, `reference.md`, and `docs/adcs_model_reading_guide.pdf` to determine whether a section below is currently live or still part of the intended higher-fidelity design.

---

## 1. Attitude Representation — Quaternions

Quaternions provide a singularity-free, four-parameter representation of 3D orientation.

### Convention

Scalar-first (Hamilton convention):

```
q = [q0; q1; q2; q3]
```

where `q0` is the scalar part and `[q1, q2, q3]` is the vector part.

**Unit quaternion constraint:**

```
q0² + q1² + q2² + q3² = 1
```

### Direction Cosine Matrix (DCM) from Quaternion

The DCM `R` rotates vectors from the **ECI frame to the body frame**. Used in the sun sensor, star tracker, magnetometer, and active gravity-gradient implementation. The same convention must be preserved across sensor and disturbance-torque blocks.

```
R = [ 1 - 2(q2² + q3²),    2(q1·q2 + q0·q3),    2(q1·q3 - q0·q2) ;
      2(q1·q2 - q0·q3),    1 - 2(q1² + q3²),     2(q2·q3 + q0·q1) ;
      2(q1·q3 + q0·q2),    2(q2·q3 - q0·q1),     1 - 2(q1² + q2²) ]
```

### Quaternion Multiplication

Used in the star tracker model to compose rotations. Given two quaternions `p` and `q`:

```
p ⊗ q = [ p0·q0 - p1·q1 - p2·q2 - p3·q3 ;
          p0·q1 + p1·q0 + p2·q3 - p3·q2 ;
          p0·q2 - p1·q3 + p2·q0 + p3·q1 ;
          p0·q3 + p1·q2 - p2·q1 + p3·q0 ]
```

### Shepperd Method (DCM → Quaternion)

Used in the TRIAD estimator to convert a computed DCM back to a quaternion. The method selects the largest diagonal element to avoid numerical singularity:

```
Compute:
  d0 = 1 + R(1,1) + R(2,2) + R(3,3)
  d1 = 1 + R(1,1) - R(2,2) - R(3,3)
  d2 = 1 - R(1,1) + R(2,2) - R(3,3)
  d3 = 1 - R(1,1) - R(2,2) + R(3,3)

Pick the largest di, then:
  If d0 is largest:
    q0 = 0.5·√d0
    q1 = (R(2,3) - R(3,2)) / (4·q0)
    q2 = (R(3,1) - R(1,3)) / (4·q0)
    q3 = (R(1,2) - R(2,1)) / (4·q0)

  (Similar expressions for d1, d2, d3 being largest)
```

### References

- [VectorNav — Attitude Transformations](https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran)
- Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*

---

## 2. Rigid Body Dynamics — Euler's Equation

Euler's equation governs rotational motion of a rigid body subject to external torques.

### Equation

```
J · ω̇ = τ_ext − ω × (J · ω)
```

Solved as:

```
ω̇ = J⁻¹ · ( τ_ext − ω × (J · ω) )
```

In the current validated branch, the live `Euler_RHS` block extends this baseline to include stored reaction-wheel momentum and wheel-body reaction torque:

```
H_total = J · ω + A_W · h_W
ω̇ = J⁻¹ · ( τ_ext − ω × H_total − τ_rw,body )
```

where:
- `J` — moment of inertia tensor (3×3, diagonal for principal axes)
- `ω` — angular velocity vector in body frame [rad/s]
- `τ_ext` — sum of all external torques [N·m]

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `J` | `diag([0.0333, 0.0333, 0.0067])` kg·m² | Uniform-density 4 kg, 10×10×30 cm cuboid baseline |

### References

- [Wikipedia — Euler's equations (rigid body dynamics)](https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics))

---

## 3. Quaternion Kinematics

Describes how the attitude quaternion evolves over time given angular velocity.

### Equation

```
q̇ = ½ · Ω(ω) · q
```

where `Ω` is the 4×4 skew-symmetric matrix constructed from the body angular velocity `ω = [ωx, ωy, ωz]`:

```
Ω(ω) = [  0,  -ωx, -ωy, -ωz ;
          ωx,   0,   ωz, -ωy ;
          ωy, -ωz,   0,   ωx ;
          ωz,  ωy,  -ωx,   0  ]
```

### Quaternion Renormalization

After each integration step, the quaternion is renormalized to prevent numerical drift from the unit constraint:

```
q ← q / ‖q‖
```

### References

- [Ashwin Narayan — How to Integrate Quaternions](https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/)

---

## 4. Orbital Mechanics — Keplerian Propagation with J2 Secular Drift

The orbit model uses a Keplerian propagator with first-order secular `J2` corrections applied to the angular elements and mean anomaly.

### Kepler's Equation

Relates mean anomaly `M` to eccentric anomaly `E`:

```
M = E − e · sin(E)
```

Solved iteratively via **Newton's method**:

```
E_{n+1} = E_n − (E_n − e·sin(E_n) − M) / (1 − e·cos(E_n))
```

### True Anomaly from Eccentric Anomaly

```
ν = 2 · atan2( √(1+e) · sin(E/2),  √(1−e) · cos(E/2) )
```

### Position in Perifocal Frame (PQW)

```
r_PQW = a·(1 − e²) / (1 + e·cos(ν)) · [ cos(ν); sin(ν); 0 ]
```

### PQW → ECI Rotation

Constructed from three successive rotations using the classical orbital elements: RAAN (Ω), argument of perigee (ω), and inclination (i):

```
R_PQW_to_ECI = Rz(−Ω) · Rx(−i) · Rz(−ω)
```

where `Rz` and `Rx` are standard rotation matrices about the Z and X axes.

### Mean Motion & Period

```
n = √(μ / a³)          [rad/s]
T = 2π / n              [s]
```

### First-Order J2 Secular Rates

Using the semi-latus rectum `p = a(1 − e²)`:

```
Ω̇ = −(3/2) J2 n (R_e / p)² cos(i)

ω̇ =  (3/4) J2 n (R_e / p)² (5 cos²(i) − 1)

Ṁ = n + (3/4) J2 n (R_e / p)² √(1 − e²) (3 cos²(i) − 1)
```

These rates are integrated linearly over time and then used to reconstruct the ECI state from the evolving mean elements.

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `μ_earth` | `3.986004418 × 10¹⁴` m³/s² | Earth gravitational parameter |
| `J2_earth` | `1.08262668 × 10⁻³` | Earth second zonal harmonic |
| `a` | `R_earth + 500 km = 6871 km` | Semi-major axis (LEO) |
| `i` | `51.6°` | ISS-like inclination |
| `e` | `≈ 0` | Near-circular orbit |
| `T` | `≈ 5670 s ≈ 94.5 min` | Orbital period |

### References

- [Wikipedia — Kepler's Equation](https://en.wikipedia.org/wiki/Kepler%27s_equation)
- [Orbital Mechanics — Elliptical Orbits](https://orbital-mechanics.space/classical-orbital-elements/elliptical-orbits.html)
- Curtis, *Orbital Mechanics for Engineering Students*

---

## 5. Magnetic Field — Tilted Dipole Model

Approximation of Earth's magnetic field using a tilted dipole, serving as a lightweight substitute for full IGRF.

### Dipole Field Equation

```
B(r) = (B₀ · R_e³ / r³) · [ 3·(m̂ · r̂)·r̂ − m̂ ]
```

where:
- `B₀` — equatorial surface field strength
- `R_e` — Earth's mean radius
- `r` — distance from Earth center to spacecraft
- `r̂` — unit vector from Earth center to spacecraft
- `m̂` — unit vector along the magnetic dipole axis

### Dipole Axis Unit Vector

The dipole is tilted `θ_m = 11.5°` toward geographic longitude `φ_m = −69°`:

```
m̂ = [ sin(θ_m) · cos(φ_m) ;
       sin(θ_m) · sin(φ_m) ;
       cos(θ_m)             ]
```

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `B₀` | `3.12 × 10⁻⁵` T | Equatorial surface field strength |
| `R_e` | `6371 km` | Earth mean radius |
| `θ_m` | `11.5°` | Dipole tilt angle |
| `φ_m` | `−69°` | Dipole tilt longitude |

### References

- [Wikipedia — Dipole model of the Earth's magnetic field](https://en.wikipedia.org/wiki/Dipole_model_of_the_Earth%27s_magnetic_field)
- [NOAA IGRF](https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html) (for full IGRF coefficients)

---

## 6. Eclipse Model — Cylindrical Shadow

Determines whether the spacecraft is in Earth's shadow using a simple cylindrical projection.

### Algorithm

Given the spacecraft position `r_sc` and the Sun direction unit vector `ŝ`:

```
1. Projection of r_sc onto Sun direction:
      proj = r_sc · ŝ

2. Perpendicular distance from r_sc to the Sun line:
      perp = ‖r_sc − proj · ŝ‖

3. Eclipse condition:
      in_eclipse = (proj < 0) AND (perp < R_earth)
```

This is a **simple cylindrical model** — no distinction between penumbra and umbra.

### References

- Montenbruck & Gill, *Satellite Orbits*, Chapter 3.4

---

## 7. Solar Ephemeris — Meeus Low-Precision Model

Used by the **onboard ephemeris** (controller side) for Sun direction. Accuracy is approximately **1°**, sufficient for coarse pointing and eclipse prediction.

### Equations

All angles in degrees unless noted. `T` is Julian centuries from J2000:

```
T = (JD − 2451545.0) / 36525
```

**Mean anomaly:**

```
M = 357.5291 + 35999.0503 · T    (mod 360°)
```

**Mean longitude:**

```
L₀ = 280.4664 + 36000.7698 · T   (mod 360°)
```

**Equation of center:**

```
C = 1.9146 · sin(M) + 0.0200 · sin(2M)
```

**Ecliptic longitude:**

```
λ = L₀ + C
```

**Mean obliquity of the ecliptic:**

```
ε = 23.4393 − 0.0130 · T
```

**Sun unit vector in ECI (equatorial coordinates):**

```
s_ECI = [ cos(λ)         ;
          cos(ε) · sin(λ) ;
          sin(ε) · sin(λ) ]
```

### References

- Jean Meeus, *Astronomical Algorithms*
- [USNO — Approximate Solar Position](https://aa.usno.navy.mil/faq/sun_approx)

---

## 8. Sensors

### 8.1 Fine Sun Sensors (FSS)

Six sun sensors mounted on the cube faces with boresights along the ±X, ±Y, ±Z body axes. Each sensor measures the Sun unit vector in its local frame.

**Model:**

```
1. Compute Sun vector in body frame: s_body = R · s_ECI
2. For each sensor i with boresight n̂_i:
      angle_i = acos(s_body · n̂_i)
      valid_i = (angle_i < 60°)   — half-cone FOV
3. Sample at `fss_sample_rate` and hold the output between samples.
4. Add Gaussian noise to each body-frame component:
      s_meas = s_body + N(0, σ²)    where σ = 0.5° (≈ 0.00873 rad)
5. Cosine-weighted average of valid sensor readings (renormalized)
```

| Parameter | Value |
|-----------|-------|
| Boresights | `±X, ±Y, ±Z` body axes |
| FOV | 60° half-cone |
| Noise | 0.5° (1σ) per component |
| Sample rate | `10 Hz` |

**Reference:** [CubeSatShop — nanoSSOC-A60 Sun Sensor](https://www.cubesatshop.com/product/nano-ssoc-a60-sun-sensor/)

---

### 8.2 Star Tracker

Provides a full attitude quaternion measurement. Boresight along the +Z body axis.

**Noise Model:**

Noise is applied as a **small-angle quaternion perturbation**:

```
δq = [ 1; δθx/2; δθy/2; δθz/2 ]    (unnormalized, then normalized)

where:
  δθz ~ N(0, σ_boresight²)        σ_boresight  = 10 arcsec = 4.85 × 10⁻⁵ rad
  δθx, δθy ~ N(0, σ_cross²)      σ_cross      = 40 arcsec = 1.94 × 10⁻⁴ rad

q_meas = q_true ⊗ δq    (quaternion multiplication)
```

**Exclusion Zones:**

| Body | Exclusion Angle | Status |
|------|----------------|--------|
| Sun | 45° from boresight | Implemented |
| Moon | 25° from boresight | Implemented |
| Earth limb | 25° beyond the apparent Earth disk | Implemented |

When blinded by an exclusion zone, the star tracker **falls back to its last valid measurement**.

**Earth-limb geometry:**

```
θ_earth = asin(R_earth / |r_sc|)
θ_bore  = acos( b̂_ECI · (−r_sc / |r_sc|) )

Blocked if:
  θ_bore ≤ θ_earth + θ_margin
```

where `θ_margin = st_exclusion_earth`. The star tracker also updates at `2 Hz` and holds the last sample between updates.

**Reference:** [CubeSatShop — ST200 Star Tracker](https://www.cubesatshop.com/product/st200-star-tracker/)

---

### 8.3 IMU (MEMS Gyroscope)

Measures body-frame angular velocity with bias and noise.

**Noise Model:**

```
Bias random walk:
  bias(k+1) = bias(k) + √(Ts) · σ_bias · randn(3,1)

Measurement:
  ω_meas = ω_true + bias + (ARW / √Ts) · randn(3,1)
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| ARW | `0.01 deg/√s` = `1.745 × 10⁻⁴ rad/√s` | Angular random walk |
| Bias instability | `1.0 deg/hr` = `4.848 × 10⁻⁶ rad/s` | Long-term drift rate |
| `σ_bias` | Derived from bias instability | Drives bias random walk |
| Sample rate | `100 Hz` | Output held between samples |
| Range | `±300 deg/s` | Per-axis saturation |

**References:**

- [VectorNav — IMU Specifications](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--error-budgets/specs-imuspecs)
- IEEE Std 952-1997 (gyroscope terminology)

---

### 8.4 Magnetometer

Measures the local magnetic field in the body frame.

**Model:**

```
B_body_true = R · B_ECI

B_meas = B_body_true + b_hard + N(0, σ²)
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| Hard-iron bias | `[50, −30, 80]` nT | Fixed offset per axis |
| White noise | `100` nT per axis (1σ) | Gaussian |
| Sample rate | `10 Hz` | Output held between samples |

**Reference:** Typical CubeSat magnetometer specs (e.g., PNI RM3100)

---

### 8.5 GNSS Receiver

Provides position and velocity in the ECI frame.

**Model:**

```
r_meas = r_true + N(0, σ_pos²)
v_meas = v_true + N(0, σ_vel²)
```

| Parameter | Value |
|-----------|-------|
| Position noise | `10` m per axis (1σ) |
| Velocity noise | `0.1` m/s per axis (1σ) |
| Sample rate | `1 Hz` |

Simple white-noise model — no outage or multipath modeling.

---

## 9. Attitude Estimation — TRIAD + MEKF

Two-vector deterministic attitude determination. Produces a DCM (and then quaternion) from simultaneous observations of two known reference vectors.

### Vectors Used

| Vector | Body Frame | Reference (ECI) Frame |
|--------|-----------|----------------------|
| Sun direction | `s_body` (from sun sensor) | `s_ECI` (from ephemeris) |
| Magnetic field | `m_body` (from magnetometer) | `m_ECI` (from dipole model) |

### Algorithm

**Body triad construction:**

```
t1_b = s_b
t2_b = (s_b × m_b) / ‖s_b × m_b‖
t3_b = t1_b × t2_b
```

**Reference triad construction (same procedure):**

```
t1_r = s_r
t2_r = (s_r × m_r) / ‖s_r × m_r‖
t3_r = t1_r × t2_r
```

**DCM from triads:**

```
M_body = [t1_b, t2_b, t3_b]       (3×3 matrix, columns are triad vectors)
M_ref  = [t1_r, t2_r, t3_r]

A = M_body · M_refᵀ
```

The resulting DCM `A` is then converted to a quaternion using the **Shepperd method** (see Section 1).

**Fallback:** If the Sun and magnetic field vectors are nearly parallel (degenerate geometry), the estimator falls back to the **star tracker quaternion** directly.

### References

- [Wikipedia — Triad method](https://en.wikipedia.org/wiki/Triad_method)
- Markley, "Attitude Determination using Two Vector Measurements"

### 9.1 Multiplicative Extended Kalman Filter (MEKF)

The estimator now uses TRIAD as a deterministic initialization / recovery step and runs a **multiplicative extended Kalman filter** for continuous attitude and gyro-bias estimation.

**State:**

```
x = [δθ ; b_g]
```

where:
- `δθ` is the 3-parameter attitude error state
- `b_g` is the gyroscope bias estimate

**Propagation:**

```
ω̂ = ω_meas − b_g
q⁻ = δq(ω̂, Δt) ⊗ q
```

The covariance is propagated with the standard MEKF linearized state transition and process-noise matrices using `mekf_sigma_g` and `mekf_sigma_b`.

**Measurement update:**

Predicted body-frame vectors are formed from the propagated quaternion:

```
v̂_B = C_BI(q⁻) v_I
```

and compared against the measured Sun and magnetic-field vectors. When the Sun vector is unavailable (eclipse), the filter performs a magnetometer-only update. When the Sun vector returns after eclipse, the TRIAD solution is used to re-anchor the quaternion estimate.

**Implemented tuning parameters:**

| Parameter | Meaning |
|-----------|---------|
| `mekf_sigma_g` | Gyro process-noise proxy |
| `mekf_sigma_b` | Gyro bias random-walk proxy |
| `mekf_sun_sigma` | Sun-vector measurement noise |
| `mekf_mag_sigma` | Normalized magnetic-vector measurement noise |

---

## 10. Control Laws

### 10.1 PD Controller (Reaction Wheels)

Proportional-derivative controller for three-axis attitude stabilization using reaction wheels.

### Equations

```
τ_cmd = ω × (J·ω) − Kp · q_e,vec − Kd · (ω_est − ω_des)
```

where:
- `q_e,vec` — vector part of the full error quaternion `q_e = q_d⁻¹ ⊗ q_est`
- `ω_des` — desired LVLH angular rate from the reference generator
- **Shortest-path logic:** if `q_e,0 < 0`, negate `q_e,vec` to ensure the controller commands the shorter rotation

### Parameters

| Parameter | Value | Units |
|-----------|-------|-------|
| `Kp` | `0.01` | N·m |
| `Kd` | `0.05` | N·m·s |
| Target quaternion | `q_des` from LVLH | Nadir-pointing |

### Reference

- Sidi, *Spacecraft Dynamics and Control*, Chapter 9

---

### 10.2 B-dot Law (Magnetorquers)

Magnetic rate-damping controller for detumbling. Uses the time derivative of the measured magnetic field as a proxy for angular velocity.

### Equation

```
m_cmd = −k_bdot · ΔB / Δt
```

where:
- `ΔB / Δt` — discrete time derivative of the body-frame magnetic field measurement
- `m_cmd` — commanded magnetic dipole moment [A·m²]

**Saturation:**

```
m_cmd = clamp(m_cmd, −0.2, +0.2)    [A·m² per axis]
```

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `k_bdot` | `5 × 10⁴` | Tuned for T-scale body-field measurements |
| Saturation | `±0.2 A·m²` | Per axis |

### References

- [Wikipedia — B-dot controller](https://en.wikipedia.org/wiki/B-dot_controller)
- Avanzini & Giulietti, "Magnetic Detumbling of a Rigid Spacecraft"

---

### 10.2.1 Web GUI Detumble Proxy

The browser-side mission GUI does not run the full magnetorquer B-dot loop. For interactive visualization it uses a deterministic rate-decay proxy:

```
omega_detumble(t) = max(omega_floor, omega_0 * exp(-t / tau))
```

with `omega_0 = 1.8 deg/s`, `omega_floor = 0.03 deg/s`, and `tau = 900 s`. The visual tumble angle integrates this decaying rate, so the spacecraft rotation visibly slows as detumbling progresses. The UI progress value is:

```
progress = clamp((omega_0 - omega_detumble) / (omega_0 - omega_floor), 0, 1)
```

This proxy is for web-simulation feedback only; the Simulink controller remains the source of truth for actuator-level detumbling behavior.

---

### 10.3 Mode Management

The controller now uses three flight states:

1. **Detumble** — when body rate is above `mode_detumble_rate`
2. **Coarse pointing** — when detumble is complete but attitude/rate thresholds for fine pointing are not met
3. **Fine pointing** — when the spacecraft is close enough to the nadir reference for full closed-loop pointing

Fine mode is also inhibited when wheel momentum is already close to saturation.

### 10.4 Reaction-Wheel Momentum Desaturation

Wheel unloading is implemented with a magnetic cross-product law:

```
h_body = A_W · h_W
m_dump = −k_dump · (h_body × B_body) / |B_body|²
```

This commands a magnetorquer dipole that removes the wheel-momentum component perpendicular to the instantaneous geomagnetic field.

| Parameter | Value |
|-----------|-------|
| `k_dump` | `0.02` |
| Enable threshold | `0.6 × rw_max_momentum × √rw_num` |

References:
- Wertz, *Spacecraft Attitude Determination and Control*
- Psiaki, "Magnetic Torquer Attitude Control via Asymptotic Periodic LQR"

---

### 10.5 Environmental Disturbance Torques

The dynamics model now includes three additional environmental disturbance torques beyond gravity gradient.

**Aerodynamic drag torque**

An exponential density model is used around the operating altitude:

```
ρ(h) = ρ_ref · exp(−(h − h_ref) / H)
```

with panel forces computed from the relative atmospheric velocity and body-face projected area:

```
F_drag,i = −(1/2) ρ |v_rel|² C_D A_i max(0, n̂_i · (−v̂_rel)) v̂_rel
τ_drag = Σ (r_COP,i × F_drag,i)
```

**Solar radiation pressure torque**

The SRP force model is also panel-based and disabled during eclipse:

```
F_srp,i = P_solar C_r A_i max(0, n̂_i · ŝ_body) ŝ_body
τ_srp = Σ (r_COP,i × F_srp,i)
```

where `P_solar = solar_flux / c`.

**Residual magnetic dipole torque**

```
τ_mag = m_residual × B_body
```

This captures bias-like magnetic disturbances caused by onboard materials and currents.

| Parameter | Value |
|-----------|-------|
| `drag_coeff` | `2.3` |
| `atm_ref_density` | `6.967 × 10⁻¹³ kg/m³` |
| `atm_scale_height` | `63.822 km` |
| `srp_coeff` | `1.3` |
| `residual_dipole` | `[5, 5, 5] mA·m²` |

Reference:
- NASA/TM-20210017131, environmental disturbance torque modeling overview

---

## 11. Actuators

### 11.1 Reaction Wheels — Pyramid Configuration

Four reaction wheels in a pyramid arrangement providing three-axis torque authority with one degree of redundancy.

### Geometry

Cant angle:

```
β = arctan(1/√2) ≈ 35.26°
```

**Spin-axis matrix** `A` (3×4): each column is the spin axis of one wheel in the body frame. The four axes are symmetrically distributed around the body Z-axis at 90° azimuth intervals, each tilted by angle `β` from the Z-axis.

### Torque Distribution

Given a commanded 3-axis torque `τ_cmd`, the per-wheel torques are computed via the **Moore-Penrose pseudoinverse**:

```
τ_wheels = A⁺ · τ_cmd

where A⁺ = Aᵀ · (A · Aᵀ)⁻¹
```

### Parameters

| Parameter | Value | Units |
|-----------|-------|-------|
| Cant angle `β` | `arctan(1/√2) ≈ 35.26°` | degrees |
| Per-wheel torque limit | `±5` | mN·m |
| Max stored momentum | `0.05` | N·m·s |

### References

- [Wikipedia — Reaction Wheel](https://en.wikipedia.org/wiki/Reaction_wheel)
- Markley & Crassidis, Ch. 7

---

### 11.2 Magnetorquers

Three magnetic torque rods aligned with the body X, Y, and Z axes.

### Torque Equation

```
τ_mag = m × B
```

where:
- `m` — magnetic dipole moment vector [A·m²]
- `B` — local magnetic field in body frame [T]

### Parameters

| Parameter | Value |
|-----------|-------|
| Max dipole | `0.2 A·m²` per axis |
| Axes | Body X, Y, Z |

### Reference

- [Wikipedia — Magnetorquer](https://en.wikipedia.org/wiki/Magnetorquer)

---

### 11.3 CMG (Representative Model)

The simulator now uses a simplified representative 3-CMG cluster. Each input is interpreted as a gimbal-rate request and converted to body torque through

```
τ_cmg = dH/dt = Ω_g × h
```

For the chosen virtual geometry, this reduces to

```
τ_body = h_cmg · [γ̇2; γ̇3; γ̇1]
```

with per-axis torque saturation applied after the mapping.

| Parameter | Value |
|-----------|-------|
| Max torque | `0.01 N·m` |
| Stored momentum | `0.5 N·m·s` |
| Max gimbal rate | `1 rad/s` |

---

## 12. Ephemeris Data

Pre-computed Sun and Moon positions used as simulation ground truth (separate from the onboard analytical ephemeris in Section 7).

### Source File

```
ephemeris_2026_weekly.csv
```

### Contents

- **53 weekly data points** spanning all of 2026
- Each row contains:
  - Timestamp
  - Sun direction (ECI unit vector: 3 components)
  - Moon direction (ECI unit vector: 3 components)

### Runtime Usage

- **Linear interpolation** between data points at each simulation time step
- Provides the reference Sun and Moon vectors for sensor models and eclipse computation

---

## Summary of Key Constants

| Constant | Symbol | Value | Units |
|----------|--------|-------|-------|
| Earth gravitational parameter | `μ` | `3.986004418 × 10¹⁴` | m³/s² |
| Earth J2 | `J2` | `1.08262668 × 10⁻³` | — |
| Earth mean radius | `R_e` | `6371` | km |
| Equatorial magnetic field | `B₀` | `3.12 × 10⁻⁵` | T |
| Dipole tilt angle | `θ_m` | `11.5` | degrees |
| Dipole tilt longitude | `φ_m` | `−69` | degrees |
| Solar radiation pressure at 1 AU | `P_solar` | `≈ 4.54 × 10⁻⁶` | N/m² |
| CubeSat inertia | `J` | `diag([0.0333, 0.0333, 0.0067])` | kg·m² |
| Orbital altitude | `h` | `500` | km |
| Orbital inclination | `i` | `51.6` | degrees |
| Orbital period | `T` | `≈ 5670` | s |
## Web ADCS GUI Physics Notes

The browser GUI in `web-adcs-gui/` uses a lightweight mission model intended for interactive visualization and parameter exploration. It is not a replacement for the Simulink dynamics, but the math should remain physically coherent with the simulator conventions.

### Orbital Propagation

For elliptical Earth orbits, the app propagates classical orbital elements with the two-body Kepler equation:

```text
n = sqrt(mu / a^3)
M(t) = M0 + n t
M = E - e sin(E)
r_pf = p / (1 + e cos(nu)) [cos(nu), sin(nu), 0]
v_pf = sqrt(mu / p) [-sin(nu), e + cos(nu), 0]
p = a (1 - e^2)
```

The perifocal state is rotated to ECI by `R3(RAAN) R1(i) R3(arg_perigee)`. The app also applies first-order J2 secular drift for RAAN and argument of perigee:

```text
RAAN_dot = -3/2 J2 n (Re / p)^2 cos(i)
argp_dot =  3/4 J2 n (Re / p)^2 (5 cos(i)^2 - 1)
```

### Transfers

The Hohmann altitude-change estimate uses:

```text
a_t = (r1 + r2) / 2
dv1 = |sqrt(mu (2/r1 - 1/a_t)) - sqrt(mu/r1)|
dv2 = |sqrt(mu/r2) - sqrt(mu (2/r2 - 1/a_t))|
t_transfer = pi sqrt(a_t^3 / mu)
```

Plane-change cost is approximated as:

```text
dv_plane = 2 v sin(delta_i / 2)
```

### Environment and ADCS

- Eclipse is modeled with a cylindrical Earth shadow and a small penumbra smoothing width.
- Sun vectors come from `ephemeris_2026_weekly.csv` when available, otherwise from a low-precision analytical solar model.
- Moon vectors are visual context only unless a CSV provides them.
- Magnetic field magnitude is a simple tilted dipole, suitable for GUI telemetry context but not high-fidelity magnetometer validation.
- Attitude quaternions are scalar-first `[q0, q1, q2, q3]` until converted at the Three.js render boundary.
- Browser-side fallback spacecraft values mirror the MATLAB default 4 kg, `0.10 x 0.10 x 0.30 m` 3U bus. When syncing from `init_adcs_params.m`, SI orbit lengths such as `orbit_alt` and `orbit_a` are converted to kilometers before mission propagation.
- The Three.js scene uses a visual-only radial altitude exaggeration factor of `3.2` for rendered orbit and spacecraft positions. Numeric propagation, telemetry, delta-V estimates, eclipse checks, target visibility, and logged values stay in physical kilometers; only the browser display transform is exaggerated so LEO paths and spacecraft clearance are readable at Earth scale.
- Near-circular browser orbit tests should compare instantaneous radius against the configured semi-major axis with an eccentricity-aware tolerance. Even `e = 0.001` produces roughly `a * e` radial variation, so hard-coding `R_e + altitude` as the instantaneous radius is too strict unless the test orbit is exactly circular.
