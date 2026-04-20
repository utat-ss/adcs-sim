# ADCS Simulation — Domain Knowledge & Skills

This document contains the **domain knowledge, mathematical models, physics, and references** used in the UTAT-SS ADCS closed-loop simulation for a 3U CubeSat. Every section lists the equations actually implemented, the parameter values chosen, and links to web sources that validate the assumptions and calculations.

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

The DCM `R` rotates vectors from the **ECI frame to the body frame**. Used in the sun sensor, star tracker, and magnetometer models.

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

where:
- `J` — moment of inertia tensor (3×3, diagonal for principal axes)
- `ω` — angular velocity vector in body frame [rad/s]
- `τ_ext` — sum of all external torques [N·m]

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `J` | `diag([0.05, 0.05, 0.08])` kg·m² | Approximate 3U CubeSat (10×10×30 cm) |

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

## 4. Orbital Mechanics — Keplerian Propagation

Two-body Keplerian orbit propagation. No J2 or higher-order perturbations are modeled.

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

### Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `μ_earth` | `3.986004418 × 10¹⁴` m³/s² | Earth gravitational parameter |
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
3. Add Gaussian noise to each body-frame component:
      s_meas = s_body + N(0, σ²)    where σ = 0.5° (≈ 0.00873 rad)
4. Weighted average of valid sensor readings (renormalized)
```

| Parameter | Value |
|-----------|-------|
| Boresights | `±X, ±Y, ±Z` body axes |
| FOV | 60° half-cone |
| Noise | 0.5° (1σ) per component |

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
| Earth limb | 25° from boresight | Not yet implemented |

When blinded by an exclusion zone, the star tracker **falls back to its last valid measurement**.

**Reference:** [CubeSatShop — ST200 Star Tracker](https://www.cubesatshop.com/product/st200-star-tracker/)

---

### 8.3 IMU (MEMS Gyroscope)

Measures body-frame angular velocity with bias and noise.

**Noise Model:**

```
Bias random walk:
  bias(k+1) = bias(k) + √(dt) · σ_bias · randn(3,1)

Measurement:
  ω_meas = ω_true + bias + (ARW / √dt) · randn(3,1)
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| ARW | `0.01 deg/√s` = `1.745 × 10⁻⁴ rad/√s` | Angular random walk |
| Bias instability | `1.0 deg/hr` = `4.848 × 10⁻⁶ rad/s` | Long-term drift rate |
| `σ_bias` | Derived from bias instability | Drives bias random walk |

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

Simple white-noise model — no outage or multipath modeling.

---

## 9. Attitude Estimation — TRIAD Method

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

---

## 10. Control Laws

### 10.1 PD Controller (Reaction Wheels)

Proportional-derivative controller for three-axis attitude stabilization using reaction wheels.

### Equations

```
τ_rw = −Kp · q_err − Kd · ω_est
```

where:
- `q_err = [q1; q2; q3]` — vector part of the estimated attitude error quaternion (small-angle approximation: `q_err ≈ θ_err / 2`)
- `ω_est` — estimated angular velocity from IMU
- **Shortest-path logic:** if `q0 < 0`, negate `q_err` to ensure rotation takes the shorter path

### Parameters

| Parameter | Value | Units |
|-----------|-------|-------|
| `Kp` | `0.002` | N·m |
| `Kd` | `0.02` | N·m·s |
| Target quaternion | `[1; 0; 0; 0]` | Identity (nadir or inertial hold) |

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
| `k_bdot` | `10⁵` | Tuned for nT-scale field measurements |
| Saturation | `±0.2 A·m²` | Per axis |

### References

- [Wikipedia — B-dot controller](https://en.wikipedia.org/wiki/B-dot_controller)
- Avanzini & Giulietti, "Magnetic Detumbling of a Rigid Spacecraft"

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

### 11.3 CMG (Placeholder)

Control Moment Gyroscope — currently a **placeholder** that outputs zero torque.

**Intended model:**

```
τ_cmg = dH/dt = h · (gimbal_rate × gimbal_axis)
```

where:
- `h` — stored angular momentum magnitude of the CMG rotor
- `gimbal_rate` — rate of gimbal angle change
- `gimbal_axis` — unit vector along the gimbal axis

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
| Earth mean radius | `R_e` | `6371` | km |
| Equatorial magnetic field | `B₀` | `3.12 × 10⁻⁵` | T |
| Dipole tilt angle | `θ_m` | `11.5` | degrees |
| Dipole tilt longitude | `φ_m` | `−69` | degrees |
| CubeSat inertia | `J` | `diag([0.05, 0.05, 0.08])` | kg·m² |
| Orbital altitude | `h` | `500` | km |
| Orbital inclination | `i` | `51.6` | degrees |
| Orbital period | `T` | `≈ 5670` | s |
