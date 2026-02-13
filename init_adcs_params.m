%% init_adcs_params.m
%  Initialize workspace parameters for the ADCS Simulator.
%  Run this BEFORE simulating adcs_sim.slx.
%
%  Usage:
%      init_adcs_params      % loads all parameters into base workspace
%      build_adcs_model      % (only needed once to create the Simulink model)
%      sim('adcs_sim')       % run the simulation

%% ════════════════════════════════════════════════════════════════════════
%  SPACECRAFT PARAMETERS
%  ════════════════════════════════════════════════════════════════════════

% Inertia tensor  [kg·m²]  (approximate 3U CubeSat)
J = diag([0.05, 0.05, 0.08]);

% Mass [kg]
mass_sc = 4.0;

%% ════════════════════════════════════════════════════════════════════════
%  INITIAL CONDITIONS
%  ════════════════════════════════════════════════════════════════════════

% Attitude quaternion  [scalar-first: q0 qx qy qz]
q0 = [1; 0; 0; 0];

% Body angular velocity  [rad/s]
omega0 = [0.01; -0.005; 0.02];

%% ════════════════════════════════════════════════════════════════════════
%  SIMULATION
%  ════════════════════════════════════════════════════════════════════════

dt    = 0.1;       % Fixed integration step  [s]
t_end = 6000;      % Duration  [s]  (~100 min, ≈ 1 orbit)

%% ════════════════════════════════════════════════════════════════════════
%  EPHEMERIS DATA  (loaded from CSV — simulation ground truth)
%  ════════════════════════════════════════════════════════════════════════

% Read the weekly ephemeris spreadsheet
ephem_csv = 'ephemeris_2026_weekly.csv';
fprintf('  Loading ephemeris from %s ... ', ephem_csv);
ephem_raw = readtable(ephem_csv, 'TextType', 'string');

% Parse ISO-8601 timestamps → datetime → seconds from epoch
ephem_epoch = datetime(2026, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');
ephem_dt    = datetime(ephem_raw.timestamp, 'InputFormat', ...
    'yyyy-MM-dd''T''HH:mm:ssXXX', 'TimeZone', 'UTC');
ephem_t_sec = seconds(ephem_dt - ephem_epoch);   % [Nx1] seconds from epoch

% Sun & Moon unit direction vectors  [Nx3]
ephem_sun  = [ephem_raw.sun_x,  ephem_raw.sun_y,  ephem_raw.sun_z];
ephem_moon = [ephem_raw.moon_x, ephem_raw.moon_y, ephem_raw.moon_z];

fprintf('done  (%d data points, %.0f days span)\n', ...
    numel(ephem_t_sec), ephem_t_sec(end)/86400);

%% ════════════════════════════════════════════════════════════════════════
%  ORBIT PARAMETERS  (LEO, ISS-like)
%  ════════════════════════════════════════════════════════════════════════

R_earth  = 6371e3;            % Earth mean radius  [m]
mu_earth = 3.986004418e14;    % Gravitational parameter  [m³/s²]

orbit_alt  = 500e3;           % Altitude above surface  [m]
orbit_inc  = 51.6;            % Inclination  [deg]
orbit_ecc  = 0.0001;          % Eccentricity  (near-circular)
orbit_RAAN = 0;               % Right ascension of ascending node  [deg]
orbit_AOP  = 0;               % Argument of perigee  [deg]
orbit_TA   = 0;               % True anomaly at epoch  [deg]

orbit_a      = R_earth + orbit_alt;                        % Semi-major axis  [m]
orbit_period = 2*pi * sqrt(orbit_a^3 / mu_earth);         % Period  [s]
orbit_n      = 2*pi / orbit_period;                        % Mean motion  [rad/s]

%% ════════════════════════════════════════════════════════════════════════
%  SENSOR PARAMETERS
%  ════════════════════════════════════════════════════════════════════════

% ── Fine Sun Sensors (photodiode-based) ──
fss_noise_sigma  = 0.5;       % Measurement noise 1σ  [deg]
fss_fov_half     = 60;        % Half-cone field of view  [deg]
fss_sample_rate  = 10;        % Sample rate  [Hz]
fss_num          = 6;         % Number of sensors

% FSS boresight unit vectors in body frame  (6 faces of a cube)
fss_boresights = [ 1  0  0;   % FSS 1: +X
                  -1  0  0;   % FSS 2: -X
                   0  1  0;   % FSS 3: +Y
                   0 -1  0;   % FSS 4: -Y
                   0  0  1;   % FSS 5: +Z
                   0  0 -1]'; % FSS 6: -Z    (3 × 6 matrix, each column = boresight)

% ── Star Tracker ──
st_noise_bore    = 10;        % Boresight axis noise 1σ  [arcsec]
st_noise_roll    = 40;        % Roll axis noise 1σ  [arcsec]
st_sample_rate   = 2;         % Sample rate  [Hz]
st_exclusion_sun   = 45;      % Sun exclusion half-angle  [deg]
st_exclusion_moon  = 25;      % Moon exclusion half-angle  [deg]
st_exclusion_earth = 25;      % Earth-limb exclusion  [deg]
st_boresight     = [0; 0; 1]; % Boresight direction in body frame

% ── IMU (MEMS gyroscope) ──
imu_arw           = 0.01;     % Angular random walk  [deg/√s]
imu_bias_instab   = 1.0;      % Bias instability  [deg/hr]
imu_sample_rate   = 100;      % Sample rate  [Hz]
imu_range         = 300;      % Measurement range  [deg/s]

% ── Magnetometer (3-axis) ──
mag_noise_sigma  = 100;       % Noise 1σ per axis  [nT]
mag_bias         = [50; -30; 80];  % Hard-iron bias  [nT]
mag_sample_rate  = 10;        % Sample rate  [Hz]

% ── GNSS Receiver ──
gnss_pos_sigma   = 10;        % Position noise 1σ  [m]
gnss_vel_sigma   = 0.1;       % Velocity noise 1σ  [m/s]
gnss_sample_rate = 1;         % Sample rate  [Hz]

%% ════════════════════════════════════════════════════════════════════════
%  ACTUATOR PARAMETERS
%  ════════════════════════════════════════════════════════════════════════

% ── Reaction Wheels (4-wheel pyramid configuration) ──
rw_max_torque    = 0.005;     % Max torque per wheel  [N·m]
rw_max_momentum  = 0.05;      % Max stored momentum  [N·m·s]
rw_max_speed     = 6000;      % Max wheel speed  [RPM]
rw_inertia       = 5e-5;      % Rotor moment of inertia  [kg·m²]
rw_num           = 4;

% Pyramid cant angle so that each wheel contributes equally to 3 axes
beta_rw = atan(1/sqrt(2));     % ≈ 35.26°  [rad]

% Spin-axis unit vectors for 4-wheel pyramid  (3 × 4 matrix)
rw_axes = [ cos(beta_rw)   0              -cos(beta_rw)   0;
            0               cos(beta_rw)   0              -cos(beta_rw);
            sin(beta_rw)    sin(beta_rw)   sin(beta_rw)    sin(beta_rw)];

% ── Magnetorquers (3-axis, aligned with body X,Y,Z) ──
mtq_max_dipole   = 0.2;       % Max magnetic dipole moment per rod  [A·m²]
mtq_axes         = eye(3);    % Dipole directions in body frame
mtq_num          = 3;

% ── Control Moment Gyroscope (single, placeholder) ──
cmg_max_torque   = 0.01;      % Max output torque  [N·m]
cmg_momentum     = 0.5;       % Rotor angular momentum  [N·m·s]
cmg_gimbal_rate  = 1.0;       % Max gimbal rate  [rad/s]

%% ════════════════════════════════════════════════════════════════════════
%  ENVIRONMENT / REFERENCE CONSTANTS
%  ════════════════════════════════════════════════════════════════════════

% Simulation epoch (Julian date)
epoch_jd = 2461083.5;         % 2026-02-12 00:00:00 UTC  (Julian date)

% Earth magnetic dipole strength (IGRF approximate)
B0_earth = 3.12e-5;           % Dipole field at equator  [T]

% Solar flux at 1 AU
solar_flux = 1361;            % [W/m²]

%% ════════════════════════════════════════════════════════════════════════
%  PRINT SUMMARY
%  ════════════════════════════════════════════════════════════════════════

fprintf('\n──────────────────────────────────────────\n');
fprintf('  ADCS Simulator Parameters Initialized\n');
fprintf('──────────────────────────────────────────\n');
fprintf('  Spacecraft mass:      %.1f kg\n', mass_sc);
fprintf('  Inertia (diag):       [%.3f  %.3f  %.3f] kg·m²\n', J(1,1), J(2,2), J(3,3));
fprintf('  Orbit altitude:       %.0f km\n', orbit_alt/1e3);
fprintf('  Orbit period:         %.1f min\n', orbit_period/60);
fprintf('  Sim duration:         %.0f s (%.1f orbits)\n', t_end, t_end/orbit_period);
fprintf('  Time step:            %.2f s\n', dt);
fprintf('  Sensors:              6 FSS, 1 ST, 1 IMU, 1 MAG, 1 GNSS\n');
fprintf('  Actuators:            4 RW (pyramid), 3 MTQ, 1 CMG\n');
fprintf('──────────────────────────────────────────\n\n');
