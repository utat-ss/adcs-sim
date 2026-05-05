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

% Mass [kg]
mass_sc = 4.0;

% 3U CubeSat outer dimensions  [m]
sc_dim_x = 0.10;
sc_dim_y = 0.10;
sc_dim_z = 0.30;

% Inertia tensor  [kg·m²]
% Use the uniform-density cuboid baseline implied by the declared bus mass
% and outer dimensions so the default rigid-body dynamics remain
% self-consistent with the rest of the parameter set.
J = diag((mass_sc / 12) * [ ...
    sc_dim_y^2 + sc_dim_z^2, ...
    sc_dim_x^2 + sc_dim_z^2, ...
    sc_dim_x^2 + sc_dim_y^2]);

% Panel geometry used by environmental torque models
% Columns correspond to +X, -X, +Y, -Y, +Z, -Z faces.
sc_face_normals = [ 1, -1,  0,  0,  0,  0;
                    0,  0,  1, -1,  0,  0;
                    0,  0,  0,  0,  1, -1 ];
sc_face_areas = [sc_dim_y*sc_dim_z, sc_dim_y*sc_dim_z, ...
                 sc_dim_x*sc_dim_z, sc_dim_x*sc_dim_z, ...
                 sc_dim_x*sc_dim_y, sc_dim_x*sc_dim_y];
sc_face_cop = [ sc_dim_x/2, -sc_dim_x/2,  0,          0,          0,          0;
                0,           0,           sc_dim_y/2, -sc_dim_y/2, 0,          0;
                0,           0,           0,          0,          sc_dim_z/2, -sc_dim_z/2 ];

%% ════════════════════════════════════════════════════════════════════════
%  INITIAL CONDITIONS
%  ════════════════════════════════════════════════════════════════════════

% Attitude quaternion  [scalar-first: q0 qx qy qz]
q0 = [1; 0; 0; 0];

% Body angular velocity  [rad/s]
omega0 = [0.01; -0.005; 0.02];

% Reaction wheel momentum state  [N·m·s]
h_W0 = [0; 0; 0; 0];

%% ════════════════════════════════════════════════════════════════════════
%  SIMULATION
%  ════════════════════════════════════════════════════════════════════════

dt    = 0.01;      % Fixed integration step  [s]  (matches fastest sensor rate)
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
J2_earth = 1.08262668e-3;     % Earth second zonal harmonic  [-]
omega_earth = [0; 0; 7.2921150e-5];  % Earth rotation rate  [rad/s]

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
%  CONTROL / MODE MANAGEMENT
%  ════════════════════════════════════════════════════════════════════════

ctrl_rw_kp              = 0.01;          % RW proportional gain
ctrl_rw_kd              = 0.05;          % RW derivative gain
ctrl_bdot_gain          = 5e4;           % B-dot detumble gain
ctrl_dump_gain          = 0.02;          % MTQ momentum-dump gain
ctrl_unload_fraction    = 0.60;          % Start unloading above this fraction of max RW momentum
mode_detumble_rate      = 0.5*pi/180;    % Enter/hold detumble above this rate  [rad/s]
mode_fine_rate          = 0.1*pi/180;    % Fine-pointing max rate threshold  [rad/s]
mode_fine_err           = 10*pi/180;     % Fine-pointing max attitude error  [rad]

%% ════════════════════════════════════════════════════════════════════════
%  ENVIRONMENT / REFERENCE CONSTANTS
%  ════════════════════════════════════════════════════════════════════════

% Simulation epoch (Julian date)
epoch_jd = 2461083.5;         % 2026-02-12 00:00:00 UTC  (Julian date)

% Earth magnetic dipole strength (IGRF approximate)
B0_earth = 3.12e-5;           % Dipole field at equator  [T]

% Solar flux at 1 AU
solar_flux = 1361;            % [W/m²]
speed_of_light = 299792458;   % [m/s]
solar_pressure_1au = solar_flux / speed_of_light;   % [N/m²]

% Environmental disturbance torque parameters
atm_ref_alt      = 500e3;      % Reference altitude for exponential atmosphere  [m]
atm_ref_density  = 6.967e-13;  % Typical mean density near 500 km  [kg/m³]
atm_scale_height = 63.822e3;   % Exponential scale height near 500 km  [m]
drag_coeff       = 2.3;        % Aerodynamic drag coefficient  [-]
srp_coeff        = 1.3;        % Effective SRP reflectivity coefficient  [-]
residual_dipole  = [5e-3; 5e-3; 5e-3];   % Residual magnetic dipole  [A·m²]

% Navigation filter parameters
mekf_sigma_g   = imu_arw * pi/180;                  % Gyro white-noise proxy  [rad/sqrt(s)]
mekf_sigma_b   = imu_bias_instab * pi/180 / 3600;   % Gyro bias random-walk proxy  [rad/s]
mekf_sun_sigma = fss_noise_sigma * pi/180;          % Sun vector angular noise  [rad]
mekf_mag_sigma = max(mag_noise_sigma * 1e-9 / B0_earth, 1e-4);  % Normalized B-vector noise

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
