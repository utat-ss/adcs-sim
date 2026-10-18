function build_sensors(sys)
%BUILD_SENSORS  Build the SENSORS subsystem inside the ADCS model.
%
%   Blocks: Sun_Sensor_Suite, Star_Tracker, IMU, Magnetometer, GNSS
%   Inports: q_in, omega_in, Sun_ECI, Moon_ECI, B_ECI, eclipse_flag, pos_ECI, vel_ECI
%   Outports: q_meas, omega_meas, sun_meas, B_body_meas, pos_meas, vel_meas, st_valid

    ar = {'autorouting','on'};

    % ── Inports (8) ──
    %  1=q_in  2=omega_in  3=Sun_ECI  4=Moon_ECI
    %  5=B_ECI 6=eclipse_flag  7=pos_ECI  8=vel_ECI
    add_inports(sys, ...
        {'q_in','omega_in','Sun_ECI','Moon_ECI', ...
         'B_ECI','eclipse_flag','pos_ECI','vel_ECI'}, ...
        30, 50, 55);

    % ── Outports (7) ──
    %  1=q_meas  2=omega_meas  3=sun_meas
    %  4=B_body_meas  5=pos_meas  6=vel_meas  7=st_valid
    add_outports(sys, ...
        {'q_meas','omega_meas','sun_meas', ...
         'B_body_meas','pos_meas','vel_meas','st_valid'}, ...
        900, 70, 90);

    % ── Clock for sensor timing and sample-indexed noise logic ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [90  10  120  30]);

    % ── Sensor parameters from workspace ──
    add_block('simulink/Sources/Constant', [sys '/FSS_Noise'], ...
        'Position', [90  700  170  720], 'Value', 'fss_noise_sigma');
    add_block('simulink/Sources/Constant', [sys '/FSS_FOV'], ...
        'Position', [90  730  170  750], 'Value', 'fss_fov_half');
    add_block('simulink/Sources/Constant', [sys '/FSS_Rate'], ...
        'Position', [90  760  170  780], 'Value', 'fss_sample_rate');
    add_block('simulink/Sources/Constant', [sys '/FSS_Boresights'], ...
        'Position', [90  790  170  810], 'Value', 'fss_boresights');

    add_block('simulink/Sources/Constant', [sys '/ST_Noise_Bore'], ...
        'Position', [90  830  170  850], 'Value', 'st_noise_bore');
    add_block('simulink/Sources/Constant', [sys '/ST_Noise_Roll'], ...
        'Position', [90  860  170  880], 'Value', 'st_noise_roll');
    add_block('simulink/Sources/Constant', [sys '/ST_Rate'], ...
        'Position', [90  890  170  910], 'Value', 'st_sample_rate');
    add_block('simulink/Sources/Constant', [sys '/ST_Excl_Sun'], ...
        'Position', [90  920  170  940], 'Value', 'st_exclusion_sun');
    add_block('simulink/Sources/Constant', [sys '/ST_Excl_Moon'], ...
        'Position', [90  950  170  970], 'Value', 'st_exclusion_moon');
    add_block('simulink/Sources/Constant', [sys '/ST_Excl_Earth'], ...
        'Position', [90  980  170 1000], 'Value', 'st_exclusion_earth');
    add_block('simulink/Sources/Constant', [sys '/ST_Boresight'], ...
        'Position', [90 1010  170 1030], 'Value', 'st_boresight');
    add_block('simulink/Sources/Constant', [sys '/R_Earth'], ...
        'Position', [90 1040  170 1060], 'Value', 'R_earth');

    add_block('simulink/Sources/Constant', [sys '/IMU_ARW'], ...
        'Position', [90 1080  170 1100], 'Value', 'imu_arw');
    add_block('simulink/Sources/Constant', [sys '/IMU_Bias_Instab'], ...
        'Position', [90 1110  170 1130], 'Value', 'imu_bias_instab');
    add_block('simulink/Sources/Constant', [sys '/IMU_Rate'], ...
        'Position', [90 1140  170 1160], 'Value', 'imu_sample_rate');
    add_block('simulink/Sources/Constant', [sys '/IMU_Range'], ...
        'Position', [90 1170  170 1190], 'Value', 'imu_range');

    add_block('simulink/Sources/Constant', [sys '/Mag_Noise'], ...
        'Position', [90 1210  170 1230], 'Value', 'mag_noise_sigma');
    add_block('simulink/Sources/Constant', [sys '/Mag_Bias'], ...
        'Position', [90 1240  170 1260], 'Value', 'mag_bias');
    add_block('simulink/Sources/Constant', [sys '/Mag_Rate'], ...
        'Position', [90 1270  170 1290], 'Value', 'mag_sample_rate');

    add_block('simulink/Sources/Constant', [sys '/GNSS_Pos_Sigma'], ...
        'Position', [90 1310  170 1330], 'Value', 'gnss_pos_sigma');
    add_block('simulink/Sources/Constant', [sys '/GNSS_Vel_Sigma'], ...
        'Position', [90 1340  170 1360], 'Value', 'gnss_vel_sigma');
    add_block('simulink/Sources/Constant', [sys '/GNSS_Rate'], ...
        'Position', [90 1370  170 1390], 'Value', 'gnss_sample_rate');

    % ── Sun Sensor Suite (single MATLAB Function block for all 6 FSS) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Sun_Sensor_Suite'], 'Position', [250  20  480  110]);
    set_mfb_script([sys '/Sun_Sensor_Suite'], sun_sensor_suite_code());
    set_mfb_data_sizes([sys '/Sun_Sensor_Suite'], ...
        {'t','1'; 'sun_eci','[3 1]'; 'q','[4 1]'; 'eclipse_flag','1'; ...
         'noise_sigma_deg','1'; 'fov_half_deg','1'; 'sample_rate','1'; ...
         'boresights','[3 6]'; 'sun_body','[3 1]'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/Sun_Sensor_ZOH'], ...
        'Position', [540  45  590  85], 'SampleTime', '1/fss_sample_rate');

    % ── Star Tracker ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Star_Tracker'], 'Position', [250  150  480  250]);
    set_mfb_script([sys '/Star_Tracker'], star_tracker_code());
    set_mfb_data_sizes([sys '/Star_Tracker'], ...
        {'t','1'; 'sun_eci','[3 1]'; 'moon_eci','[3 1]'; 'pos_eci','[3 1]'; ...
         'q_true','[4 1]'; 'noise_bore_arcsec','1'; 'noise_roll_arcsec','1'; ...
         'sample_rate','1'; 'excl_sun_deg','1'; 'excl_moon_deg','1'; ...
         'excl_earth_deg','1'; 'boresight_body','[3 1]'; 'Re','1'; ...
         'q_candidate','[4 1]'; 'valid','1'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/ST_Candidate_ZOH'], ...
        'Position', [540  150  590  190], 'SampleTime', '1/st_sample_rate');
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/ST_Valid_ZOH'], ...
        'Position', [540  210  590  240], 'SampleTime', '1/st_sample_rate');
    add_block('simulink/Signal Routing/Switch', [sys '/ST_Validity_Switch'], ...
        'Position', [650  165  700  225], 'Threshold', '0.5', ...
        'Criteria', 'u2 >= Threshold');
    add_block('simulink/Discrete/Memory', [sys '/ST_Last_Valid'], ...
        'Position', [760  255  820  285], 'InitialCondition', 'q0');

    % ── IMU ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/IMU'], 'Position', [250  290  480  360]);
    set_mfb_script([sys '/IMU'], imu_code());
    set_mfb_data_sizes([sys '/IMU'], ...
        {'t','1'; 'omega_true','[3 1]'; 'arw_deg_sqrt_s','1'; ...
         'bias_instab_deg_hr','1'; 'sample_rate','1'; 'range_deg_s','1'; ...
         'omega_meas','[3 1]'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/IMU_ZOH'], ...
        'Position', [540  305  590  345], 'SampleTime', '1/imu_sample_rate');

    % ── Magnetometer ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Magnetometer'], 'Position', [250  390  480  470]);
    set_mfb_script([sys '/Magnetometer'], magnetometer_code());
    set_mfb_data_sizes([sys '/Magnetometer'], ...
        {'t','1'; 'B_eci','[3 1]'; 'q','[4 1]'; 'noise_sigma_nT','1'; ...
         'bias_nT','[3 1]'; 'sample_rate','1'; 'B_body_meas','[3 1]'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/Magnetometer_ZOH'], ...
        'Position', [540  410  590  450], 'SampleTime', '1/mag_sample_rate');

    % ── GNSS (split position/velocity blocks to avoid multi-output MFB inference issues) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/GNSS_Position'], 'Position', [250  500  480  560]);
    set_mfb_script([sys '/GNSS_Position'], gnss_position_code());
    set_mfb_data_sizes([sys '/GNSS_Position'], ...
        {'t','1'; 'pos_eci','[3 1]'; 'pos_sigma','1'; 'sample_rate','1'; ...
         'pos_meas','[3 1]'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/GNSS_Position_ZOH'], ...
        'Position', [540  510  590  550], 'SampleTime', '1/gnss_sample_rate');
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/GNSS_Velocity'], 'Position', [250  585  480  645]);
    set_mfb_script([sys '/GNSS_Velocity'], gnss_velocity_code());
    set_mfb_data_sizes([sys '/GNSS_Velocity'], ...
        {'t','1'; 'vel_eci','[3 1]'; 'vel_sigma','1'; 'sample_rate','1'; ...
         'vel_meas','[3 1]'});
    add_block('simulink/Discrete/Zero-Order Hold', [sys '/GNSS_Velocity_ZOH'], ...
        'Position', [540  595  590  635], 'SampleTime', '1/gnss_sample_rate');

    % ══════════════════════════════════════════════════════════════════════
    %  WIRING
    % ══════════════════════════════════════════════════════════════════════

    % t, Sun_ECI, q_in, eclipse_flag → Sun_Sensor_Suite
    add_line(sys, 'Clock/1',        'Sun_Sensor_Suite/1', ar{:});
    add_line(sys, 'Sun_ECI/1',      'Sun_Sensor_Suite/2', ar{:});
    add_line(sys, 'q_in/1',         'Sun_Sensor_Suite/3', ar{:});
    add_line(sys, 'eclipse_flag/1', 'Sun_Sensor_Suite/4', ar{:});
    add_line(sys, 'FSS_Noise/1',    'Sun_Sensor_Suite/5', ar{:});
    add_line(sys, 'FSS_FOV/1',      'Sun_Sensor_Suite/6', ar{:});
    add_line(sys, 'FSS_Rate/1',     'Sun_Sensor_Suite/7', ar{:});
    add_line(sys, 'FSS_Boresights/1','Sun_Sensor_Suite/8', ar{:});

    % t, Sun_ECI, Moon_ECI, pos_ECI, q_in → Star_Tracker
    add_line(sys, 'Clock/1',    'Star_Tracker/1', ar{:});
    add_line(sys, 'Sun_ECI/1',  'Star_Tracker/2', ar{:});
    add_line(sys, 'Moon_ECI/1', 'Star_Tracker/3', ar{:});
    add_line(sys, 'pos_ECI/1',  'Star_Tracker/4', ar{:});
    add_line(sys, 'q_in/1',     'Star_Tracker/5', ar{:});
    add_line(sys, 'ST_Noise_Bore/1', 'Star_Tracker/6', ar{:});
    add_line(sys, 'ST_Noise_Roll/1', 'Star_Tracker/7', ar{:});
    add_line(sys, 'ST_Rate/1',       'Star_Tracker/8', ar{:});
    add_line(sys, 'ST_Excl_Sun/1',   'Star_Tracker/9', ar{:});
    add_line(sys, 'ST_Excl_Moon/1',  'Star_Tracker/10', ar{:});
    add_line(sys, 'ST_Excl_Earth/1', 'Star_Tracker/11', ar{:});
    add_line(sys, 'ST_Boresight/1',  'Star_Tracker/12', ar{:});
    add_line(sys, 'R_Earth/1',       'Star_Tracker/13', ar{:});

    % t, omega_in → IMU
    add_line(sys, 'Clock/1',    'IMU/1', ar{:});
    add_line(sys, 'omega_in/1', 'IMU/2', ar{:});
    add_line(sys, 'IMU_ARW/1',          'IMU/3', ar{:});
    add_line(sys, 'IMU_Bias_Instab/1',  'IMU/4', ar{:});
    add_line(sys, 'IMU_Rate/1',         'IMU/5', ar{:});
    add_line(sys, 'IMU_Range/1',        'IMU/6', ar{:});

    % t, B_ECI, q_in → Magnetometer
    add_line(sys, 'Clock/1', 'Magnetometer/1', ar{:});
    add_line(sys, 'B_ECI/1', 'Magnetometer/2', ar{:});
    add_line(sys, 'q_in/1',  'Magnetometer/3', ar{:});
    add_line(sys, 'Mag_Noise/1', 'Magnetometer/4', ar{:});
    add_line(sys, 'Mag_Bias/1',  'Magnetometer/5', ar{:});
    add_line(sys, 'Mag_Rate/1',  'Magnetometer/6', ar{:});

    % t, pos_ECI → GNSS position
    add_line(sys, 'Clock/1',   'GNSS_Position/1', ar{:});
    add_line(sys, 'pos_ECI/1', 'GNSS_Position/2', ar{:});
    add_line(sys, 'GNSS_Pos_Sigma/1', 'GNSS_Position/3', ar{:});
    add_line(sys, 'GNSS_Rate/1',      'GNSS_Position/4', ar{:});

    % t, vel_ECI → GNSS velocity
    add_line(sys, 'Clock/1',   'GNSS_Velocity/1', ar{:});
    add_line(sys, 'vel_ECI/1', 'GNSS_Velocity/2', ar{:});
    add_line(sys, 'GNSS_Vel_Sigma/1', 'GNSS_Velocity/3', ar{:});
    add_line(sys, 'GNSS_Rate/1',      'GNSS_Velocity/4', ar{:});

    % Output sampling and held star-tracker validity.
    add_line(sys, 'Sun_Sensor_Suite/1', 'Sun_Sensor_ZOH/1', ar{:});
    add_line(sys, 'Star_Tracker/1',     'ST_Candidate_ZOH/1', ar{:});
    add_line(sys, 'Star_Tracker/2',     'ST_Valid_ZOH/1', ar{:});
    add_line(sys, 'ST_Candidate_ZOH/1', 'ST_Validity_Switch/1', ar{:});
    add_line(sys, 'ST_Valid_ZOH/1',     'ST_Validity_Switch/2', ar{:});
    add_line(sys, 'ST_Last_Valid/1',    'ST_Validity_Switch/3', ar{:});
    add_line(sys, 'ST_Validity_Switch/1', 'ST_Last_Valid/1', ar{:});
    add_line(sys, 'IMU/1',              'IMU_ZOH/1', ar{:});
    add_line(sys, 'Magnetometer/1',     'Magnetometer_ZOH/1', ar{:});
    add_line(sys, 'GNSS_Position/1',    'GNSS_Position_ZOH/1', ar{:});
    add_line(sys, 'GNSS_Velocity/1',    'GNSS_Velocity_ZOH/1', ar{:});

    % Outputs → outports
    add_line(sys, 'ST_Validity_Switch/1', 'q_meas/1',      ar{:});
    add_line(sys, 'IMU_ZOH/1',          'omega_meas/1',  ar{:});
    add_line(sys, 'Sun_Sensor_ZOH/1',   'sun_meas/1',    ar{:});
    add_line(sys, 'Magnetometer_ZOH/1', 'B_body_meas/1', ar{:});
    add_line(sys, 'GNSS_Position_ZOH/1','pos_meas/1',    ar{:});
    add_line(sys, 'GNSS_Velocity_ZOH/1','vel_meas/1',    ar{:});
    add_line(sys, 'ST_Valid_ZOH/1',     'st_valid/1',    ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Sun Sensor Suite (6 FSS combined)
% ─────────────────────────────────────────────────────────────────────────
function s = sun_sensor_suite_code()
    L = {
    'function sun_body = fcn(t, sun_eci, q, eclipse_flag, noise_sigma_deg, fov_half_deg, sample_rate, boresights)'
    '%#codegen'
    '% Six-face FSS vector measurement with eclipse/FOV gating.'
    '  if eclipse_flag > 0.5'
    '      sun_body = [0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  sun_mag = sqrt(sum(sun_eci.^2));'
    '  if sun_mag < 1e-9'
    '      sun_body = [0; 0; 0];'
    '      return;'
    '  end'
    '  s_hat = sun_eci / sun_mag;'
    ''
    '  R = q_to_dcm_bi(q);'
    '  true_body = R * s_hat;'
    '  fov_cos = cos(fov_half_deg * pi/180.0);'
    '  valid_count = 0.0;'
    '  best_dot = -2.0;'
    '  for k = 1:6'
    '      bnorm = sqrt(boresights(1,k)^2 + boresights(2,k)^2 + boresights(3,k)^2);'
    '      if bnorm > 1e-12'
    '          bx = boresights(1,k) / bnorm;'
    '          by = boresights(2,k) / bnorm;'
    '          bz = boresights(3,k) / bnorm;'
    '      else'
    '          bx = 0; by = 0; bz = 0;'
    '      end'
    '      d = true_body(1)*bx + true_body(2)*by + true_body(3)*bz;'
    '      if d > best_dot'
    '          best_dot = d;'
    '      end'
    '      if d >= fov_cos'
    '          valid_count = valid_count + 1.0;'
    '      end'
    '  end'
    '  if valid_count < 0.5 || best_dot < fov_cos'
    '      sun_body = [0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  sigma = noise_sigma_deg * pi/180.0;'
    '  nvec = noise3(t, sample_rate, 101.0);'
    '  meas = true_body + sigma * nvec;'
    '  mn = sqrt(meas(1)^2 + meas(2)^2 + meas(3)^2);'
    '  if mn > 1e-12'
    '      sun_body = meas / mn;'
    '  else'
    '      sun_body = [0; 0; 0];'
    '  end'
    'end'
    ''
    'function C = q_to_dcm_bi(q)'
    '  qn = max(sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2), 1e-12);'
    '  q0=q(1)/qn; q1=q(2)/qn; q2=q(3)/qn; q3=q(4)/qn;'
    '  C = [1-2*(q2*q2+q3*q3),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1*q1+q3*q3),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1*q1+q2*q2)];'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Star Tracker
% ─────────────────────────────────────────────────────────────────────────
function s = star_tracker_code()
    L = {
    'function [q_candidate, valid] = fcn(t, sun_eci, moon_eci, pos_eci, q_true, noise_bore_arcsec, noise_roll_arcsec, sample_rate, excl_sun_deg, excl_moon_deg, excl_earth_deg, boresight_body, Re)'
    '%#codegen'
    '% Quaternion attitude measurement with Sun, Moon, and Earth exclusion zones.'
    '  qn = sqrt(q_true(1)^2 + q_true(2)^2 + q_true(3)^2 + q_true(4)^2);'
    '  if qn < 1e-12'
    '      q = [1; 0; 0; 0];'
    '  else'
    '      q = q_true / qn;'
    '  end'
    ''
    '  C_bi = q_to_dcm_bi(q);'
    '  bore_b = boresight_body / max(sqrt(boresight_body(1)^2 + boresight_body(2)^2 + boresight_body(3)^2), 1e-12);'
    '  bore_eci = C_bi'' * bore_b;'
    '  valid = 1.0;'
    ''
    '  if vector_in_cone(bore_eci, sun_eci, excl_sun_deg * pi/180.0)'
    '      valid = 0.0;'
    '  end'
    '  if vector_in_cone(bore_eci, moon_eci, excl_moon_deg * pi/180.0)'
    '      valid = 0.0;'
    '  end'
    ''
    '  rnorm = sqrt(pos_eci(1)^2 + pos_eci(2)^2 + pos_eci(3)^2);'
    '  if rnorm > Re + 1.0'
    '      earth_center = -pos_eci / rnorm;'
    '      earth_radius = asin(min(max(Re / rnorm, 0.0), 1.0));'
    '      earth_excl = min(earth_radius + excl_earth_deg*pi/180.0, pi);'
    '      if vector_in_cone(bore_eci, earth_center, earth_excl)'
    '          valid = 0.0;'
    '      end'
    '  end'
    ''
    '  % Boresight error tilts the optical axis; roll rotates about that axis.'
    '  sigma_bore = noise_bore_arcsec * (pi/180.0) / 3600.0;'
    '  sigma_roll = noise_roll_arcsec * (pi/180.0) / 3600.0;'
    '  helper = [1; 0; 0];'
    '  if abs(bore_b(1)) > 0.9'
    '      helper = [0; 1; 0];'
    '  end'
    '  transverse1 = cross(bore_b, helper);'
    '  transverse1 = transverse1 / sqrt(sum(transverse1.^2));'
    '  transverse2 = cross(bore_b, transverse1);'
    '  eta = noise3(t, sample_rate, 201.0);'
    '  dtheta = sigma_bore*(eta(1)*transverse1 + eta(2)*transverse2) + sigma_roll*eta(3)*bore_b;'
    '  dq = small_angle_quat(dtheta);'
    '  q_candidate = quat_multiply(q, dq);'
    '  q_candidate = normalize_quat(q_candidate);'
    '  if q_candidate(1) < 0'
    '      q_candidate = -q_candidate;'
    '  end'
    'end'
    ''
    'function tf = vector_in_cone(a, b, half_angle)'
    '  an = sqrt(a(1)^2 + a(2)^2 + a(3)^2);'
    '  bn = sqrt(b(1)^2 + b(2)^2 + b(3)^2);'
    '  if an < 1e-12 || bn < 1e-12'
    '      tf = false;'
    '      return;'
    '  end'
    '  dotp = (a(1)*b(1) + a(2)*b(2) + a(3)*b(3)) / (an*bn);'
    '  tf = dotp >= cos(half_angle);'
    'end'
    ''
    'function C = q_to_dcm_bi(q)'
    '  q0=q(1); q1=q(2); q2=q(3); q3=q(4);'
    '  C = [1-2*(q2*q2+q3*q3),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1*q1+q3*q3),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1*q1+q2*q2)];'
    'end'
    ''
    'function q = small_angle_quat(dtheta)'
    '  angle = sqrt(dtheta(1)^2 + dtheta(2)^2 + dtheta(3)^2);'
    '  if angle < 1e-12'
    '      q = [1; 0.5*dtheta(1); 0.5*dtheta(2); 0.5*dtheta(3)];'
    '  else'
    '      axis = dtheta / angle;'
    '      q = [cos(0.5*angle); axis(1)*sin(0.5*angle); axis(2)*sin(0.5*angle); axis(3)*sin(0.5*angle)];'
    '  end'
    'end'
    ''
    'function c = quat_multiply(a, b)'
    '  c = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);'
    '       a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);'
    '       a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);'
    '       a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];'
    'end'
    ''
    'function qn = normalize_quat(q)'
    '  n = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);'
    '  if n > 1e-12'
    '      qn = q / n;'
    '  else'
    '      qn = [1; 0; 0; 0];'
    '  end'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  IMU (MEMS gyroscope)
% ─────────────────────────────────────────────────────────────────────────
function s = imu_code()
    L = {
    'function omega_meas = fcn(t, omega_true, arw_deg_sqrt_s, bias_instab_deg_hr, sample_rate, range_deg_s)'
    '%#codegen'
    '% MEMS gyro with sample-indexed white noise and drifting bias.'
    '  dt_s = 1.0 / sample_rate;'
    '  arw = arw_deg_sqrt_s * pi/180.0;'
    '  white_sigma = arw / sqrt(dt_s);'
    '  eta = noise3(t, sample_rate, 301.0);'
    '  bias_sigma = bias_instab_deg_hr * pi/180.0 / 3600.0;'
    '  beta = noise3(t, 0.1, 331.0);'
    '  bias = bias_sigma * sqrt(max(t, dt_s)) * beta;'
    '  omega_meas = omega_true + bias + white_sigma * eta;'
    '  limit = range_deg_s * pi/180.0;'
    '  for k = 1:3'
    '      if omega_meas(k) > limit'
    '          omega_meas(k) = limit;'
    '      elseif omega_meas(k) < -limit'
    '          omega_meas(k) = -limit;'
    '      end'
    '  end'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetometer (3-axis)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetometer_code()
    L = {
    'function B_body_meas = fcn(t, B_eci, q, noise_sigma_nT, bias_nT, sample_rate)'
    '%#codegen'
    '% Body-frame magnetic field with hard-iron bias and white noise.'
    '  B_body_meas = zeros(3,1);'
    '  qn = max(sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2), 1e-12);'
    '  q0 = q(1) / qn;'
    '  q1 = q(2) / qn;'
    '  q2 = q(3) / qn;'
    '  q3 = q(4) / qn;'
    ''
    '  c11 = 1 - 2*(q2*q2 + q3*q3);'
    '  c12 = 2*(q1*q2 + q0*q3);'
    '  c13 = 2*(q1*q3 - q0*q2);'
    '  c21 = 2*(q1*q2 - q0*q3);'
    '  c22 = 1 - 2*(q1*q1 + q3*q3);'
    '  c23 = 2*(q2*q3 + q0*q1);'
    '  c31 = 2*(q1*q3 + q0*q2);'
    '  c32 = 2*(q2*q3 - q0*q1);'
    '  c33 = 1 - 2*(q1*q1 + q2*q2);'
    ''
    '  bx = c11*B_eci(1) + c12*B_eci(2) + c13*B_eci(3);'
    '  by = c21*B_eci(1) + c22*B_eci(2) + c23*B_eci(3);'
    '  bz = c31*B_eci(1) + c32*B_eci(2) + c33*B_eci(3);'
    ''
    '  noise_t = (noise_sigma_nT * 1e-9) * noise3(t, sample_rate, 401.0);'
    '  bias_t = bias_nT * 1e-9;'
    '  B_body_meas(1) = bx + bias_t(1) + noise_t(1);'
    '  B_body_meas(2) = by + bias_t(2) + noise_t(2);'
    '  B_body_meas(3) = bz + bias_t(3) + noise_t(3);'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  GNSS Position Receiver
% ─────────────────────────────────────────────────────────────────────────
function s = gnss_position_code()
    L = {
    'function pos_meas = fcn(t, pos_eci, pos_sigma, sample_rate)'
    '%#codegen'
    '% GNSS ECI position measurement with white noise.'
    '  pos_meas = pos_eci + pos_sigma * noise3(t, sample_rate, 501.0);'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  GNSS Velocity Receiver
% ─────────────────────────────────────────────────────────────────────────
function s = gnss_velocity_code()
    L = {
    'function vel_meas = fcn(t, vel_eci, vel_sigma, sample_rate)'
    '%#codegen'
    '% GNSS ECI velocity measurement with white noise.'
    '  vel_meas = vel_eci + vel_sigma * noise3(t, sample_rate, 601.0);'
    'end'
    ''
    'function y = noise3(t, rate, seed)'
    '  k = floor(t * rate + 1e-9);'
    '  y = [randn_hash(k, 1.0, seed); randn_hash(k, 2.0, seed); randn_hash(k, 3.0, seed)];'
    'end'
    ''
    'function z = randn_hash(k, chan, seed)'
    '  u1 = hash01(k + 17.0*chan + seed);'
    '  u2 = hash01(k + 37.0*chan + seed + 9.0);'
    '  u1 = max(u1, 1e-12);'
    '  z = sqrt(-2.0*log(u1)) * cos(2.0*pi*u2);'
    'end'
    ''
    'function u = hash01(x)'
    '  u = sin(12.9898*x + 78.233) * 43758.5453;'
    '  u = u - floor(u);'
    'end'
    };
    s = strjoin(L, newline);
end
