function build_sensors(sys)
%BUILD_SENSORS  Build the SENSORS subsystem inside the ADCS model.
%
%   Blocks: Sun_Sensor_Suite, Star_Tracker, IMU, Magnetometer, GNSS
%   Inports: q_in, omega_in, Sun_ECI, Moon_ECI, B_ECI, eclipse_flag, pos_ECI, vel_ECI
%   Outports: q_meas, omega_meas, sun_meas, B_body_meas, pos_meas, vel_meas

    ar = {'autorouting','on'};

    % ── Inports (8) ──
    %  1=q_in  2=omega_in  3=Sun_ECI  4=Moon_ECI
    %  5=B_ECI 6=eclipse_flag  7=pos_ECI  8=vel_ECI
    add_inports(sys, ...
        {'q_in','omega_in','Sun_ECI','Moon_ECI', ...
         'B_ECI','eclipse_flag','pos_ECI','vel_ECI'}, ...
        30, 50, 55);

    % ── Outports (6) ──
    %  1=q_meas  2=omega_meas  3=sun_meas
    %  4=B_body_meas  5=pos_meas  6=vel_meas
    add_outports(sys, ...
        {'q_meas','omega_meas','sun_meas', ...
         'B_body_meas','pos_meas','vel_meas'}, ...
        900, 70, 90);

    % ── Clock for sensor sample-and-hold logic ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [90  10  120  30]);

    % ── Sun Sensor Suite (single MATLAB Function block for all 6 FSS) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Sun_Sensor_Suite'], 'Position', [250  20  480  110]);
    set_mfb_script([sys '/Sun_Sensor_Suite'], sun_sensor_suite_code());
    set_mfb_data_sizes([sys '/Sun_Sensor_Suite'], ...
        {'t','1'; 'sun_eci','[3 1]'; 'q','[4 1]'; 'eclipse_flag','1'; 'sun_body','[3 1]'});

    % ── Star Tracker ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Star_Tracker'], 'Position', [250  150  480  250]);
    set_mfb_script([sys '/Star_Tracker'], star_tracker_code());
    set_mfb_data_sizes([sys '/Star_Tracker'], ...
        {'t','1'; 'sun_eci','[3 1]'; 'moon_eci','[3 1]'; 'pos_eci','[3 1]'; ...
         'q_true','[4 1]'; 'q_meas','[4 1]'});

    % ── IMU ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/IMU'], 'Position', [250  290  480  360]);
    set_mfb_script([sys '/IMU'], imu_code());
    set_mfb_data_sizes([sys '/IMU'], ...
        {'t','1'; 'omega_true','[3 1]'; 'omega_meas','[3 1]'});

    % ── Magnetometer ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Magnetometer'], 'Position', [250  390  480  470]);
    set_mfb_script([sys '/Magnetometer'], magnetometer_code());
    set_mfb_data_sizes([sys '/Magnetometer'], ...
        {'t','1'; 'B_eci','[3 1]'; 'q','[4 1]'; 'B_body_meas','[3 1]'});

    % ── GNSS (split position/velocity blocks to avoid multi-output MFB inference issues) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/GNSS_Position'], 'Position', [250  500  480  560]);
    set_mfb_script([sys '/GNSS_Position'], gnss_position_code());
    set_mfb_data_sizes([sys '/GNSS_Position'], ...
        {'t','1'; 'pos_eci','[3 1]'; 'pos_meas','[3 1]'});
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/GNSS_Velocity'], 'Position', [250  585  480  645]);
    set_mfb_script([sys '/GNSS_Velocity'], gnss_velocity_code());
    set_mfb_data_sizes([sys '/GNSS_Velocity'], ...
        {'t','1'; 'vel_eci','[3 1]'; 'vel_meas','[3 1]'});

    % ══════════════════════════════════════════════════════════════════════
    %  WIRING
    % ══════════════════════════════════════════════════════════════════════

    % t, Sun_ECI, q_in, eclipse_flag → Sun_Sensor_Suite
    add_line(sys, 'Clock/1',        'Sun_Sensor_Suite/1', ar{:});
    add_line(sys, 'Sun_ECI/1',      'Sun_Sensor_Suite/2', ar{:});
    add_line(sys, 'q_in/1',         'Sun_Sensor_Suite/3', ar{:});
    add_line(sys, 'eclipse_flag/1', 'Sun_Sensor_Suite/4', ar{:});

    % t, Sun_ECI, Moon_ECI, pos_ECI, q_in → Star_Tracker
    add_line(sys, 'Clock/1',    'Star_Tracker/1', ar{:});
    add_line(sys, 'Sun_ECI/1',  'Star_Tracker/2', ar{:});
    add_line(sys, 'Moon_ECI/1', 'Star_Tracker/3', ar{:});
    add_line(sys, 'pos_ECI/1',  'Star_Tracker/4', ar{:});
    add_line(sys, 'q_in/1',     'Star_Tracker/5', ar{:});

    % t, omega_in → IMU
    add_line(sys, 'Clock/1',    'IMU/1', ar{:});
    add_line(sys, 'omega_in/1', 'IMU/2', ar{:});

    % t, B_ECI, q_in → Magnetometer
    add_line(sys, 'Clock/1', 'Magnetometer/1', ar{:});
    add_line(sys, 'B_ECI/1', 'Magnetometer/2', ar{:});
    add_line(sys, 'q_in/1',  'Magnetometer/3', ar{:});

    % t, pos_ECI → GNSS position
    add_line(sys, 'Clock/1',   'GNSS_Position/1', ar{:});
    add_line(sys, 'pos_ECI/1', 'GNSS_Position/2', ar{:});

    % t, vel_ECI → GNSS velocity
    add_line(sys, 'Clock/1',   'GNSS_Velocity/1', ar{:});
    add_line(sys, 'vel_ECI/1', 'GNSS_Velocity/2', ar{:});

    % Outputs → outports
    add_line(sys, 'Star_Tracker/1',     'q_meas/1',      ar{:});
    add_line(sys, 'IMU/1',              'omega_meas/1',  ar{:});
    add_line(sys, 'Sun_Sensor_Suite/1', 'sun_meas/1',    ar{:});
    add_line(sys, 'Magnetometer/1',     'B_body_meas/1', ar{:});
    add_line(sys, 'GNSS_Position/1',    'pos_meas/1',    ar{:});
    add_line(sys, 'GNSS_Velocity/1',    'vel_meas/1',    ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Sun Sensor Suite (6 FSS combined)
% ─────────────────────────────────────────────────────────────────────────
function s = sun_sensor_suite_code()
    L = {
    'function sun_body = fcn(t, sun_eci, q, eclipse_flag)'
    '%#codegen'
    '% Deterministic sun-vector body rotation with eclipse gating.'
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
    '  q0=q(1); q1=q(2); q2=q(3); q3=q(4);'
    '  R = [1-2*(q2^2+q3^2),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1^2+q2^2)];'
    '  sun_body = R * s_hat;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Star Tracker
% ─────────────────────────────────────────────────────────────────────────
function s = star_tracker_code()
    L = {
    'function q_meas = fcn(t, sun_eci, moon_eci, pos_eci, q_true)'
    '%#codegen'
    '% Deterministic passthrough used during compile hardening.'
    '  q_meas = q_true;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  IMU (MEMS gyroscope)
% ─────────────────────────────────────────────────────────────────────────
function s = imu_code()
    L = {
    'function omega_meas = fcn(t, omega_true)'
    '%#codegen'
    '% Temporary passthrough used to isolate MATLAB Function analysis issues.'
    '  omega_meas = omega_true;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetometer (3-axis)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetometer_code()
    L = {
    'function B_body_meas = fcn(t, B_eci, q)'
    '%#codegen'
    '% Deterministic body-frame field rotation.'
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
    '  B_body_meas(1) = bx;'
    '  B_body_meas(2) = by;'
    '  B_body_meas(3) = bz;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  GNSS Position Receiver
% ─────────────────────────────────────────────────────────────────────────
function s = gnss_position_code()
    L = {
    'function pos_meas = fcn(t, pos_eci)'
    '%#codegen'
    '% Temporary passthrough used to isolate MATLAB Function analysis issues.'
    '  pos_meas = pos_eci;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  GNSS Velocity Receiver
% ─────────────────────────────────────────────────────────────────────────
function s = gnss_velocity_code()
    L = {
    'function vel_meas = fcn(t, vel_eci)'
    '%#codegen'
    '% Temporary passthrough used to isolate MATLAB Function analysis issues.'
    '  vel_meas = vel_eci;'
    'end'
    };
    s = strjoin(L, newline);
end
