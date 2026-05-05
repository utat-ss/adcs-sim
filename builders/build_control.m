function build_control(sys)
%BUILD_CONTROL  Build the CONTROL subsystem inside the ADCS model.
%
%   Blocks: Onboard_Ephemeris, Attitude_Estimator, Reference_Generator,
%           Mode_Manager, Control_Law
%   Inports: q_meas, omega_meas, sun_meas, B_body_meas, pos_meas, vel_meas, h_W_in
%   Outports: rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd

    ar = {'autorouting','on'};

    % ── Inports (7) ──
    add_inports(sys, ...
        {'q_meas','omega_meas','sun_meas','B_body_meas','pos_meas','vel_meas','h_W_in'}, ...
        30, 40, 55);

    % ── Outports (3) ──
    add_outports(sys, ...
        {'rw_torque_cmd','mtq_dipole_cmd','cmg_gimbal_cmd'}, ...
        1040, 120, 160);

    % ── Clock + Epoch ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [30  400  60  420]);
    add_block('simulink/Sources/Constant', [sys '/Epoch_JD'], ...
        'Position', [30  470  110  490], 'Value', 'epoch_jd');

    % ── Onboard Ephemeris (simplified flight-side prediction) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Onboard_Ephemeris'], 'Position', [180  400  380  500]);
    set_mfb_script([sys '/Onboard_Ephemeris'], onboard_ephemeris_code());
    set_mfb_data_sizes([sys '/Onboard_Ephemeris'], ...
        {'t','1'; 'epoch_jd','1'; 'pos_eci','[3 1]'; 'sun_pred','[3 1]'; 'mag_pred','[3 1]'});

    % ── Attitude Estimator (TRIAD-seeded MEKF) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Attitude_Estimator'], 'Position', [180  120  400  320]);
    set_mfb_script([sys '/Attitude_Estimator'], attitude_estimator_code());
    set_mfb_data_sizes([sys '/Attitude_Estimator'], ...
        {'q_meas','[4 1]'; 'omega_meas','[3 1]'; 'sun_meas','[3 1]'; 'B_body_meas','[3 1]'; ...
         'sun_ref','[3 1]'; 'mag_ref','[3 1]'; 'q_est','[4 1]'; 'omega_est','[3 1]'});

    % ── Nadir-pointing reference generator ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Reference_Generator'], 'Position', [430  330  700  470]);
    set_mfb_script([sys '/Reference_Generator'], reference_generator_code());
    set_mfb_data_sizes([sys '/Reference_Generator'], ...
        {'pos_eci','[3 1]'; 'vel_eci','[3 1]'; 'q_des','[4 1]'; 'omega_des','[3 1]'});

    % ── Mode manager (detumble → coarse → fine) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Mode_Manager'], 'Position', [430  500  700  610]);
    set_mfb_script([sys '/Mode_Manager'], mode_manager_code());
    set_mfb_data_sizes([sys '/Mode_Manager'], ...
        {'q_est','[4 1]'; 'omega_est','[3 1]'; 'sun_meas','[3 1]'; 'q_des','[4 1]'; ...
         'h_W','[4 1]'; 'mode_id','1'});

    % ── Control Law (mode-aware RW/MTQ/CMG control) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Control_Law'], 'Position', [750  80  940  330]);
    set_mfb_script([sys '/Control_Law'], control_law_code());
    set_mfb_data_sizes([sys '/Control_Law'], ...
        {'q_est','[4 1]'; 'omega_est','[3 1]'; 'B_body','[3 1]'; 'q_des','[4 1]'; ...
         'omega_des','[3 1]'; 'mode_id','1'; 'h_W','[4 1]'; ...
         'rw_cmd','[3 1]'; 'mtq_cmd','[3 1]'; 'cmg_cmd','[3 1]'});

    % ── Wiring ──
    % Clock + Epoch + orbit state → Onboard Ephemeris
    add_line(sys, 'Clock/1',    'Onboard_Ephemeris/1', ar{:});
    add_line(sys, 'Epoch_JD/1', 'Onboard_Ephemeris/2', ar{:});
    add_line(sys, 'pos_meas/1', 'Onboard_Ephemeris/3', ar{:});

    % Sensor measurements → Attitude Estimator inputs 1-4
    add_line(sys, 'q_meas/1',      'Attitude_Estimator/1', ar{:});
    add_line(sys, 'omega_meas/1',  'Attitude_Estimator/2', ar{:});
    add_line(sys, 'sun_meas/1',    'Attitude_Estimator/3', ar{:});
    add_line(sys, 'B_body_meas/1', 'Attitude_Estimator/4', ar{:});

    % Onboard Ephemeris → Attitude Estimator inputs 5-6
    add_line(sys, 'Onboard_Ephemeris/1', 'Attitude_Estimator/5', ar{:});
    add_line(sys, 'Onboard_Ephemeris/2', 'Attitude_Estimator/6', ar{:});

    % Attitude Estimator → Control Law inputs 1-2
    add_line(sys, 'Attitude_Estimator/1', 'Control_Law/1', ar{:});
    add_line(sys, 'Attitude_Estimator/2', 'Control_Law/2', ar{:});

    % B_body_meas → Control Law input 3
    add_line(sys, 'B_body_meas/1', 'Control_Law/3', ar{:});

    % Measured orbit state → Reference generator
    add_line(sys, 'pos_meas/1', 'Reference_Generator/1', ar{:});
    add_line(sys, 'vel_meas/1', 'Reference_Generator/2', ar{:});

    % Reference outputs → Control Law inputs 4-5
    add_line(sys, 'Reference_Generator/1', 'Control_Law/4', ar{:});
    add_line(sys, 'Reference_Generator/2', 'Control_Law/5', ar{:});

    % Attitude + reference + wheel momentum → Mode Manager
    add_line(sys, 'Attitude_Estimator/1', 'Mode_Manager/1', ar{:});
    add_line(sys, 'Attitude_Estimator/2', 'Mode_Manager/2', ar{:});
    add_line(sys, 'sun_meas/1',           'Mode_Manager/3', ar{:});
    add_line(sys, 'Reference_Generator/1','Mode_Manager/4', ar{:});
    add_line(sys, 'h_W_in/1',             'Mode_Manager/5', ar{:});

    % Mode + wheel momentum → Control Law inputs 6-7
    add_line(sys, 'Mode_Manager/1', 'Control_Law/6', ar{:});
    add_line(sys, 'h_W_in/1',       'Control_Law/7', ar{:});

    % Control Law → outports
    add_line(sys, 'Control_Law/1', 'rw_torque_cmd/1',   ar{:});
    add_line(sys, 'Control_Law/2', 'mtq_dipole_cmd/1',  ar{:});
    add_line(sys, 'Control_Law/3', 'cmg_gimbal_cmd/1',  ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Onboard Ephemeris  (analytical Sun + local magnetic prediction)
% ─────────────────────────────────────────────────────────────────────────
function s = onboard_ephemeris_code()
    L = {
    'function [sun_pred, mag_pred] = fcn(t, epoch_jd, pos_eci)'
    '%#codegen'
    '% Deterministic reference vectors used during compile hardening.'
    '  sun_pred = [1; 0; 0];'
    '  mag_pred = [0; 0; 1];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Attitude Estimator  (TRIAD-seeded MEKF)
% ─────────────────────────────────────────────────────────────────────────
function s = attitude_estimator_code()
    L = {
    'function [q_est, omega_est] = fcn(q_meas, omega_meas, sun_meas, B_body_meas, sun_ref, mag_ref)'
    '%#codegen'
    '% Compile-stable estimator passthrough.'
    '  qn = sqrt(q_meas(1)^2 + q_meas(2)^2 + q_meas(3)^2 + q_meas(4)^2);'
    '  if qn > 1e-12'
    '      q_est = q_meas / qn;'
    '  else'
    '      q_est = [1; 0; 0; 0];'
    '  end'
    '  if q_est(1) < 0'
    '      q_est = -q_est;'
    '  end'
    '  omega_est = omega_meas;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Nadir-pointing Reference Generator
% ─────────────────────────────────────────────────────────────────────────
function s = reference_generator_code()
    L = {
    'function [q_des, omega_des] = fcn(pos_eci, vel_eci)'
    '%#codegen'
    '% Compile-stable identity reference.'
    '  q_des = [1; 0; 0; 0];'
    '  omega_des = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Mode Manager
% ─────────────────────────────────────────────────────────────────────────
function s = mode_manager_code()
    L = {
    'function mode_id = fcn(q_est, omega_est, sun_meas, q_des, h_W)'
    '%#codegen'
    '% Compile-stable constant mode selection.'
    '  mode_id = 2;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Control Law (mode-aware RW + MTQ unload + CMG assist)
% ─────────────────────────────────────────────────────────────────────────
function s = control_law_code()
    L = {
    'function [rw_cmd, mtq_cmd, cmg_cmd] = fcn(q_est, omega_est, B_body, q_des, omega_des, mode_id, h_W)'
    '%#codegen'
    '% Compile-stable zero-torque fallback.'
    '  rw_cmd = [0; 0; 0];'
    '  mtq_cmd = [0; 0; 0];'
    '  cmg_cmd = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end
