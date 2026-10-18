function build_control(sys)
%BUILD_CONTROL  Build the CONTROL subsystem inside the ADCS model.
%
%   Blocks: Onboard_Ephemeris, Attitude_Estimator, Reference_Generator,
%           Mode_Manager, Control_Law
%   Inports: q_meas, omega_meas, sun_meas, B_body_meas, pos_meas, vel_meas,
%            h_W_in, st_valid
%   Outports: rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd

    ar = {'autorouting','on'};

    % ── Inports (8) ──
    add_inports(sys, ...
        {'q_meas','omega_meas','sun_meas','B_body_meas','pos_meas','vel_meas','h_W_in','st_valid'}, ...
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
    add_block('simulink/Sources/Constant', [sys '/Control_Params'], ...
        'Position', [520  650  650  670], ...
        'Value', ['[ctrl_rw_kp; ctrl_rw_kd; ctrl_bdot_gain; ctrl_dump_gain; ' ...
                  'ctrl_unload_fraction; mode_detumble_rate; mode_fine_rate; ' ...
                  'mode_fine_err; rw_max_momentum; mtq_max_dipole]']);
    add_block('simulink/Sources/Constant', [sys '/RW_Axes'], ...
        'Position', [520  690  650  710], 'Value', 'rw_axes');
    add_block('simulink/Sources/Constant', [sys '/RW_Speed_Params'], ...
        'Position', [520  730  650  750], ...
        'Value', '[rw_inertia; rw_max_speed]');

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
         'sun_ref','[3 1]'; 'mag_ref','[3 1]'; 'st_valid','1'; ...
         'q_est','[4 1]'; 'omega_est','[3 1]'; 'att_valid','1'});

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
         'h_W','[4 1]'; 'att_valid','1'; 'control_params','[10 1]'; ...
         'rw_speed_params','[2 1]'; 'mode_id','1'});

    % ── Control Law (mode-aware RW/MTQ/CMG control) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Control_Law'], 'Position', [750  80  940  330]);
    set_mfb_script([sys '/Control_Law'], control_law_code());
    set_mfb_data_sizes([sys '/Control_Law'], ...
        {'q_est','[4 1]'; 'omega_est','[3 1]'; 'B_body','[3 1]'; 'q_des','[4 1]'; ...
         'omega_des','[3 1]'; 'mode_id','1'; 'h_W','[4 1]'; ...
         'control_params','[10 1]'; 'rw_axes','[3 4]'; 'rw_speed_params','[2 1]'; ...
         'rw_cmd','[3 1]'; 'mtq_cmd','[3 1]'; 'cmg_cmd','[3 1]'});

    % ── Internal control-state logs for validation and dashboard replay ──
    add_block('simulink/Sinks/To Workspace', [sys '/Q_Est_To_Workspace'], ...
        'Position', [970  360  1090  390], 'VariableName', 'q_est_log', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [sys '/Q_Des_To_Workspace'], ...
        'Position', [970  400  1090  430], 'VariableName', 'q_des_log', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [sys '/Att_Valid_To_Workspace'], ...
        'Position', [970  440  1090  470], 'VariableName', 'att_valid_log', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [sys '/Mode_ID_To_Workspace'], ...
        'Position', [970  480  1090  510], 'VariableName', 'mode_id_log', 'SaveFormat', 'Timeseries');

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
    add_line(sys, 'st_valid/1',           'Attitude_Estimator/7', ar{:});

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
    add_line(sys, 'Attitude_Estimator/3', 'Mode_Manager/6', ar{:});
    add_line(sys, 'Control_Params/1',     'Mode_Manager/7', ar{:});
    add_line(sys, 'RW_Speed_Params/1',    'Mode_Manager/8', ar{:});

    % Mode + wheel momentum → Control Law inputs 6-7
    add_line(sys, 'Mode_Manager/1', 'Control_Law/6', ar{:});
    add_line(sys, 'h_W_in/1',       'Control_Law/7', ar{:});
    add_line(sys, 'Control_Params/1','Control_Law/8', ar{:});
    add_line(sys, 'RW_Axes/1',       'Control_Law/9', ar{:});
    add_line(sys, 'RW_Speed_Params/1','Control_Law/10', ar{:});

    % Control Law → outports
    add_line(sys, 'Control_Law/1', 'rw_torque_cmd/1',   ar{:});
    add_line(sys, 'Control_Law/2', 'mtq_dipole_cmd/1',  ar{:});
    add_line(sys, 'Control_Law/3', 'cmg_gimbal_cmd/1',  ar{:});

    add_line(sys, 'Attitude_Estimator/1', 'Q_Est_To_Workspace/1', ar{:});
    add_line(sys, 'Reference_Generator/1','Q_Des_To_Workspace/1', ar{:});
    add_line(sys, 'Attitude_Estimator/3', 'Att_Valid_To_Workspace/1', ar{:});
    add_line(sys, 'Mode_Manager/1',       'Mode_ID_To_Workspace/1', ar{:});
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
    '% Low-precision onboard Sun and dipole magnetic-field prediction.'
    '  jd = epoch_jd + t/86400.0;'
    '  T = (jd - 2451545.0) / 36525.0;'
    '  deg = pi/180.0;'
    '  M = mod(357.5291 + 35999.0503*T, 360.0) * deg;'
    '  L0 = mod(280.4664 + 36000.7698*T, 360.0) * deg;'
    '  C = (1.9146*sin(M) + 0.0200*sin(2.0*M)) * deg;'
    '  lambda = L0 + C;'
    '  eps = (23.4393 - 0.0130*T) * deg;'
    '  sun_pred = [cos(lambda); cos(eps)*sin(lambda); sin(eps)*sin(lambda)];'
    '  sn = sqrt(sun_pred(1)^2 + sun_pred(2)^2 + sun_pred(3)^2);'
    '  if sn > 1e-12'
    '      sun_pred = sun_pred / sn;'
    '  else'
    '      sun_pred = [1; 0; 0];'
    '  end'
    ''
    '  R_earth = 6371e3;'
    '  B0 = 3.12e-5;'
    '  r = sqrt(pos_eci(1)^2 + pos_eci(2)^2 + pos_eci(3)^2);'
    '  if r > 1e-6'
    '      rhat = pos_eci / r;'
    '      tilt = 11.5*deg;'
    '      lon = -69.0*deg;'
    '      m_ecef = [sin(tilt)*cos(lon); sin(tilt)*sin(lon); cos(tilt)];'
    '      gmst_deg = 280.46061837 + 360.98564736629*(jd - 2451545.0) + 0.000387933*T*T - (T*T*T)/38710000.0;'
    '      gmst = mod(gmst_deg, 360.0) * deg;'
    '      cg = cos(gmst); sg = sin(gmst);'
    '      mhat = [cg*m_ecef(1) - sg*m_ecef(2); sg*m_ecef(1) + cg*m_ecef(2); m_ecef(3)];'
    '      mag_pred = B0 * (R_earth/r)^3 * (3.0*dot(mhat, rhat)*rhat - mhat);'
    '  else'
    '      mag_pred = [0; 0; B0];'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Attitude Estimator  (TRIAD-seeded MEKF)
% ─────────────────────────────────────────────────────────────────────────
function s = attitude_estimator_code()
    L = {
    'function [q_est, omega_est, att_valid] = fcn(q_meas, omega_meas, sun_meas, B_body_meas, sun_ref, mag_ref, st_valid)'
    '%#codegen'
    '% Prefer a fresh valid tracker quaternion; otherwise use TRIAD.'
    '% If neither path is observable, q_est is only a numerical hold/fallback'
    '% and att_valid is false so downstream mode logic stays degraded.'
    '  q_fallback = [1; 0; 0; 0];'
    '  qn = sqrt(sum(q_meas.^2));'
    '  if qn > 1e-9'
    '      q_fallback = q_meas / qn;'
    '      if q_fallback(1) < 0'
    '          q_fallback = -q_fallback;'
    '      end'
    '  end'
    '  if st_valid > 0.5 && qn > 1e-9'
    '      q_est = q_fallback;'
    '      omega_est = omega_meas;'
    '      att_valid = 1.0;'
    '      return;'
    '  end'
    ''
    '  sbn = sqrt(sum(sun_meas.^2));'
    '  mbn = sqrt(sum(B_body_meas.^2));'
    '  srn = sqrt(sum(sun_ref.^2));'
    '  mrn = sqrt(sum(mag_ref.^2));'
    '  if sbn < 1e-9 || mbn < 1e-12 || srn < 1e-9 || mrn < 1e-12'
    '      q_est = q_fallback;'
    '      omega_est = omega_meas;'
    '      att_valid = 0.0;'
    '      return;'
    '  end'
    ''
    '  s_b = sun_meas / sbn;'
    '  m_b = B_body_meas / mbn;'
    '  s_r = sun_ref / srn;'
    '  m_r = mag_ref / mrn;'
    '  c_b = [s_b(2)*m_b(3)-s_b(3)*m_b(2); s_b(3)*m_b(1)-s_b(1)*m_b(3); s_b(1)*m_b(2)-s_b(2)*m_b(1)];'
    '  c_r = [s_r(2)*m_r(3)-s_r(3)*m_r(2); s_r(3)*m_r(1)-s_r(1)*m_r(3); s_r(1)*m_r(2)-s_r(2)*m_r(1)];'
    '  c_b_n = sqrt(sum(c_b.^2));'
    '  c_r_n = sqrt(sum(c_r.^2));'
    '  if c_b_n < 1e-4 || c_r_n < 1e-4'
    '      q_est = q_fallback;'
    '      omega_est = omega_meas;'
    '      att_valid = 0.0;'
    '      return;'
    '  end'
    '  t1_b = s_b;'
    '  t2_b = c_b / c_b_n;'
    '  t3_b = [t1_b(2)*t2_b(3)-t1_b(3)*t2_b(2); t1_b(3)*t2_b(1)-t1_b(1)*t2_b(3); t1_b(1)*t2_b(2)-t1_b(2)*t2_b(1)];'
    '  t1_r = s_r;'
    '  t2_r = c_r / c_r_n;'
    '  t3_r = [t1_r(2)*t2_r(3)-t1_r(3)*t2_r(2); t1_r(3)*t2_r(1)-t1_r(1)*t2_r(3); t1_r(1)*t2_r(2)-t1_r(2)*t2_r(1)];'
    '  M_b = [t1_b, t2_b, t3_b];'
    '  M_r = [t1_r, t2_r, t3_r];'
    '  C_bi = M_b * M_r'';'
    ''
    '  d0 = 1.0 + C_bi(1,1) + C_bi(2,2) + C_bi(3,3);'
    '  d1 = 1.0 + C_bi(1,1) - C_bi(2,2) - C_bi(3,3);'
    '  d2 = 1.0 - C_bi(1,1) + C_bi(2,2) - C_bi(3,3);'
    '  d3 = 1.0 - C_bi(1,1) - C_bi(2,2) + C_bi(3,3);'
    '  if d0 >= d1 && d0 >= d2 && d0 >= d3'
    '      q0 = 0.5*sqrt(max(d0, 0.0));'
    '      den = max(4.0*q0, 1e-12);'
    '      q_est = [q0; (C_bi(2,3)-C_bi(3,2))/den; (C_bi(3,1)-C_bi(1,3))/den; (C_bi(1,2)-C_bi(2,1))/den];'
    '  elseif d1 >= d2 && d1 >= d3'
    '      q1 = 0.5*sqrt(max(d1, 0.0));'
    '      den = max(4.0*q1, 1e-12);'
    '      q_est = [(C_bi(2,3)-C_bi(3,2))/den; q1; (C_bi(1,2)+C_bi(2,1))/den; (C_bi(3,1)+C_bi(1,3))/den];'
    '  elseif d2 >= d3'
    '      q2 = 0.5*sqrt(max(d2, 0.0));'
    '      den = max(4.0*q2, 1e-12);'
    '      q_est = [(C_bi(3,1)-C_bi(1,3))/den; (C_bi(1,2)+C_bi(2,1))/den; q2; (C_bi(2,3)+C_bi(3,2))/den];'
    '  else'
    '      q3 = 0.5*sqrt(max(d3, 0.0));'
    '      den = max(4.0*q3, 1e-12);'
    '      q_est = [(C_bi(1,2)-C_bi(2,1))/den; (C_bi(3,1)+C_bi(1,3))/den; (C_bi(2,3)+C_bi(3,2))/den; q3];'
    '  end'
    '  qn2 = sqrt(sum(q_est.^2));'
    '  if qn2 > 1e-9'
    '      q_est = q_est / qn2;'
    '      att_valid = 1.0;'
    '  else'
    '      q_est = q_fallback;'
    '      att_valid = 0.0;'
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
    '% LVLH/nadir reference, returned as ECI->desired-body quaternion.'
    '  rn = sqrt(sum(pos_eci.^2));'
    '  vn = sqrt(sum(vel_eci.^2));'
    '  if rn < 1e-6 || vn < 1e-9'
    '      q_des = [1; 0; 0; 0];'
    '      omega_des = [0; 0; 0];'
    '      return;'
    '  end'
    '  z_b_i = -pos_eci / rn;'
    '  v_proj = vel_eci - dot(vel_eci, z_b_i)*z_b_i;'
    '  vpn = sqrt(sum(v_proj.^2));'
    '  if vpn < 1e-9'
    '      q_des = [1; 0; 0; 0];'
    '      omega_des = [0; 0; 0];'
    '      return;'
    '  end'
    '  x_b_i = v_proj / vpn;'
    '  y_b_i = [z_b_i(2)*x_b_i(3)-z_b_i(3)*x_b_i(2); z_b_i(3)*x_b_i(1)-z_b_i(1)*x_b_i(3); z_b_i(1)*x_b_i(2)-z_b_i(2)*x_b_i(1)];'
    '  yn = sqrt(sum(y_b_i.^2));'
    '  if yn < 1e-9'
    '      q_des = [1; 0; 0; 0];'
    '      omega_des = [0; 0; 0];'
    '      return;'
    '  end'
    '  y_b_i = y_b_i / yn;'
    '  x_b_i = [y_b_i(2)*z_b_i(3)-y_b_i(3)*z_b_i(2); y_b_i(3)*z_b_i(1)-y_b_i(1)*z_b_i(3); y_b_i(1)*z_b_i(2)-y_b_i(2)*z_b_i(1)];'
    '  C_bi = [x_b_i''; y_b_i''; z_b_i''];'
    ''
    '  d0 = 1.0 + C_bi(1,1) + C_bi(2,2) + C_bi(3,3);'
    '  d1 = 1.0 + C_bi(1,1) - C_bi(2,2) - C_bi(3,3);'
    '  d2 = 1.0 - C_bi(1,1) + C_bi(2,2) - C_bi(3,3);'
    '  d3 = 1.0 - C_bi(1,1) - C_bi(2,2) + C_bi(3,3);'
    '  if d0 >= d1 && d0 >= d2 && d0 >= d3'
    '      q0 = 0.5*sqrt(max(d0, 0.0)); den = max(4.0*q0, 1e-12);'
    '      q_des = [q0; (C_bi(2,3)-C_bi(3,2))/den; (C_bi(3,1)-C_bi(1,3))/den; (C_bi(1,2)-C_bi(2,1))/den];'
    '  elseif d1 >= d2 && d1 >= d3'
    '      q1 = 0.5*sqrt(max(d1, 0.0)); den = max(4.0*q1, 1e-12);'
    '      q_des = [(C_bi(2,3)-C_bi(3,2))/den; q1; (C_bi(1,2)+C_bi(2,1))/den; (C_bi(3,1)+C_bi(1,3))/den];'
    '  elseif d2 >= d3'
    '      q2 = 0.5*sqrt(max(d2, 0.0)); den = max(4.0*q2, 1e-12);'
    '      q_des = [(C_bi(3,1)-C_bi(1,3))/den; (C_bi(1,2)+C_bi(2,1))/den; q2; (C_bi(2,3)+C_bi(3,2))/den];'
    '  else'
    '      q3 = 0.5*sqrt(max(d3, 0.0)); den = max(4.0*q3, 1e-12);'
    '      q_des = [(C_bi(1,2)-C_bi(2,1))/den; (C_bi(3,1)+C_bi(1,3))/den; (C_bi(2,3)+C_bi(3,2))/den; q3];'
    '  end'
    '  q_des = q_des / max(sqrt(sum(q_des.^2)), 1e-12);'
    '  if q_des(1) < 0'
    '      q_des = -q_des;'
    '  end'
    '  h_orb = [pos_eci(2)*vel_eci(3)-pos_eci(3)*vel_eci(2); pos_eci(3)*vel_eci(1)-pos_eci(1)*vel_eci(3); pos_eci(1)*vel_eci(2)-pos_eci(2)*vel_eci(1)];'
    '  omega_mag = sqrt(sum(h_orb.^2)) / max(rn*rn, 1e-12);'
    '  omega_des = [0; -omega_mag; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Mode Manager
% ─────────────────────────────────────────────────────────────────────────
function s = mode_manager_code()
    L = {
    'function mode_id = fcn(q_est, omega_est, sun_meas, q_des, h_W, att_valid, control_params, rw_speed_params)'
    '%#codegen'
    '% 1=detumble, 2=coarse pointing, 3=fine pointing.'
    '  rate = sqrt(sum(omega_est.^2));'
    '  qd = [q_des(1); -q_des(2); -q_des(3); -q_des(4)];'
    '  q_err = quat_multiply(qd, q_est);'
    '  q_err = q_err / max(sqrt(sum(q_err.^2)), 1e-12);'
    '  if q_err(1) < 0'
    '      q_err = -q_err;'
    '  end'
    '  err_ang = 2.0*atan2(sqrt(q_err(2)^2 + q_err(3)^2 + q_err(4)^2), max(min(q_err(1),1.0),-1.0));'
    '  sun_valid = sqrt(sum(sun_meas.^2)) > 1e-9;'
    '  unload_fraction = control_params(5);'
    '  detumble_rate = control_params(6);'
    '  fine_rate = control_params(7);'
    '  fine_err = control_params(8);'
    '  max_momentum = control_params(9);'
    '  speed_limit = abs(rw_speed_params(1)) * abs(rw_speed_params(2)) * 2.0*pi/60.0;'
    '  effective_h_limit = min(max(abs(max_momentum), 1e-9), max(speed_limit, 1e-9));'
    '  hfrac = max(abs(h_W)) / effective_h_limit;'
    '  if att_valid < 0.5'
    '      mode_id = 1;'
    '  elseif rate > detumble_rate'
    '      mode_id = 1;'
    '  elseif sun_valid && rate < fine_rate && err_ang < fine_err && hfrac < unload_fraction'
    '      mode_id = 3;'
    '  else'
    '      mode_id = 2;'
    '  end'
    'end'
    ''
    'function q = quat_multiply(p, r)'
    '  q = [p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4);'
    '       p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3);'
    '       p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2);'
    '       p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Control Law (mode-aware RW + MTQ unload; CMG disabled)
% ─────────────────────────────────────────────────────────────────────────
function s = control_law_code()
    L = {
    'function [rw_cmd, mtq_cmd, cmg_cmd] = fcn(q_est, omega_est, B_body, q_des, omega_des, mode_id, h_W, control_params, rw_axes, rw_speed_params)'
    '%#codegen'
    '% Mode-aware body torque and magnetic dipole commands.'
    '  cmg_cmd = [0; 0; 0];'
    '  rw_cmd = [0; 0; 0];'
    '  mtq_cmd = [0; 0; 0];'
    ''
    '  Bn = sqrt(sum(B_body.^2));'
    '  kp_fine = control_params(1);'
    '  kd_fine = control_params(2);'
    '  bdot_gain = control_params(3);'
    '  dump_gain = control_params(4);'
    '  unload_fraction = control_params(5);'
    '  max_momentum = control_params(9);'
    '  max_dipole = abs(control_params(10));'
    '  speed_limit = abs(rw_speed_params(1)) * abs(rw_speed_params(2)) * 2.0*pi/60.0;'
    '  effective_h_limit = min(max(abs(max_momentum), 1e-9), max(speed_limit, 1e-9));'
    '  if mode_id < 1.5'
    '      if Bn > 1e-9'
    '          m_raw = bdot_gain * [omega_est(2)*B_body(3)-omega_est(3)*B_body(2); omega_est(3)*B_body(1)-omega_est(1)*B_body(3); omega_est(1)*B_body(2)-omega_est(2)*B_body(1)];'
    '          mtq_cmd = min(max(m_raw, -max_dipole), max_dipole);'
    '      end'
    '      return;'
    '  end'
    ''
    '  qd = [q_des(1); -q_des(2); -q_des(3); -q_des(4)];'
    '  q_err = quat_multiply(qd, q_est);'
    '  q_err = q_err / max(sqrt(sum(q_err.^2)), 1e-12);'
    '  if q_err(1) < 0'
    '      q_err = -q_err;'
    '  end'
    '  C_err = dcm_from_quat(q_err);'
    '  omega_des_body = C_err * omega_des;'
    '  rate_err = omega_est - omega_des_body;'
    '  if mode_id > 2.5'
    '      kp = kp_fine;'
    '      kd = kd_fine;'
    '  else'
    '      kp = 0.4 * kp_fine;'
    '      kd = 0.5 * kd_fine;'
    '  end'
    '  rw_cmd = -kp*q_err(2:4) - kd*rate_err;'
    ''
    '  A = rw_axes;'
    '  G = A * A'';'
    '  if abs(det(G)) < 1e-12'
    '      rw_cmd = [0; 0; 0];'
    '      return;'
    '  end'
    '  tau_w = A'' * (G \ (-rw_cmd));'
    '  for k = 1:4'
    '      if abs(h_W(k)) > 0.95*effective_h_limit && tau_w(k)*h_W(k) > 0.0'
    '          tau_w(k) = 0.0;'
    '      end'
    '  end'
    '  rw_cmd = -A * tau_w;'
    ''
    '  H_body = A * h_W;'
    '  if Bn > 1e-9 && max(abs(h_W)) > unload_fraction*effective_h_limit'
    '      tau_dump = -dump_gain * H_body;'
    '      B2 = max(Bn*Bn, 1e-18);'
    '      m_dump = [B_body(2)*tau_dump(3)-B_body(3)*tau_dump(2); B_body(3)*tau_dump(1)-B_body(1)*tau_dump(3); B_body(1)*tau_dump(2)-B_body(2)*tau_dump(1)] / B2;'
    '      mtq_cmd = min(max(m_dump, -max_dipole), max_dipole);'
    '  end'
    'end'
    ''
    'function q = quat_multiply(p, r)'
    '  q = [p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4);'
    '       p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3);'
    '       p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2);'
    '       p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];'
    'end'
    ''
    'function C = dcm_from_quat(q)'
    '  q0=q(1); q1=q(2); q2=q(3); q3=q(4);'
    '  C = [1-2*(q2*q2+q3*q3),   2*(q1*q2+q0*q3),   2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3), 1-2*(q1*q1+q3*q3),   2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),   2*(q2*q3-q0*q1), 1-2*(q1*q1+q2*q2)];'
    'end'
    };
    s = strjoin(L, newline);
end
