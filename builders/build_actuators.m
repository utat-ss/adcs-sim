function build_actuators(sys)
%BUILD_ACTUATORS  Build the ACTUATORS subsystem inside the ADCS model.
%
%   Blocks: RW_Assembly, MTQ_Assembly, CMG, Ext_Sum
%   Inports: rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd, B_body_in, h_W_in
%   Outports: ext_torque, rw_torque_body, rw_tau_wheels

    ar = {'autorouting','on'};

    % ── Inports (5) ──
    add_inports(sys, {'rw_torque_cmd','mtq_dipole_cmd','cmg_gimbal_cmd','B_body_in','h_W_in'}, ...
        30, 50, 90);

    % ── Outports (3) ──
    add_outports(sys, {'ext_torque','rw_torque_body','rw_tau_wheels'}, 860, 150, 90);

    % ── Workspace-backed actuator parameters ──
    add_block('simulink/Sources/Constant', [sys '/RW_Axes'], ...
        'Position', [30  420  130  440], 'Value', 'rw_axes');
    add_block('simulink/Sources/Constant', [sys '/RW_Limits'], ...
        'Position', [30  460  130  480], ...
        'Value', '[rw_max_torque; rw_max_momentum; rw_inertia; rw_max_speed; dt]');
    add_block('simulink/Sources/Constant', [sys '/MTQ_Max_Dipole'], ...
        'Position', [30  500  130  520], 'Value', 'mtq_max_dipole');

    % ── Reaction Wheel Assembly ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/RW_Assembly'], 'Position', [220  20  440  120]);
    set_mfb_script([sys '/RW_Assembly'], rw_assembly_code());
    set_mfb_data_sizes([sys '/RW_Assembly'], ...
        {'torque_cmd','[3 1]'; 'h_W','[4 1]'; 'rw_axes','[3 4]'; 'rw_limits','[5 1]'; ...
         'torque_body','[3 1]'; 'tau_wheels','[4 1]'});

    % ── Magnetorquer Assembly ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/MTQ_Assembly'], 'Position', [220  160  440  260]);
    set_mfb_script([sys '/MTQ_Assembly'], mtq_assembly_code());
    set_mfb_data_sizes([sys '/MTQ_Assembly'], ...
        {'dipole_cmd','[3 1]'; 'B_body','[3 1]'; 'mtq_max_dipole','1'; 'torque_body','[3 1]'});

    % ── CMG (simplified representative cluster) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/CMG'], 'Position', [220  300  440  370]);
    set_mfb_script([sys '/CMG'], cmg_code());
    set_mfb_data_sizes([sys '/CMG'], ...
        {'gimbal_cmd','[3 1]'; 'torque_body','[3 1]'});

    % ── External torque sum (MTQ + CMG only) ──
    add_block('simulink/Math Operations/Sum', [sys '/Ext_Sum'], ...
        'Position', [580  230  610  300], 'Inputs', '++');

    % ── Wiring ──
    add_line(sys, 'rw_torque_cmd/1',  'RW_Assembly/1',  ar{:});
    add_line(sys, 'h_W_in/1',         'RW_Assembly/2',  ar{:});
    add_line(sys, 'RW_Axes/1',        'RW_Assembly/3',  ar{:});
    add_line(sys, 'RW_Limits/1',      'RW_Assembly/4',  ar{:});
    add_line(sys, 'mtq_dipole_cmd/1', 'MTQ_Assembly/1', ar{:});
    add_line(sys, 'B_body_in/1',      'MTQ_Assembly/2', ar{:});
    add_line(sys, 'MTQ_Max_Dipole/1', 'MTQ_Assembly/3', ar{:});
    add_line(sys, 'cmg_gimbal_cmd/1', 'CMG/1',          ar{:});

    add_line(sys, 'MTQ_Assembly/1', 'Ext_Sum/1', ar{:});
    add_line(sys, 'CMG/1',          'Ext_Sum/2', ar{:});

    add_line(sys, 'Ext_Sum/1',     'ext_torque/1',     ar{:});
    add_line(sys, 'RW_Assembly/1', 'rw_torque_body/1', ar{:});
    add_line(sys, 'RW_Assembly/2', 'rw_tau_wheels/1',  ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Reaction Wheel Assembly
% ─────────────────────────────────────────────────────────────────────────
function s = rw_assembly_code()
    L = {
    'function [torque_body, tau_wheels] = fcn(torque_cmd, h_W, rw_axes, rw_limits)'
    '%#codegen'
    '% Four-wheel pyramid. torque_body is the torque applied to the spacecraft.'
    '% Wheel torque is clipped by torque limit and by the remaining one-step'
    '% momentum/speed headroom, so h_W + dt*tau_wheels stays inside limits.'
    '  A = rw_axes;'
    '  G = A * A'';'
    '  max_torque = abs(rw_limits(1));'
    '  max_momentum = abs(rw_limits(2));'
    '  rw_inertia = abs(rw_limits(3));'
    '  max_speed_rpm = abs(rw_limits(4));'
    '  step_dt = abs(rw_limits(5));'
    '  speed_momentum = rw_inertia * max_speed_rpm * 2.0*pi/60.0;'
    '  h_limit = min(max(max_momentum, 0.0), max(speed_momentum, 0.0));'
    '  if abs(det(G)) < 1e-12 || max_torque <= 0.0 || h_limit <= 0.0 || step_dt <= 0.0'
    '      torque_body = [0; 0; 0];'
    '      tau_wheels = [0; 0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  tau_wheels = A'' * (G \ (-torque_cmd));'
    '  tau_wheels = min(max(tau_wheels, -max_torque), max_torque);'
    '  for k = 1:4'
    '      h_now = min(max(h_W(k), -h_limit), h_limit);'
    '      pos_rate_limit = max((h_limit - h_now) / step_dt, 0.0);'
    '      neg_rate_limit = max((h_now + h_limit) / step_dt, 0.0);'
    '      tau_wheels(k) = min(max(tau_wheels(k), -neg_rate_limit), pos_rate_limit);'
    '  end'
    '  torque_body = -A * tau_wheels;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetorquer Assembly
% ─────────────────────────────────────────────────────────────────────────
function s = mtq_assembly_code()
    L = {
    'function torque_body = fcn(dipole_cmd, B_body, mtq_max_dipole)'
    '%#codegen'
    '% Magnetic torque from commanded dipole: tau = m x B.'
    '  limit = max(abs(mtq_max_dipole), 0.0);'
    '  m = min(max(dipole_cmd, -limit), limit);'
    '  torque_body = [m(2)*B_body(3)-m(3)*B_body(2);'
    '                 m(3)*B_body(1)-m(1)*B_body(3);'
    '                 m(1)*B_body(2)-m(2)*B_body(1)];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  CMG (simplified representative cluster)
% ─────────────────────────────────────────────────────────────────────────
function s = cmg_code()
    L = {
    'function torque_body = fcn(gimbal_cmd)'
    '%#codegen'
    '% CMG dynamics are intentionally disabled until the gimbal contract is modeled.'
    '  torque_body = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end
