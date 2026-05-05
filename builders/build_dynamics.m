function build_dynamics(sys)
%BUILD_DYNAMICS  Build the DYNAMICS subsystem inside the ADCS model.
%
%   Blocks: Euler_RHS, Gravity_Gradient, Aero_Drag_Torque, SRP_Torque,
%           Residual_Mag_Torque, Ext_Torque_Sum, Quat_RHS, Quat_Norm,
%           Integ_omega, Integ_hW, Integ_q
%   Inports: Torque_ext, rw_torque_body, rw_tau_wheels, pos_ECI, vel_ECI,
%            Sun_ECI, eclipse_flag, B_ECI
%   Outports: q_out, omega_out, h_W_out

    ar = {'autorouting','on'};

    % ── Inports (8) ──
    add_inports(sys, ...
        {'Torque_ext','rw_torque_body','rw_tau_wheels','pos_ECI', ...
         'vel_ECI','Sun_ECI','eclipse_flag','B_ECI'}, ...
        30, 60, 45);

    % ── Outports (3) ──
    add_outports(sys, {'q_out','omega_out','h_W_out'}, 980, 80, 180);

    % ── Parameters from workspace ──
    add_block('simulink/Sources/Constant', [sys '/J_param'], ...
        'Position', [30  430  110  450], 'Value', 'J');

    % ── Euler equation RHS ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Euler_RHS'], 'Position', [380  190  600  330]);
    set_mfb_script([sys '/Euler_RHS'], euler_rhs_code());
    set_mfb_data_sizes([sys '/Euler_RHS'], ...
        {'torque_ext','[3 1]'; 'omega','[3 1]'; 'J','[3 3]'; 'h_W','[4 1]'; ...
         'rw_torque_body','[3 1]'; 'omega_dot','[3 1]'});

    % ── Gravity-gradient torque ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Gravity_Gradient'], 'Position', [380  380  600  500]);
    set_mfb_script([sys '/Gravity_Gradient'], gravity_gradient_code());
    set_mfb_data_sizes([sys '/Gravity_Gradient'], ...
        {'pos_eci','[3 1]'; 'q','[4 1]'; 'J','[3 3]'; 'tau_gg','[3 1]'});

    % ── Aerodynamic drag torque ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Aero_Drag_Torque'], 'Position', [380  540  600  650]);
    set_mfb_script([sys '/Aero_Drag_Torque'], aero_drag_code());
    set_mfb_data_sizes([sys '/Aero_Drag_Torque'], ...
        {'pos_eci','[3 1]'; 'vel_eci','[3 1]'; 'q','[4 1]'; 'tau_drag','[3 1]'});

    % ── Solar radiation pressure torque ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/SRP_Torque'], 'Position', [380  690  600  800]);
    set_mfb_script([sys '/SRP_Torque'], srp_torque_code());
    set_mfb_data_sizes([sys '/SRP_Torque'], ...
        {'sun_eci','[3 1]'; 'eclipse_flag','1'; 'q','[4 1]'; 'tau_srp','[3 1]'});

    % ── Residual magnetic dipole torque ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Residual_Mag_Torque'], 'Position', [380  840  600  930]);
    set_mfb_script([sys '/Residual_Mag_Torque'], residual_mag_torque_code());
    set_mfb_data_sizes([sys '/Residual_Mag_Torque'], ...
        {'B_eci','[3 1]'; 'q','[4 1]'; 'tau_mag','[3 1]'});

    % ── Sum external torques: commanded + environmental disturbances ──
    add_block('simulink/Math Operations/Sum', [sys '/Ext_Torque_Sum'], ...
        'Position', [300  220  330  320], 'Inputs', '+++++');

    % ── Integrators ──
    add_block('simulink/Continuous/Integrator', [sys '/Integ_omega'], ...
        'Position', [700  240  740  280], 'InitialCondition', 'omega0');

    add_block('simulink/Continuous/Integrator', [sys '/Integ_hW'], ...
        'Position', [700  450  740  490], 'InitialCondition', 'h_W0');

    add_block('simulink/Continuous/Integrator', [sys '/Integ_q'], ...
        'Position', [700  70  740  110], 'InitialCondition', 'q0');

    % ── Quaternion kinematics and normalizer ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Quat_RHS'], 'Position', [380  40  600  140]);
    set_mfb_script([sys '/Quat_RHS'], quat_rhs_code());
    set_mfb_data_sizes([sys '/Quat_RHS'], ...
        {'omega','[3 1]'; 'q','[4 1]'; 'q_dot','[4 1]'});

    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Quat_Norm'], 'Position', [820  70  940  110]);
    set_mfb_script([sys '/Quat_Norm'], quat_norm_code());
    set_mfb_data_sizes([sys '/Quat_Norm'], ...
        {'q','[4 1]'; 'q_out','[4 1]'});

    % ═════════════  W I R I N G  ═════════════
    add_line(sys, 'Torque_ext/1',       'Ext_Torque_Sum/1',  ar{:});
    add_line(sys, 'Gravity_Gradient/1', 'Ext_Torque_Sum/2',  ar{:});
    add_line(sys, 'Aero_Drag_Torque/1', 'Ext_Torque_Sum/3',  ar{:});
    add_line(sys, 'SRP_Torque/1',       'Ext_Torque_Sum/4',  ar{:});
    add_line(sys, 'Residual_Mag_Torque/1', 'Ext_Torque_Sum/5', ar{:});

    add_line(sys, 'Ext_Torque_Sum/1', 'Euler_RHS/1', ar{:});
    add_line(sys, 'Integ_omega/1',    'Euler_RHS/2', ar{:});
    add_line(sys, 'J_param/1',        'Euler_RHS/3', ar{:});
    add_line(sys, 'Integ_hW/1',       'Euler_RHS/4', ar{:});
    add_line(sys, 'rw_torque_body/1', 'Euler_RHS/5', ar{:});

    add_line(sys, 'Euler_RHS/1',    'Integ_omega/1', ar{:});
    add_line(sys, 'Integ_omega/1',  'omega_out/1',   ar{:});

    add_line(sys, 'rw_tau_wheels/1', 'Integ_hW/1',   ar{:});
    add_line(sys, 'Integ_hW/1',      'h_W_out/1',    ar{:});

    add_line(sys, 'Integ_omega/1', 'Quat_RHS/1', ar{:});
    add_line(sys, 'Quat_Norm/1',   'Quat_RHS/2', ar{:});

    add_line(sys, 'Quat_RHS/1',   'Integ_q/1',   ar{:});
    add_line(sys, 'Integ_q/1',    'Quat_Norm/1', ar{:});
    add_line(sys, 'Quat_Norm/1',  'q_out/1',     ar{:});

    add_line(sys, 'pos_ECI/1',     'Gravity_Gradient/1', ar{:});
    add_line(sys, 'Quat_Norm/1',   'Gravity_Gradient/2', ar{:});
    add_line(sys, 'J_param/1',     'Gravity_Gradient/3', ar{:});

    add_line(sys, 'pos_ECI/1',     'Aero_Drag_Torque/1', ar{:});
    add_line(sys, 'vel_ECI/1',     'Aero_Drag_Torque/2', ar{:});
    add_line(sys, 'Quat_Norm/1',   'Aero_Drag_Torque/3', ar{:});

    add_line(sys, 'Sun_ECI/1',       'SRP_Torque/1', ar{:});
    add_line(sys, 'eclipse_flag/1',  'SRP_Torque/2', ar{:});
    add_line(sys, 'Quat_Norm/1',     'SRP_Torque/3', ar{:});

    add_line(sys, 'B_ECI/1',       'Residual_Mag_Torque/1', ar{:});
    add_line(sys, 'Quat_Norm/1',   'Residual_Mag_Torque/2', ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  Euler equation RHS
% ─────────────────────────────────────────────────────────────────────────
function s = euler_rhs_code()
    L = {
    'function omega_dot = fcn(torque_ext, omega, J, h_W, rw_torque_body)'
    '%#codegen'
    '% Euler equation with reaction-wheel momentum coupling:'
    '%   omega_dot = J \ (torque_ext - omega×(J*omega + A_W*h_W) - rw_torque_body)'
    ''
    '  beta = atan(1/sqrt(2));'
    '  cb = cos(beta); sb = sin(beta);'
    '  A_W = [ cb,  0, -cb,  0;'
    '           0, cb,   0, -cb;'
    '          sb, sb,  sb,  sb ];'
    ''
    '  H_total = J * omega + A_W * h_W;'
    '  cross_term = [ omega(2)*H_total(3) - omega(3)*H_total(2);'
    '                 omega(3)*H_total(1) - omega(1)*H_total(3);'
    '                 omega(1)*H_total(2) - omega(2)*H_total(1) ];'
    '  omega_dot = J \ (torque_ext - cross_term - rw_torque_body);'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Gravity-gradient torque
% ─────────────────────────────────────────────────────────────────────────
function s = gravity_gradient_code()
    L = {
    'function tau_gg = fcn(pos_eci, q, J)'
    '%#codegen'
    '% Gravity-gradient torque in body frame.'
    '% tau_gg = (3*mu/R^3) * cross(r_body, J*r_body)'
    ''
    '  mu = 3.986004418e14;'
    '  R = sqrt(pos_eci(1)^2 + pos_eci(2)^2 + pos_eci(3)^2);'
    '  if R < 1e-9'
    '      tau_gg = [0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  r_hat_eci = -pos_eci / R;'
    ''
    '  q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);'
    '  % Use the same ECI->body DCM convention as the active sensor models.'
    '  C_bi = [1-2*(q2*q2+q3*q3),   2*(q1*q2+q0*q3),   2*(q1*q3-q0*q2);'
    '            2*(q1*q2-q0*q3), 1-2*(q1*q1+q3*q3),   2*(q2*q3+q0*q1);'
    '            2*(q1*q3+q0*q2),   2*(q2*q3-q0*q1), 1-2*(q1*q1+q2*q2)];'
    ''
    '  r_body = C_bi * r_hat_eci;'
    '  Jr = J * r_body;'
    '  c = [r_body(2)*Jr(3)-r_body(3)*Jr(2);'
    '       r_body(3)*Jr(1)-r_body(1)*Jr(3);'
    '       r_body(1)*Jr(2)-r_body(2)*Jr(1)];'
    '  tau_gg = (3*mu/(R^3)) * c;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Aerodynamic drag torque
% ─────────────────────────────────────────────────────────────────────────
function s = aero_drag_code()
    L = {
    'function tau_drag = fcn(pos_eci, vel_eci, q)'
    '%#codegen'
    '% Compile-stable zero disturbance fallback.'
    '  tau_drag = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Solar radiation pressure torque
% ─────────────────────────────────────────────────────────────────────────
function s = srp_torque_code()
    L = {
    'function tau_srp = fcn(sun_eci, eclipse_flag, q)'
    '%#codegen'
    '% Compile-stable zero disturbance fallback.'
    '  tau_srp = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Residual magnetic dipole torque
% ─────────────────────────────────────────────────────────────────────────
function s = residual_mag_torque_code()
    L = {
    'function tau_mag = fcn(B_eci, q)'
    '%#codegen'
    '% Compile-stable zero disturbance fallback.'
    '  tau_mag = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Quaternion kinematics RHS
% ─────────────────────────────────────────────────────────────────────────
function s = quat_rhs_code()
    L = {
    'function q_dot = fcn(omega, q)'
    '%#codegen'
    '% Quaternion kinematics: q_dot = 0.5 * Omega(omega) * q'
    '% Convention: q = [q0; q1; q2; q3]  (scalar-first)'
    ''
    '  wx = omega(1); wy = omega(2); wz = omega(3);'
    ''
    '  % Omega matrix for scalar-first convention'
    '  Omega = [  0,  -wx, -wy, -wz;'
    '            wx,    0,  wz, -wy;'
    '            wy, -wz,    0,  wx;'
    '            wz,  wy, -wx,    0 ];'
    ''
    '  q_dot = 0.5 * Omega * q;'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Quaternion normalisation
% ─────────────────────────────────────────────────────────────────────────
function s = quat_norm_code()
    L = {
    'function q_out = fcn(q)'
    '%#codegen'
    '% Normalise quaternion.'
    '  n = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);'
    '  if n > 0'
    '      q_out = q / n;'
    '  else'
    '      q_out = [1; 0; 0; 0];'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end
