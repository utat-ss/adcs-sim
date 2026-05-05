function build_actuators(sys)
%BUILD_ACTUATORS  Build the ACTUATORS subsystem inside the ADCS model.
%
%   Blocks: RW_Assembly, MTQ_Assembly, CMG, Ext_Sum
%   Inports: rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd, B_body_in
%   Outports: ext_torque, rw_torque_body, rw_tau_wheels

    ar = {'autorouting','on'};

    % ── Inports (4) ──
    add_inports(sys, {'rw_torque_cmd','mtq_dipole_cmd','cmg_gimbal_cmd','B_body_in'}, ...
        30, 50, 90);

    % ── Outports (3) ──
    add_outports(sys, {'ext_torque','rw_torque_body','rw_tau_wheels'}, 860, 150, 90);

    % ── Reaction Wheel Assembly ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/RW_Assembly'], 'Position', [220  20  440  120]);
    set_mfb_script([sys '/RW_Assembly'], rw_assembly_code());
    set_mfb_data_sizes([sys '/RW_Assembly'], ...
        {'torque_cmd','[3 1]'; 'torque_body','[3 1]'; 'tau_wheels','[4 1]'});

    % ── Magnetorquer Assembly ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/MTQ_Assembly'], 'Position', [220  160  440  260]);
    set_mfb_script([sys '/MTQ_Assembly'], mtq_assembly_code());
    set_mfb_data_sizes([sys '/MTQ_Assembly'], ...
        {'dipole_cmd','[3 1]'; 'B_body','[3 1]'; 'torque_body','[3 1]'});

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
    add_line(sys, 'mtq_dipole_cmd/1', 'MTQ_Assembly/1', ar{:});
    add_line(sys, 'B_body_in/1',      'MTQ_Assembly/2', ar{:});
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
    'function [torque_body, tau_wheels] = fcn(torque_cmd)'
    '%#codegen'
    '% Compile-stable zero-torque fallback.'
    '  torque_body = [0; 0; 0];'
    '  tau_wheels = [0; 0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end


% ─────────────────────────────────────────────────────────────────────────
%  Magnetorquer Assembly
% ─────────────────────────────────────────────────────────────────────────
function s = mtq_assembly_code()
    L = {
    'function torque_body = fcn(dipole_cmd, B_body)'
    '%#codegen'
    '% Compile-stable zero-torque fallback.'
    '  torque_body = [0; 0; 0];'
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
    '% Compile-stable zero-torque fallback.'
    '  torque_body = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end
