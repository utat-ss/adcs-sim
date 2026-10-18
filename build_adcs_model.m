function build_adcs_model()
%BUILD_ADCS_MODEL  Programmatically create the ADCS Simulator Simulink model.
%
%   build_adcs_model()
%
%   Generates 'adcs_sim.slx' with the following hierarchy:
%
%   TOP LEVEL
%   ├── ENVIRONMENT    (builders/build_environment.m)
%   │   ├── Ephemeris_Truth, Orbit_Propagator, Magnetic_Field_Model, Eclipse_Model
%   │
%   ├── SENSORS        (builders/build_sensors.m)
%   │   ├── Sun_Sensor_Suite, Star_Tracker, IMU, Magnetometer, GNSS
%   │
%   ├── CONTROL        (builders/build_control.m)
%   │   ├── Onboard_Ephemeris, Attitude_Estimator, Reference_Generator, Control_Law
%   │
%   ├── ACTUATORS      (builders/build_actuators.m)
%   │   ├── RW_Assembly, MTQ_Assembly, CMG
%   │
%   ├── DYNAMICS       (builders/build_dynamics.m)
%   │   ├── Euler_RHS, Gravity_Gradient, Aero_Drag_Torque,
%   │   │   SRP_Torque, Residual_Mag_Torque, Quat_RHS, Quat_Norm, Integrators
%   │
%   ├── Attitude_Scope, Rates_Scope
%   └── To Workspace logging for state, environment, command, and torque paths
%
%   Feedback loops:
%       DYNAMICS.q_out     → SENSORS.q_in
%       DYNAMICS.omega_out → SENSORS.omega_in
%
%   Forward path:
%       ENVIRONMENT → SENSORS → CONTROL → ACTUATORS → DYNAMICS
%
%   Note: if Simulink cannot find blocks at the paths used below,
%   replace 'simulink/Sources/In1' with 'simulink/Commonly Used Blocks/In1',
%   and  'simulink/Sinks/Out1'  with 'simulink/Commonly Used Blocks/Out1'.
%   The paths used here work from R2016b onward.

%  ───────────────────────────────────────────────────────────────────────
%  0.  HOUSEKEEPING — add builders/ and utils/ to path
%  ───────────────────────────────────────────────────────────────────────
thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir, 'utils'));
addpath(fullfile(thisDir, 'builders'));

mdl = 'adcs_sim';
mdl_file = fullfile(thisDir, [mdl '.slx']);
mdl_cache_file = fullfile(thisDir, [mdl '.slxc']);

if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(mdl_cache_file), delete(mdl_cache_file); end

if isfile(mdl_file)
    load_system(mdl_file);
    Simulink.BlockDiagram.deleteContents(mdl);
else
    new_system(mdl);
end

open_system(mdl);

set_param(mdl, ...
    'Solver',    'ode4', ...
    'FixedStep', 'dt',   ...
    'StopTime',  't_end');

ar = {'autorouting','on'};          % shorthand used everywhere

%  ───────────────────────────────────────────────────────────────────────
%  1.  TOP-LEVEL BLOCKS
%  ───────────────────────────────────────────────────────────────────────
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/ENVIRONMENT'], ...
    'Position', [80  200  230  460]);
csub([mdl '/ENVIRONMENT']);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/SENSORS'], ...
    'Position', [380  60  530  560]);
csub([mdl '/SENSORS']);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/CONTROL'], ...
    'Position', [700  100  850  460]);
csub([mdl '/CONTROL']);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/ACTUATORS'], ...
    'Position', [1020  160  1170  420]);
csub([mdl '/ACTUATORS']);

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/DYNAMICS'], ...
    'Position', [1340  140  1490  400]);
csub([mdl '/DYNAMICS']);

add_block('simulink/Sinks/Scope', [mdl '/Attitude_Scope'], ...
    'Position', [1600  180  1650  220]);
add_block('simulink/Sinks/Scope', [mdl '/Rates_Scope'], ...
    'Position', [1600  330  1650  370]);
add_block('simulink/Sinks/To Workspace', [mdl '/Q_To_Workspace'], ...
    'Position', [1600  240  1700  270], 'VariableName', 'q_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Omega_To_Workspace'], ...
    'Position', [1600  390  1700  420], 'VariableName', 'omega_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/HW_To_Workspace'], ...
    'Position', [1600  500  1700  530], 'VariableName', 'h_W_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Sun_ECI_To_Workspace'], ...
    'Position', [250  520  370  550], 'VariableName', 'sun_eci_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Moon_ECI_To_Workspace'], ...
    'Position', [250  560  370  590], 'VariableName', 'moon_eci_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/B_ECI_To_Workspace'], ...
    'Position', [250  600  370  630], 'VariableName', 'B_eci_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Pos_ECI_To_Workspace'], ...
    'Position', [250  640  370  670], 'VariableName', 'pos_eci_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Vel_ECI_To_Workspace'], ...
    'Position', [250  680  370  710], 'VariableName', 'vel_eci_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Eclipse_To_Workspace'], ...
    'Position', [250  720  370  750], 'VariableName', 'eclipse_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Q_Meas_To_Workspace'], ...
    'Position', [550  600  670  630], 'VariableName', 'q_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Omega_Meas_To_Workspace'], ...
    'Position', [550  640  670  670], 'VariableName', 'omega_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Sun_Meas_To_Workspace'], ...
    'Position', [550  680  670  710], 'VariableName', 'sun_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/B_Body_Meas_To_Workspace'], ...
    'Position', [550  720  670  750], 'VariableName', 'B_body_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Pos_Meas_To_Workspace'], ...
    'Position', [550  760  670  790], 'VariableName', 'pos_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Vel_Meas_To_Workspace'], ...
    'Position', [550  800  670  830], 'VariableName', 'vel_meas_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/RW_Command_To_Workspace'], ...
    'Position', [870  500  990  530], 'VariableName', 'rw_cmd_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/MTQ_Command_To_Workspace'], ...
    'Position', [870  540  990  570], 'VariableName', 'mtq_cmd_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/CMG_Command_To_Workspace'], ...
    'Position', [870  580  990  610], 'VariableName', 'cmg_cmd_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/Ext_Torque_To_Workspace'], ...
    'Position', [1190  450  1310  480], 'VariableName', 'ext_torque_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/RW_Body_Torque_To_Workspace'], ...
    'Position', [1190  490  1310  520], 'VariableName', 'rw_torque_body_log', 'SaveFormat', 'Timeseries');
add_block('simulink/Sinks/To Workspace', [mdl '/RW_Wheel_Torque_To_Workspace'], ...
    'Position', [1190  530  1310  560], 'VariableName', 'rw_tau_wheels_log', 'SaveFormat', 'Timeseries');

%  ───────────────────────────────────────────────────────────────────────
%  2.  BUILD EACH SUBSYSTEM  (delegated to builders/)
%  ───────────────────────────────────────────────────────────────────────
build_environment([mdl '/ENVIRONMENT']);
build_sensors([mdl '/SENSORS']);
build_control([mdl '/CONTROL']);
build_actuators([mdl '/ACTUATORS']);
build_dynamics([mdl '/DYNAMICS']);

%  ───────────────────────────────────────────────────────────────────────
%  3.  TOP-LEVEL SIGNAL ROUTING
%  ───────────────────────────────────────────────────────────────────────
%
%  ENVIRONMENT output port mapping:
%    1 = Sun_vec_ECI   2 = Moon_vec_ECI   3 = B_ECI
%    4 = pos_ECI       5 = vel_ECI        6 = eclipse_flag
%
%  SENSORS input port mapping:
%    1 = q_in          2 = omega_in       3 = Sun_ECI
%    4 = Moon_ECI      5 = B_ECI          6 = eclipse_flag
%    7 = pos_ECI       8 = vel_ECI
%
%  SENSORS output port mapping:
%    1 = q_meas        2 = omega_meas     3 = sun_meas
%    4 = B_body_meas   5 = pos_meas       6 = vel_meas
%    7 = tracker_valid, when exposed by the sensor builder
%
%  CONTROL input ports  1-6 match SENSORS output ports 1-6
%  CONTROL input port   7 = h_W_in
%  CONTROL input port   8 = tracker_valid, when exposed by the control builder
%  CONTROL output ports 1 = rw_torque_cmd  2 = mtq_dipole_cmd  3 = cmg_gimbal_cmd
%
%  ACTUATORS input ports 1-3 match CONTROL output ports 1-3
%  ACTUATORS input port  4 = B_body_in
%  ACTUATORS input port  5 = h_W_in, when exposed by the actuator builder
%  ACTUATORS output ports: 1 = ext_torque  2 = rw_torque_body  3 = rw_tau_wheels
%
%  DYNAMICS input ports:
%    1 = Torque_ext      2 = rw_torque_body   3 = rw_tau_wheels   4 = pos_ECI
%    5 = vel_ECI         6 = Sun_ECI          7 = eclipse_flag    8 = B_ECI
%  DYNAMICS output ports: 1 = q_out   2 = omega_out   3 = h_W_out

% ── ENVIRONMENT → SENSORS ──
add_line(mdl, 'ENVIRONMENT/1', 'SENSORS/3', ar{:});   % Sun_vec_ECI → Sun_ECI
add_line(mdl, 'ENVIRONMENT/2', 'SENSORS/4', ar{:});   % Moon_vec_ECI → Moon_ECI
add_line(mdl, 'ENVIRONMENT/3', 'SENSORS/5', ar{:});   % B_ECI → B_ECI
add_line(mdl, 'ENVIRONMENT/4', 'SENSORS/7', ar{:});   % pos_ECI → pos_ECI
add_line(mdl, 'ENVIRONMENT/5', 'SENSORS/8', ar{:});   % vel_ECI → vel_ECI
add_line(mdl, 'ENVIRONMENT/6', 'SENSORS/6', ar{:});   % eclipse_flag → eclipse_flag
add_line(mdl, 'ENVIRONMENT/1', 'Sun_ECI_To_Workspace/1',  ar{:});
add_line(mdl, 'ENVIRONMENT/2', 'Moon_ECI_To_Workspace/1', ar{:});
add_line(mdl, 'ENVIRONMENT/3', 'B_ECI_To_Workspace/1',    ar{:});
add_line(mdl, 'ENVIRONMENT/4', 'Pos_ECI_To_Workspace/1',  ar{:});
add_line(mdl, 'ENVIRONMENT/5', 'Vel_ECI_To_Workspace/1',  ar{:});
add_line(mdl, 'ENVIRONMENT/6', 'Eclipse_To_Workspace/1',  ar{:});

% ── SENSORS → CONTROL  (1:1 mapping, ports 1-6) ──
for k = 1:6
    add_line(mdl, ['SENSORS/' num2str(k)], ['CONTROL/' num2str(k)], ar{:});
end
add_line(mdl, 'SENSORS/1', 'Q_Meas_To_Workspace/1',      ar{:});
add_line(mdl, 'SENSORS/2', 'Omega_Meas_To_Workspace/1',  ar{:});
add_line(mdl, 'SENSORS/3', 'Sun_Meas_To_Workspace/1',    ar{:});
add_line(mdl, 'SENSORS/4', 'B_Body_Meas_To_Workspace/1', ar{:});
add_line(mdl, 'SENSORS/5', 'Pos_Meas_To_Workspace/1',    ar{:});
add_line(mdl, 'SENSORS/6', 'Vel_Meas_To_Workspace/1',    ar{:});

sensor_ports = get_param([mdl '/SENSORS'], 'Ports');
control_ports = get_param([mdl '/CONTROL'], 'Ports');
if sensor_ports(2) >= 7
    add_block('simulink/Sinks/To Workspace', [mdl '/Tracker_Valid_To_Workspace'], ...
        'Position', [550  840  670  870], 'VariableName', 'tracker_valid_log', ...
        'SaveFormat', 'Timeseries');
    add_line(mdl, 'SENSORS/7', 'Tracker_Valid_To_Workspace/1', ar{:});
    if control_ports(1) >= 8
        add_line(mdl, 'SENSORS/7', 'CONTROL/8', ar{:});
    end
end

% ── DYNAMICS → CONTROL (wheel momentum feedback) ──
add_line(mdl, 'DYNAMICS/3', 'CONTROL/7', ar{:});       % h_W_out → h_W_in

% ── CONTROL → ACTUATORS  (1:1 mapping, ports 1-3) ──
for k = 1:3
    add_line(mdl, ['CONTROL/' num2str(k)], ['ACTUATORS/' num2str(k)], ar{:});
end
add_line(mdl, 'CONTROL/1', 'RW_Command_To_Workspace/1',  ar{:});
add_line(mdl, 'CONTROL/2', 'MTQ_Command_To_Workspace/1', ar{:});
add_line(mdl, 'CONTROL/3', 'CMG_Command_To_Workspace/1', ar{:});

% ── SENSORS → ACTUATORS (B-body feedback for MTQ) ──
add_line(mdl, 'SENSORS/4', 'ACTUATORS/4', ar{:});     % B_body_meas → B_body_in

actuator_ports = get_param([mdl '/ACTUATORS'], 'Ports');
if actuator_ports(1) >= 5
    add_line(mdl, 'DYNAMICS/3', 'ACTUATORS/5', ar{:}); % h_W_out → h_W_in
end

% ── ACTUATORS → DYNAMICS ──
add_line(mdl, 'ACTUATORS/1', 'DYNAMICS/1', ar{:});    % ext_torque → Torque_ext
add_line(mdl, 'ACTUATORS/2', 'DYNAMICS/2', ar{:});    % rw_torque_body → rw_torque_body
add_line(mdl, 'ACTUATORS/3', 'DYNAMICS/3', ar{:});    % rw_tau_wheels → rw_tau_wheels
add_line(mdl, 'ACTUATORS/1', 'Ext_Torque_To_Workspace/1',     ar{:});
add_line(mdl, 'ACTUATORS/2', 'RW_Body_Torque_To_Workspace/1', ar{:});
add_line(mdl, 'ACTUATORS/3', 'RW_Wheel_Torque_To_Workspace/1', ar{:});
add_line(mdl, 'ENVIRONMENT/4', 'DYNAMICS/4', ar{:});  % pos_ECI → pos_ECI
add_line(mdl, 'ENVIRONMENT/5', 'DYNAMICS/5', ar{:});  % vel_ECI → vel_ECI
add_line(mdl, 'ENVIRONMENT/1', 'DYNAMICS/6', ar{:});  % Sun_vec_ECI → Sun_ECI
add_line(mdl, 'ENVIRONMENT/6', 'DYNAMICS/7', ar{:});  % eclipse_flag → eclipse_flag
add_line(mdl, 'ENVIRONMENT/3', 'DYNAMICS/8', ar{:});  % B_ECI → B_ECI

% ── DYNAMICS feedback → SENSORS ──
add_line(mdl, 'DYNAMICS/1', 'SENSORS/1', ar{:});       % q_out → q_in
add_line(mdl, 'DYNAMICS/2', 'SENSORS/2', ar{:});       % omega_out → omega_in

% ── DYNAMICS → Scopes  (branches from same outputs) ──
add_line(mdl, 'DYNAMICS/1', 'Attitude_Scope/1', ar{:});
add_line(mdl, 'DYNAMICS/2', 'Rates_Scope/1',    ar{:});

% ── DYNAMICS → To Workspace logging ──
add_line(mdl, 'DYNAMICS/1', 'Q_To_Workspace/1',     ar{:});
add_line(mdl, 'DYNAMICS/2', 'Omega_To_Workspace/1', ar{:});
add_line(mdl, 'DYNAMICS/3', 'HW_To_Workspace/1',    ar{:});

%  ───────────────────────────────────────────────────────────────────────
%  4.  SAVE
%  ───────────────────────────────────────────────────────────────────────
save_system(mdl, mdl_file);
fprintf('Model "%s.slx" created and saved successfully.\n', mdl);
fprintf('Run init_adcs_params before simulating.\n');

end  % build_adcs_model
