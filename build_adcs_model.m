function build_adcs_model()
%BUILD_ADCS_MODEL  Programmatically create the ADCS Simulator Simulink model.
%
%   build_adcs_model()
%
%   Generates 'adcs_sim.slx' with the following hierarchy:
%
%   TOP LEVEL
%   ├── ENVIRONMENT
%   │   ├── Ephemeris_Truth         → Sun_vec_ECI, Moon_vec_ECI
%   │   ├── Orbit_Propagator        → pos_ECI, vel_ECI
%   │   ├── Magnetic_Field_Model    → B_ECI       (input: pos_ECI)
%   │   └── Eclipse_Model           → eclipse_flag (inputs: pos_ECI, Sun_vec_ECI)
%   │
%   ├── SENSORS
%   │   ├── FSS_1 … FSS_6          → fss_raw  (inputs: Sun_ECI, q_true, eclipse_flag)
%   │   ├── Sun_Sensor_Processing   → sun_meas (inputs: fss_1…6_raw)
%   │   ├── Star_Tracker            → q_meas   (inputs: Sun_ECI, Moon_ECI, q_true)
%   │   ├── IMU                     → omega_meas (input: omega_true)
%   │   ├── Magnetometer            → B_body_meas (inputs: B_ECI, q_true)
%   │   └── GNSS                    → pos_meas, vel_meas (inputs: pos_ECI, vel_ECI)
%   │
%   ├── CONTROL
%   │   ├── Onboard_Ephemeris       → Sun_predicted, Mag_predicted
%   │   ├── Attitude_Estimator      → q_est, omega_est
%   │   └── Control_Law             → rw_torque_cmd, mtq_dipole_cmd, cmg_gimbal_cmd
%   │
%   ├── ACTUATORS
%   │   ├── RW_1 … RW_4            → torque_out  (input: torque_cmd)
%   │   ├── MTQ_X, MTQ_Y, MTQ_Z    → torque_out  (input: dipole_cmd)
%   │   ├── CMG                     → torque_out  (input: gimbal_cmd)
%   │   └── Torque_Sum              (sums all 8 actuator torques)
%   │
%   ├── DYNAMICS                    (placeholder: constant q, omega)
%   │   → q_out  [4x1]
%   │   → omega_out [3x1]
%   │
%   ├── Attitude_Scope
%   └── Rates_Scope
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
%  0.  HOUSEKEEPING
%  ───────────────────────────────────────────────────────────────────────
mdl = 'adcs_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'Solver',    'ode4', ...
    'FixedStep', '0.1',  ...
    'StopTime',  '6000');

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

%  ───────────────────────────────────────────────────────────────────────
%  2.  BUILD EACH SUBSYSTEM
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
%
%  CONTROL input ports  1-6 match SENSORS output ports 1-6
%  CONTROL output ports 1 = rw_torque_cmd  2 = mtq_dipole_cmd  3 = cmg_gimbal_cmd
%
%  ACTUATORS input ports 1-3 match CONTROL output ports 1-3
%  ACTUATORS output port 1 = torque_out
%
%  DYNAMICS input port 1 = Torque_in
%  DYNAMICS output ports 1 = q_out   2 = omega_out

% ── ENVIRONMENT → SENSORS ──
add_line(mdl, 'ENVIRONMENT/1', 'SENSORS/3', ar{:});   % Sun_vec_ECI → Sun_ECI
add_line(mdl, 'ENVIRONMENT/2', 'SENSORS/4', ar{:});   % Moon_vec_ECI → Moon_ECI
add_line(mdl, 'ENVIRONMENT/3', 'SENSORS/5', ar{:});   % B_ECI → B_ECI
add_line(mdl, 'ENVIRONMENT/4', 'SENSORS/7', ar{:});   % pos_ECI → pos_ECI
add_line(mdl, 'ENVIRONMENT/5', 'SENSORS/8', ar{:});   % vel_ECI → vel_ECI
add_line(mdl, 'ENVIRONMENT/6', 'SENSORS/6', ar{:});   % eclipse_flag → eclipse_flag

% ── SENSORS → CONTROL  (1:1 mapping, ports 1-6) ──
for k = 1:6
    add_line(mdl, ['SENSORS/' num2str(k)], ['CONTROL/' num2str(k)], ar{:});
end

% ── CONTROL → ACTUATORS  (1:1 mapping, ports 1-3) ──
for k = 1:3
    add_line(mdl, ['CONTROL/' num2str(k)], ['ACTUATORS/' num2str(k)], ar{:});
end

% ── ACTUATORS → DYNAMICS ──
add_line(mdl, 'ACTUATORS/1', 'DYNAMICS/1', ar{:});    % torque_out → Torque_in

% ── DYNAMICS feedback → SENSORS ──
add_line(mdl, 'DYNAMICS/1', 'SENSORS/1', ar{:});       % q_out → q_in
add_line(mdl, 'DYNAMICS/2', 'SENSORS/2', ar{:});       % omega_out → omega_in

% ── DYNAMICS → Scopes  (branches from same outputs) ──
add_line(mdl, 'DYNAMICS/1', 'Attitude_Scope/1', ar{:});
add_line(mdl, 'DYNAMICS/2', 'Rates_Scope/1',    ar{:});

%  ───────────────────────────────────────────────────────────────────────
%  4.  SAVE
%  ───────────────────────────────────────────────────────────────────────
save_system(mdl);
fprintf('Model "%s.slx" created and saved successfully.\n', mdl);
fprintf('Run init_adcs_params before simulating.\n');

end  % build_adcs_model


% ═══════════════════════════════════════════════════════════════════════════
%                         L O C A L   H E L P E R S
% ═══════════════════════════════════════════════════════════════════════════

function csub(path)
%CSUB  Clear the default In1→Out1 inside a freshly-added Subsystem block.
    try delete_line(path, 'In1/1', 'Out1/1'); catch, end
    try delete_block([path '/In1']);           catch, end
    try delete_block([path '/Out1']);          catch, end
end

% ─────────────────────────────────────────────────────────────────────────
function leaf(parent, name, ins, outs, pos)
%LEAF  Create a stub (blank) subsystem with named inports and outports.
%      Inputs are terminated internally; outputs are driven by Ground blocks.
    fp = [parent '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', fp, 'Position', pos);
    csub(fp);

    sp = 45;                       % vertical pixel spacing
    for i = 1:numel(ins)
        y = 30 + sp*(i-1);
        add_block('simulink/Sources/In1', ...
            [fp '/' ins{i}], ...
            'Position', [50  y  80  y+14], 'Port', num2str(i));
        add_block('simulink/Sinks/Terminator', ...
            [fp '/Term_' num2str(i)], ...
            'Position', [160 y  180 y+14]);
        add_line(fp, [ins{i} '/1'], ['Term_' num2str(i) '/1']);
    end
    for i = 1:numel(outs)
        y = 30 + sp*(i-1);
        add_block('simulink/Sources/Ground', ...
            [fp '/Gnd_' num2str(i)], ...
            'Position', [260 y  280 y+14]);
        add_block('simulink/Sinks/Out1', ...
            [fp '/' outs{i}], ...
            'Position', [360 y  390 y+14], 'Port', num2str(i));
        add_line(fp, ['Gnd_' num2str(i) '/1'], [outs{i} '/1']);
    end
end

% ─────────────────────────────────────────────────────────────────────────
function add_inports(sys, names, x, y0, dy)
%ADD_INPORTS  Add a column of Inport blocks.
    for i = 1:numel(names)
        y = y0 + dy*(i-1);
        add_block('simulink/Sources/In1', [sys '/' names{i}], ...
            'Position', [x  y  x+30  y+14], 'Port', num2str(i));
    end
end

% ─────────────────────────────────────────────────────────────────────────
function add_outports(sys, names, x, y0, dy)
%ADD_OUTPORTS  Add a column of Outport blocks.
    for i = 1:numel(names)
        y = y0 + dy*(i-1);
        add_block('simulink/Sinks/Out1', [sys '/' names{i}], ...
            'Position', [x  y  x+30  y+14], 'Port', num2str(i));
    end
end


% ═══════════════════════════════════════════════════════════════════════════
%                     E N V I R O N M E N T
% ═══════════════════════════════════════════════════════════════════════════
function build_environment(sys)
    ar = {'autorouting','on'};

    % ── Outports (parent-level output interface) ──
    %  1: Sun_vec_ECI   2: Moon_vec_ECI   3: B_ECI
    %  4: pos_ECI       5: vel_ECI        6: eclipse_flag
    add_outports(sys, ...
        {'Sun_vec_ECI','Moon_vec_ECI','B_ECI','pos_ECI','vel_ECI','eclipse_flag'}, ...
        700, 40, 70);

    % ── Time sources ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [30  120  60  140]);

    % ── Ephemeris data arrays (loaded from CSV in init_adcs_params) ──
    add_block('simulink/Sources/Constant', [sys '/Ephem_t'], ...
        'Position', [30  30  110  50], 'Value', 'ephem_t_sec');
    add_block('simulink/Sources/Constant', [sys '/Ephem_sun'], ...
        'Position', [30  70  110  90], 'Value', 'ephem_sun');
    add_block('simulink/Sources/Constant', [sys '/Ephem_moon'], ...
        'Position', [30  110  110  130], 'Value', 'ephem_moon');

    % ── Ephemeris_Truth  (CSV-based interpolation) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Ephemeris_Truth'], 'Position', [200  30  400  140]);

    % ── Orbit_Propagator  (Keplerian 2-body from orbital elements) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Orbit_Propagator'], 'Position', [200  180  400  250]);

    % ── Magnetic_Field_Model  (tilted-dipole IGRF approximation) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Magnetic_Field_Model'], 'Position', [200  290  400  350]);

    % ── Eclipse_Model  (cylindrical Earth shadow) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Eclipse_Model'], 'Position', [200  390  400  460]);

    % ── Set MATLAB Function block scripts ──
    set_mfb_script([sys '/Ephemeris_Truth'],     ephemeris_truth_code());
    set_mfb_script([sys '/Orbit_Propagator'],    orbit_propagator_code());
    set_mfb_script([sys '/Magnetic_Field_Model'],magnetic_field_code());
    set_mfb_script([sys '/Eclipse_Model'],       eclipse_model_code());

    % ── Wiring ──
    % Clock + ephemeris data → Ephemeris_Truth (4 inputs: t, ephem_t, sun_data, moon_data)
    add_line(sys, 'Clock/1',      'Ephemeris_Truth/1', ar{:});
    add_line(sys, 'Ephem_t/1',    'Ephemeris_Truth/2', ar{:});
    add_line(sys, 'Ephem_sun/1',  'Ephemeris_Truth/3', ar{:});
    add_line(sys, 'Ephem_moon/1', 'Ephemeris_Truth/4', ar{:});
    add_line(sys, 'Clock/1',      'Orbit_Propagator/1', ar{:});

    % Ephemeris → outports
    add_line(sys, 'Ephemeris_Truth/1', 'Sun_vec_ECI/1',  ar{:});
    add_line(sys, 'Ephemeris_Truth/2', 'Moon_vec_ECI/1', ar{:});

    % Orbit Propagator → outports
    add_line(sys, 'Orbit_Propagator/1', 'pos_ECI/1', ar{:});
    add_line(sys, 'Orbit_Propagator/2', 'vel_ECI/1', ar{:});

    % pos_ECI branch → Magnetic_Field_Model
    add_line(sys, 'Orbit_Propagator/1', 'Magnetic_Field_Model/1', ar{:});
    add_line(sys, 'Magnetic_Field_Model/1', 'B_ECI/1',            ar{:});

    % pos_ECI + Sun_vec → Eclipse_Model
    add_line(sys, 'Orbit_Propagator/1', 'Eclipse_Model/1', ar{:});
    add_line(sys, 'Ephemeris_Truth/1',  'Eclipse_Model/2', ar{:});
    add_line(sys, 'Eclipse_Model/1',    'eclipse_flag/1',  ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%                          S E N S O R S
%  Fine Sun Sensors 1-6, Star Tracker, IMU, Magnetometer, GNSS
%  Each sensor has: coordinate transform, noise, and (where applicable)
%  sample-and-hold, FOV check, or exclusion logic.
% ═══════════════════════════════════════════════════════════════════════════
function build_sensors(sys)
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

    % ── Sun Sensor Suite (single MATLAB Function block for all 6 FSS) ──
    %    Inputs: Sun_ECI, q_true, eclipse_flag
    %    Output: sun_meas [3x1] (unit Sun vector in body frame, with noise)
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Sun_Sensor_Suite'], 'Position', [250  20  480  110]);
    set_mfb_script([sys '/Sun_Sensor_Suite'], sun_sensor_suite_code());

    % ── Star Tracker ──
    %    Inputs: Sun_ECI, Moon_ECI, q_true
    %    Output: q_meas [4x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Star_Tracker'], 'Position', [250  150  480  240]);
    set_mfb_script([sys '/Star_Tracker'], star_tracker_code());

    % ── IMU ──
    %    Input: omega_true [3x1]
    %    Output: omega_meas [3x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/IMU'], 'Position', [250  280  480  340]);
    set_mfb_script([sys '/IMU'], imu_code());

    % ── Magnetometer ──
    %    Inputs: B_ECI [3x1], q_true [4x1]
    %    Output: B_body_meas [3x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Magnetometer'], 'Position', [250  380  480  450]);
    set_mfb_script([sys '/Magnetometer'], magnetometer_code());

    % ── GNSS ──
    %    Inputs: pos_ECI [3x1], vel_ECI [3x1]
    %    Output: pos_meas [3x1], vel_meas [3x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/GNSS'], 'Position', [250  490  480  570]);
    set_mfb_script([sys '/GNSS'], gnss_code());

    % ══════════════════════════════════════════════════════════════════════
    %  WIRING
    % ══════════════════════════════════════════════════════════════════════

    % Sun_ECI, q_in, eclipse_flag → Sun_Sensor_Suite
    add_line(sys, 'Sun_ECI/1',      'Sun_Sensor_Suite/1', ar{:});
    add_line(sys, 'q_in/1',         'Sun_Sensor_Suite/2', ar{:});
    add_line(sys, 'eclipse_flag/1', 'Sun_Sensor_Suite/3', ar{:});

    % Sun_ECI, Moon_ECI, q_in → Star_Tracker
    add_line(sys, 'Sun_ECI/1',  'Star_Tracker/1', ar{:});
    add_line(sys, 'Moon_ECI/1', 'Star_Tracker/2', ar{:});
    add_line(sys, 'q_in/1',    'Star_Tracker/3', ar{:});

    % omega_in → IMU
    add_line(sys, 'omega_in/1', 'IMU/1', ar{:});

    % B_ECI, q_in → Magnetometer
    add_line(sys, 'B_ECI/1', 'Magnetometer/1', ar{:});
    add_line(sys, 'q_in/1', 'Magnetometer/2',  ar{:});

    % pos_ECI, vel_ECI → GNSS
    add_line(sys, 'pos_ECI/1', 'GNSS/1', ar{:});
    add_line(sys, 'vel_ECI/1', 'GNSS/2', ar{:});

    % Outputs → outports
    add_line(sys, 'Star_Tracker/1',      'q_meas/1',      ar{:});
    add_line(sys, 'IMU/1',               'omega_meas/1',  ar{:});
    add_line(sys, 'Sun_Sensor_Suite/1',  'sun_meas/1',    ar{:});
    add_line(sys, 'Magnetometer/1',      'B_body_meas/1', ar{:});
    add_line(sys, 'GNSS/1',             'pos_meas/1',    ar{:});
    add_line(sys, 'GNSS/2',             'vel_meas/1',    ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%                          C O N T R O L
%  Onboard ephemeris (simplified, intentionally different from truth),
%  TRIAD attitude estimator, and PD + B-dot control law.
% ═══════════════════════════════════════════════════════════════════════════
function build_control(sys)
    ar = {'autorouting','on'};

    % ── Inports (6) ──
    add_inports(sys, ...
        {'q_meas','omega_meas','sun_meas','B_body_meas','pos_meas','vel_meas'}, ...
        30, 50, 60);

    % ── Outports (3) ──
    add_outports(sys, ...
        {'rw_torque_cmd','mtq_dipole_cmd','cmg_gimbal_cmd'}, ...
        750, 80, 120);

    % ── Simulation time for onboard ephemeris ──
    add_block('simulink/Sources/Clock', [sys '/Clock'], ...
        'Position', [30  420  60  440]);
    add_block('simulink/Sources/Constant', [sys '/Epoch_JD'], ...
        'Position', [30  470  110  490], 'Value', 'epoch_jd');

    % ── Onboard Ephemeris (simplified — deliberately lower fidelity) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Onboard_Ephemeris'], 'Position', [180  400  380  500]);
    set_mfb_script([sys '/Onboard_Ephemeris'], onboard_ephemeris_code());

    % ── Attitude Estimator (TRIAD method) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Attitude_Estimator'], 'Position', [180  120  400  320]);
    set_mfb_script([sys '/Attitude_Estimator'], triad_estimator_code());

    % ── Control Law (PD for RW + B-dot for MTQ) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Control_Law'], 'Position', [480  60  700  280]);
    set_mfb_script([sys '/Control_Law'], control_law_code());

    % ── Wiring ──
    % Clock + Epoch → Onboard Ephemeris
    add_line(sys, 'Clock/1',    'Onboard_Ephemeris/1', ar{:});
    add_line(sys, 'Epoch_JD/1', 'Onboard_Ephemeris/2', ar{:});

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

    % B_body_meas → Control Law input 3 (for B-dot magnetorquer control)
    add_line(sys, 'B_body_meas/1', 'Control_Law/3', ar{:});

    % Control Law → outports
    add_line(sys, 'Control_Law/1', 'rw_torque_cmd/1',   ar{:});
    add_line(sys, 'Control_Law/2', 'mtq_dipole_cmd/1',  ar{:});
    add_line(sys, 'Control_Law/3', 'cmg_gimbal_cmd/1',  ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%                        A C T U A T O R S
%  Reaction wheels with saturation + momentum tracking,
%  magnetorquers with dipole saturation, and a placeholder CMG.
% ═══════════════════════════════════════════════════════════════════════════
function build_actuators(sys)
    ar = {'autorouting','on'};

    % ── Inports (3) ──
    add_inports(sys, {'rw_torque_cmd','mtq_dipole_cmd','cmg_gimbal_cmd'}, ...
        30, 50, 120);

    % ── Outport (1) ──
    add_outports(sys, {'torque_out'}, 850, 250, 0);

    % ── Reaction Wheel Assembly (MATLAB Function block) ──
    %  Applies per-wheel torque saturation, maps through pyramid geometry,
    %  and tracks stored angular momentum.
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/RW_Assembly'], 'Position', [220  20  420  100]);
    set_mfb_script([sys '/RW_Assembly'], rw_assembly_code());

    % ── Magnetorquer Assembly (MATLAB Function block) ──
    %  Saturates dipole commands and computes torque = m × B.
    %  Needs B_ECI in body frame — for now uses a unit vector placeholder;
    %  later feed B_body from SENSORS via additional inport.
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/MTQ_Assembly'], 'Position', [220  140  420  230]);
    set_mfb_script([sys '/MTQ_Assembly'], mtq_assembly_code());

    % ── CMG (MATLAB Function block, placeholder) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/CMG'], 'Position', [220  280  420  350]);
    set_mfb_script([sys '/CMG'], cmg_code());

    % ── Torque Sum (3 inputs: RW + MTQ + CMG = net torque 3x1) ──
    add_block('simulink/Math Operations/Sum', [sys '/Torque_Sum'], ...
        'Position', [550  160  580  260], 'Inputs', '+++');

    % ── Wiring ──
    add_line(sys, 'rw_torque_cmd/1',   'RW_Assembly/1',  ar{:});
    add_line(sys, 'mtq_dipole_cmd/1',  'MTQ_Assembly/1', ar{:});
    add_line(sys, 'cmg_gimbal_cmd/1',  'CMG/1',          ar{:});

    add_line(sys, 'RW_Assembly/1',  'Torque_Sum/1', ar{:});
    add_line(sys, 'MTQ_Assembly/1', 'Torque_Sum/2', ar{:});
    add_line(sys, 'CMG/1',          'Torque_Sum/3', ar{:});

    add_line(sys, 'Torque_Sum/1', 'torque_out/1', ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%                         D Y N A M I C S
%  Implements rigid-body rotational dynamics:
%    J * omega_dot = Torque_ext - omega x (J * omega)   (Euler equation)
%    q_dot = 0.5 * Omega(omega) * q                     (quaternion kinematics)
%  with quaternion renormalisation at every step.
% ═══════════════════════════════════════════════════════════════════════════
function build_dynamics(sys)
    ar = {'autorouting','on'};

    % ── Inport ──
    add_block('simulink/Sources/In1', [sys '/Torque_in'], ...
        'Position', [30  200  60  214], 'Port', '1');

    % ── Outports ──
    add_outports(sys, {'q_out','omega_out'}, 900, 80, 200);

    % ── Inertia tensor from workspace ──
    add_block('simulink/Sources/Constant', [sys '/J_param'], ...
        'Position', [30  300  110  320], 'Value', 'J');

    % ── Euler equation RHS  (MATLAB Function block) ──
    %    inputs:  torque [3x1], omega [3x1], J [3x3]
    %    output:  omega_dot [3x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Euler_RHS'], 'Position', [250  160  420  260]);
    set_mfb_script([sys '/Euler_RHS'], euler_rhs_code());

    % ── Integrator for omega ──
    add_block('simulink/Continuous/Integrator', [sys '/Integ_omega'], ...
        'Position', [500  190  540  230], ...
        'InitialCondition', 'omega0');

    % ── Quaternion kinematics RHS  (MATLAB Function block) ──
    %    inputs:  omega [3x1], q [4x1]
    %    output:  q_dot [4x1]
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Quat_RHS'], 'Position', [250  30  420  110]);
    set_mfb_script([sys '/Quat_RHS'], quat_rhs_code());

    % ── Integrator for q ──
    add_block('simulink/Continuous/Integrator', [sys '/Integ_q'], ...
        'Position', [500  50  540  90], ...
        'InitialCondition', 'q0');

    % ── Quaternion normalisation (MATLAB Function block) ──
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [sys '/Quat_Norm'], 'Position', [620  50  740  90]);
    set_mfb_script([sys '/Quat_Norm'], quat_norm_code());

    % ── Set MATLAB Function block scripts ──
    % (already done above via set_mfb_script calls)

    % ═════════════  W I R I N G  ═════════════

    % Torque_in → Euler_RHS input 1
    add_line(sys, 'Torque_in/1', 'Euler_RHS/1', ar{:});

    % J_param → Euler_RHS input 3
    add_line(sys, 'J_param/1', 'Euler_RHS/3', ar{:});

    % Euler_RHS → Integrator_omega → omega signal
    add_line(sys, 'Euler_RHS/1',    'Integ_omega/1', ar{:});
    add_line(sys, 'Integ_omega/1',  'omega_out/1',   ar{:});

    % omega feedback → Euler_RHS input 2
    add_line(sys, 'Integ_omega/1', 'Euler_RHS/2', ar{:});

    % omega → Quat_RHS input 1
    add_line(sys, 'Integ_omega/1', 'Quat_RHS/1', ar{:});

    % Quat_RHS → Integrator_q → Normalise → q_out
    add_line(sys, 'Quat_RHS/1',   'Integ_q/1',   ar{:});
    add_line(sys, 'Integ_q/1',    'Quat_Norm/1', ar{:});
    add_line(sys, 'Quat_Norm/1',  'q_out/1',     ar{:});

    % q feedback → Quat_RHS input 2
    add_line(sys, 'Quat_Norm/1',  'Quat_RHS/2', ar{:});
end


% ═══════════════════════════════════════════════════════════════════════════
%          M A T L A B   F U N C T I O N   B L O C K   H E L P E R
% ═══════════════════════════════════════════════════════════════════════════
function set_mfb_script(blockPath, scriptStr)
%SET_MFB_SCRIPT  Set the embedded MATLAB code of a MATLAB Function block.
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    if isempty(chart)
        % Force diagram update so Stateflow chart object is created
        mdl = bdroot(blockPath);
        set_param(mdl, 'SimulationCommand', 'update');
        chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    end
    if ~isempty(chart)
        chart.Script = scriptStr;
    else
        warning('Could not set script for %s', blockPath);
    end
end


% ═══════════════════════════════════════════════════════════════════════════
%          E M B E D D E D   C O D E   G E N E R A T O R S
%  Each function returns a string with valid MATLAB code for a
%  MATLAB Function block.  The function name MUST be 'fcn'.
% ═══════════════════════════════════════════════════════════════════════════

% ─────────────────────────────────────────────────────────────────────────
%  ENVIRONMENT  —  Ephemeris Truth  (CSV-based lookup with interpolation)
% ─────────────────────────────────────────────────────────────────────────
function s = ephemeris_truth_code()
    L = {
    'function [sun_eci, moon_eci] = fcn(t, ephem_t, ephem_sun, ephem_moon)'
    '%#codegen'
    '% Ground-truth Sun & Moon direction vectors interpolated from'
    '% pre-loaded ephemeris CSV data (ephemeris_2026_weekly.csv).'
    '%'
    '% Inputs:  t          – simulation time [s]'
    '%          ephem_t    – [Nx1] time breakpoints [s] from epoch'
    '%          ephem_sun  – [Nx3] Sun unit direction vectors (ECI)'
    '%          ephem_moon – [Nx3] Moon unit direction vectors (ECI)'
    '% Outputs: sun_eci    – Sun direction unit vector ECI [3x1]'
    '%          moon_eci   – Moon direction unit vector ECI [3x1]'
    ''
    '  N = size(ephem_t, 1);'
    ''
    '  % ── Clamp to data range ──'
    '  tc = t;'
    '  if tc < ephem_t(1)'
    '      tc = ephem_t(1);'
    '  elseif tc > ephem_t(N)'
    '      tc = ephem_t(N);'
    '  end'
    ''
    '  % ── Find bracketing interval  (linear search, N is small ~53) ──'
    '  idx = 1;'
    '  for k = 1:N-1'
    '      if ephem_t(k+1) >= tc'
    '          idx = k;'
    '          break;'
    '      end'
    '  end'
    ''
    '  % ── Linear interpolation fraction ──'
    '  dt_seg = ephem_t(idx+1) - ephem_t(idx);'
    '  if dt_seg > 0'
    '      frac = (tc - ephem_t(idx)) / dt_seg;'
    '  else'
    '      frac = 0;'
    '  end'
    ''
    '  % ── Interpolate Sun ──'
    '  s1 = ephem_sun(idx, :)'';'
    '  s2 = ephem_sun(idx+1, :)'';'
    '  sun_raw = s1 + frac * (s2 - s1);'
    '  sn = sqrt(sun_raw(1)^2 + sun_raw(2)^2 + sun_raw(3)^2);'
    '  if sn > 0'
    '      sun_eci = sun_raw / sn;'
    '  else'
    '      sun_eci = [1; 0; 0];'
    '  end'
    ''
    '  % ── Interpolate Moon ──'
    '  m1 = ephem_moon(idx, :)'';'
    '  m2 = ephem_moon(idx+1, :)'';'
    '  moon_raw = m1 + frac * (m2 - m1);'
    '  mn = sqrt(moon_raw(1)^2 + moon_raw(2)^2 + moon_raw(3)^2);'
    '  if mn > 0'
    '      moon_eci = moon_raw / mn;'
    '  else'
    '      moon_eci = [0; 1; 0];'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  ENVIRONMENT  —  Orbit Propagator  (Keplerian 2-body, no perturbations)
% ─────────────────────────────────────────────────────────────────────────
function s = orbit_propagator_code()
    L = {
    'function [pos_eci, vel_eci] = fcn(t)'
    '%#codegen'
    '% Keplerian orbit propagator (2-body, no J2).'
    '% Orbital elements read from workspace via coder.extrinsic.'
    ''
    '  mu = 3.986004418e14;'         % m^3/s^2
    '  Re = 6371e3;'                 % m
    ''
    '  % ── Orbital elements (could be tunable parameters) ──'
    '  alt  = 500e3;'                % m  (will override from workspace later)
    '  inc  = 51.6 * pi/180;'
    '  ecc  = 0.0001;'
    '  RAAN = 0;'
    '  AOP  = 0;'
    '  TA0  = 0;'
    ''
    '  a = Re + alt;'
    '  n = sqrt(mu / a^3);'          % mean motion
    ''
    '  % ── Propagate mean anomaly ──'
    '  MA = n * t;'                  % mean anomaly at time t (TA0=0 ⇒ MA0=0)
    ''
    '  % ── Solve Kepler''s equation  M = E - e sin(E)  (Newton) ──'
    '  E = MA;'
    '  for iter = 1:10'
    '      dE = (MA - E + ecc*sin(E)) / (1 - ecc*cos(E));'
    '      E = E + dE;'
    '      if abs(dE) < 1e-12, break; end'
    '  end'
    ''
    '  % ── True anomaly ──'
    '  nu = 2 * atan2(sqrt(1+ecc)*sin(E/2), sqrt(1-ecc)*cos(E/2));'
    ''
    '  % ── Perifocal frame (PQW) ──'
    '  r_mag = a * (1 - ecc*cos(E));'
    '  p_pqw = r_mag * [cos(nu); sin(nu); 0];'
    '  v_pqw = sqrt(mu*a)/r_mag * [-sin(E); sqrt(1-ecc^2)*cos(E); 0];'
    ''
    '  % ── Rotation PQW → ECI ──'
    '  cO = cos(RAAN); sO = sin(RAAN);'
    '  cw = cos(AOP);  sw = sin(AOP);'
    '  ci = cos(inc);  si = sin(inc);'
    ''
    '  R = [ cO*cw - sO*sw*ci,  -cO*sw - sO*cw*ci,   sO*si;'
    '        sO*cw + cO*sw*ci,  -sO*sw + cO*cw*ci,  -cO*si;'
    '        sw*si,              cw*si,               ci   ];'
    ''
    '  pos_eci = R * p_pqw;'
    '  vel_eci = R * v_pqw;'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  ENVIRONMENT  —  Magnetic Field (tilted dipole)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetic_field_code()
    L = {
    'function B_eci = fcn(pos_eci)'
    '%#codegen'
    '% Tilted-dipole approximation of Earth''s magnetic field in ECI.'
    '% Valid for LEO quick-look; replace with IGRF for higher fidelity.'
    ''
    '  Re = 6371e3;'              % Earth radius [m]
    '  B0 = 3.12e-5;'             % equatorial surface field [T]
    ''
    '  % Magnetic dipole axis (tilted ~11.5 deg toward ~290 deg lon)'
    '  theta_m = 11.5 * pi/180;'
    '  phi_m   = -69.0 * pi/180;'  % geographic longitude of magnetic pole
    '  m_hat = [ sin(theta_m)*cos(phi_m);'
    '            sin(theta_m)*sin(phi_m);'
    '            cos(theta_m) ];'
    ''
    '  r_vec = pos_eci;'
    '  r_mag = sqrt(r_vec(1)^2 + r_vec(2)^2 + r_vec(3)^2);'
    '  if r_mag < Re'
    '      r_mag = Re;'  % clamp to surface
    '  end'
    '  r_hat = r_vec / r_mag;'
    ''
    '  % Dipole field:  B = (B0*Re^3/r^3) * [3(m·r̂)r̂ - m]'
    '  m_dot_r = m_hat(1)*r_hat(1) + m_hat(2)*r_hat(2) + m_hat(3)*r_hat(3);'
    '  coeff   = B0 * (Re / r_mag)^3;'
    '  B_eci   = coeff * (3*m_dot_r*r_hat - m_hat);'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  ENVIRONMENT  —  Eclipse Model (cylindrical shadow)
% ─────────────────────────────────────────────────────────────────────────
function s = eclipse_model_code()
    L = {
    'function eclipse_flag = fcn(pos_eci, sun_eci)'
    '%#codegen'
    '% Cylindrical Earth shadow model.'
    '% Returns 1.0 if spacecraft is in eclipse, 0.0 if sunlit.'
    ''
    '  Re = 6371e3;'  % m
    ''
    '  % Unit vector from Earth to Sun'
    '  sun_mag = sqrt(sun_eci(1)^2 + sun_eci(2)^2 + sun_eci(3)^2);'
    '  s_hat = sun_eci / sun_mag;'
    ''
    '  % Project spacecraft position onto Sun direction'
    '  proj = pos_eci(1)*s_hat(1) + pos_eci(2)*s_hat(2) + pos_eci(3)*s_hat(3);'
    ''
    '  % Perpendicular distance from Sun-Earth line'
    '  perp_vec = pos_eci - proj * s_hat;'
    '  perp_dist = sqrt(perp_vec(1)^2 + perp_vec(2)^2 + perp_vec(3)^2);'
    ''
    '  % In eclipse if behind Earth AND within shadow cylinder'
    '  if proj < 0 && perp_dist < Re'
    '      eclipse_flag = 1.0;'
    '  else'
    '      eclipse_flag = 0.0;'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  DYNAMICS  —  Euler equation RHS
% ─────────────────────────────────────────────────────────────────────────
function s = euler_rhs_code()
    L = {
    'function omega_dot = fcn(torque, omega, J)'
    '%#codegen'
    '% Euler''s rigid-body rotation equation:'
    '%   J * omega_dot = torque - omega x (J * omega)'
    ''
    '  Jw = J * omega;'
    '  cross_term = [ omega(2)*Jw(3) - omega(3)*Jw(2);'
    '                 omega(3)*Jw(1) - omega(1)*Jw(3);'
    '                 omega(1)*Jw(2) - omega(2)*Jw(1) ];'
    '  omega_dot = J \ (torque - cross_term);'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  DYNAMICS  —  Quaternion kinematics RHS
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
%  DYNAMICS  —  Quaternion normalisation
% ─────────────────────────────────────────────────────────────────────────
function s = quat_norm_code()
    L = {
    'function q_out = fcn(q)'
    '%#codegen'
    '% Normalise quaternion to unit length (prevents integrator drift).'
    ''
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

% ─────────────────────────────────────────────────────────────────────────
%  ACTUATORS  —  Reaction Wheel Assembly
% ─────────────────────────────────────────────────────────────────────────
function s = rw_assembly_code()
    L = {
    'function torque_body = fcn(torque_cmd)'
    '%#codegen'
    '% 4-wheel pyramid reaction wheel assembly.'
    '% Saturates commanded torque per-wheel, maps to body torque via'
    '% the spin-axis geometry matrix A (3x4).'
    '%'
    '% Input:  torque_cmd [3x1] - desired body torque [N·m]'
    '% Output: torque_body [3x1] - achievable body torque [N·m]'
    ''
    '  rw_max = 0.005;'    % N·m per wheel
    ''
    '  % Pyramid spin-axis unit vectors  (cant angle ≈ 35.26°)'
    '  beta = atan(1/sqrt(2));'
    '  cb = cos(beta); sb = sin(beta);'
    '  A = [ cb,  0, -cb,  0;'
    '         0, cb,   0, -cb;'
    '        sb, sb,  sb,  sb ];'
    ''
    '  % Pseudoinverse distribution: tau_wheels = A^+ * torque_cmd'
    '  % For 4 wheels and 3 axes, A^+ = A''*(A*A'')^{-1}'
    '  tau_w = A'' * ((A*A'') \ torque_cmd);'
    ''
    '  % Per-wheel saturation'
    '  for i = 1:4'
    '      if tau_w(i) > rw_max'
    '          tau_w(i) = rw_max;'
    '      elseif tau_w(i) < -rw_max'
    '          tau_w(i) = -rw_max;'
    '      end'
    '  end'
    ''
    '  % Map saturated wheel torques back to body frame'
    '  torque_body = A * tau_w;'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  ACTUATORS  —  Magnetorquer Assembly
% ─────────────────────────────────────────────────────────────────────────
function s = mtq_assembly_code()
    L = {
    'function torque_body = fcn(dipole_cmd)'
    '%#codegen'
    '% 3-axis magnetorquer model.'
    '% Saturates dipole moment, then computes torque = m × B.'
    '% B_body is approximated as constant here; will be replaced with'
    '% a fed-back sensor input in a future iteration.'
    '%'
    '% Input:  dipole_cmd [3x1] - commanded dipole moment [A·m²]'
    '% Output: torque_body [3x1] - resulting torque [N·m]'
    ''
    '  mtq_max = 0.2;'   % A·m² per axis
    ''
    '  % Saturate'
    '  m = dipole_cmd;'
    '  for i = 1:3'
    '      if m(i) > mtq_max'
    '          m(i) = mtq_max;'
    '      elseif m(i) < -mtq_max'
    '          m(i) = -mtq_max;'
    '      end'
    '  end'
    ''
    '  % Placeholder B field in body frame [T]'
    '  % (In full model, feed B_body_meas from SENSORS as an input)'
    '  B_body = [2e-5; -1e-5; 4e-5];'
    ''
    '  % Torque = m × B'
    '  torque_body = [ m(2)*B_body(3) - m(3)*B_body(2);'
    '                  m(3)*B_body(1) - m(1)*B_body(3);'
    '                  m(1)*B_body(2) - m(2)*B_body(1) ];'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  ACTUATORS  —  CMG (placeholder)
% ─────────────────────────────────────────────────────────────────────────
function s = cmg_code()
    L = {
    'function torque_body = fcn(gimbal_cmd)'
    '%#codegen'
    '% Placeholder CMG model. Outputs zero torque for now.'
    '% In full model: torque = dH/dt = h * gimbal_rate × gimbal_axis'
    ''
    '  torque_body = [0; 0; 0];'
    ''
    '  % Suppress unused input warning'
    '  if false'
    '      disp(gimbal_cmd);'
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  CONTROL  —  Onboard Ephemeris  (analytical Meeus model)
%  This is the model the satellite carries onboard.  It uses the same
%  Meeus algorithm that was previously the simulation truth, so it is
%  good (~1° Sun, ~5° Moon) but NOT identical to the CSV ground truth.
% ─────────────────────────────────────────────────────────────────────────
function s = onboard_ephemeris_code()
    L = {
    'function [sun_pred, mag_pred] = fcn(t, epoch_jd)'
    '%#codegen'
    '% Onboard analytical Sun & magnetic-dipole reference vectors.'
    '% Uses Meeus low-precision solar ephemeris (~1 deg accuracy)'
    '% and a fixed magnetic dipole axis.'
    '% This is what the flight software would compute onboard.'
    '%'
    '% Inputs:  t         – simulation time [s]'
    '%          epoch_jd  – Julian date at t = 0'
    '% Outputs: sun_pred  – Sun direction unit vector in ECI [3x1]'
    '%          mag_pred  – magnetic dipole axis in ECI [3x1]'
    ''
    '  jd = epoch_jd + t / 86400;'
    '  T  = (jd - 2451545.0) / 36525;'
    ''
    '  % —— Sun (Meeus low-precision) ——'
    '  M  = mod(357.5291 + 35999.0503*T, 360) * pi/180;'
    '  L0 = mod(280.4664 + 36000.7698*T, 360);'
    '  C  = 1.9146*sin(M) + 0.0200*sin(2*M);'
    '  lam = mod(L0 + C, 360) * pi/180;'
    '  eps = (23.4393 - 0.0130*T) * pi/180;'
    '  sun_pred = [ cos(lam);'
    '               cos(eps)*sin(lam);'
    '               sin(eps)*sin(lam) ];'
    '  sn = sqrt(sun_pred(1)^2+sun_pred(2)^2+sun_pred(3)^2);'
    '  if sn > 0'
    '      sun_pred = sun_pred / sn;'
    '  end'
    ''
    '  % —— Magnetic dipole axis (fixed, IGRF approximate) ——'
    '  theta_m = 11.5 * pi/180;'
    '  phi_m   = -69.0 * pi/180;'
    '  mag_pred = [ sin(theta_m)*cos(phi_m);'
    '               sin(theta_m)*sin(phi_m);'
    '               cos(theta_m) ];'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  CONTROL  —  TRIAD Attitude Estimator
% ─────────────────────────────────────────────────────────────────────────
function s = triad_estimator_code()
    L = {
    'function [q_est, omega_est] = fcn(q_meas, omega_meas, sun_meas, B_body_meas, sun_ref, mag_ref)'
    '%#codegen'
    '% TRIAD attitude estimator.'
    '% Uses two vector observations (Sun, magnetic field) in body and'
    '% reference frames to compute the attitude quaternion.'
    '% Falls back to the measured quaternion if vectors are degenerate.'
    '%'
    '% Inputs: q_meas      [4x1] - star-tracker quaternion measurement'
    '%         omega_meas  [3x1] - gyro measurement [rad/s]'
    '%         sun_meas    [3x1] - Sun direction in body frame (unit vec)'
    '%         B_body_meas [3x1] - B-field in body frame (unit vec)'
    '%         sun_ref     [3x1] - Sun direction in ECI (from onboard ephem)'
    '%         mag_ref     [3x1] - B-field direction in ECI (from onboard ephem)'
    '% Outputs: q_est      [4x1] - estimated quaternion [scalar-first]'
    '%          omega_est  [3x1] - estimated body rates [rad/s]'
    ''
    '  % —— Normalise observation vectors ——'
    '  s_b = sun_meas;'
    '  n_sb = sqrt(s_b(1)^2 + s_b(2)^2 + s_b(3)^2);'
    '  if n_sb > 1e-10'
    '      s_b = s_b / n_sb;'
    '  end'
    ''
    '  m_b = B_body_meas;'
    '  n_mb = sqrt(m_b(1)^2 + m_b(2)^2 + m_b(3)^2);'
    '  if n_mb > 1e-10'
    '      m_b = m_b / n_mb;'
    '  end'
    ''
    '  s_r = sun_ref;'
    '  n_sr = sqrt(s_r(1)^2 + s_r(2)^2 + s_r(3)^2);'
    '  if n_sr > 1e-10'
    '      s_r = s_r / n_sr;'
    '  end'
    ''
    '  m_r = mag_ref;'
    '  n_mr = sqrt(m_r(1)^2 + m_r(2)^2 + m_r(3)^2);'
    '  if n_mr > 1e-10'
    '      m_r = m_r / n_mr;'
    '  end'
    ''
    '  % —— TRIAD bases ——'
    '  % Body frame triad'
    '  t1_b = s_b;'
    '  cp = [s_b(2)*m_b(3)-s_b(3)*m_b(2);'
    '        s_b(3)*m_b(1)-s_b(1)*m_b(3);'
    '        s_b(1)*m_b(2)-s_b(2)*m_b(1)];'
    '  n_cp = sqrt(cp(1)^2+cp(2)^2+cp(3)^2);'
    ''
    '  if n_cp < 1e-10 || n_sb < 1e-10 || n_mb < 1e-10'
    '      % Vectors degenerate — fall back to star-tracker'
    '      q_est = q_meas;'
    '      omega_est = omega_meas;'
    '      return;'
    '  end'
    ''
    '  t2_b = cp / n_cp;'
    '  t3_b = [t1_b(2)*t2_b(3)-t1_b(3)*t2_b(2);'
    '          t1_b(3)*t2_b(1)-t1_b(1)*t2_b(3);'
    '          t1_b(1)*t2_b(2)-t1_b(2)*t2_b(1)];'
    ''
    '  % Reference frame triad'
    '  t1_r = s_r;'
    '  cp_r = [s_r(2)*m_r(3)-s_r(3)*m_r(2);'
    '          s_r(3)*m_r(1)-s_r(1)*m_r(3);'
    '          s_r(1)*m_r(2)-s_r(2)*m_r(1)];'
    '  n_cp_r = sqrt(cp_r(1)^2+cp_r(2)^2+cp_r(3)^2);'
    '  t2_r = cp_r / n_cp_r;'
    '  t3_r = [t1_r(2)*t2_r(3)-t1_r(3)*t2_r(2);'
    '          t1_r(3)*t2_r(1)-t1_r(1)*t2_r(3);'
    '          t1_r(1)*t2_r(2)-t1_r(2)*t2_r(1)];'
    ''
    '  % —— DCM: body from reference  A = M_body * M_ref'' ——'
    '  M_b = [t1_b, t2_b, t3_b];'
    '  M_r = [t1_r, t2_r, t3_r];'
    '  A   = M_b * M_r'';'
    ''
    '  % —— DCM to quaternion (Shepperd method) ——'
    '  tr = A(1,1) + A(2,2) + A(3,3);'
    '  qw = sqrt(max(0, 1+tr)) / 2;'
    '  qx = sqrt(max(0, 1+A(1,1)-A(2,2)-A(3,3))) / 2;'
    '  qy = sqrt(max(0, 1-A(1,1)+A(2,2)-A(3,3))) / 2;'
    '  qz = sqrt(max(0, 1-A(1,1)-A(2,2)+A(3,3))) / 2;'
    ''
    '  % Resolve sign ambiguities'
    '  if A(3,2)-A(2,3) < 0, qx = -qx; end'
    '  if A(1,3)-A(3,1) < 0, qy = -qy; end'
    '  if A(2,1)-A(1,2) < 0, qz = -qz; end'
    ''
    '  q_est = [qw; qx; qy; qz];'
    '  nq = sqrt(q_est(1)^2+q_est(2)^2+q_est(3)^2+q_est(4)^2);'
    '  if nq > 0'
    '      q_est = q_est / nq;'
    '  end'
    ''
    '  % Omega estimate = gyro (no filtering yet)'
    '  omega_est = omega_meas;'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  CONTROL  —  Control Law (PD for RW + B-dot for MTQ)
% ─────────────────────────────────────────────────────────────────────────
function s = control_law_code()
    L = {
    'function [rw_cmd, mtq_cmd, cmg_cmd] = fcn(q_est, omega_est, B_body)'
    '%#codegen'
    '% Combined control law:'
    '%   - PD controller on reaction wheels for attitude hold'
    '%   - B-dot law on magnetorquers for rate damping'
    '%   - CMG command = zero (placeholder)'
    '%'
    '% Inputs: q_est     [4x1] - estimated quaternion [scalar-first]'
    '%         omega_est [3x1] - estimated body rates [rad/s]'
    '%         B_body    [3x1] - magnetic field in body frame [T or nT]'
    '% Outputs: rw_cmd   [3x1] - reaction wheel torque command [N·m]'
    '%          mtq_cmd  [3x1] - magnetorquer dipole command [A·m²]'
    '%          cmg_cmd  [3x1] - CMG gimbal command (zeros)'
    ''
    '  % —— PD gains ——'
    '  Kp = 0.002;'   % proportional (N·m)
    '  Kd = 0.02;'    % derivative   (N·m·s)
    ''
    '  % Target attitude: identity quaternion [1;0;0;0]'
    '  % Quaternion error: q_err ≈ 2 * q_vec when q_scalar ≈ 1'
    '  q_err = q_est(2:4);'   % vector part = rotation error (small-angle)
    '  if q_est(1) < 0'
    '      q_err = -q_err;'   % ensure shortest path
    '  end'
    ''
    '  rw_cmd = -Kp * q_err - Kd * omega_est;'
    ''
    '  % —— B-dot magnetorquer law ——'
    '  % m = -k_bdot * (dB/dt)'
    '  % Since we don''t have dB/dt here, use  m = -k * (omega × B)  as proxy'
    '  persistent B_prev'
    '  if isempty(B_prev)'
    '      B_prev = B_body;'
    '  end'
    '  k_bdot = 1e5;'    % gain (tuned for nT-scale fields)
    '  dB = B_body - B_prev;'
    '  B_prev = B_body;'
    '  mtq_cmd = -k_bdot * dB;'
    ''
    '  % Saturate dipole command'
    '  mtq_max = 0.2;'
    '  for i = 1:3'
    '      if mtq_cmd(i) > mtq_max'
    '          mtq_cmd(i) = mtq_max;'
    '      elseif mtq_cmd(i) < -mtq_max'
    '          mtq_cmd(i) = -mtq_max;'
    '      end'
    '  end'
    ''
    '  cmg_cmd = [0; 0; 0];'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  SENSORS  —  Sun Sensor Suite (6 FSS combined)
% ─────────────────────────────────────────────────────────────────────────
function s = sun_sensor_suite_code()
    L = {
    'function sun_body = fcn(sun_eci, q, eclipse_flag)'
    '%#codegen'
    '% Six Fine Sun Sensors on CubeSat faces.'
    '% Rotates Sun vector to body frame, checks which sensors see the Sun,'
    '% adds per-sensor noise, and combines via weighted average.'
    '% Returns zero vector in eclipse.'
    '%'
    '% Inputs: sun_eci      [3x1]  - Sun position ECI [km]'
    '%         q            [4x1]  - true attitude quaternion [scalar-first]'
    '%         eclipse_flag scalar - 1 if in eclipse'
    '% Output: sun_body     [3x1]  - measured Sun unit vector in body frame'
    ''
    '  % Eclipse check'
    '  if eclipse_flag > 0.5'
    '      sun_body = [0; 0; 0];'
    '      return;'
    '  end'
    ''
    '  % —— Rotate Sun to body frame via quaternion ——'
    '  sun_mag = sqrt(sun_eci(1)^2 + sun_eci(2)^2 + sun_eci(3)^2);'
    '  if sun_mag < 1e-6'
    '      sun_body = [0; 0; 0];'
    '      return;'
    '  end'
    '  s_hat = sun_eci / sun_mag;'
    ''
    '  % q = [q0 q1 q2 q3],  DCM from ECI to body: R = R(q)'
    '  q0=q(1); q1=q(2); q2=q(3); q3=q(4);'
    '  R = [1-2*(q2^2+q3^2),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1^2+q2^2)];'
    '  s_body = R * s_hat;'
    ''
    '  % FSS boresight directions (6 cube faces)'
    '  bore = [ 1  0  0;'
    '          -1  0  0;'
    '           0  1  0;'
    '           0 -1  0;'
    '           0  0  1;'
    '           0  0 -1 ]'';'  % 3x6
    ''
    '  % FOV half-angle [rad]'
    '  fov_half = 60 * pi/180;'
    '  noise_sigma = 0.5 * pi/180;'  % 0.5 deg 1-sigma
    ''
    '  % Check each sensor and accumulate'
    '  sum_vec = [0; 0; 0];'
    '  n_valid = 0;'
    '  for k = 1:6'
    '      cos_ang = bore(1,k)*s_body(1) + bore(2,k)*s_body(2) + bore(3,k)*s_body(3);'
    '      if cos_ang > cos(fov_half)'
    '          % Sensor sees the Sun: add noise'
    '          % (using simple additive Gaussian on the unit vector components)'
    '          noisy = s_body + noise_sigma * randn(3,1);'
    '          nn = sqrt(noisy(1)^2 + noisy(2)^2 + noisy(3)^2);'
    '          if nn > 0'
    '              noisy = noisy / nn;'
    '          end'
    '          sum_vec = sum_vec + noisy;'
    '          n_valid = n_valid + 1;'
    '      end'
    '  end'
    ''
    '  if n_valid > 0'
    '      sun_body = sum_vec / n_valid;'
    '      nb = sqrt(sun_body(1)^2 + sun_body(2)^2 + sun_body(3)^2);'
    '      if nb > 0'
    '          sun_body = sun_body / nb;'
    '      end'
    '  else'
    '      sun_body = [0; 0; 0];'  % no sensor has Sun in FOV
    '  end'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  SENSORS  —  Star Tracker
% ─────────────────────────────────────────────────────────────────────────
function s = star_tracker_code()
    L = {
    'function q_meas = fcn(sun_eci, moon_eci, q_true)'
    '%#codegen'
    '% Star tracker model with Sun/Moon exclusion and noise.'
    '%'
    '% Inputs: sun_eci  [3x1] - Sun position ECI'
    '%         moon_eci [3x1] - Moon position ECI'
    '%         q_true   [4x1] - true attitude quaternion [scalar-first]'
    '% Output: q_meas   [4x1] - measured quaternion (+ noise or last-valid)'
    ''
    '  persistent q_prev'
    '  if isempty(q_prev)'
    '      q_prev = [1; 0; 0; 0];'
    '  end'
    ''
    '  % Star tracker boresight in body frame'
    '  bore = [0; 0; 1];'
    ''
    '  % Convert bore to ECI'
    '  q0=q_true(1); q1=q_true(2); q2=q_true(3); q3=q_true(4);'
    '  R = [1-2*(q2^2+q3^2),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1^2+q2^2)];'
    '  bore_eci = R'' * bore;'  % body-to-ECI
    ''
    '  % Sun exclusion (45 deg half-angle)'
    '  sun_mag = sqrt(sun_eci(1)^2+sun_eci(2)^2+sun_eci(3)^2);'
    '  if sun_mag > 0'
    '      cos_sun = (bore_eci(1)*sun_eci(1)+bore_eci(2)*sun_eci(2)+bore_eci(3)*sun_eci(3)) / sun_mag;'
    '      if cos_sun > cos(45*pi/180)'
    '          q_meas = q_prev;'  % blinded
    '          return;'
    '      end'
    '  end'
    ''
    '  % Moon exclusion (25 deg half-angle)'
    '  moon_mag = sqrt(moon_eci(1)^2+moon_eci(2)^2+moon_eci(3)^2);'
    '  if moon_mag > 0'
    '      cos_moon = (bore_eci(1)*moon_eci(1)+bore_eci(2)*moon_eci(2)+bore_eci(3)*moon_eci(3)) / moon_mag;'
    '      if cos_moon > cos(25*pi/180)'
    '          q_meas = q_prev;'  % blinded
    '          return;'
    '      end'
    '  end'
    ''
    '  % Add noise (small-angle random rotation)'
    '  bore_noise  = 10 / 206265;'  % 10 arcsec → rad
    '  cross_noise = 40 / 206265;'  % 40 arcsec → rad
    '  err_axis = [bore_noise*randn; cross_noise*randn; cross_noise*randn];'
    '  half = err_axis / 2;'
    '  dq = [1; half(1); half(2); half(3)];'
    '  ndq = sqrt(dq(1)^2+dq(2)^2+dq(3)^2+dq(4)^2);'
    '  dq = dq / ndq;'
    ''
    '  % Quaternion multiply: q_meas = q_true ⊗ dq'
    '  a=q_true(1); b=q_true(2); c=q_true(3); d=q_true(4);'
    '  p=dq(1);     r=dq(2);     s=dq(3);     tt=dq(4);'
    '  q_meas = [a*p-b*r-c*s-d*tt;'
    '            a*r+b*p+c*tt-d*s;'
    '            a*s-b*tt+c*p+d*r;'
    '            a*tt+b*s-c*r+d*p];'
    ''
    '  nq = sqrt(q_meas(1)^2+q_meas(2)^2+q_meas(3)^2+q_meas(4)^2);'
    '  q_meas = q_meas / nq;'
    '  q_prev = q_meas;'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  SENSORS  —  IMU (MEMS gyroscope)
% ─────────────────────────────────────────────────────────────────────────
function s = imu_code()
    L = {
    'function omega_meas = fcn(omega_true)'
    '%#codegen'
    '% MEMS gyroscope model with bias + white noise.'
    '%'
    '% Input:  omega_true [3x1] - true angular velocity [rad/s]'
    '% Output: omega_meas [3x1] - measured angular velocity [rad/s]'
    ''
    '  persistent bias'
    '  if isempty(bias)'
    '      bias = [0; 0; 0];'
    '  end'
    ''
    '  dt = 0.1;'  % sample period [s]
    '  arw = 0.01 * pi/180;'           % angular random walk [rad/√s]
    '  bias_instab = 1.0 * pi/180 / 3600;'  % bias instability [rad/s]
    ''
    '  % Random-walk bias drift'
    '  bias = bias + sqrt(dt) * bias_instab * randn(3,1);'
    ''
    '  % Measurement = truth + bias + white noise'
    '  omega_meas = omega_true + bias + (arw / sqrt(dt)) * randn(3,1);'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  SENSORS  —  Magnetometer (3-axis)
% ─────────────────────────────────────────────────────────────────────────
function s = magnetometer_code()
    L = {
    'function B_body_meas = fcn(B_eci, q)'
    '%#codegen'
    '% 3-axis magnetometer: rotates truth to body frame, adds bias + noise.'
    '%'
    '% Inputs: B_eci [3x1] - true magnetic field in ECI [T]'
    '%         q     [4x1] - true attitude quaternion [scalar-first]'
    '% Output: B_body_meas [3x1] - measured B-field in body frame [T]'
    ''
    '  % Rotate ECI → body'
    '  q0=q(1); q1=q(2); q2=q(3); q3=q(4);'
    '  R = [1-2*(q2^2+q3^2),  2*(q1*q2+q0*q3),  2*(q1*q3-q0*q2);'
    '       2*(q1*q2-q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3+q0*q1);'
    '       2*(q1*q3+q0*q2),  2*(q2*q3-q0*q1),  1-2*(q1^2+q2^2)];'
    '  B_body = R * B_eci;'
    ''
    '  % Hard-iron bias [T]  (50, -30, 80 nT converted to T)'
    '  bias = [50e-9; -30e-9; 80e-9];'
    ''
    '  % White noise 1σ = 100 nT per axis'
    '  noise_sigma = 100e-9;'
    ''
    '  B_body_meas = B_body + bias + noise_sigma * randn(3,1);'
    'end'
    };
    s = strjoin(L, newline);
end

% ─────────────────────────────────────────────────────────────────────────
%  SENSORS  —  GNSS Receiver
% ─────────────────────────────────────────────────────────────────────────
function s = gnss_code()
    L = {
    'function [pos_meas, vel_meas] = fcn(pos_eci, vel_eci)'
    '%#codegen'
    '% GNSS receiver model: adds white noise to position and velocity.'
    '%'
    '% Inputs: pos_eci [3x1] - true position [m]'
    '%         vel_eci [3x1] - true velocity [m/s]'
    '% Outputs: pos_meas [3x1], vel_meas [3x1]'
    ''
    '  pos_sigma = 10.0;'   % m, 1-sigma per axis
    '  vel_sigma = 0.1;'    % m/s, 1-sigma per axis
    ''
    '  pos_meas = pos_eci + pos_sigma * randn(3,1);'
    '  vel_meas = vel_eci + vel_sigma * randn(3,1);'
    'end'
    };
    s = strjoin(L, newline);
end
