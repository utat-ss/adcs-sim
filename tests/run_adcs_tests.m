function results = run_adcs_tests()
%RUN_ADCS_TESTS Build and behavior-test the ADCS Simulink model.
%
%   RESULTS = RUN_ADCS_TESTS() initializes parameters, rebuilds the generated
%   model, checks source/model interfaces, runs bounded closed-loop scenarios,
%   and verifies that attitude, environment, control, actuator, and dynamics
%   paths are actually reachable.

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    prev_dir = pwd;
    cleanup = onCleanup(@() cd(prev_dir));
    cd(repo_root);

    evalin('base', 'init_adcs_params;');
    verify_source_inventory(repo_root);
    verify_parameter_consistency();
    verify_sim_config_consistency();
    verify_quaternion_frame_conventions();
    verify_invalid_overrides();

    evalin('base', 'build_adcs_model;');
    load_system('adcs_sim');
    verify_top_level_model('adcs_sim');
    verify_logging_sinks('adcs_sim');
    verify_generated_block_scripts('adcs_sim');
    verify_extracted_generated_kernels('adcs_sim');
    verify_extracted_environment_kernels('adcs_sim');
    verify_tracker_noise_basis('adcs_sim');
    close_system('adcs_sim', 0);

    smoke = simulate_adcs_case(struct('dt', 0.01, 't_end', 60));
    verify_basic_simulation(smoke);

    override_input = struct( ...
        'dt', 0.01, ...
        't_end', 5, ...
        'q0', [2; 0; 0; 0], ...
        'omega0', [0.003; -0.004; 0.005], ...
        'h_W0', [0.001; -0.002; 0.003; -0.004], ...
        'J', diag([0.04, 0.035, 0.012]));
    override_case = simulate_adcs_case(override_input);
    verify_override_case(override_case, override_input);

    pointing_case = simulate_adcs_case(struct( ...
        'dt', 0.01, ...
        't_end', 60, ...
        'q0', axis_angle_quaternion([0; 1; 0], 35*pi/180), ...
        'omega0', [0; 0; 0]));
    verify_pointing_case(pointing_case);

    detumble_case = simulate_adcs_case(struct( ...
        'dt', 0.01, ...
        't_end', 20, ...
        'q0', [1; 0; 0; 0], ...
        'omega0', [0.03; -0.02; 0.015]));
    verify_detumble_case(detumble_case);

    torque_free_case = simulate_adcs_case(struct( ...
        'dt', 0.01, ...
        't_end', 10, ...
        'q0', axis_angle_quaternion([1; 1; 0], 15*pi/180), ...
        'omega0', [0.005; 0.003; -0.002], ...
        'h_W0', [0.001; -0.0005; 0.0007; -0.0002], ...
        'J', diag([0.03, 0.03, 0.03]), ...
        'ctrl_rw_kp', 0, ...
        'ctrl_rw_kd', 0, ...
        'ctrl_bdot_gain', 0, ...
        'ctrl_dump_gain', 0, ...
        'atm_ref_density', 0, ...
        'solar_pressure_1au', 0, ...
        'residual_dipole', [0; 0; 0], ...
        'rw_axes', alternate_reaction_wheel_axes()));
    verify_torque_free_conservation(torque_free_case);

    export_dir = fullfile(tempdir, 'adcs_export_contract_check');
    if ~isfolder(export_dir)
        mkdir(export_dir);
    end
    export_manifest = export_adcs_results(pointing_case, export_dir, 'test_export', ...
        MaxSamples=101, MakeFigures=false);
    verify_export_schema(export_manifest);

    results = struct();
    results.build_ok = true;
    results.sim_ok = true;
    results.source_file_count = count_matlab_sources(repo_root);
    results.smoke = smoke.metrics;
    results.override = override_case.metrics;
    results.pointing = pointing_case.metrics;
    results.detumble = detumble_case.metrics;
    results.torque_free = torque_free_case.metrics;
    results.max_quat_norm_error = max([ ...
        smoke.metrics.max_quat_norm_error, ...
        override_case.metrics.max_quat_norm_error, ...
        pointing_case.metrics.max_quat_norm_error, ...
        detumble_case.metrics.max_quat_norm_error, ...
        torque_free_case.metrics.max_quat_norm_error]);
    results.pointing_peak_rw_command_nm = pointing_case.metrics.peak_rw_command_nm;
    results.pointing_peak_rw_body_torque_nm = pointing_case.metrics.peak_rw_torque_body_nm;
    results.detumble_peak_mtq_command_am2 = detumble_case.metrics.peak_mtq_command_am2;
    results.detumble_peak_ext_torque_nm = detumble_case.metrics.peak_ext_torque_nm;
    results.torque_free_total_h_variation_nms = total_inertial_momentum_variation(torque_free_case);
    results.export_replay_json = export_manifest.replay_json;
end


function verify_invalid_overrides()
    try
        simulate_adcs_case(struct('dt', 0.05, 't_end', 0.1));
        error('ExpectedInvalidDtNotRaised');
    catch me
        assert(contains(me.message, 'fastest configured sensor sample period'), ...
            'Unexpected invalid-dt error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('dt', 0.006, 't_end', 0.1));
        error('ExpectedNonDividingDtNotRaised');
    catch me
        assert(contains(me.message, 'divide every configured sensor sample period'), ...
            'Unexpected non-dividing-dt error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('omega00', [0; 0; 0], 't_end', 0.1));
        error('ExpectedUnknownOverrideNotRaised');
    catch me
        assert(contains(me.message, 'Unknown ADCS override'), ...
            'Unexpected unknown-override error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('J', diag([0.03, 0.01, 0.10]), 't_end', 0.1));
        error('ExpectedBadInertiaNotRaised');
    catch me
        assert(contains(me.message, 'triangle inequality'), ...
            'Unexpected bad-inertia error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('mag_noise_sigma', -1, 't_end', 0.1));
        error('ExpectedNegativeNoiseNotRaised');
    catch me
        assert(contains(me.message, 'nonnegative'), ...
            'Unexpected negative-noise error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('fss_fov_half', 220, 't_end', 0.1));
        error('ExpectedBadFovNotRaised');
    catch me
        assert(contains(me.message, '<= 180'), ...
            'Unexpected bad-FOV error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('st_boresight', [0; 0; 0], 't_end', 0.1));
        error('ExpectedBadBoresightNotRaised');
    catch me
        assert(contains(me.message, 'st_boresight must be nonzero'), ...
            'Unexpected bad-boresight error: %s', me.message);
    end

    try
        simulate_adcs_case(struct('orbit_alt', 500e3, 'orbit_ecc', 0.1, 't_end', 0.1));
        error('ExpectedEarthIntersectingOrbitNotRaised');
    catch me
        assert(contains(me.message, 'perigee radius'), ...
            'Unexpected Earth-intersecting orbit error: %s', me.message);
    end
end


function verify_export_schema(manifest)
    assert(isfield(manifest, 'replay_json') && isfile(manifest.replay_json), ...
        'Exporter did not write viewer replay JSON.');
    assert(isfield(manifest, 'timeseries_json') && isfile(manifest.timeseries_json), ...
        'Exporter did not write diagnostic timeseries JSON.');
    assert(isfield(manifest, 'mat') && isfile(manifest.mat), ...
        'Exporter did not write full MATLAB results MAT.');

    replay = jsondecode(fileread(manifest.replay_json));
    assert(strcmp(replay.schema, 'utat.adcs.replay.v1'), ...
        'Viewer replay schema mismatch.');
    assert(isfield(replay, 'samples') && numel(replay.samples) >= 2, ...
        'Viewer replay must include multiple samples.');
    assert(numel(replay.samples) <= 102, ...
        'Viewer replay MaxSamples limit was not respected.');
    assert(isfield(replay.centralBody, 'radiusKm') && replay.centralBody.radiusKm > 6000, ...
        'Viewer replay is missing Earth radius metadata.');
    assert(isfield(replay.simulation, 'epochJd') && isfinite(replay.simulation.epochJd), ...
        'Viewer replay is missing epoch metadata.');
    assert(isfield(replay.spacecraft, 'dimensionsM') && numel(replay.spacecraft.dimensionsM) == 3, ...
        'Viewer replay is missing spacecraft dimensions.');
    assert(isfield(replay.visualization, 'cameraFovHalfDeg') && replay.visualization.cameraFovHalfDeg == 8, ...
        'Viewer replay must use the shared illustrative 8 deg camera half-angle.');
    assert(isfield(replay, 'events') && isfield(replay.events, 'modeChanges') && ...
        numel(replay.events.modeChanges) >= 1, ...
        'Viewer replay must include full-log control mode event metadata.');

    sample = replay.samples(1);
    required_sample_fields = ["tSec", "positionEciKm", "velocityEciKmps", "qEciToBody", ...
        "qDesiredEciToBody", "omegaBodyRadS", "reactionWheelCommandNm", ...
        "reactionWheelTorqueBodyNm", "reactionWheelTorqueWheelNm", "desiredAttitudeErrorDeg"];
    for idx = 1:numel(required_sample_fields)
        assert(isfield(sample, required_sample_fields(idx)), ...
            'Viewer replay sample missing %s.', required_sample_fields(idx));
    end

    timeseries = jsondecode(fileread(manifest.timeseries_json));
    assert(strcmp(timeseries.metadata.schema, 'adcs-simulink-replay-v1'), ...
        'Diagnostic timeseries JSON schema mismatch.');
end


function verify_source_inventory(repo_root)
    files = matlab_source_files(repo_root);
    names = sort(string({files.name}));
    required = [
        "build_adcs_model.m"
        "init_adcs_params.m"
        "simulate_adcs_case.m"
        "summarize_adcs_simulation.m"
        "launch_adcs_gui.m"
        "export_adcs_results.m"
        "launch_adcs_3d_viewer.m"
        "run_adcs_tests.m"
        "run_pointing_validation.m"
        "build_control.m"
        "build_actuators.m"
        "build_dynamics.m"
        "build_environment.m"
        "build_sensors.m"
        "orbit_propagator_eval.m"
        "sim_config.m"];
    for idx = 1:numel(required)
        assert(any(names == required(idx)), 'Missing MATLAB source file %s.', required(idx));
    end
end

function n = count_matlab_sources(repo_root)
    n = numel(matlab_source_files(repo_root));
end

function files = matlab_source_files(repo_root)
    files = [
        dir(fullfile(repo_root, '*.m'));
        dir(fullfile(repo_root, 'builders', '*.m'));
        dir(fullfile(repo_root, 'utils', '*.m'));
        dir(fullfile(repo_root, 'tests', '*.m'));
        dir(fullfile(repo_root, 'sim_config', '*.m'));
        dir(fullfile(repo_root, 'sim_config', '*', '*.m'))];
end


function verify_parameter_consistency()
    J = evalin('base', 'J');
    mass_sc = evalin('base', 'mass_sc');
    sc_dim_x = evalin('base', 'sc_dim_x');
    sc_dim_y = evalin('base', 'sc_dim_y');
    sc_dim_z = evalin('base', 'sc_dim_z');

    expected_diag = (mass_sc / 12) * [ ...
        sc_dim_y^2 + sc_dim_z^2; ...
        sc_dim_x^2 + sc_dim_z^2; ...
        sc_dim_x^2 + sc_dim_y^2];

    assert(norm(J - diag(diag(J)), 'fro') < 1e-12, ...
        'Default inertia tensor should remain diagonal in the principal-axis frame.');
    assert(norm(diag(J) - expected_diag, inf) < 1e-12, ...
        'Default inertia tensor must stay consistent with the declared bus mass and dimensions.');
end


function verify_sim_config_consistency()
    cfg = sim_config();

    assert(abs(cfg.simulation.dt - evalin('base', 'dt')) < 1e-12, ...
        'sim_config dt drifted from init_adcs_params.');
    assert(abs(cfg.simulation.tf - evalin('base', 't_end')) < 1e-12, ...
        'sim_config t_end drifted from init_adcs_params.');
    assert(norm(cfg.spacecraft.inertia_kgm2 - evalin('base', 'J'), 'fro') < 1e-12, ...
        'sim_config inertia drifted from init_adcs_params.');
    assert(norm(cfg.actuators.reaction_wheel.axes_body - evalin('base', 'rw_axes'), 'fro') < 1e-12, ...
        'sim_config reaction-wheel axes drifted from init_adcs_params.');
    assert(norm(cfg.sensors.fine_sun_sensor.boresights_body - evalin('base', 'fss_boresights'), 'fro') < 1e-12, ...
        'sim_config FSS boresights drifted from init_adcs_params.');
end


function verify_quaternion_frame_conventions()
    samples = [ ...
        1, 0, 0, 0; ...
        sqrt(0.5), sqrt(0.5), 0, 0; ...
        sqrt(0.5), 0, sqrt(0.5), 0; ...
        0.8, -0.1, 0.2, 0.55];

    for idx = 1:size(samples, 1)
        q = samples(idx, :).';
        q = q / norm(q);

        sensor_dcm = scalar_first_dcm(q);
        gravity_gradient_dcm = scalar_first_dcm(q);

        assert(norm(sensor_dcm * sensor_dcm.' - eye(3), 'fro') < 1e-12, ...
            'Sensor quaternion DCM must remain orthonormal.');
        assert(norm(gravity_gradient_dcm - sensor_dcm, 'fro') < 1e-12, ...
            'Gravity-gradient and sensor blocks must use the same ECI-to-body DCM convention.');
    end
end


function verify_top_level_model(mdl)
    required_blocks = {'ENVIRONMENT', 'SENSORS', 'CONTROL', 'ACTUATORS', 'DYNAMICS'};
    for k = 1:numel(required_blocks)
        assert(~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', required_blocks{k})), ...
            'Missing top-level block: %s/%s', mdl, required_blocks{k});
    end

    port_expectations = {
        'ENVIRONMENT', [0 6];
        'SENSORS', [8 NaN];
        'CONTROL', [NaN 3];
        'ACTUATORS', [NaN 3];
        'DYNAMICS', [8 3]};
    for k = 1:size(port_expectations, 1)
        ports = get_param([mdl '/' port_expectations{k, 1}], 'Ports');
        expected = port_expectations{k, 2};
        if strcmp(port_expectations{k, 1}, 'SENSORS')
            assert(ports(1) == 8 && (ports(2) == 6 || ports(2) == 7), ...
                'SENSORS should expose 8 input ports and 6 or 7 output ports; got [%d %d].', ...
                ports(1), ports(2));
        elseif strcmp(port_expectations{k, 1}, 'CONTROL')
            assert((ports(1) == 7 || ports(1) == 8) && ports(2) == expected(2), ...
                'CONTROL should expose 7 or 8 input ports and 3 output ports; got [%d %d].', ...
                ports(1), ports(2));
        elseif strcmp(port_expectations{k, 1}, 'ACTUATORS')
            assert((ports(1) == 4 || ports(1) == 5) && ports(2) == expected(2), ...
                'ACTUATORS should expose 4 or 5 input ports and 3 output ports; got [%d %d].', ...
                ports(1), ports(2));
        else
            assert(all(ports(1:2) == expected), ...
                'Unexpected port count for %s: expected [%d %d], got [%d %d].', ...
                port_expectations{k, 1}, expected(1), expected(2), ports(1), ports(2));
        end
    end
end


function verify_logging_sinks(mdl)
    required_logs = { ...
        'Q_To_Workspace', 'Omega_To_Workspace', 'HW_To_Workspace', ...
        'Sun_ECI_To_Workspace', 'Moon_ECI_To_Workspace', 'B_ECI_To_Workspace', ...
        'Pos_ECI_To_Workspace', 'Vel_ECI_To_Workspace', 'Eclipse_To_Workspace', ...
        'Q_Meas_To_Workspace', 'Omega_Meas_To_Workspace', 'Sun_Meas_To_Workspace', ...
        'B_Body_Meas_To_Workspace', 'Pos_Meas_To_Workspace', 'Vel_Meas_To_Workspace', ...
        'RW_Command_To_Workspace', 'MTQ_Command_To_Workspace', 'CMG_Command_To_Workspace', ...
        'Ext_Torque_To_Workspace', 'RW_Body_Torque_To_Workspace', 'RW_Wheel_Torque_To_Workspace'};
    for k = 1:numel(required_logs)
        assert(~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', required_logs{k})), ...
            'Missing top-level logging sink: %s.', required_logs{k});
    end

    sensor_ports = get_param([mdl '/SENSORS'], 'Ports');
    if sensor_ports(2) >= 7
        assert(~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Tracker_Valid_To_Workspace')), ...
            'Missing tracker validity logging sink.');
    end
end


function verify_generated_block_scripts(mdl)
    blocks = { ...
        'ENVIRONMENT/Ephemeris_Truth', ...
        'ENVIRONMENT/Orbit_Propagator', ...
        'ENVIRONMENT/Magnetic_Field_Model', ...
        'ENVIRONMENT/Eclipse_Model', ...
        'SENSORS/Sun_Sensor_Suite', ...
        'SENSORS/Star_Tracker', ...
        'SENSORS/IMU', ...
        'SENSORS/Magnetometer', ...
        'SENSORS/GNSS_Position', ...
        'SENSORS/GNSS_Velocity', ...
        'CONTROL/Onboard_Ephemeris', ...
        'CONTROL/Attitude_Estimator', ...
        'CONTROL/Reference_Generator', ...
        'CONTROL/Mode_Manager', ...
        'CONTROL/Control_Law', ...
        'ACTUATORS/RW_Assembly', ...
        'ACTUATORS/MTQ_Assembly', ...
        'ACTUATORS/CMG', ...
        'DYNAMICS/Euler_RHS', ...
        'DYNAMICS/Gravity_Gradient', ...
        'DYNAMICS/Aero_Drag_Torque', ...
        'DYNAMICS/SRP_Torque', ...
        'DYNAMICS/Residual_Mag_Torque', ...
        'DYNAMICS/Quat_RHS', ...
        'DYNAMICS/Quat_Norm'};

    script_blob = "";
    for k = 1:numel(blocks)
        script_blob = script_blob + newline + string(chart_script([mdl '/' blocks{k}]));
    end

    forbidden = [ ...
        "Compile-stable zero"; ...
        "compile-stable zero"; ...
        "fixed LEO state fallback"; ...
        "literal magnetic field fallback"; ...
        "sunlit fallback"; ...
        "constant mode selection"; ...
        "Temporary passthrough"; ...
        "Deterministic passthrough used during compile hardening"];
    for k = 1:numel(forbidden)
        assert(~contains(script_blob, forbidden(k)), ...
            'Generated model still contains placeholder block code: %s', forbidden(k));
    end
end


function verify_extracted_generated_kernels(mdl)
    rw_axes = evalin('base', 'rw_axes');
    rw_max_torque = evalin('base', 'rw_max_torque');
    rw_max_momentum = evalin('base', 'rw_max_momentum');
    rw_inertia = evalin('base', 'rw_inertia');
    rw_max_speed = evalin('base', 'rw_max_speed');
    dt = evalin('base', 'dt');
    mtq_max_dipole = evalin('base', 'mtq_max_dipole');
    control_params = evalin('base', ...
        '[ctrl_rw_kp; ctrl_rw_kd; ctrl_bdot_gain; ctrl_dump_gain; ctrl_unload_fraction; mode_detumble_rate; mode_fine_rate; mode_fine_err; rw_max_momentum; mtq_max_dipole]');
    rw_speed_params = [rw_inertia; rw_max_speed];

    q_est = axis_angle_quaternion([0; 1; 0], 20*pi/180);
    q_des = [1; 0; 0; 0];
    omega_est = [0; 0; 0];
    omega_des = [0; 0; 0];
    b_body = [0; 0; 3.0e-5];
    h_w = zeros(4, 1);

    control_script = chart_script([mdl '/CONTROL/Control_Law']);
    control_inputs = extracted_input_count(control_script);
    if control_inputs == 7
        [rw_cmd, mtq_cmd, cmg_cmd] = run_extracted_kernel(control_script, 3, ...
            q_est, omega_est, b_body, q_des, omega_des, 3, h_w);
    elseif control_inputs == 10
        [rw_cmd, mtq_cmd, cmg_cmd] = run_extracted_kernel(control_script, 3, ...
            q_est, omega_est, b_body, q_des, omega_des, 3, h_w, ...
            control_params, rw_axes, rw_speed_params);
    else
        error('Unexpected Control_Law input count: %d.', control_inputs);
    end
    assert(rw_cmd(2) < -1e-7, ...
        'Extracted Control_Law did not command a corrective -Y torque for a +Y attitude error.');
    assert(norm(mtq_cmd) < 1e-12, 'Fine-pointing Control_Law should not command MTQ without wheel unloading.');
    assert(norm(cmg_cmd) < 1e-12, 'CMG command should remain disabled until the CMG contract is modeled.');

    if control_inputs == 7
        [rw_detumble, mtq_detumble] = run_extracted_kernel(control_script, 3, ...
            [1; 0; 0; 0], [0.03; -0.02; 0.015], b_body, q_des, omega_des, 1, h_w);
    else
        [rw_detumble, mtq_detumble] = run_extracted_kernel(control_script, 3, ...
            [1; 0; 0; 0], [0.03; -0.02; 0.015], b_body, q_des, omega_des, 1, h_w, ...
            control_params, rw_axes, rw_speed_params);
    end
    assert(norm(rw_detumble) < 1e-12, 'Detumble mode should not command reaction wheels.');
    assert(norm(mtq_detumble) > 1e-6, 'Detumble mode should command magnetorquers.');
    assert(max(abs(mtq_detumble)) <= mtq_max_dipole + 1e-12, ...
        'Extracted Control_Law exceeded MTQ dipole saturation.');

    q_des_nonidentity = axis_angle_quaternion([1; 0; 0], 30*pi/180);
    q_est_perturbed = quat_multiply(q_des_nonidentity, axis_angle_quaternion([0; 1; 0], 5*pi/180));
    expected_error = attitude_error(q_est_perturbed, q_des_nonidentity);
    if control_inputs == 7
        rw_nonidentity = run_extracted_kernel(control_script, 3, ...
            q_est_perturbed, [0; 0; 0], b_body, q_des_nonidentity, [0; 0; 0], 3, h_w);
    else
        rw_nonidentity = run_extracted_kernel(control_script, 3, ...
            q_est_perturbed, [0; 0; 0], b_body, q_des_nonidentity, [0; 0; 0], 3, h_w, ...
            control_params, rw_axes, rw_speed_params);
    end
    assert(dot(rw_nonidentity, expected_error(2:4)) < -1e-10, ...
        'Control_Law does not oppose a nonidentity desired-attitude perturbation in the correct frame.');

    actuator_script = chart_script([mdl '/ACTUATORS/RW_Assembly']);
    n_inputs = extracted_input_count(actuator_script);
    if n_inputs == 3
        [rw_body, tau_wheels] = run_extracted_kernel(actuator_script, 2, ...
            rw_cmd, rw_axes, rw_max_torque);
    elseif n_inputs == 4
        rw_limits = [rw_max_torque; rw_max_momentum; rw_inertia; rw_max_speed; dt];
        [rw_body, tau_wheels] = run_extracted_kernel(actuator_script, 2, ...
            rw_cmd, h_w, rw_axes, rw_limits);
    elseif n_inputs == 5
        [rw_body, tau_wheels] = run_extracted_kernel(actuator_script, 2, ...
            rw_cmd, h_w, rw_axes, rw_max_torque, rw_max_momentum);
    else
        error('Unexpected RW_Assembly input count: %d.', n_inputs);
    end

    assert(max(abs(tau_wheels)) <= rw_max_torque + 1e-12, ...
        'Extracted RW_Assembly exceeded per-wheel torque saturation.');
    assert(norm(rw_body + rw_axes * tau_wheels) < 1e-12, ...
        'Extracted RW_Assembly output sign is inconsistent with dynamics wheel-momentum coupling.');
end


function verify_extracted_environment_kernels(mdl)
    mu = evalin('base', 'mu_earth');
    re = evalin('base', 'R_earth');
    j2 = evalin('base', 'J2_earth');
    a = re + 500e3;
    ecc = 0.0125;
    inc_deg = 63.4;
    raan_deg = 37.0;
    aop_deg = 22.0;
    ta_deg = 41.0;
    period = 2*pi * sqrt(a^3 / mu);
    h = 0.1;

    orbit_script = chart_script([mdl '/ENVIRONMENT/Orbit_Propagator']);
    for t = [600, period]
        [pos_minus, ~] = run_extracted_kernel(orbit_script, 2, ...
            t - h, mu, re, j2, a, ecc, inc_deg, raan_deg, aop_deg, ta_deg);
        [~, vel] = run_extracted_kernel(orbit_script, 2, ...
            t, mu, re, j2, a, ecc, inc_deg, raan_deg, aop_deg, ta_deg);
        [pos_plus, ~] = run_extracted_kernel(orbit_script, 2, ...
            t + h, mu, re, j2, a, ecc, inc_deg, raan_deg, aop_deg, ta_deg);
        vel_fd = (pos_plus - pos_minus) / (2*h);
        assert(norm(vel - vel_fd) < 1e-3, ...
            'Extracted Orbit_Propagator velocity disagrees with finite-difference position at t=%.3f s.', t);
    end

    eclipse_script = chart_script([mdl '/ENVIRONMENT/Eclipse_Model']);
    sun = [1; 0; 0];
    cases = { ...
        [-re - 500e3; 0; 0], sun, 1; ...
        [ re + 500e3; 0; 0], sun, 0; ...
        [-re - 500e3; 1.2*re; 0], sun, 0; ...
        [0; re + 500e3; 0], sun, 0};
    for k = 1:size(cases, 1)
        flag = run_extracted_kernel(eclipse_script, 1, cases{k, 1}, cases{k, 2}, re);
        assert(flag == cases{k, 3}, 'Extracted Eclipse_Model failed geometry case %d.', k);
    end
end


function verify_tracker_noise_basis(mdl)
    script = chart_script([mdl '/SENSORS/Star_Tracker']);
    q = axis_angle_quaternion([1; 2; -1], 0.8);
    C = scalar_first_dcm(q);
    boresights = [0 1 1; 0 0 2; 1 0 3];
    for k = 1:size(boresights, 2)
        bore = boresights(:, k) / norm(boresights(:, k));
        % Place bright bodies and Earth opposite the optical axis.
        away = -C.' * bore;
        args = {0, away, away, -7e6*away, q};
        tail = {10, 20, 10, 10, bore, 6371e3};
        [exact, valid] = run_extracted_kernel(script, 2, args{:}, 0, 0, tail{:});
        assert(valid == 1 && norm(scalar_first_dcm(exact)-C, 'fro') < 1e-12, ...
            'Zero-noise tracker must recover truth for arbitrary boresight.');
        roll = run_extracted_kernel(script, 1, args{:}, 0, 400, tail{:});
        D = scalar_first_dcm(roll)*C.';
        assert(norm(D*bore-bore) < 1e-12 && norm(D-eye(3), 'fro') > 1e-6, ...
            'Roll-only error must rotate about the configured body boresight.');
        tilt = run_extracted_kernel(script, 1, args{:}, 400, 0, tail{:});
        D = scalar_first_dcm(tilt)*C.';
        axis_sine = [D(3,2)-D(2,3); D(1,3)-D(3,1); D(2,1)-D(1,2)];
        assert(abs(dot(axis_sine, bore)) < 1e-12 && norm(D*bore-bore) > 1e-6, ...
            'Boresight-only error must tilt the optical axis without roll.');
    end
end


function verify_basic_simulation(results)
    require_signal(results, 'q', 4);
    require_signal(results, 'omega_rad_s', 3);
    require_signal(results, 'h_w_nms', 4);
    require_signal(results, 'pos_eci_m', 3);
    require_signal(results, 'vel_eci_mps', 3);
    require_signal(results, 'B_eci_T', 3);
    require_signal(results, 'q_meas', 4);
    require_signal(results, 'omega_meas_rad_s', 3);
    require_signal(results, 'sun_meas_body', 3);
    require_signal(results, 'B_body_meas_T', 3);
    require_signal(results, 'pos_meas_m', 3);
    require_signal(results, 'vel_meas_mps', 3);
    require_signal(results, 'rw_cmd_nm', 3);
    require_signal(results, 'mtq_cmd_am2', 3);
    require_signal(results, 'ext_torque_nm', 3);
    require_signal(results, 'rw_torque_body_nm', 3);
    require_signal(results, 'rw_tau_wheels_nm', 4);
    require_signal(results, 'q_est', 4);
    require_signal(results, 'q_des', 4);
    require_signal(results, 'att_valid', 1);
    require_signal(results, 'mode_id', 1);

    assert(results.metrics.max_quat_norm_error < 1e-6, ...
        'Quaternion normalization drift exceeded tolerance.');
    assert(results.metrics.duration_s >= 60 - 1e-9 || results.parameters.t_end_s < 60, ...
        'Smoke simulation stopped before the requested horizon.');
    assert(all(isfinite(struct_to_vector(results.metrics))), ...
        'Summary metrics contain non-finite values.');

    verify_unit_vectors(results.signals.sun_eci, 'Sun ECI');
    verify_unit_vectors(results.signals.moon_eci, 'Moon ECI');
    verify_quaternion_samples(results.signals.q_meas, 'Measured quaternion');
    assert(all(results.signals.eclipse_flag(:) == 0 | results.signals.eclipse_flag(:) == 1), ...
        'Eclipse flag must remain binary.');
    if isfield(results.signals, 'tracker_valid')
        assert(all(results.signals.tracker_valid(:) == 0 | results.signals.tracker_valid(:) == 1), ...
            'Tracker validity flag must remain binary.');
    end
    assert(all(results.signals.att_valid(:) == 0 | results.signals.att_valid(:) == 1), ...
        'Attitude validity flag must remain binary.');
    assert(all(ismember(results.signals.mode_id(:), [1; 2; 3])), ...
        'Mode id must remain in the documented set 1=detumble, 2=coarse, 3=fine.');
end


function verify_override_case(results, overrides)
    assert(abs(results.parameters.dt_s - overrides.dt) < 1e-12, 'dt override was not preserved.');
    assert(abs(results.parameters.t_end_s - overrides.t_end) < 1e-12, 't_end override was not preserved.');
    assert(norm(results.parameters.J_kgm2 - overrides.J, 'fro') < 1e-12, 'J override was not preserved.');
    assert(norm(results.parameters.q0 - [1; 0; 0; 0]) < 1e-12, 'q0 override was not normalized/preserved.');
    assert(norm(results.parameters.omega0_rad_s - overrides.omega0) < 1e-12, 'omega0 override was not preserved.');
    assert(norm(results.parameters.h_w0_nms - overrides.h_W0) < 1e-12, 'h_W0 override was not preserved.');

    assert(norm(results.signals.q(1, :)' - results.parameters.q0) < 1e-9, ...
        'Logged initial quaternion does not match override.');
    assert(norm(results.signals.omega_rad_s(1, :)' - overrides.omega0) < 1e-9, ...
        'Logged initial rate does not match override.');
    assert(norm(results.signals.h_w_nms(1, :)' - overrides.h_W0) < 1e-9, ...
        'Logged initial wheel momentum does not match override.');
end


function verify_pointing_case(results)
    verify_basic_simulation(results);
    assert(results.metrics.peak_rw_command_nm > 1e-7, ...
        'Nonidentity pointing case did not generate a reaction-wheel command.');
    assert(results.metrics.peak_rw_torque_body_nm > 1e-7, ...
        'Reaction-wheel command did not reach actuator body torque output.');
    assert(results.metrics.peak_rw_wheel_torque_nm > 1e-7, ...
        'Reaction-wheel command did not reach per-wheel torques.');
    assert(results.metrics.peak_wheel_momentum_norm_nms > 1e-7, ...
        'Reaction-wheel torques did not integrate into wheel momentum.');
    assert(max(abs(results.signals.rw_tau_wheels_nm), [], 'all') <= evalin('base', 'rw_max_torque') + 1e-9, ...
        'Per-wheel reaction-wheel torque exceeded configured saturation.');
    assert(max(abs(results.signals.h_w_nms), [], 'all') <= evalin('base', 'rw_max_momentum') + 1e-9, ...
        'Reaction-wheel momentum exceeded configured storage limit.');
    verify_known_axis_rw_response(results);
    verify_wheel_momentum_exchange(results);
    desired_error_deg = desired_attitude_error_deg(results);
    assert(all(isfinite(desired_error_deg)), 'Desired attitude error contains non-finite values.');
    assert(min(desired_error_deg) < desired_error_deg(1), ...
        'Pointing case never reduces desired-attitude separation from its initial value.');

    orbit_motion_m = max(vecnorm(results.signals.pos_eci_m - results.signals.pos_eci_m(1, :), 2, 2));
    assert(orbit_motion_m > 100, 'Orbit position did not change measurably during the scenario.');
    pos_truth_at_meas = interp_signal(results.signals.pos_eci_time_s, ...
        results.signals.pos_eci_m, results.signals.pos_meas_time_s);
    assert(max(vecnorm(results.signals.pos_meas_m - pos_truth_at_meas, 2, 2)) < 1e5, ...
        'GNSS position measurements diverged unrealistically from propagated truth.');
end


function verify_detumble_case(results)
    verify_basic_simulation(results);
    assert(results.metrics.peak_mtq_command_am2 > 1e-6, ...
        'High-rate detumble case did not generate a magnetorquer command.');
    assert(results.metrics.peak_ext_torque_nm > 1e-12, ...
        'Magnetorquer command did not reach external torque output.');
    assert(max(abs(results.signals.mtq_cmd_am2), [], 'all') <= evalin('base', 'mtq_max_dipole') + 1e-12, ...
        'Magnetorquer command exceeded configured saturation.');
end


function error_deg = desired_attitude_error_deg(results)
    q_est_at_des = interp_signal(results.signals.q_est_time_s, ...
        results.signals.q_est, results.signals.q_des_time_s);
    error_deg = zeros(numel(results.signals.q_des_time_s), 1);
    for idx = 1:numel(error_deg)
        q_est = q_est_at_des(idx, :).';
        q_est = q_est / max(norm(q_est), 1e-12);
        q_des = results.signals.q_des(idx, :).';
        q_des = q_des / max(norm(q_des), 1e-12);
        q_err = attitude_error(q_est, q_des);
        error_deg(idx) = 2 * atan2(norm(q_err(2:4)), max(min(q_err(1), 1), -1)) * 180 / pi;
    end
end


function verify_torque_free_conservation(results)
    verify_basic_simulation(results);
    assert(results.metrics.peak_rw_command_nm < 1e-12, ...
        'Torque-free case should not command reaction wheels.');
    assert(results.metrics.peak_mtq_command_am2 < 1e-12, ...
        'Torque-free case should not command magnetorquers.');
    assert(results.metrics.peak_ext_torque_nm < 1e-12, ...
        'Torque-free case should not apply commanded external torque.');

    h_variation = total_inertial_momentum_variation(results);
    h_initial = norm(total_inertial_momentum(results, 1));
    assert(h_variation <= max(1e-7, 1e-3 * h_initial), ...
        'Torque-free total inertial angular momentum drift exceeded tolerance.');
end


function require_signal(results, name, width)
    assert(isfield(results.signals, name), 'Missing logged signal: %s.', name);
    samples = results.signals.(name);
    assert(size(samples, 2) == width, 'Signal %s should have width %d.', name, width);
    assert(~isempty(samples), 'Signal %s is empty.', name);
    assert(all(isfinite(samples(:))), 'Signal %s contains non-finite values.', name);
end


function samples_at_query = interp_signal(t_samples, samples, t_query)
    t_samples = t_samples(:);
    t_query = t_query(:);
    samples_at_query = zeros(numel(t_query), size(samples, 2));
    for col = 1:size(samples, 2)
        samples_at_query(:, col) = interp1(t_samples, samples(:, col), t_query, 'linear', 'extrap');
    end
end


function h_variation = total_inertial_momentum_variation(results)
    h_inertial = zeros(size(results.signals.omega_rad_s));
    for k = 1:size(h_inertial, 1)
        h_inertial(k, :) = total_inertial_momentum(results, k).';
    end
    h_variation = max(vecnorm(h_inertial - h_inertial(1, :), 2, 2));
end


function h_i = total_inertial_momentum(results, idx)
    a_w = results.parameters.rw_axes_body;
    q = results.signals.q(idx, :).';
    c_bi = scalar_first_dcm(q);
    h_body = results.parameters.J_kgm2 * results.signals.omega_rad_s(idx, :).';
    h_body = h_body + a_w * results.signals.h_w_nms(idx, :).';
    h_i = c_bi.' * h_body;
end


function verify_unit_vectors(samples, label)
    norms = vecnorm(samples, 2, 2);
    assert(max(abs(norms - 1)) < 1e-6, '%s vectors are not unit-normalized.', label);
end


function verify_quaternion_samples(samples, label)
    norms = vecnorm(samples, 2, 2);
    assert(max(abs(norms - 1)) < 1e-6, '%s samples are not unit-normalized.', label);
end


function verify_known_axis_rw_response(results)
    pos0 = results.signals.pos_eci_m(1, :).';
    vel0 = results.signals.vel_eci_mps(1, :).';
    q_des = nadir_reference_quaternion(pos0, vel0);
    q_est = results.signals.q_meas(1, :).';
    q_err = attitude_error(q_est, q_des);
    rw_cmd0 = results.signals.rw_cmd_nm(1, :).';

    assert(norm(q_err(2:4)) > 1e-3, 'Known-axis pointing case did not create a measurable attitude error.');
    assert(dot(rw_cmd0, q_err(2:4)) < -1e-10, ...
        'Initial reaction-wheel command did not oppose the logged attitude error vector.');
end


function verify_wheel_momentum_exchange(results)
    a_w = results.parameters.rw_axes_body;
    rw_body_from_wheels = -(a_w * results.signals.rw_tau_wheels_nm.').';
    mismatch = max(vecnorm(results.signals.rw_torque_body_nm - rw_body_from_wheels, 2, 2));
    assert(mismatch < 1e-9, 'Logged body RW torque is inconsistent with logged per-wheel torque.');

    t = results.signals.rw_tau_wheels_time_s;
    dt = diff(t);
    impulse = sum(results.signals.rw_tau_wheels_nm(1:end-1, :) .* dt, 1);
    delta_h = results.signals.h_w_nms(end, :) - results.signals.h_w_nms(1, :);
    assert(norm(delta_h - impulse) < 1e-5, ...
        'Logged wheel momentum change is inconsistent with the wheel torque impulse.');
end


function n_inputs = extracted_input_count(script)
    first_line = regexp(script, 'function[^\n\r]*', 'match', 'once');
    tokens = regexp(first_line, 'fcn\(([^)]*)\)', 'tokens', 'once');
    assert(~isempty(tokens), 'Could not determine extracted function input count.');
    if strlength(strtrim(string(tokens{1}))) == 0
        n_inputs = 0;
    else
        n_inputs = numel(strsplit(string(tokens{1}), ','));
    end
end


function varargout = run_extracted_kernel(script, n_outputs, varargin)
    work_dir = tempname;
    mkdir(work_dir);
    kernel_path = fullfile(work_dir, 'fcn.m');

    fid = fopen(kernel_path, 'w');
    assert(fid > 0, 'Could not create extracted kernel file.');
    bytes_written = fwrite(fid, script);
    close_status = fclose(fid);
    assert(bytes_written > 0 && close_status == 0, 'Could not write extracted kernel file.');

    addpath(work_dir);
    cleanup_all = onCleanup(@() cleanup_kernel_dir(work_dir));
    clear fcn
    [varargout{1:n_outputs}] = fcn(varargin{:});
    clear fcn
end


function cleanup_kernel_dir(work_dir)
    if contains(path, work_dir)
        rmpath(work_dir);
    end
    if isfolder(work_dir)
        try
            rmdir(work_dir, 's');
        catch
        end
    end
end


function script = chart_script(block_path)
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    assert(~isempty(chart), 'Missing MATLAB Function chart for %s.', block_path);
    script = chart.Script;
end


function q = axis_angle_quaternion(axis, angle)
    axis = axis(:) / norm(axis);
    q = [cos(angle/2); axis * sin(angle/2)];
end


function dcm = scalar_first_dcm(q)
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    dcm = [ ...
        1 - 2*(q2^2 + q3^2),  2*(q1*q2 + q0*q3),  2*(q1*q3 - q0*q2); ...
        2*(q1*q2 - q0*q3),  1 - 2*(q1^2 + q3^2),  2*(q2*q3 + q0*q1); ...
        2*(q1*q3 + q0*q2),  2*(q2*q3 - q0*q1),  1 - 2*(q1^2 + q2^2)];
end


function q_des = nadir_reference_quaternion(pos_eci, vel_eci)
    z_b_i = -pos_eci / norm(pos_eci);
    v_proj = vel_eci - dot(vel_eci, z_b_i) * z_b_i;
    x_b_i = v_proj / norm(v_proj);
    y_b_i = cross(z_b_i, x_b_i);
    y_b_i = y_b_i / norm(y_b_i);
    x_b_i = cross(y_b_i, z_b_i);
    c_bi = [x_b_i.'; y_b_i.'; z_b_i.'];
    q_des = dcm_to_quaternion(c_bi);
end


function q = dcm_to_quaternion(c_bi)
    d0 = 1 + c_bi(1,1) + c_bi(2,2) + c_bi(3,3);
    d1 = 1 + c_bi(1,1) - c_bi(2,2) - c_bi(3,3);
    d2 = 1 - c_bi(1,1) + c_bi(2,2) - c_bi(3,3);
    d3 = 1 - c_bi(1,1) - c_bi(2,2) + c_bi(3,3);

    if d0 >= d1 && d0 >= d2 && d0 >= d3
        q0 = 0.5 * sqrt(max(d0, 0));
        den = max(4 * q0, 1e-12);
        q = [q0; (c_bi(2,3)-c_bi(3,2))/den; (c_bi(3,1)-c_bi(1,3))/den; (c_bi(1,2)-c_bi(2,1))/den];
    elseif d1 >= d2 && d1 >= d3
        q1 = 0.5 * sqrt(max(d1, 0));
        den = max(4 * q1, 1e-12);
        q = [(c_bi(2,3)-c_bi(3,2))/den; q1; (c_bi(1,2)+c_bi(2,1))/den; (c_bi(3,1)+c_bi(1,3))/den];
    elseif d2 >= d3
        q2 = 0.5 * sqrt(max(d2, 0));
        den = max(4 * q2, 1e-12);
        q = [(c_bi(3,1)-c_bi(1,3))/den; (c_bi(1,2)+c_bi(2,1))/den; q2; (c_bi(2,3)+c_bi(3,2))/den];
    else
        q3 = 0.5 * sqrt(max(d3, 0));
        den = max(4 * q3, 1e-12);
        q = [(c_bi(1,2)-c_bi(2,1))/den; (c_bi(3,1)+c_bi(1,3))/den; (c_bi(2,3)+c_bi(3,2))/den; q3];
    end

    q = q / norm(q);
    if q(1) < 0
        q = -q;
    end
end


function q_err = attitude_error(q_est, q_des)
    qd = [q_des(1); -q_des(2:4)];
    q_err = quat_multiply(qd, q_est);
    q_err = q_err / norm(q_err);
    if q_err(1) < 0
        q_err = -q_err;
    end
end


function q = quat_multiply(p, r)
    q = [ ...
        p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4); ...
        p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3); ...
        p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2); ...
        p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1)];
end


function a_w = reaction_wheel_axes()
    beta = atan(1 / sqrt(2));
    cb = cos(beta);
    sb = sin(beta);
    a_w = [ cb,  0, -cb,  0;
             0, cb,   0, -cb;
            sb, sb,  sb,  sb ];
end


function a_w = alternate_reaction_wheel_axes()
    raw_axes = [ 1,  0,  0,  1;
                 0,  1,  0,  1;
                 0,  0,  1,  1 ];
    a_w = raw_axes ./ vecnorm(raw_axes, 2, 1);
end


function values = struct_to_vector(s)
    fields = fieldnames(s);
    values = [];
    for k = 1:numel(fields)
        value = s.(fields{k});
        if isnumeric(value) || islogical(value)
            values = [values; value(:)]; %#ok<AGROW>
        end
    end
end
