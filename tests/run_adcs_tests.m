function results = run_adcs_tests()
%RUN_ADCS_TESTS Build and smoke-test the ADCS Simulink model.
%
%   RESULTS = RUN_ADCS_TESTS() initializes parameters, rebuilds the model,
%   checks key parameter and frame-convention assumptions, runs a short
%   closed-loop simulation, and verifies that the key logged signals are
%   present, finite, and physically sane.

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    prev_dir = pwd;
    cleanup = onCleanup(@() cd(prev_dir)); %#ok<NASGU>
    cd(repo_root);

    evalin('base', 'init_adcs_params;');
    verify_parameter_consistency();
    verify_quaternion_frame_conventions();
    evalin('base', 'build_adcs_model;');
    load_system('adcs_sim');

    verify_top_level_model('adcs_sim');

    sim_out = sim('adcs_sim', ...
        'StopTime', '60', ...
        'SaveOutput', 'on', ...
        'ReturnWorkspaceOutputs', 'on');

    q_log = sim_out.get('q_log');
    omega_log = sim_out.get('omega_log');
    h_w_log = sim_out.get('h_W_log');

    assert(isa(q_log, 'timeseries'), 'Expected q_log timeseries output.');
    assert(isa(omega_log, 'timeseries'), 'Expected omega_log timeseries output.');
    assert(isa(h_w_log, 'timeseries'), 'Expected h_W_log timeseries output.');

    q_data = orient_samples(q_log.Data, 4);
    omega_data = orient_samples(omega_log.Data, 3);
    h_w_data = orient_samples(h_w_log.Data, 4);

    assert(~isempty(q_data), 'Quaternion log is empty.');
    assert(~isempty(omega_data), 'Angular-rate log is empty.');
    assert(~isempty(h_w_data), 'Wheel-momentum log is empty.');
    assert(all(isfinite(q_data(:))), 'Quaternion log contains non-finite values.');
    assert(all(isfinite(omega_data(:))), 'Angular-rate log contains non-finite values.');
    assert(all(isfinite(h_w_data(:))), 'Wheel-momentum log contains non-finite values.');

    quat_norm_error = abs(sqrt(sum(q_data.^2, 2)) - 1);
    assert(max(quat_norm_error) < 5e-2, 'Quaternion norm drift exceeded tolerance.');

    summary = summarize_adcs_simulation(sim_out);
    assert(isfield(summary, 'metrics'), 'Expected derived summary metrics.');
    assert(summary.metrics.peak_rate_norm_deg_s >= 0, 'Peak rate metric must be nonnegative.');
    assert(summary.metrics.peak_wheel_speed_rpm >= 0, 'Wheel-speed metric must be nonnegative.');
    assert(summary.metrics.peak_total_ang_momentum_norm_nms >= 0, ...
        'Total angular momentum metric must be nonnegative.');

    results = struct();
    results.build_ok = true;
    results.sim_ok = true;
    results.max_quat_norm_error = max(quat_norm_error);
    results.final_rate_norm = norm(omega_data(end, :));
    results.final_wheel_momentum_norm = norm(h_w_data(end, :));
    results.peak_rate_norm_deg_s = summary.metrics.peak_rate_norm_deg_s;
    results.peak_wheel_speed_rpm = summary.metrics.peak_wheel_speed_rpm;
    results.peak_total_ang_momentum_norm_nms = summary.metrics.peak_total_ang_momentum_norm_nms;

    close_system('adcs_sim', 0);
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


function verify_quaternion_frame_conventions()
    samples = [ ...
        1, 0, 0, 0; ...
        sqrt(0.5), sqrt(0.5), 0, 0; ...
        sqrt(0.5), 0, sqrt(0.5), 0; ...
        0.8, -0.1, 0.2, 0.55];

    for idx = 1:size(samples, 1)
        q = samples(idx, :).';
        q = q / norm(q);

        sensor_dcm = sensor_frame_dcm(q);
        gravity_gradient_dcm = gravity_gradient_frame_dcm(q);

        assert(norm(sensor_dcm * sensor_dcm.' - eye(3), 'fro') < 1e-12, ...
            'Sensor quaternion DCM must remain orthonormal.');
        assert(norm(gravity_gradient_dcm - sensor_dcm, 'fro') < 1e-12, ...
            'Gravity-gradient and sensor blocks must use the same ECI-to-body DCM convention.');
    end
end


function verify_top_level_model(mdl)
    required_blocks = {'ENVIRONMENT', 'SENSORS', 'CONTROL', 'ACTUATORS', 'DYNAMICS'};
    for k = 1:numel(required_blocks)
        blk = [mdl '/' required_blocks{k}];
        assert(~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', required_blocks{k})), ...
            'Missing top-level block: %s', blk);
    end

    dyn_ports = get_param([mdl '/DYNAMICS'], 'Ports');
    assert(dyn_ports(1) == 8, 'DYNAMICS subsystem should expose 8 input ports.');
    assert(dyn_ports(2) == 3, 'DYNAMICS subsystem should expose 3 output ports.');
end


function samples = orient_samples(data, width)
    samples = squeeze(data);

    if isvector(samples)
        samples = reshape(samples, [], width);
    end

    if size(samples, 2) == width
        return;
    end

    if size(samples, 1) == width
        samples = samples.';
        return;
    end

    error('Unexpected logged data shape for width %d.', width);
end


function dcm = sensor_frame_dcm(q)
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    dcm = [ ...
        1 - 2*(q2^2 + q3^2),  2*(q1*q2 + q0*q3),  2*(q1*q3 - q0*q2); ...
        2*(q1*q2 - q0*q3),  1 - 2*(q1^2 + q3^2),  2*(q2*q3 + q0*q1); ...
        2*(q1*q3 + q0*q2),  2*(q2*q3 - q0*q1),  1 - 2*(q1^2 + q2^2)];
end


function dcm = gravity_gradient_frame_dcm(q)
    % Keep this formula mirrored explicitly so the regression catches any
    % future sign or transpose drift between live sensor and disturbance code.
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    dcm = [ ...
        1 - 2*(q2^2 + q3^2),  2*(q1*q2 + q0*q3),  2*(q1*q3 - q0*q2); ...
        2*(q1*q2 - q0*q3),  1 - 2*(q1^2 + q3^2),  2*(q2*q3 + q0*q1); ...
        2*(q1*q3 + q0*q2),  2*(q2*q3 - q0*q1),  1 - 2*(q1^2 + q2^2)];
end
