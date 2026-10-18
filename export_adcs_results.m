function manifest = export_adcs_results(results, output_dir, base_name, options)
%EXPORT_ADCS_RESULTS Save ADCS simulation results for review and replay.
%
%   MANIFEST = EXPORT_ADCS_RESULTS(RESULTS, OUTPUT_DIR, BASE_NAME) writes:
%     - BASE_NAME.mat      full MATLAB results struct
%     - BASE_NAME_replay.json     viewer replay JSON, schema utat.adcs.replay.v1
%     - BASE_NAME_timeseries.json downsampled diagnostic signal JSON
%     - BASE_NAME_metrics.csv / .json
%     - figures/*.png      review plots from the same run

    arguments
        results (1,1) struct
        output_dir (1,:) char = fullfile(pwd, 'validation_logs')
        base_name (1,:) char = 'final_adcs_results'
        options.MaxSamples (1,1) double {mustBePositive} = 3001
        options.MakeFigures (1,1) logical = true
    end

    assert(isfield(results, 'signals') && isfield(results, 'metrics'), ...
        'Expected results from simulate_adcs_case.');

    if ~isfolder(output_dir)
        mkdir(output_dir);
    end

    fig_dir = fullfile(output_dir, 'figures');
    if options.MakeFigures && ~isfolder(fig_dir)
        mkdir(fig_dir);
    end

    mat_path = fullfile(output_dir, [base_name '.mat']);
    replay_json_path = fullfile(output_dir, [base_name '_replay.json']);
    timeseries_json_path = fullfile(output_dir, [base_name '_timeseries.json']);
    metrics_csv_path = fullfile(output_dir, [base_name '_metrics.csv']);
    metrics_json_path = fullfile(output_dir, [base_name '_metrics.json']);

    save(mat_path, 'results', '-v7.3');
    writecell(results.metric_rows, metrics_csv_path);

    enriched_metrics = enrich_metrics(results);
    metrics_json = jsonencode(enriched_metrics, 'PrettyPrint', true);
    write_text(metrics_json_path, metrics_json);

    replay = build_browser_replay(results, options.MaxSamples);
    write_text(replay_json_path, jsonencode(replay, 'PrettyPrint', true));

    timeseries = build_timeseries_replay(results, options.MaxSamples);
    write_text(timeseries_json_path, jsonencode(timeseries, 'PrettyPrint', true));

    figure_paths = strings(0, 1);
    if options.MakeFigures
        figure_paths = write_review_figures(results, fig_dir, base_name);
    end

    manifest = struct( ...
        'mat', mat_path, ...
        'json', replay_json_path, ...
        'replay_json', replay_json_path, ...
        'timeseries_json', timeseries_json_path, ...
        'metrics_csv', metrics_csv_path, ...
        'metrics_json', metrics_json_path, ...
        'figures', figure_paths);
end


function replay = build_browser_replay(results, max_samples)
    params = results.parameters;
    t = viewer_time_grid(results, max_samples);
    desired_error = desired_attitude_error_series(results);

    pos_eci_km = interp_optional(results, 'pos_eci_time_s', 'pos_eci_m', t, 3, NaN) / 1e3;
    vel_eci_kmps = interp_optional(results, 'vel_eci_time_s', 'vel_eci_mps', t, 3, NaN) / 1e3;
    q = normalize_rows(interp_optional(results, 'q_time_s', 'q', t, 4, [1 0 0 0]));
    q_est = normalize_rows(interp_optional(results, 'q_est_time_s', 'q_est', t, 4, [1 0 0 0]));
    q_des = normalize_rows(interp_optional(results, 'q_des_time_s', 'q_des', t, 4, [1 0 0 0]));
    omega = interp_optional(results, 'omega_time_s', 'omega_rad_s', t, 3, NaN);
    sun = interp_optional(results, 'sun_eci_time_s', 'sun_eci', t, 3, NaN);
    moon = interp_optional(results, 'moon_eci_time_s', 'moon_eci', t, 3, NaN);
    b_eci = interp_optional(results, 'B_eci_time_s', 'B_eci_T', t, 3, NaN);
    eclipse = interp_optional(results, 'eclipse_time_s', 'eclipse_flag', t, 1, 0);
    tracker_valid = interp_optional(results, 'tracker_valid_time_s', 'tracker_valid', t, 1, NaN);
    att_valid = interp_optional(results, 'att_valid_time_s', 'att_valid', t, 1, NaN);
    mode_id = interp_optional(results, 'mode_id_time_s', 'mode_id', t, 1, NaN);
    h_w = interp_optional(results, 'h_w_time_s', 'h_w_nms', t, 4, NaN);
    rw_cmd = interp_optional(results, 'rw_cmd_time_s', 'rw_cmd_nm', t, 3, NaN);
    rw_torque_body = interp_optional(results, 'rw_torque_body_time_s', 'rw_torque_body_nm', t, 3, NaN);
    rw_tau_wheels = interp_optional(results, 'rw_tau_wheels_time_s', 'rw_tau_wheels_nm', t, 4, NaN);
    mtq_cmd = interp_optional(results, 'mtq_cmd_time_s', 'mtq_cmd_am2', t, 3, NaN);
    ext_torque = interp_optional(results, 'ext_torque_time_s', 'ext_torque_nm', t, 3, NaN);
    qerr = interp1(desired_error.time_s, desired_error.values, t, 'linear', 'extrap');

    sample = struct( ...
        'tSec', 0, ...
        'positionEciKm', [NaN NaN NaN], ...
        'velocityEciKmps', [NaN NaN NaN], ...
        'qEciToBody', [1 0 0 0], ...
        'qEstimatedEciToBody', [1 0 0 0], ...
        'qDesiredEciToBody', [1 0 0 0], ...
        'omegaBodyRadS', [NaN NaN NaN], ...
        'sunEci', [NaN NaN NaN], ...
        'moonEci', [NaN NaN NaN], ...
        'magneticFieldEciT', [NaN NaN NaN], ...
        'eclipse', false, ...
        'trackerValid', NaN, ...
        'attitudeValid', NaN, ...
        'modeId', NaN, ...
        'reactionWheelMomentumNms', [NaN NaN NaN NaN], ...
        'reactionWheelCommandNm', [NaN NaN NaN], ...
        'reactionWheelTorqueBodyNm', [NaN NaN NaN], ...
        'reactionWheelTorqueWheelNm', [NaN NaN NaN NaN], ...
        'magnetorquerCommandAm2', [NaN NaN NaN], ...
        'externalTorqueNm', [NaN NaN NaN], ...
        'desiredAttitudeErrorDeg', NaN);
    samples = repmat(sample, numel(t), 1);
    for idx = 1:numel(t)
        samples(idx).tSec = t(idx);
        samples(idx).positionEciKm = row(pos_eci_km(idx, :));
        samples(idx).velocityEciKmps = row(vel_eci_kmps(idx, :));
        samples(idx).qEciToBody = row(q(idx, :));
        samples(idx).qEstimatedEciToBody = row(q_est(idx, :));
        samples(idx).qDesiredEciToBody = row(q_des(idx, :));
        samples(idx).omegaBodyRadS = row(omega(idx, :));
        samples(idx).sunEci = row(sun(idx, :));
        samples(idx).moonEci = row(moon(idx, :));
        samples(idx).magneticFieldEciT = row(b_eci(idx, :));
        samples(idx).eclipse = eclipse(idx) > 0.5;
        samples(idx).trackerValid = tracker_valid(idx);
        samples(idx).attitudeValid = att_valid(idx);
        samples(idx).modeId = mode_id(idx);
        samples(idx).reactionWheelMomentumNms = row(h_w(idx, :));
        samples(idx).reactionWheelCommandNm = row(rw_cmd(idx, :));
        samples(idx).reactionWheelTorqueBodyNm = row(rw_torque_body(idx, :));
        samples(idx).reactionWheelTorqueWheelNm = row(rw_tau_wheels(idx, :));
        samples(idx).magnetorquerCommandAm2 = row(mtq_cmd(idx, :));
        samples(idx).externalTorqueNm = row(ext_torque(idx, :));
        samples(idx).desiredAttitudeErrorDeg = qerr(idx);
    end

    replay = struct();
    replay.schema = 'utat.adcs.replay.v1';
    replay.provenance = struct( ...
        'model', 'adcs_sim', ...
        'source', 'MATLAB/Simulink exported by export_adcs_results.m', ...
        'generatedUtc', char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z''')), ...
        'durationSec', results.metrics.duration_s, ...
        'sampleCount', numel(samples), ...
        'samplePolicy', sprintf('uniform position-grid stride to at most %d replay samples', max_samples));
    replay.centralBody = struct( ...
        'name', 'Earth', ...
        'frame', 'ECI', ...
        'radiusKm', get_param_value(params, 'earth_radius_m', 6371e3) / 1e3, ...
        'muM3S2', get_param_value(params, 'earth_mu_m3_s2', 3.986004418e14), ...
        'j2', get_param_value(params, 'earth_j2', 1.08262668e-3));
    replay.simulation = struct( ...
        'epochJd', get_param_value(params, 'epoch_jd', NaN), ...
        'dtSec', get_param_value(params, 'dt_s', NaN), ...
        'tEndSec', get_param_value(params, 't_end_s', results.metrics.duration_s), ...
        'attitudeQuaternionConvention', 'scalar-first passive ECI-to-body', ...
        'positionFrame', 'ECI', ...
        'positionUnits', 'km');
    replay.spacecraft = struct( ...
        'dimensionsM', row(get_param_value(params, 'spacecraft_dimensions_m', [NaN; NaN; NaN])), ...
        'massKg', get_param_value(params, 'mass_sc_kg', NaN), ...
        'rwAxesBody', get_param_value(params, 'rw_axes_body', NaN(3, 4)), ...
        'mtqAxesBody', get_param_value(params, 'mtq_axes_body', eye(3)));
    replay.visualization = struct( ...
        'cameraBoresightBody', row(get_param_value(params, 'visualization_camera_boresight_body', [0; 0; 1])), ...
        'cameraFovHalfDeg', get_param_value(params, 'visualization_camera_fov_half_deg', 8), ...
        'cameraNote', get_param_value(params, 'visualization_camera_note', ...
            'Illustrative +Z viewer cone; not a modeled payload camera or tracker acceptance angle.'), ...
        'starTrackerBoresightBody', row(get_param_value(params, 'st_boresight_body', [0; 0; 1])), ...
        'starTrackerExclusionSunDeg', get_param_value(params, 'st_exclusion_sun_deg', NaN), ...
        'starTrackerExclusionMoonDeg', get_param_value(params, 'st_exclusion_moon_deg', NaN), ...
        'starTrackerExclusionEarthDeg', get_param_value(params, 'st_exclusion_earth_deg', NaN));
    replay.metrics = enrich_metrics(results);
    replay.events = struct('modeChanges', mode_change_events(results));
    replay.samples = samples;
end


function replay = build_timeseries_replay(results, max_samples)
    replay = struct();
    replay.metadata = struct( ...
        'schema', 'adcs-simulink-replay-v1', ...
        'generated_utc', char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z''')), ...
        'model', 'adcs_sim', ...
        'duration_s', results.metrics.duration_s, ...
        'sample_policy', sprintf('uniform stride to at most %d samples per signal', max_samples));
    replay.parameters = results.parameters;
    replay.metrics = enrich_metrics(results);
    replay.timeseries = struct();

    replay.timeseries.q = make_series(results, 'q_time_s', 'q', '-', 'quaternion_scalar_first', ...
        'Truth attitude quaternion, scalar-first, passive ECI-to-body.', max_samples);
    replay.timeseries.q_est = make_series(results, 'q_est_time_s', 'q_est', '-', 'quaternion_scalar_first', ...
        'Estimated attitude quaternion used by control.', max_samples);
    replay.timeseries.q_des = make_series(results, 'q_des_time_s', 'q_des', '-', 'quaternion_scalar_first', ...
        'Desired LVLH/nadir attitude quaternion.', max_samples);
    replay.timeseries.omega_rad_s = make_series(results, 'omega_time_s', 'omega_rad_s', 'rad/s', 'body', ...
        'Truth body angular velocity.', max_samples);
    replay.timeseries.pos_eci_m = make_series(results, 'pos_eci_time_s', 'pos_eci_m', 'm', 'ECI', ...
        'Propagated spacecraft position.', max_samples);
    replay.timeseries.vel_eci_mps = make_series(results, 'vel_eci_time_s', 'vel_eci_mps', 'm/s', 'ECI', ...
        'Propagated spacecraft velocity.', max_samples);
    replay.timeseries.sun_eci = make_series(results, 'sun_eci_time_s', 'sun_eci', '-', 'ECI', ...
        'Sun unit vector.', max_samples);
    replay.timeseries.moon_eci = make_series(results, 'moon_eci_time_s', 'moon_eci', '-', 'ECI', ...
        'Moon unit vector.', max_samples);
    replay.timeseries.B_eci_T = make_series(results, 'B_eci_time_s', 'B_eci_T', 'T', 'ECI', ...
        'Magnetic field vector.', max_samples);
    replay.timeseries.sun_meas_body = make_series(results, 'sun_meas_time_s', 'sun_meas_body', '-', 'body', ...
        'Sampled sun sensor body vector.', max_samples);
    replay.timeseries.B_body_meas_T = make_series(results, 'B_body_meas_time_s', 'B_body_meas_T', 'T', 'body', ...
        'Sampled magnetometer body vector.', max_samples);
    replay.timeseries.rw_cmd_nm = make_series(results, 'rw_cmd_time_s', 'rw_cmd_nm', 'N*m', 'body', ...
        'Reaction-wheel body torque command.', max_samples);
    replay.timeseries.mtq_cmd_am2 = make_series(results, 'mtq_cmd_time_s', 'mtq_cmd_am2', 'A*m^2', 'body', ...
        'Magnetorquer dipole command.', max_samples);
    replay.timeseries.ext_torque_nm = make_series(results, 'ext_torque_time_s', 'ext_torque_nm', 'N*m', 'body', ...
        'External commanded torque entering dynamics.', max_samples);
    replay.timeseries.rw_torque_body_nm = make_series(results, 'rw_torque_body_time_s', 'rw_torque_body_nm', 'N*m', 'body', ...
        'Spacecraft torque from reaction wheels.', max_samples);
    replay.timeseries.rw_tau_wheels_nm = make_series(results, 'rw_tau_wheels_time_s', 'rw_tau_wheels_nm', 'N*m', 'wheel', ...
        'Per-wheel momentum derivative.', max_samples);
    replay.timeseries.h_w_nms = make_series(results, 'h_w_time_s', 'h_w_nms', 'N*m*s', 'wheel', ...
        'Per-wheel stored momentum.', max_samples);
    replay.timeseries.mode_id = make_series(results, 'mode_id_time_s', 'mode_id', '-', 'mode', ...
        'Control mode id: 1 detumble, 2 coarse, 3 fine.', max_samples);
    replay.timeseries.att_valid = make_series(results, 'att_valid_time_s', 'att_valid', '-', 'flag', ...
        'Attitude estimate validity flag.', max_samples);
    if isfield(results.signals, 'tracker_valid')
        replay.timeseries.tracker_valid = make_series(results, 'tracker_valid_time_s', 'tracker_valid', '-', 'flag', ...
            'Star tracker validity flag.', max_samples);
    end
    replay.timeseries.eclipse_flag = make_series(results, 'eclipse_time_s', 'eclipse_flag', '-', 'flag', ...
        'Cylindrical eclipse flag.', max_samples);

    desired_error = desired_attitude_error_series(results);
    replay.timeseries.desired_attitude_error_deg = downsample_series(desired_error.time_s, ...
        desired_error.values, 'deg', 'scalar', 'Estimated-to-desired attitude separation.', max_samples);
end


function t = viewer_time_grid(results, max_samples)
    if isfield(results.signals, 'pos_eci_time_s')
        base_t = results.signals.pos_eci_time_s(:);
    else
        base_t = results.signals.q_time_s(:);
    end
    stride = max(1, ceil(numel(base_t) / max_samples));
    idx = 1:stride:numel(base_t);
    if idx(end) ~= numel(base_t)
        idx = [idx numel(base_t)];
    end
    t = unique(base_t(idx), 'stable');
end


function values = interp_optional(results, time_name, value_name, t_query, width, fill_value)
    if isfield(results.signals, time_name) && isfield(results.signals, value_name)
        values = interp_signal(results.signals.(time_name), results.signals.(value_name), t_query);
        return;
    end
    if isscalar(fill_value)
        fill_row = repmat(fill_value, 1, width);
    else
        fill_row = reshape(fill_value, 1, []);
    end
    values = repmat(fill_row, numel(t_query), 1);
end


function q = normalize_rows(q)
    for idx = 1:size(q, 1)
        n = norm(q(idx, :));
        if n < 1e-12
            q(idx, :) = [1 0 0 0];
        else
            q(idx, :) = q(idx, :) / n;
            if q(idx, 1) < 0
                q(idx, :) = -q(idx, :);
            end
        end
    end
end


function y = get_param_value(params, name, default_value)
    if isfield(params, name)
        y = params.(name);
    else
        y = default_value;
    end
end


function y = row(x)
    y = reshape(x, 1, []);
end


function events = mode_change_events(results)
    empty_event = struct('tSec', {}, 'modeId', {});
    events = empty_event;
    if ~isfield(results.signals, 'mode_id_time_s') || ~isfield(results.signals, 'mode_id')
        return;
    end
    t = results.signals.mode_id_time_s(:);
    mode = round(results.signals.mode_id(:));
    if isempty(t) || isempty(mode)
        return;
    end
    change_idx = [1; find(diff(mode) ~= 0) + 1];
    events = repmat(struct('tSec', 0, 'modeId', 0), numel(change_idx), 1);
    for idx = 1:numel(change_idx)
        events(idx).tSec = t(change_idx(idx));
        events(idx).modeId = mode(change_idx(idx));
    end
end


function metrics = enrich_metrics(results)
    metrics = results.metrics;
    if isfield(results.signals, 'q_des') && isfield(results.signals, 'q_est')
        desired_error = desired_attitude_error_series(results);
        metrics.initial_desired_attitude_error_deg = desired_error.values(1);
        metrics.final_desired_attitude_error_deg = desired_error.values(end);
        metrics.min_desired_attitude_error_deg = min(desired_error.values);
    end
    if isfield(results.signals, 'tracker_valid')
        metrics.tracker_valid_fraction = mean(results.signals.tracker_valid(:) > 0.5);
    end
end


function series = make_series(results, time_name, value_name, units, frame, description, max_samples)
    if ~isfield(results.signals, time_name) || ~isfield(results.signals, value_name)
        series = struct('time_s', [], 'values', [], 'units', units, 'frame', frame, 'description', description);
        return;
    end
    series = downsample_series(results.signals.(time_name), results.signals.(value_name), units, frame, description, max_samples);
end


function series = downsample_series(time_s, values, units, frame, description, max_samples)
    time_s = time_s(:);
    stride = max(1, ceil(numel(time_s) / max_samples));
    idx = 1:stride:numel(time_s);
    if idx(end) ~= numel(time_s)
        idx = [idx numel(time_s)];
    end
    series = struct( ...
        'time_s', time_s(idx), ...
        'values', values(idx, :), ...
        'units', units, ...
        'frame', frame, ...
        'description', description);
end


function figure_paths = write_review_figures(results, fig_dir, base_name)
    figure_paths = strings(0, 1);

    desired_error = desired_attitude_error_series(results);
    figure_paths(end + 1) = plot_panel(fig_dir, [base_name '_attitude.png'], ...
        @(tl) plot_attitude_panel(tl, results, desired_error));
    figure_paths(end + 1) = plot_panel(fig_dir, [base_name '_rates.png'], ...
        @(tl) plot_rates_panel(tl, results));
    figure_paths(end + 1) = plot_panel(fig_dir, [base_name '_wheels_control.png'], ...
        @(tl) plot_wheels_control_panel(tl, results));
    figure_paths(end + 1) = plot_panel(fig_dir, [base_name '_environment_orbit.png'], ...
        @(tl) plot_environment_panel(tl, results));
    figure_paths(end + 1) = plot_panel(fig_dir, [base_name '_sensors_modes.png'], ...
        @(tl) plot_sensors_modes_panel(tl, results));
end


function path_out = plot_panel(fig_dir, file_name, plot_fn)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 800], ...
        'DefaultAxesColor', 'w', 'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', ...
        'DefaultAxesZColor', 'k', 'DefaultTextColor', 'k', ...
        'DefaultLegendTextColor', 'k', 'DefaultLegendColor', 'w', ...
        'DefaultAxesColorOrder', [0 .35 .65; .8 .25 .05; .6 .42 0; .45 .15 .55; .2 .5 .15; .1 .5 .6]);
    cleanup = onCleanup(@() close(fig));
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    plot_fn(tl);
    path_out = fullfile(fig_dir, file_name);
    exportgraphics(fig, path_out, 'Resolution', 150);
    clear cleanup
end


function plot_attitude_panel(tl, results, desired_error)
    nexttile(tl);
    plot(results.signals.q_time_s, results.signals.q, 'LineWidth', 1.0);
    title('Truth quaternion'); xlabel('Time [s]'); ylabel('q'); grid on;
    legend('q0','q1','q2','q3', 'Location', 'best');

    nexttile(tl);
    plot(desired_error.time_s, desired_error.values, 'LineWidth', 1.2);
    title('Estimated-to-desired attitude separation'); xlabel('Time [s]'); ylabel('deg'); grid on;

    nexttile(tl);
    plot(results.signals.q_est_time_s, results.signals.q_est, 'LineWidth', 1.0);
    title('Estimated quaternion'); xlabel('Time [s]'); ylabel('q est'); grid on;

    nexttile(tl);
    plot(results.signals.q_des_time_s, results.signals.q_des, 'LineWidth', 1.0);
    title('Desired quaternion'); xlabel('Time [s]'); ylabel('q des'); grid on;
end


function plot_rates_panel(tl, results)
    nexttile(tl);
    plot(results.signals.omega_time_s, results.signals.omega_rad_s * 180/pi, 'LineWidth', 1.0);
    title('Body rates'); xlabel('Time [s]'); ylabel('deg/s'); grid on;
    legend('\omega_x','\omega_y','\omega_z', 'Location', 'best');

    nexttile(tl);
    plot(results.signals.omega_time_s, results.signals.omega_norm_deg_s, 'LineWidth', 1.2);
    title('Rate norm'); xlabel('Time [s]'); ylabel('deg/s'); grid on;

    nexttile(tl);
    plot(results.signals.omega_time_s, results.signals.kinetic_energy_j, 'LineWidth', 1.2);
    title('Rotational kinetic energy'); xlabel('Time [s]'); ylabel('J'); grid on;

    nexttile(tl);
    plot(results.signals.omega_time_s, results.signals.total_ang_momentum_norm_nms, 'LineWidth', 1.2);
    title('Total angular momentum norm'); xlabel('Time [s]'); ylabel('N*m*s'); grid on;
end


function plot_wheels_control_panel(tl, results)
    nexttile(tl);
    plot(results.signals.h_w_time_s, results.signals.h_w_nms, 'LineWidth', 1.0);
    title('Wheel momentum'); xlabel('Time [s]'); ylabel('N*m*s'); grid on;

    nexttile(tl);
    plot(results.signals.rw_cmd_time_s, results.signals.rw_cmd_nm, 'LineWidth', 1.0);
    title('RW body torque command'); xlabel('Time [s]'); ylabel('N*m'); grid on;

    nexttile(tl);
    plot(results.signals.mtq_cmd_time_s, results.signals.mtq_cmd_am2, 'LineWidth', 1.0);
    title('MTQ dipole command'); xlabel('Time [s]'); ylabel('A*m^2'); grid on;

    nexttile(tl);
    plot(results.signals.rw_tau_wheels_time_s, results.signals.rw_tau_wheels_nm, 'LineWidth', 1.0);
    title('Per-wheel torque'); xlabel('Time [s]'); ylabel('N*m'); grid on;
end


function plot_environment_panel(tl, results)
    nexttile(tl);
    plot3(results.signals.pos_eci_m(:,1)/1e3, results.signals.pos_eci_m(:,2)/1e3, ...
        results.signals.pos_eci_m(:,3)/1e3, 'LineWidth', 1.0);
    axis equal; grid on; title('ECI orbit arc'); xlabel('x [km]'); ylabel('y [km]'); zlabel('z [km]');

    nexttile(tl);
    plot(results.signals.pos_eci_time_s, vecnorm(results.signals.pos_eci_m, 2, 2)/1e3, 'LineWidth', 1.2);
    title('Orbital radius'); xlabel('Time [s]'); ylabel('km'); grid on;

    nexttile(tl);
    plot(results.signals.B_eci_time_s, vecnorm(results.signals.B_eci_T, 2, 2) * 1e6, 'LineWidth', 1.2);
    title('Magnetic field magnitude'); xlabel('Time [s]'); ylabel('\muT'); grid on;

    nexttile(tl);
    stairs(results.signals.eclipse_time_s, results.signals.eclipse_flag, 'LineWidth', 1.2);
    title('Eclipse flag'); xlabel('Time [s]'); ylabel('flag'); ylim([-0.1 1.1]); grid on;
end


function plot_sensors_modes_panel(tl, results)
    nexttile(tl);
    plot(results.signals.sun_meas_time_s, results.signals.sun_meas_body, 'LineWidth', 1.0);
    title('Sun sensor body vector'); xlabel('Time [s]'); ylabel('unit vector'); grid on;

    nexttile(tl);
    plot(results.signals.B_body_meas_time_s, results.signals.B_body_meas_T * 1e6, 'LineWidth', 1.0);
    title('Magnetometer body vector'); xlabel('Time [s]'); ylabel('\muT'); grid on;

    nexttile(tl);
    stairs(results.signals.mode_id_time_s, results.signals.mode_id, 'LineWidth', 1.2);
    title('Control mode'); xlabel('Time [s]'); ylabel('mode id'); ylim([0.5 3.5]); grid on;

    nexttile(tl);
    hold on;
    stairs(results.signals.att_valid_time_s, results.signals.att_valid, 'LineWidth', 1.2);
    if isfield(results.signals, 'tracker_valid')
        stairs(results.signals.tracker_valid_time_s, results.signals.tracker_valid, 'LineWidth', 1.2);
        legend('att valid', 'ST valid', 'Location', 'best');
    else
        legend('att valid', 'Location', 'best');
    end
    hold off;
    title('Estimator validity'); xlabel('Time [s]'); ylabel('flag'); ylim([-0.1 1.1]); grid on;
end


function desired_error = desired_attitude_error_series(results)
    q_est_at_des = interp_signal(results.signals.q_est_time_s, results.signals.q_est, results.signals.q_des_time_s);
    values = zeros(numel(results.signals.q_des_time_s), 1);
    for idx = 1:numel(values)
        q_est = q_est_at_des(idx, :).';
        q_est = q_est / max(norm(q_est), 1e-12);
        q_des = results.signals.q_des(idx, :).';
        q_des = q_des / max(norm(q_des), 1e-12);
        q_err = attitude_error(q_est, q_des);
        values(idx) = 2 * atan2(norm(q_err(2:4)), max(min(q_err(1), 1), -1)) * 180 / pi;
    end
    desired_error = struct('time_s', results.signals.q_des_time_s(:), 'values', values);
end


function samples_at_query = interp_signal(t_samples, samples, t_query)
    samples_at_query = zeros(numel(t_query), size(samples, 2));
    for col = 1:size(samples, 2)
        samples_at_query(:, col) = interp1(t_samples(:), samples(:, col), t_query(:), 'linear', 'extrap');
    end
end


function q_err = attitude_error(q_est, q_des)
    qd = [q_des(1); -q_des(2:4)];
    q_err = quat_multiply(qd, q_est);
    q_err = q_err / max(norm(q_err), 1e-12);
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


function write_text(path, text)
    fid = fopen(path, 'w');
    assert(fid > 0, 'Could not open %s for writing.', path);
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, text, 'char');
    clear cleaner
end
