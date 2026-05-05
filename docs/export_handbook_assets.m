function export_handbook_assets()
%EXPORT_HANDBOOK_ASSETS Export screenshots and figures for the ADCS handbook.

    docs_dir = fileparts(mfilename('fullpath'));
    repo_root = fileparts(docs_dir);
    assets_dir = fullfile(docs_dir, 'handbook_assets');
    if ~exist(assets_dir, 'dir')
        mkdir(assets_dir);
    end

    prev_dir = pwd;
    cleanup = onCleanup(@() cd(prev_dir)); %#ok<NASGU>
    cd(repo_root);

    init_adcs_params;
    build_adcs_model;
    load_system('adcs_sim');

    export_system_png('adcs_sim', fullfile(assets_dir, 'top_level_model.png'));
    export_system_png('adcs_sim/ENVIRONMENT', fullfile(assets_dir, 'environment_subsystem.png'));
    export_system_png('adcs_sim/SENSORS', fullfile(assets_dir, 'sensors_subsystem.png'));
    export_system_png('adcs_sim/CONTROL', fullfile(assets_dir, 'control_subsystem.png'));
    export_system_png('adcs_sim/ACTUATORS', fullfile(assets_dir, 'actuators_subsystem.png'));
    export_system_png('adcs_sim/DYNAMICS', fullfile(assets_dir, 'dynamics_subsystem.png'));

    export_gui_png(fullfile(assets_dir, 'gui_inputs.png'), fullfile(assets_dir, 'gui_results.png'));
    export_summary_plot(fullfile(assets_dir, 'default_run_summary.png'));

    close_system('adcs_sim', 0);
end


function export_system_png(system_name, out_path)
    open_system(system_name);
    try
        set_param(system_name, 'ZoomFactor', 'FitSystem');
    catch
        % Older or headless sessions sometimes reject FitSystem. Printing still works.
    end
    drawnow;
    pause(0.5);
    print(['-s' system_name], '-dpng', '-r180', out_path);
end


function export_gui_png(inputs_path, results_path)
    fig = launch_adcs_gui();
    drawnow;
    pause(1.0);
    exportapp(fig, inputs_path);

    run_button = findall(fig, 'Type', 'uibutton', 'Text', 'Run simulation');
    if isempty(run_button)
        delete(fig);
        error('Could not locate the Run simulation button in the GUI.');
    end

    callback = run_button.ButtonPushedFcn;
    callback(run_button, []);
    drawnow;
    pause(1.0);
    exportapp(fig, results_path);
    delete(fig);
end


function export_summary_plot(out_path)
    results = simulate_adcs_case();

    fig = figure('Color', 'w', 'Position', [100 100 1300 900], 'Visible', 'off');
    tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(results.signals.q_time_s, results.signals.q, 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Quaternion');
    title('Quaternion history');
    legend({'q0', 'q1', 'q2', 'q3'}, 'Location', 'best');

    nexttile;
    omega_deg_s = results.signals.omega_rad_s * 180 / pi;
    plot(results.signals.omega_time_s, omega_deg_s, 'LineWidth', 1.2);
    hold on;
    plot(results.signals.omega_time_s, results.signals.omega_norm_deg_s, 'k--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Rate [deg/s]');
    title('Body rates');
    legend({'wx', 'wy', 'wz', '||\omega||'}, 'Location', 'best');

    nexttile;
    plot(results.signals.h_w_time_s, results.signals.h_w_nms, 'LineWidth', 1.2);
    hold on;
    plot(results.signals.h_w_time_s, results.signals.h_w_norm_nms, 'k--', 'LineWidth', 1.5);
    hold off;
    grid on;
    xlabel('Time [s]');
    ylabel('Momentum [N*m*s]');
    title('Reaction-wheel momentum');
    legend({'h_1', 'h_2', 'h_3', 'h_4', '||h_W||'}, 'Location', 'best');

    nexttile;
    axis off;
    text(0.0, 1.0, 'Default run snapshot', 'FontWeight', 'bold', 'FontSize', 13, 'VerticalAlignment', 'top');
    text(0.0, 0.86, sprintf('Duration: %.0f s', results.metrics.duration_s), 'FontSize', 11);
    text(0.0, 0.76, sprintf('Peak rate norm: %.3f deg/s', results.metrics.peak_rate_norm_deg_s), 'FontSize', 11);
    text(0.0, 0.66, sprintf('Peak attitude excursion: %.3f deg', results.metrics.peak_attitude_excursion_deg), 'FontSize', 11);
    text(0.0, 0.56, sprintf('Peak wheel momentum utilization: %.1f%%', ...
        results.metrics.peak_wheel_momentum_utilization_pct), 'FontSize', 11);
    text(0.0, 0.46, sprintf('Peak wheel speed utilization: %.1f%%', ...
        results.metrics.peak_wheel_speed_utilization_pct), 'FontSize', 11);
    text(0.0, 0.36, sprintf('Peak total angular momentum norm: %.4f N*m*s', ...
        results.metrics.peak_total_ang_momentum_norm_nms), 'FontSize', 11);
    text(0.0, 0.20, ['Current validated branch note: orbit propagation, magnetics, eclipse, ' ...
        'estimation, control, and actuator torque generation include compile-safe fallback blocks.'], ...
        'FontSize', 10, 'Interpreter', 'none');

    exportgraphics(fig, out_path, 'Resolution', 180);
    close(fig);
end
