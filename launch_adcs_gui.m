function fig = launch_adcs_gui()
%LAUNCH_ADCS_GUI Launch an interactive ADCS simulation dashboard.

    defaults = load_default_inputs();

    fig = uifigure('Name', 'ADCS Simulator Dashboard', 'Position', [100 100 1500 900]);
    main_grid = uigridlayout(fig, [1 2]);
    main_grid.ColumnWidth = {340, '1x'};
    main_grid.RowHeight = {'1x'};
    main_grid.Padding = [12 12 12 12];

    input_panel = uipanel(main_grid, 'Title', 'Simulation inputs');
    input_panel.Layout.Row = 1;
    input_panel.Layout.Column = 1;

    input_grid = uigridlayout(input_panel, [15 2]);
    input_grid.RowHeight = repmat({'fit'}, 1, 15);
    input_grid.ColumnWidth = {140, '1x'};
    input_grid.Padding = [10 10 10 10];

    row = 1;
    [dt_field, row] = add_numeric_field(input_grid, row, 'dt [s]', defaults.dt);
    [t_end_field, row] = add_numeric_field(input_grid, row, 't_end [s]', defaults.t_end);
    [q0_fields, row] = add_vector_field(input_grid, row, 'q0 [q0 q1 q2 q3]', defaults.q0);
    [omega_fields, row] = add_vector_field(input_grid, row, 'omega0 [rad/s]', defaults.omega0);
    [h_w_fields, row] = add_vector_field(input_grid, row, 'h_W0 [N*m*s]', defaults.h_W0);
    [j_diag_fields, row] = add_vector_field(input_grid, row, 'J diag [kg*m^2]', defaults.j_diag);

    run_button = uibutton(input_grid, 'push', ...
        'Text', 'Run simulation', ...
        'ButtonPushedFcn', @on_run_button);
    run_button.Layout.Row = row;
    run_button.Layout.Column = [1 2];
    row = row + 1;

    reset_button = uibutton(input_grid, 'push', ...
        'Text', 'Reset defaults', ...
        'ButtonPushedFcn', @on_reset_button);
    reset_button.Layout.Row = row;
    reset_button.Layout.Column = [1 2];
    row = row + 1;

    status_label = uilabel(input_grid, ...
        'Text', 'Ready. Set parameters and run the simulator.', ...
        'WordWrap', 'on');
    status_label.Layout.Row = row;
    status_label.Layout.Column = [1 2];
    row = row + 1;

    note_label = uilabel(input_grid, ...
        'Text', ['The GUI uses init_adcs_params.m as the source of truth, ' ...
                 'applies the selected overrides, rebuilds adcs_sim, then runs sim().'], ...
        'WordWrap', 'on');
    note_label.Layout.Row = row;
    note_label.Layout.Column = [1 2];
    output_tabs = uitabgroup(main_grid);
    output_tabs.Layout.Row = 1;
    output_tabs.Layout.Column = 2;

    plots_tab = uitab(output_tabs, 'Title', 'Plots');
    plots_grid = uigridlayout(plots_tab, [2 2]);
    plots_grid.RowHeight = {'1x', '1x'};
    plots_grid.ColumnWidth = {'1x', '1x'};
    plots_grid.Padding = [10 10 10 10];

    q_axes = uiaxes(plots_grid);
    q_axes.Layout.Row = 1;
    q_axes.Layout.Column = 1;
    title(q_axes, 'Quaternion history');
    xlabel(q_axes, 'Time [s]');
    ylabel(q_axes, 'Quaternion');

    omega_axes = uiaxes(plots_grid);
    omega_axes.Layout.Row = 1;
    omega_axes.Layout.Column = 2;
    title(omega_axes, 'Body rates');
    xlabel(omega_axes, 'Time [s]');
    ylabel(omega_axes, 'Rate [deg/s]');

    h_w_axes = uiaxes(plots_grid);
    h_w_axes.Layout.Row = 2;
    h_w_axes.Layout.Column = 1;
    title(h_w_axes, 'Wheel momentum');
    xlabel(h_w_axes, 'Time [s]');
    ylabel(h_w_axes, 'Momentum [N*m*s]');

    derived_axes = uiaxes(plots_grid);
    derived_axes.Layout.Row = 2;
    derived_axes.Layout.Column = 2;
    title(derived_axes, 'Derived behavior metrics');
    xlabel(derived_axes, 'Time [s]');
    ylabel(derived_axes, 'Value');

    metrics_tab = uitab(output_tabs, 'Title', 'Metrics');
    metrics_grid = uigridlayout(metrics_tab, [1 1]);
    metrics_grid.Padding = [10 10 10 10];
    metrics_table = uitable(metrics_grid, ...
        'Data', cell(0, 3), ...
        'ColumnName', {'Metric', 'Value', 'Unit'}, ...
        'ColumnEditable', [false false false]);

    params_tab = uitab(output_tabs, 'Title', 'Parameters');
    params_grid = uigridlayout(params_tab, [1 1]);
    params_grid.Padding = [10 10 10 10];
    params_table = uitable(params_grid, ...
        'Data', cell(0, 3), ...
        'ColumnName', {'Parameter', 'Value', 'Unit'}, ...
        'ColumnEditable', [false false false]);

    on_reset_button([], []);

    if nargout == 0
        clear fig
    end

    function on_run_button(~, ~)
        run_button.Enable = 'off';
        reset_button.Enable = 'off';
        status_label.Text = 'Running init_adcs_params, rebuilding adcs_sim, and simulating...';
        drawnow;

        try
            [overrides, note_text] = collect_overrides();
            results = simulate_adcs_case(overrides);
            fig.UserData = results;

            update_plots(results);
            metrics_table.Data = results.metric_rows;
            params_table.Data = results.parameter_rows;

            status_label.Text = sprintf(['Simulation complete. Peak rate %.3f deg/s, ' ...
                'peak wheel momentum utilization %.1f%%. %s'], ...
                results.metrics.peak_rate_norm_deg_s, ...
                results.metrics.peak_wheel_momentum_utilization_pct, ...
                note_text);
        catch me
            status_label.Text = 'Simulation failed.';
            uialert(fig, getReport(me, 'extended', 'hyperlinks', 'off'), 'ADCS simulation error');
        end

        run_button.Enable = 'on';
        reset_button.Enable = 'on';
    end

    function on_reset_button(~, ~)
        dt_field.Value = defaults.dt;
        t_end_field.Value = defaults.t_end;
        set_vector_fields(q0_fields, defaults.q0);
        set_vector_fields(omega_fields, defaults.omega0);
        set_vector_fields(h_w_fields, defaults.h_W0);
        set_vector_fields(j_diag_fields, defaults.j_diag);
        status_label.Text = 'Defaults loaded from init_adcs_params.m.';

        metrics_table.Data = cell(0, 3);
        params_table.Data = default_parameter_rows(defaults);
        cla(q_axes);
        cla(omega_axes);
        cla(h_w_axes);
        cla(derived_axes);
    end

    function [overrides, note_text] = collect_overrides()
        q0 = vector_field_values(q0_fields);
        q_norm = norm(q0);
        assert(q_norm > 0, 'q0 must be nonzero.');

        note_parts = {};
        if abs(q_norm - 1) > 1e-9
            q0 = q0 / q_norm;
            note_parts{end + 1} = 'q0 was normalized before the run.';
        end

        j_diag = vector_field_values(j_diag_fields);
        assert(all(j_diag > 0), 'All inertia diagonal terms must be positive.');

        overrides = struct( ...
            'dt', dt_field.Value, ...
            't_end', t_end_field.Value, ...
            'q0', q0(:), ...
            'omega0', vector_field_values(omega_fields).', ...
            'h_W0', vector_field_values(h_w_fields).', ...
            'J', diag(j_diag));

        if isempty(note_parts)
            note_text = 'Plots and summary tables use the latest run.';
        else
            note_text = strjoin(note_parts, ' ');
        end
    end

    function update_plots(results)
        plot(q_axes, results.signals.q_time_s, results.signals.q, 'LineWidth', 1.2);
        legend(q_axes, {'q0', 'q1', 'q2', 'q3'}, 'Location', 'best');
        grid(q_axes, 'on');

        omega_deg_s = results.signals.omega_rad_s * 180 / pi;
        plot(omega_axes, results.signals.omega_time_s, omega_deg_s, 'LineWidth', 1.2);
        hold(omega_axes, 'on');
        plot(omega_axes, results.signals.omega_time_s, results.signals.omega_norm_deg_s, ...
            'k--', 'LineWidth', 1.5);
        hold(omega_axes, 'off');
        legend(omega_axes, {'wx', 'wy', 'wz', '||omega||'}, 'Location', 'best');
        grid(omega_axes, 'on');

        plot(h_w_axes, results.signals.h_w_time_s, results.signals.h_w_nms, 'LineWidth', 1.2);
        hold(h_w_axes, 'on');
        plot(h_w_axes, results.signals.h_w_time_s, results.signals.h_w_norm_nms, ...
            'k--', 'LineWidth', 1.5);
        hold(h_w_axes, 'off');
        legend(h_w_axes, {'h1', 'h2', 'h3', 'h4', '||h_W||'}, 'Location', 'best');
        grid(h_w_axes, 'on');

        plot(derived_axes, results.signals.omega_time_s, results.signals.kinetic_energy_j, ...
            'LineWidth', 1.5);
        hold(derived_axes, 'on');
        plot(derived_axes, results.signals.h_w_time_s, results.signals.h_w_norm_nms, ...
            '--', 'LineWidth', 1.5);
        plot(derived_axes, results.signals.omega_time_s, results.signals.total_ang_momentum_norm_nms, ...
            ':', 'LineWidth', 1.8);
        hold(derived_axes, 'off');
        legend(derived_axes, {'Kinetic energy [J]', 'Wheel momentum norm [N*m*s]', ...
            'Total angular momentum norm [N*m*s]'}, 'Location', 'best');
        grid(derived_axes, 'on');
    end
end


function defaults = load_default_inputs()
    repo_root = fileparts(mfilename('fullpath'));
    prev_dir = pwd;
    cleanup = onCleanup(@() cd(prev_dir));
    cd(repo_root);

    evalin('base', 'init_adcs_params;');
    defaults = struct( ...
        'dt', evalin('base', 'dt'), ...
        't_end', evalin('base', 't_end'), ...
        'q0', evalin('base', 'q0').', ...
        'omega0', evalin('base', 'omega0').', ...
        'h_W0', evalin('base', 'h_W0').', ...
        'j_diag', diag(evalin('base', 'J')).');
end


function rows = default_parameter_rows(defaults)
    rows = {
        'Time step dt', sprintf('%.6g', defaults.dt), 's';
        'Simulation horizon t_end', sprintf('%.6g', defaults.t_end), 's';
        'Initial quaternion q0', format_vector(defaults.q0), '-';
        'Initial body rate omega0', format_vector(defaults.omega0), 'rad/s';
        'Initial wheel momentum h_W0', format_vector(defaults.h_W0), 'N*m*s';
        'Inertia diagonal', format_vector(defaults.j_diag), 'kg*m^2'
        };
end


function [field, next_row] = add_numeric_field(parent, row, label_text, default_value)
    label = uilabel(parent, 'Text', label_text);
    label.Layout.Row = row;
    label.Layout.Column = 1;

    field = uieditfield(parent, 'numeric', 'Value', default_value);
    field.Layout.Row = row;
    field.Layout.Column = 2;
    next_row = row + 1;
end


function [fields, next_row] = add_vector_field(parent, row, label_text, default_values)
    label = uilabel(parent, 'Text', label_text);
    label.Layout.Row = row;
    label.Layout.Column = 1;

    value_grid = uigridlayout(parent, [1 numel(default_values)]);
    value_grid.ColumnWidth = repmat({'1x'}, 1, numel(default_values));
    value_grid.RowHeight = {'fit'};
    value_grid.Padding = [0 0 0 0];
    value_grid.ColumnSpacing = 5;
    value_grid.Layout.Row = row;
    value_grid.Layout.Column = 2;

    fields = gobjects(1, numel(default_values));
    for idx = 1:numel(default_values)
        fields(idx) = uieditfield(value_grid, 'numeric', 'Value', default_values(idx));
        fields(idx).Layout.Row = 1;
        fields(idx).Layout.Column = idx;
    end

    next_row = row + 1;
end


function values = vector_field_values(fields)
    values = arrayfun(@(field) field.Value, fields);
end


function set_vector_fields(fields, values)
    for idx = 1:numel(fields)
        fields(idx).Value = values(idx);
    end
end


function text = format_vector(values)
    text = sprintf('[%s]', strjoin(arrayfun(@(v) sprintf('%.4g', v), values(:).', ...
        'UniformOutput', false), '  '));
end
