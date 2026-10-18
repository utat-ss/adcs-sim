function app = launch_adcs_3d_viewer(results_input, varargin)
%LAUNCH_ADCS_3D_VIEWER Native MATLAB 3-D playback for saved ADCS results.
%   launch_adcs_3d_viewer() opens a file picker or the newest supported
%   validation_logs MAT file. The MAT file must contain a summarized results
%   struct from summarize_adcs_simulation. This viewer does not run Simulink,
%   rebuild the model, or modify saved results.
%
%   launch_adcs_3d_viewer(results) opens an already-loaded results struct.
%   launch_adcs_3d_viewer(matFile) loads a MAT file containing results.
%   launch_adcs_3d_viewer('selftest','Visible','off','ExportPath',png) runs a
%   minimal UI smoke test with an internal fixture; this is only a viewer test.

    if nargin < 1
        results_input = [];
    end

    opts = parse_options(varargin{:});
    if ischar(results_input) || (isstring(results_input) && isscalar(results_input))
        token = char(results_input);
        if strcmpi(token, 'selftest')
            [results, source_label] = selftest_results();
        else
            [results, source_label] = load_results_from_file(token);
        end
    elseif isstruct(results_input) && isfield(results_input, 'signals')
        results = results_input;
        source_label = 'workspace results struct';
    elseif isempty(results_input)
        [results, source_label] = choose_results_file();
    else
        error('launch_adcs_3d_viewer:BadInput', ...
            'Input must be empty, a results struct, a MAT filename, or ''selftest''.');
    end

    data = prepare_viewer_data(results, source_label);
    app = build_viewer(data, opts);

    if ~isempty(opts.ExportPath)
        drawnow;
        exportapp(app.Figure, opts.ExportPath);
    end
end

function opts = parse_options(varargin)
    opts.Visible = 'on';
    opts.ExportPath = '';
    opts.PlayStep = 1;
    k = 1;
    while k <= numel(varargin)
        name = validatestring(varargin{k}, {'Visible','ExportPath','PlayStep'});
        value = varargin{k+1};
        switch name
            case 'Visible'
                opts.Visible = validatestring(value, {'on','off'});
            case 'ExportPath'
                opts.ExportPath = char(value);
            case 'PlayStep'
                opts.PlayStep = max(1, round(value));
        end
        k = k + 2;
    end
end

function [results, label] = choose_results_file()
    repo_dir = fileparts(mfilename('fullpath'));
    validation_dir = fullfile(repo_dir, 'validation_logs');
    candidates = [dir(fullfile(validation_dir, '*behavior*.mat')); dir(fullfile(validation_dir, '*scenario*.mat')); dir(fullfile(validation_dir, '*.mat'))];
    if ~isempty(candidates)
        [~, order] = sort([candidates.datenum], 'descend');
        last_error = [];
        for ii = 1:numel(order)
            file_name = fullfile(candidates(order(ii)).folder, candidates(order(ii)).name);
            try
                [results, label] = load_results_from_file(file_name);
                return;
            catch err
                last_error = err;
            end
        end
        if ~isempty(last_error)
            warning('launch_adcs_3d_viewer:NoAutoResults', ...
                'No validation_logs MAT file contained a supported results struct. Last loader error: %s', last_error.message);
        end
    end

    [file, path] = uigetfile('*.mat', 'Select ADCS summarized results MAT file', validation_dir);
    if isequal(file, 0)
        error('launch_adcs_3d_viewer:NoResultsFile', 'No results MAT file selected.');
    end
    [results, label] = load_results_from_file(fullfile(path, file));
end

function tf = is_supported_results(candidate)
    tf = isstruct(candidate) && isfield(candidate, 'signals') && ...
        isfield(candidate.signals, 'pos_eci_m') && ...
        isfield(candidate.signals, 'vel_eci_mps') && ...
        isfield(candidate.signals, 'q');
end
function [results, label] = load_results_from_file(file_name)
    file_name = char(file_name);
    if ~isfile(file_name)
        error('launch_adcs_3d_viewer:MissingFile', 'Results file not found: %s', file_name);
    end
    raw = load(file_name);
    names = fieldnames(raw);
    if isfield(raw, 'results') && is_supported_results(raw.results)
        results = raw.results;
    else
        supported_names = cell(size(names));
        supported_count = 0;
        for k = 1:numel(names)
            candidate = raw.(names{k});
            if is_supported_results(candidate)
                supported_count = supported_count + 1;
                supported_names{supported_count} = names{k};
            end
        end
        supported_names = supported_names(1:supported_count);
        if isscalar(supported_names)
            results = raw.(supported_names{1});
        elseif supported_count > 1
            error('launch_adcs_3d_viewer:AmbiguousResultsStruct', ...
                'MAT file contains multiple supported results structs: %s. Pass the desired struct directly.', strjoin(supported_names, ', '));
        else
            error('launch_adcs_3d_viewer:NoResultsStruct', ...
                'MAT file must contain a summarized results struct with signals.pos_eci_m, signals.vel_eci_mps, and signals.q.');
        end
    end
    label = file_name;
end

function data = prepare_viewer_data(results, source_label)
    s = results.signals;
    assert(isfield(s, 'pos_eci_m'), 'Results must include signals.pos_eci_m.');
    assert(isfield(s, 'vel_eci_mps'), 'Results must include signals.vel_eci_mps.');
    assert(isfield(s, 'q'), 'Results must include signals.q.');

    t = get_time_vector(s, 'pos_eci_time_s', size(s.pos_eci_m, 1));
    pos = orient_samples(s.pos_eci_m, 3);
    vel = resample_signal(get_time_vector(s, 'vel_eci_time_s', size(s.vel_eci_mps, 1)), orient_samples(s.vel_eci_mps, 3), t);
    q = resample_signal(get_time_vector(s, 'q_time_s', size(s.q, 1)), orient_samples(s.q, 4), t);
    q = normalize_rows(q);

    data = struct();
    data.SourceLabel = source_label;
    data.Results = results;
    data.t = t(:);
    data.pos_m = pos;
    data.vel_mps = vel;
    data.q = q;
    data.sun_eci = optional_resampled(s, 'sun_eci', 'sun_eci_time_s', t, 3);
    data.q_des = optional_resampled(s, 'q_des', 'q_des_time_s', t, 4);
    data.q_est = optional_resampled(s, 'q_est', 'q_est_time_s', t, 4);
    data.tracker_valid = optional_resampled(s, 'tracker_valid', 'tracker_valid_time_s', t, 1);
    data.mode_id = optional_resampled(s, 'mode_id', 'mode_id_time_s', t, 1);
    data.eclipse = optional_resampled(s, 'eclipse_flag', 'eclipse_time_s', t, 1);
    data.omega = optional_resampled(s, 'omega_rad_s', 'omega_time_s', t, 3);

    data.Re_m = 6371000.0;
    data.EarthRadiusLabel = 'fallback spherical Earth radius 6371 km';
    if isfield(results, 'parameters') && isfield(results.parameters, 'earth_radius_m')
        data.Re_m = results.parameters.earth_radius_m;
        data.EarthRadiusLabel = 'results.parameters.earth_radius_m';
    end
    data.epoch_jd = 2461083.5;
    if isfield(results, 'parameters') && isfield(results.parameters, 'epoch_jd')
        data.epoch_jd = results.parameters.epoch_jd;
    end
    data.body_dims_m = [0.10, 0.10, 0.30];
    if isfield(results, 'parameters') && isfield(results.parameters, 'spacecraft_dimensions_m')
        data.body_dims_m = results.parameters.spacecraft_dimensions_m(:).';
    end
    data.st_boresight_body = [0; 0; 1];
    data.st_fov_half_deg = 8.0;
    data.st_fov_label = 'illustrative default +Z camera cone, 8 deg half-angle; not validated tracker metadata';
    if isfield(results, 'parameters') && isfield(results.parameters, 'st_boresight_body') && isfield(results.parameters, 'st_fov_half_deg')
        data.st_boresight_body = results.parameters.st_boresight_body(:);
        data.st_fov_half_deg = results.parameters.st_fov_half_deg;
        data.st_fov_label = 'star tracker FOV from saved results metadata';
    end
    data.st_exclusion_earth_deg = 25;
end

function t = get_time_vector(signals, name, n)
    if isfield(signals, name)
        t = signals.(name)(:);
    else
        t = (0:n-1).';
    end
end

function y = optional_resampled(signals, sample_name, time_name, t, width)
    if ~isfield(signals, sample_name)
        y = [];
        return;
    end
    samples = orient_samples(signals.(sample_name), width);
    sample_t = get_time_vector(signals, time_name, size(samples, 1));
    y = resample_signal(sample_t, samples, t);
    if width == 4
        y = normalize_rows(y);
    end
end

function y = resample_signal(sample_t, samples, t)
    sample_t = sample_t(:);
    if size(samples, 1) == 1
        y = repmat(samples, numel(t), 1);
        return;
    end
    [sample_t, unique_idx] = unique(sample_t, 'stable');
    samples = samples(unique_idx, :);
    y = interp1(sample_t, samples, t(:), 'linear', 'extrap');
end

function samples = orient_samples(samples, width)
    samples = squeeze(samples);
    if isvector(samples)
        samples = samples(:);
    end
    if size(samples, 2) == width
        return;
    elseif size(samples, 1) == width
        samples = samples.';
    else
        error('launch_adcs_3d_viewer:BadSignalWidth', 'Expected signal width %d.', width);
    end
end

function rows = normalize_rows(rows)
    n = sqrt(sum(rows.^2, 2));
    n(n < 1e-12) = 1;
    rows = rows ./ n;
    flip = rows(:,1) < 0;
    rows(flip,:) = -rows(flip,:);
end

function app = build_viewer(data, opts)
    app = struct();
    app.Data = data;
    app.Index = 1;
    app.Playing = false;
    app.Speed = 1;
    app.CameraMode = 'earth';
    app.PlayStartClock = [];
    app.PlayStartTime = [];
    app.Figure = uifigure('Name', 'ADCS 3-D Playback Viewer', 'Visible', opts.Visible, ...
        'Position', [100 80 1320 820], 'Color', [0.07 0.08 0.10]);
    app.Grid = uigridlayout(app.Figure, [2 2]);
    app.Grid.RowHeight = {'1x', 92};
    app.Grid.ColumnWidth = {'1x', 330};

    app.Axes = uiaxes(app.Grid);
    app.Axes.Layout.Row = 1;
    app.Axes.Layout.Column = 1;
    hold(app.Axes, 'on');
    grid(app.Axes, 'on');
    axis(app.Axes, 'equal');
    app.Axes.Color = [0.02 0.025 0.04];
    app.Axes.Projection = 'perspective';
    app.Axes.XColor = [0.85 0.85 0.85];
    app.Axes.YColor = [0.85 0.85 0.85];
    app.Axes.ZColor = [0.85 0.85 0.85];
    xlabel(app.Axes, 'ECI X [km]');
    ylabel(app.Axes, 'ECI Y [km]');
    zlabel(app.Axes, 'ECI Z [km]');
    view(app.Axes, 35, 22);

    app.Side = uipanel(app.Grid, 'Title', 'Playback / Layers / Telemetry', ...
        'ForegroundColor', [0.92 0.92 0.92], 'BackgroundColor', [0.10 0.11 0.13]);
    app.Side.Layout.Row = [1 2];
    app.Side.Layout.Column = 2;
    sideGrid = uigridlayout(app.Side, [20 1]);
    sideGrid.RowHeight = {28, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 26, 30, 30, 30, 30, 54, 54};
    sideGrid.RowSpacing = 6;
    sideGrid.Padding = [10 8 10 8];
    if isprop(sideGrid, 'Scrollable')
        sideGrid.Scrollable = 'on';
    end

    app.TitleLabel = uilabel(sideGrid, 'Text', 'ADCS 3-D playback', 'FontWeight', 'bold', 'FontColor', [1 1 1]);
    app.TitleLabel.Layout.Row = 1;

    app.TimeLabel = uitextarea(sideGrid, 'Editable', 'off', 'FontName', 'Consolas', ...
        'FontSize', 11, 'FontColor', [0.95 0.95 0.95], 'BackgroundColor', [0.06 0.065 0.08]);
    app.TimeLabel.Layout.Row = [2 5];

    app.LayerOrbit = uicheckbox(sideGrid, 'Text', 'Orbit path', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerOrbit.Layout.Row = 6;
    app.LayerEarth = uicheckbox(sideGrid, 'Text', 'Earth sphere/grid', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerEarth.Layout.Row = 7;
    app.LayerBodyAxes = uicheckbox(sideGrid, 'Text', 'Body axes', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerBodyAxes.Layout.Row = 8;
    app.LayerEciAxes = uicheckbox(sideGrid, 'Text', 'ECI axes', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerEciAxes.Layout.Row = 9;
    app.LayerLvlhAxes = uicheckbox(sideGrid, 'Text', 'LVLH axes', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerLvlhAxes.Layout.Row = 10;
    app.LayerVectors = uicheckbox(sideGrid, 'Text', 'Sun/nadir/velocity vectors', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerVectors.Layout.Row = 11;
    app.LayerFov = uicheckbox(sideGrid, 'Text', 'Camera cone + footprint', 'Value', true, 'FontColor', [0.95 0.95 0.95]);
    app.LayerFov.Layout.Row = 12;
    app.LayerDesired = uicheckbox(sideGrid, 'Text', 'Desired attitude axes', 'Value', ~isempty(data.q_des), 'FontColor', [0.95 0.95 0.95]);
    app.LayerDesired.Layout.Row = 13;

    app.CameraLabel = uilabel(sideGrid, 'Text', 'Camera presets', 'FontWeight', 'bold', 'FontColor', [1 1 1]);
    app.CameraLabel.Layout.Row = 14;
    app.CamEarth = uibutton(sideGrid, 'Text', 'Earth view', 'ButtonPushedFcn', @(src,~) camera_earth(ancestor(src, 'figure')));
    app.CamEarth.Layout.Row = 15;
    app.CamSpacecraft = uibutton(sideGrid, 'Text', 'Spacecraft chase', 'ButtonPushedFcn', @(src,~) camera_spacecraft(ancestor(src, 'figure')));
    app.CamSpacecraft.Layout.Row = 16;
    app.CamTop = uibutton(sideGrid, 'Text', 'ECI top view', 'ButtonPushedFcn', @(src,~) camera_top(ancestor(src, 'figure')));
    app.CamTop.Layout.Row = 17;
    app.Export = uibutton(sideGrid, 'Text', 'Export screenshot...', 'ButtonPushedFcn', @(src,~) export_screenshot(ancestor(src, 'figure')));
    app.Export.Layout.Row = 18;

    app.Note = uitextarea(sideGrid, 'Editable', 'off', 'Value', { ...
        'Visualization assumptions:', ...
        ['- Earth sphere: ' data.EarthRadiusLabel], ...
        ['- Tracker cone: ' data.st_fov_label], ...
        '- Footprint is ray/sphere intersection with horizon gaps.', ...
        '- Viewer uses saved results only.'}, ...
        'FontSize', 10, 'FontColor', [0.92 0.92 0.92], 'BackgroundColor', [0.06 0.065 0.08]);
    app.Note.Layout.Row = [19 20];

    controlPanel = uipanel(app.Grid, 'BackgroundColor', [0.10 0.11 0.13]);
    controlPanel.Layout.Row = 2;
    controlPanel.Layout.Column = 1;
    controlGrid = uigridlayout(controlPanel, [2 6]);
    controlGrid.ColumnWidth = {75, '1x', 85, 95, 95, 95};
    controlGrid.RowHeight = {32, 32};
    controlGrid.Padding = [10 8 10 8];

    app.PlayButton = uibutton(controlGrid, 'Text', 'Play', 'ButtonPushedFcn', @(~,~) toggle_play(app.Figure));
    app.Slider = uislider(controlGrid, 'Limits', [1, numel(data.t)], 'Value', 1, ...
        'MajorTicks', [], 'MinorTicks', [], 'ValueChangedFcn', @(~,~) slider_changed(app.Figure), ...
        'ValueChangingFcn', @(~,evt) slider_changing(app.Figure, evt));
    app.Slider.Layout.Column = 2;
    app.SpeedDrop = uidropdown(controlGrid, 'Items', {'0.25x','0.5x','1x','2x','5x','10x'}, ...
        'Value', '1x', 'ValueChangedFcn', @(~,~) speed_changed(app.Figure));
    app.StepBack = uibutton(controlGrid, 'Text', '- step', 'ButtonPushedFcn', @(~,~) step_index(app.Figure, -opts.PlayStep));
    app.StepForward = uibutton(controlGrid, 'Text', '+ step', 'ButtonPushedFcn', @(~,~) step_index(app.Figure, opts.PlayStep));
    app.Reset = uibutton(controlGrid, 'Text', 'Reset view', 'ButtonPushedFcn', @(src,~) reset_camera(ancestor(src, 'figure')));
    app.Source = uilabel(controlGrid, 'Text', sprintf('Source: %s', data.SourceLabel), ...
        'FontColor', [0.88 0.88 0.88], 'Interpreter', 'none');
    app.Source.Layout.Row = 2;
    app.Source.Layout.Column = [1 6];

    app.Timer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, ...
        'TimerFcn', @(~,~) timer_step(app.Figure));
    app.Figure.UserData = app;
    app.Figure.CloseRequestFcn = @(fig,~) close_viewer(fig);

    create_static_artists(app.Figure);
    update_scene(app.Figure, 1);
    reset_camera(app.Figure);
end

function create_static_artists(fig)
    app = getapp(fig);
    ax = app.Axes;
    d = app.Data;
    Re_km = d.Re_m / 1e3;

    [xe, ye, ze] = sphere(96);
    app.Earth = surf(ax, Re_km*xe, Re_km*ye, Re_km*ze, ...
        'FaceColor', [0.08 0.23 0.54], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.88, 'DisplayName', 'Earth');
    app.EarthGrid = create_earth_grid(ax, Re_km);
    app.Orbit = plot3(ax, d.pos_m(:,1)/1e3, d.pos_m(:,2)/1e3, d.pos_m(:,3)/1e3, ...
        'Color', [0.9 0.9 0.9], 'LineWidth', 1.1, 'DisplayName', 'Orbit path');
    lim = max(vecnorm(d.pos_m, 2, 2))/1e3 * 1.15;
    app.EciX = plot3(ax, [0 lim], [0 0], [0 0], 'r-', 'LineWidth', 1.2);
    app.EciY = plot3(ax, [0 0], [0 lim], [0 0], 'g-', 'LineWidth', 1.2);
    app.EciZ = plot3(ax, [0 0], [0 0], [0 lim], 'b-', 'LineWidth', 1.2);
    xlim(ax, [-lim lim]); ylim(ax, [-lim lim]); zlim(ax, [-lim lim]);

    [verts, faces] = body_mesh(d.body_dims_m * 35000 / 1e3);
    app.BodyPatch = patch(ax, 'Vertices', verts, 'Faces', faces, 'FaceColor', [0.75 0.78 0.82], ...
        'EdgeColor', [0.10 0.10 0.12], 'LineWidth', 0.8, 'FaceAlpha', 0.96);
    app.BodyAxes = gobjects(3,1);
    app.LvlhAxes = gobjects(3,1);
    app.DesiredAxes = gobjects(3,1);
    app.Vectors = gobjects(3,1);
    colors = eye(3);
    for k = 1:3
        app.BodyAxes(k) = plot3(ax, nan, nan, nan, '-', 'Color', colors(k,:), 'LineWidth', 2.5);
        app.LvlhAxes(k) = plot3(ax, nan, nan, nan, '--', 'Color', 0.65*colors(k,:) + 0.25, 'LineWidth', 1.8);
        app.DesiredAxes(k) = plot3(ax, nan, nan, nan, ':', 'Color', colors(k,:), 'LineWidth', 2.0);
        app.Vectors(k) = plot3(ax, nan, nan, nan, '-', 'LineWidth', 1.8);
    end
    app.Vectors(1).Color = [1.0 0.85 0.10];
    app.Vectors(2).Color = [0.60 0.80 1.0];
    app.Vectors(3).Color = [1.0 0.35 0.35];
    app.FovCone = surf(ax, nan(2), nan(2), nan(2), 'FaceColor', [0.1 0.8 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.16);
    app.FovFootprint = plot3(ax, nan, nan, nan, '-', 'Color', [0.1 0.9 1.0], 'LineWidth', 1.5);
    fig.UserData = app;
end

function h = create_earth_grid(ax, radius_km)
    latitudes = -60:30:60;
    longitudes = 0:30:330;
    h = gobjects(numel(latitudes) + numel(longitudes), 1);
    theta = linspace(0, 2*pi, 181);
    idx = 1;
    for lat = latitudes
        z = radius_km * sind(lat) * ones(size(theta));
        rho = radius_km * cosd(lat);
        h(idx) = plot3(ax, rho*cos(theta), rho*sin(theta), z, '-', ...
            'Color', [0.34 0.48 0.70], 'LineWidth', 0.45);
        idx = idx + 1;
    end
    phi = linspace(-pi/2, pi/2, 121);
    for lon = longitudes
        x = radius_km * cos(phi) * cosd(lon);
        y = radius_km * cos(phi) * sind(lon);
        z = radius_km * sin(phi);
        h(idx) = plot3(ax, x, y, z, '-', 'Color', [0.34 0.48 0.70], 'LineWidth', 0.45);
        idx = idx + 1;
    end
end
function [verts, faces] = body_mesh(dims_km)
    x = dims_km(1)/2; y = dims_km(2)/2; z = dims_km(3)/2;
    verts = [-x -y -z; x -y -z; x y -z; -x y -z; -x -y z; x -y z; x y z; -x y z];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function update_scene(fig, idx)
    app = getapp(fig);
    d = app.Data;
    idx = min(max(round(idx), 1), numel(d.t));
    app.Index = idx;
    app.Slider.Value = idx;
    r_km = d.pos_m(idx,:).' / 1e3;
    v = d.vel_mps(idx,:).';
    q = d.q(idx,:).';
    C_bi = q_to_dcm_bi(q);
    C_ib = C_bi.';

    [verts0, ~] = body_mesh(d.body_dims_m * 35000 / 1e3);
    verts = (C_ib * verts0.').' + r_km.';
    app.BodyPatch.Vertices = verts;

    axis_len = max(150, 0.035 * norm(r_km));
    for k = 1:3
        endpoint = r_km + C_ib(:,k) * axis_len;
        set(app.BodyAxes(k), 'XData', [r_km(1), endpoint(1)], 'YData', [r_km(2), endpoint(2)], 'ZData', [r_km(3), endpoint(3)]);
    end

    C_li = lvlh_dcm(d.pos_m(idx,:).', v);
    C_il = C_li.';
    for k = 1:3
        endpoint = r_km + C_il(:,k) * axis_len * 0.85;
        set(app.LvlhAxes(k), 'XData', [r_km(1), endpoint(1)], 'YData', [r_km(2), endpoint(2)], 'ZData', [r_km(3), endpoint(3)]);
    end

    if ~isempty(d.q_des)
        Cd_bi = q_to_dcm_bi(d.q_des(idx,:).');
        Cd_ib = Cd_bi.';
        for k = 1:3
            endpoint = r_km + Cd_ib(:,k) * axis_len * 1.15;
            set(app.DesiredAxes(k), 'XData', [r_km(1), endpoint(1)], 'YData', [r_km(2), endpoint(2)], 'ZData', [r_km(3), endpoint(3)]);
        end
    end

    sun_dir = [nan; nan; nan];
    if ~isempty(d.sun_eci)
        sun_dir = unit(d.sun_eci(idx,:).');
        set_vector(app.Vectors(1), r_km, r_km + sun_dir * axis_len * 1.5);
    end
    set_vector(app.Vectors(2), r_km, r_km - unit(r_km) * axis_len * 1.25);
    if norm(v) > 0
        set_vector(app.Vectors(3), r_km, r_km + unit(v) * axis_len * 1.25);
    end

    bore_eci = C_ib * unit(d.st_boresight_body);
    update_fov(app, r_km, bore_eci, axis_len);
    apply_layers(app);
    update_telemetry(app, idx, sun_dir);
    apply_camera_mode(app);
    fig.UserData = app;
end

function set_vector(h, a, b)
    set(h, 'XData', [a(1), b(1)], 'YData', [a(2), b(2)], 'ZData', [a(3), b(3)]);
end

function update_fov(app, r_km, bore_eci, axis_len)
    half = app.Data.st_fov_half_deg * pi/180;
    [u, v] = perpendicular_basis(bore_eci);
    theta = linspace(0, 2*pi, 64);
    ray_dirs = cos(half) * bore_eci + sin(half) * (u*cos(theta) + v*sin(theta));
    near = r_km;
    far = r_km + axis_len * 2.0 * ray_dirs;
    X = [repmat(near(1), 1, numel(theta)); far(1,:)];
    Y = [repmat(near(2), 1, numel(theta)); far(2,:)];
    Z = [repmat(near(3), 1, numel(theta)); far(3,:)];
    set(app.FovCone, 'XData', X, 'YData', Y, 'ZData', Z);

    Re_km = app.Data.Re_m / 1e3;
    fp = nan(3, numel(theta));
    for k = 1:numel(theta)
        fp(:,k) = ray_sphere_intersection(r_km, unit(ray_dirs(:,k)), Re_km);
    end
    good = all(isfinite(fp), 1);
    if any(good)
        fp(:, ~good) = nan;
        set(app.FovFootprint, 'XData', fp(1,:), 'YData', fp(2,:), 'ZData', fp(3,:));
    else
        set(app.FovFootprint, 'XData', nan, 'YData', nan, 'ZData', nan);
    end
end

function p = ray_sphere_intersection(r, d, radius)
    b = 2 * dot(r, d);
    c = dot(r, r) - radius^2;
    disc = b^2 - 4*c;
    if disc < 0
        p = [nan; nan; nan];
        return;
    end
    roots_lambda = sort([(-b - sqrt(disc))/2, (-b + sqrt(disc))/2]);
    lambda = roots_lambda(find(roots_lambda > 0, 1, 'first'));
    if isempty(lambda)
        p = [nan; nan; nan];
    else
        p = r + lambda*d;
    end
end

function apply_layers(app)
    set_visible(app.Orbit, app.LayerOrbit.Value);
    set_visible([app.Earth; app.EarthGrid(:)], app.LayerEarth.Value);
    set_visible([app.EciX app.EciY app.EciZ], app.LayerEciAxes.Value);
    set_visible(app.BodyAxes, app.LayerBodyAxes.Value);
    set_visible(app.LvlhAxes, app.LayerLvlhAxes.Value);
    set_visible(app.Vectors, app.LayerVectors.Value);
    set_visible([app.FovCone app.FovFootprint], app.LayerFov.Value);
    set_visible(app.DesiredAxes, app.LayerDesired.Value && ~isempty(app.Data.q_des));
end

function set_visible(h, tf)
    if tf
        set(h, 'Visible', 'on');
    else
        set(h, 'Visible', 'off');
    end
end

function update_telemetry(app, idx, sun_dir)
    d = app.Data;
    utc = jd_to_utc(d.epoch_jd + d.t(idx)/86400);
    rate = nan;
    if ~isempty(d.omega)
        rate = norm(d.omega(idx,:)) * 180/pi;
    end
    tracker = 'n/a';
    if ~isempty(d.tracker_valid)
        tracker = sprintf('%.0f', d.tracker_valid(idx));
    end
    mode = 'n/a';
    if ~isempty(d.mode_id)
        mode = sprintf('%.0f', d.mode_id(idx));
    end
    eclipse = 'n/a';
    if ~isempty(d.eclipse)
        eclipse = sprintf('%.0f', d.eclipse(idx));
    end
    if all(isfinite(sun_dir))
        sun_text = sprintf('[% .3f % .3f % .3f]', sun_dir);
    else
        sun_text = 'not logged';
    end
    app.TimeLabel.Value = { ...
        sprintf('t = %.3f s   sample %d / %d', d.t(idx), idx, numel(d.t)), ...
        sprintf('UTC ≈ %s', char(utc, 'yyyy-MM-dd HH:mm:ss z')), ...
        sprintf('|r| = %.1f km   |v| = %.3f km/s', norm(d.pos_m(idx,:))/1e3, norm(d.vel_mps(idx,:))/1e3), ...
        sprintf('|omega| = %.4f deg/s', rate), ...
        sprintf('tracker_valid=%s   mode=%s   eclipse=%s', tracker, mode, eclipse), ...
        sprintf('Sun ECI = %s', sun_text)};
end

function C = q_to_dcm_bi(q)
    q = q(:) / max(norm(q), 1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    C = [1-2*(q2*q2+q3*q3), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2); ...
         2*(q1*q2-q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3+q0*q1); ...
         2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), 1-2*(q1*q1+q2*q2)];
end

function C_li = lvlh_dcm(r, v)
    z = -unit(r);
    vp = v - dot(v, z)*z;
    if norm(vp) < 1e-12
        x = [1;0;0];
    else
        x = unit(vp);
    end
    y = unit(cross(z, x));
    x = unit(cross(y, z));
    C_li = [x.'; y.'; z.'];
end

function u = unit(v)
    v = v(:);
    n = norm(v);
    if n < 1e-12
        u = zeros(size(v));
    else
        u = v / n;
    end
end

function [u, v] = perpendicular_basis(w)
    w = unit(w);
    if abs(w(3)) < 0.8
        a = [0;0;1];
    else
        a = [0;1;0];
    end
    u = unit(cross(a, w));
    v = unit(cross(w, u));
end

function utc = jd_to_utc(jd)
    utc = datetime(2000, 1, 1, 12, 0, 0, 'TimeZone', 'UTC') + days(jd - 2451545.0);
end

function toggle_play(fig)
    app = getapp(fig);
    if strcmp(app.Timer.Running, 'on')
        stop(app.Timer);
        app.PlayButton.Text = 'Play';
    else
        app.PlayStartClock = tic;
        app.PlayStartTime = app.Data.t(app.Index);
        app.PlayButton.Text = 'Pause';
        fig.UserData = app;
        start(app.Timer);
        return;
    end
    fig.UserData = app;
end

function timer_step(fig)
    if ~isvalid(fig)
        return;
    end
    app = getapp(fig);
    if isempty(app.PlayStartClock) || isempty(app.PlayStartTime)
        app.PlayStartClock = tic;
        app.PlayStartTime = app.Data.t(app.Index);
        fig.UserData = app;
    end
    elapsed = toc(app.PlayStartClock) * app.Speed;
    target_t = app.PlayStartTime + elapsed;
    t0 = app.Data.t(1);
    t1 = app.Data.t(end);
    if target_t > t1
        target_t = t0 + mod(target_t - t0, max(t1 - t0, eps));
        app.PlayStartClock = tic;
        app.PlayStartTime = target_t;
        fig.UserData = app;
    end
    [~, new_idx] = min(abs(app.Data.t - target_t));
    update_scene(fig, new_idx);
end

function slider_changed(fig)
    app = getapp(fig);
    update_scene(fig, app.Slider.Value);
end

function slider_changing(fig, evt)
    update_scene(fig, evt.Value);
end

function speed_changed(fig)
    app = getapp(fig);
    app.Speed = sscanf(app.SpeedDrop.Value, '%fx');
    if strcmp(app.Timer.Running, 'on')
        app.PlayStartClock = tic;
        app.PlayStartTime = app.Data.t(app.Index);
    end
    fig.UserData = app;
end

function step_index(fig, delta)
    app = getapp(fig);
    update_scene(fig, app.Index + delta);
end

function reset_camera(fig)
    app = getapp(fig);
    app.CameraMode = 'free';
    restore_global_limits(app);
    ax = app.Axes;
    campos(ax, [1 -1 0.65] * max(abs(xlim(ax))) * 1.7);
    camtarget(ax, [0 0 0]);
    camup(ax, [0 0 1]);
    set(ax, 'CameraViewAngleMode', 'auto');
    fig.UserData = app;
end

function camera_earth(fig)
    app = getapp(fig);
    app.CameraMode = 'earth';
    apply_camera_mode(app);
    fig.UserData = app;
end

function camera_spacecraft(fig)
    app = getapp(fig);
    app.CameraMode = 'spacecraft';
    apply_camera_mode(app);
    fig.UserData = app;
end

function camera_top(fig)
    app = getapp(fig);
    app.CameraMode = 'top';
    apply_camera_mode(app);
    fig.UserData = app;
end

function apply_camera_mode(app)
    switch app.CameraMode
        case 'earth'
            restore_global_limits(app);
            set(app.Axes, 'CameraViewAngleMode', 'auto');
            camtarget(app.Axes, [0 0 0]);
            campos(app.Axes, [1.8 -1.8 1.1] * app.Data.Re_m/1e3 * 3);
            camup(app.Axes, [0 0 1]);
        case 'spacecraft'
            idx = app.Index;
            r = app.Data.pos_m(idx,:)/1e3;
            v = unit(app.Data.vel_mps(idx,:).');
            radial = unit(r.');
            % Planet-sized axis bounds can clip nearby spacecraft geometry in
            % a perspective camera. Keep the chase view bounds local.
            span = 150;
            xlim(app.Axes, r(1)+[-span span]);
            ylim(app.Axes, r(2)+[-span span]);
            zlim(app.Axes, r(3)+[-span span]);
            grid(app.Axes, 'off');
            if norm(v) < 1e-12
                v = [0;1;0];
            end
            offset = -80*v.' + 35*radial.' + [0 0 18];
            camtarget(app.Axes, r);
            campos(app.Axes, r + offset);
            camup(app.Axes, radial.');
            camva(app.Axes, 28);
        case 'top'
            restore_global_limits(app);
            set(app.Axes, 'CameraViewAngleMode', 'auto');
            camtarget(app.Axes, [0 0 0]);
            view(app.Axes, 0, 90);
    end
end

function restore_global_limits(app)
    lim = max(vecnorm(app.Data.pos_m, 2, 2))/1e3 * 1.15;
    xlim(app.Axes, [-lim lim]);
    ylim(app.Axes, [-lim lim]);
    zlim(app.Axes, [-lim lim]);
    grid(app.Axes, 'on');
end

function export_screenshot(fig)
    app = getapp(fig);
    [file, path] = uiputfile('*.png', 'Export viewer screenshot', 'adcs_3d_viewer.png');
    if isequal(file, 0)
        return;
    end
    exportapp(app.Figure, fullfile(path, file));
end

function close_viewer(fig)
    app = getapp(fig);
    if isfield(app, 'Timer') && isa(app.Timer, 'timer') && isvalid(app.Timer)
        stop(app.Timer);
        delete(app.Timer);
    end
    delete(fig);
end

function app = getapp(fig)
    app = fig.UserData;
end

function [results, label] = selftest_results()
    t = linspace(0, 600, 121).';
    Re = 6378137;
    rmag = Re + 500e3;
    n = 2*pi/5400;
    th = n*t;
    pos = rmag * [cos(th), sin(th), 0.25*sin(0.5*th)];
    vel = [gradient(pos(:,1), t), gradient(pos(:,2), t), gradient(pos(:,3), t)];
    q = zeros(numel(t), 4);
    for k = 1:numel(t)
        a = 0.2*sin(th(k));
        q(k,:) = [cos(a/2), 0, sin(a/2), 0];
    end
    sun = repmat(unit([0.8; -0.55; -0.24]).', numel(t), 1);
    results.signals = struct('pos_eci_time_s', t, 'pos_eci_m', pos, ...
        'vel_eci_time_s', t, 'vel_eci_mps', vel, 'q_time_s', t, 'q', q, ...
        'sun_eci_time_s', t, 'sun_eci', sun, 'omega_time_s', t, ...
        'omega_rad_s', zeros(numel(t), 3), 'tracker_valid_time_s', t, ...
        'tracker_valid', ones(numel(t), 1), 'mode_id_time_s', t, 'mode_id', 3*ones(numel(t), 1));
    results.parameters = struct('epoch_jd', 2461083.5, 'spacecraft_dimensions_m', [0.10 0.10 0.30]);
    label = 'internal viewer smoke-test fixture (not validated simulation output)';
end






