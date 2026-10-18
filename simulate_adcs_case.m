function results = simulate_adcs_case(overrides)
%SIMULATE_ADCS_CASE Run the ADCS simulator with optional parameter overrides.

    if nargin < 1
        overrides = struct();
    end

    validateattributes(overrides, {'struct'}, {'scalar'}, mfilename, 'overrides');

    repo_root = fileparts(mfilename('fullpath'));
    prev_dir = pwd;
    cleanup = onCleanup(@() cd(prev_dir));
    cd(repo_root);

    evalin('base', 'init_adcs_params;');
    apply_overrides(overrides);
    evalin('base', 'build_adcs_model;');
    load_system('adcs_sim');

    stop_time = sprintf('%.15g', evalin('base', 't_end'));
    sim_out = sim('adcs_sim', ...
        'StopTime', stop_time, ...
        'SaveOutput', 'on', ...
        'ReturnWorkspaceOutputs', 'on');

    results = summarize_adcs_simulation(sim_out);
    results.overrides = overrides;

    close_system('adcs_sim', 0);
end


function apply_overrides(overrides)
    names = fieldnames(overrides);
    allowed_names = configurable_names();
    for idx = 1:numel(names)
        name = names{idx};
        value = overrides.(name);
        assert(any(strcmp(name, allowed_names)), ...
            'Unknown ADCS override "%s". Check spelling or add it to simulate_adcs_case configurable_names().', ...
            name);

        switch name
            case 'q0'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', 4}, mfilename, 'q0');
                value = value(:);
                q_norm = norm(value);
                assert(q_norm > 0, 'q0 must be nonzero.');
                value = value / q_norm;
            case 'omega0'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, 'omega0');
                value = value(:);
            case 'h_W0'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', 4}, mfilename, 'h_W0');
                value = value(:);
            case 'J'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [3 3]}, mfilename, 'J');
            case {'dt', 't_end'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, name);
            case {'mass_sc', 'sc_dim_x', 'sc_dim_y', 'sc_dim_z', 'R_earth', 'mu_earth', ...
                  'orbit_alt', 'rw_max_torque', 'rw_max_momentum', 'rw_max_speed', ...
                  'rw_inertia', 'mtq_max_dipole', 'speed_of_light', 'atm_scale_height'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, name);
            case {'solar_flux', 'solar_pressure_1au', 'atm_ref_density'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'}, mfilename, name);
            case {'fss_sample_rate', 'st_sample_rate', 'imu_sample_rate', 'mag_sample_rate', 'gnss_sample_rate'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, name);
            case {'fss_noise_sigma', 'st_noise_bore', 'st_noise_roll', 'imu_arw', ...
                  'imu_bias_instab', 'mag_noise_sigma', 'gnss_pos_sigma', 'gnss_vel_sigma', ...
                  'cmg_max_torque', 'cmg_momentum', 'cmg_gimbal_rate', 'ctrl_rw_kp', ...
                  'ctrl_rw_kd', 'ctrl_bdot_gain', 'ctrl_dump_gain', 'mode_detumble_rate', ...
                  'mode_fine_rate', 'mode_fine_err', 'B0_earth', 'drag_coeff', 'srp_coeff'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'}, mfilename, name);
            case 'imu_range'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, name);
            case {'fss_fov_half', 'st_exclusion_sun', 'st_exclusion_moon', 'st_exclusion_earth'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', '>', 0, '<=', 180}, mfilename, name);
            case 'ctrl_unload_fraction'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', '>=', 0, '<=', 1}, mfilename, name);
            case {'orbit_inc', 'orbit_RAAN', 'orbit_AOP', 'orbit_TA', 'epoch_jd', 'J2_earth'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, mfilename, name);
            case 'orbit_ecc'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', '>=', 0, '<', 1}, mfilename, name);
            case {'residual_dipole', 'mag_bias', 'st_boresight'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', 3}, mfilename, name);
                value = value(:);
                if strcmp(name, 'st_boresight')
                    assert(norm(value) > 1e-12, 'st_boresight must be nonzero.');
                    value = value / norm(value);
                end
            case 'rw_axes'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [3 4]}, mfilename, name);
            case 'mtq_axes'
                validateattributes(value, {'numeric'}, {'real', 'finite', 'size', [3 3]}, mfilename, name);
            otherwise
                validateattributes(value, {'numeric', 'logical', 'char', 'string'}, {'nonempty'}, mfilename, name);
        end

        assignin('base', name, value);
    end

    recompute_derived_parameters(isfield(overrides, 'J'), isfield(overrides, 'solar_pressure_1au'));
    validate_final_configuration();
end


function names = configurable_names()
    names = { ...
        'mass_sc', 'sc_dim_x', 'sc_dim_y', 'sc_dim_z', 'J', ...
        'q0', 'omega0', 'h_W0', 'dt', 't_end', ...
        'R_earth', 'mu_earth', 'J2_earth', 'orbit_alt', 'orbit_inc', ...
        'orbit_ecc', 'orbit_RAAN', 'orbit_AOP', 'orbit_TA', ...
        'fss_noise_sigma', 'fss_fov_half', 'fss_sample_rate', ...
        'st_noise_bore', 'st_noise_roll', 'st_sample_rate', 'st_exclusion_sun', ...
        'st_exclusion_moon', 'st_exclusion_earth', 'st_boresight', ...
        'imu_arw', 'imu_bias_instab', 'imu_sample_rate', 'imu_range', ...
        'mag_noise_sigma', 'mag_bias', 'mag_sample_rate', ...
        'gnss_pos_sigma', 'gnss_vel_sigma', 'gnss_sample_rate', ...
        'rw_max_torque', 'rw_max_momentum', 'rw_max_speed', 'rw_inertia', 'rw_axes', ...
        'mtq_max_dipole', 'mtq_axes', 'cmg_max_torque', 'cmg_momentum', 'cmg_gimbal_rate', ...
        'ctrl_rw_kp', 'ctrl_rw_kd', 'ctrl_bdot_gain', 'ctrl_dump_gain', ...
        'ctrl_unload_fraction', 'mode_detumble_rate', 'mode_fine_rate', 'mode_fine_err', ...
        'epoch_jd', 'B0_earth', 'solar_flux', 'speed_of_light', 'solar_pressure_1au', ...
        'atm_ref_density', 'atm_scale_height', 'drag_coeff', 'srp_coeff', 'residual_dipole'};
end


function recompute_derived_parameters(j_explicit, solar_pressure_explicit)
    mass_sc = evalin('base', 'mass_sc');
    sc_dim_x = evalin('base', 'sc_dim_x');
    sc_dim_y = evalin('base', 'sc_dim_y');
    sc_dim_z = evalin('base', 'sc_dim_z');

    if ~j_explicit
        assignin('base', 'J', diag((mass_sc / 12) * [ ...
            sc_dim_y^2 + sc_dim_z^2, ...
            sc_dim_x^2 + sc_dim_z^2, ...
            sc_dim_x^2 + sc_dim_y^2]));
    end

    assignin('base', 'sc_face_areas', [sc_dim_y*sc_dim_z, sc_dim_y*sc_dim_z, ...
        sc_dim_x*sc_dim_z, sc_dim_x*sc_dim_z, sc_dim_x*sc_dim_y, sc_dim_x*sc_dim_y]);
    assignin('base', 'sc_face_cop', [ sc_dim_x/2, -sc_dim_x/2,  0,          0,          0,          0; ...
                                      0,           0,           sc_dim_y/2, -sc_dim_y/2, 0,          0; ...
                                      0,           0,           0,          0,          sc_dim_z/2, -sc_dim_z/2 ]);

    r_earth = evalin('base', 'R_earth');
    orbit_alt = evalin('base', 'orbit_alt');
    mu_earth = evalin('base', 'mu_earth');
    orbit_a = r_earth + orbit_alt;
    orbit_period = 2*pi * sqrt(orbit_a^3 / mu_earth);
    assignin('base', 'orbit_a', orbit_a);
    assignin('base', 'orbit_period', orbit_period);
    assignin('base', 'orbit_n', 2*pi / orbit_period);

    if ~solar_pressure_explicit
        assignin('base', 'solar_pressure_1au', evalin('base', 'solar_flux / speed_of_light'));
    end

    assignin('base', 'mekf_sigma_g', evalin('base', 'imu_arw * pi/180'));
    assignin('base', 'mekf_sigma_b', evalin('base', 'imu_bias_instab * pi/180 / 3600'));
    assignin('base', 'mekf_sun_sigma', evalin('base', 'fss_noise_sigma * pi/180'));
    assignin('base', 'mekf_mag_sigma', evalin('base', 'max(mag_noise_sigma * 1e-9 / B0_earth, 1e-4)'));
end


function validate_final_configuration()
    J = evalin('base', 'J');
    assert(norm(J - J.', 'fro') < 1e-12, 'J must be symmetric.');
    principal_inertia = eig(J);
    assert(all(isfinite(principal_inertia)) && all(principal_inertia > 0), ...
        'J must be finite and positive definite.');
    assert(principal_inertia(1) + principal_inertia(2) >= principal_inertia(3) - 1e-12, ...
        'J principal moments violate the rigid-body inertia triangle inequality.');

    h_W0 = evalin('base', 'h_W0');
    rw_max_momentum = evalin('base', 'rw_max_momentum');
    rw_inertia = evalin('base', 'rw_inertia');
    rw_max_speed = evalin('base', 'rw_max_speed');
    assert(max(abs(h_W0)) <= rw_max_momentum + 1e-12, ...
        'h_W0 exceeds rw_max_momentum.');
    wheel_speed0_rpm = max(abs(h_W0 / rw_inertia)) * 60 / (2*pi);
    assert(wheel_speed0_rpm <= rw_max_speed + 1e-12, ...
        'h_W0 exceeds rw_max_speed through rw_inertia.');

    sample_rates = evalin('base', '[fss_sample_rate, st_sample_rate, imu_sample_rate, mag_sample_rate, gnss_sample_rate]');
    assert(all(isfinite(sample_rates)) && all(sample_rates > 0), ...
        'All sensor sample rates must be positive and finite.');
    sensor_periods = 1 ./ sample_rates;
    dt = evalin('base', 'dt');
    min_sensor_period = min(sensor_periods);
    assert(dt <= min_sensor_period + eps(min_sensor_period), ...
        'dt must be no larger than the fastest configured sensor sample period (%.6g s).', ...
        min_sensor_period);
    period_ratios = sensor_periods ./ dt;
    assert(max(abs(period_ratios - round(period_ratios))) < 1e-9, ...
        'dt must divide every configured sensor sample period exactly.');
    assert(evalin('base', 't_end') >= dt, 't_end must be at least one integration step.');

    orbit_a = evalin('base', 'orbit_a');
    orbit_ecc = evalin('base', 'orbit_ecc');
    r_earth = evalin('base', 'R_earth');
    assert(orbit_a * (1 - orbit_ecc) > r_earth, ...
        'Orbit perigee radius must remain above the Earth radius in this model.');

    rw_axes = evalin('base', 'rw_axes');
    assert(all(size(rw_axes) == [3 4]) && rank(rw_axes) == 3, ...
        'rw_axes must be a full-rank 3x4 mapping.');
    assert(max(abs(vecnorm(rw_axes, 2, 1) - 1)) < 1e-12, ...
        'Each reaction-wheel axis must be unit length.');
    mtq_axes = evalin('base', 'mtq_axes');
    assert(all(size(mtq_axes) == [3 3]) && rank(mtq_axes) == 3, ...
        'mtq_axes must be a full-rank 3x3 mapping.');
    assert(max(abs(vecnorm(mtq_axes, 2, 1) - 1)) < 1e-12, ...
        'Each magnetorquer axis must be unit length.');

    epoch_jd = evalin('base', 'epoch_jd');
    t_end = evalin('base', 't_end');
    ephem_t_sec = evalin('base', 'ephem_t_sec');
    sim_start_ephem_s = (epoch_jd - 2461041.5) * 86400;
    sim_stop_ephem_s = sim_start_ephem_s + t_end;
    assert(sim_start_ephem_s >= ephem_t_sec(1) - 1e-6 && sim_stop_ephem_s <= ephem_t_sec(end) + 1e-6, ...
        'Configured epoch_jd and t_end exceed the loaded ephemeris CSV coverage.');
end
