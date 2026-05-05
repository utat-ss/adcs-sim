function results = summarize_adcs_simulation(sim_out)
%SUMMARIZE_ADCS_SIMULATION Collect logged signals and derived ADCS metrics.

    q_log = require_timeseries(sim_out, 'q_log');
    omega_log = require_timeseries(sim_out, 'omega_log');
    h_w_log = require_timeseries(sim_out, 'h_W_log');

    q = orient_samples(q_log.Data, 4);
    omega = orient_samples(omega_log.Data, 3);
    h_w = orient_samples(h_w_log.Data, 4);

    t_q = q_log.Time(:);
    t_omega = omega_log.Time(:);
    t_h_w = h_w_log.Time(:);

    q_norm = sqrt(sum(q.^2, 2));
    q_unit = normalize_rows(q);
    q0 = normalize_quaternion(evalin('base', 'q0'));

    J = evalin('base', 'J');
    dt = evalin('base', 'dt');
    t_end = evalin('base', 't_end');
    omega0 = evalin('base', 'omega0');
    h_w0 = evalin('base', 'h_W0');
    mass_sc = evalin('base', 'mass_sc');
    orbit_alt = evalin('base', 'orbit_alt');
    orbit_period = evalin('base', 'orbit_period');
    rw_max_momentum = evalin('base', 'rw_max_momentum');
    rw_max_speed = evalin('base', 'rw_max_speed');
    rw_inertia = evalin('base', 'rw_inertia');
    rw_max_torque = evalin('base', 'rw_max_torque');
    mtq_max_dipole = evalin('base', 'mtq_max_dipole');
    cmg_max_torque = evalin('base', 'cmg_max_torque');

    quat_norm_error = abs(q_norm - 1);
    attitude_excursion_deg = quaternion_separation_deg(q_unit, q0);

    rate_norm = vecnorm(omega, 2, 2);
    rate_norm_deg_s = rate_norm * 180 / pi;
    peak_rate_axes_deg_s = max(abs(omega), [], 1) * 180 / pi;
    rms_rate_axes_deg_s = sqrt(mean(omega.^2, 1)) * 180 / pi;

    omega_dot = finite_difference(t_omega, omega);
    omega_dot_norm = vecnorm(omega_dot, 2, 2);
    omega_dot_norm_deg_s2 = omega_dot_norm * 180 / pi;

    h_w_norm = vecnorm(h_w, 2, 2);
    peak_wheel_momentum_axes = max(abs(h_w), [], 1);
    peak_wheel_momentum_util = max(peak_wheel_momentum_axes) / rw_max_momentum;

    wheel_speed_rpm = (h_w / rw_inertia) * (60 / (2 * pi));
    peak_wheel_speed_rpm = max(max(abs(wheel_speed_rpm)));
    peak_wheel_speed_util = peak_wheel_speed_rpm / rw_max_speed;

    a_w = reaction_wheel_axes();
    body_ang_momentum = (J * omega.').';
    wheel_body_momentum = (a_w * h_w.').';
    total_ang_momentum = body_ang_momentum + wheel_body_momentum;
    total_ang_momentum_norm = vecnorm(total_ang_momentum, 2, 2);

    kinetic_energy = 0.5 * sum(omega .* body_ang_momentum, 2);

    results = struct();
    results.signals = struct( ...
        'q_time_s', t_q, ...
        'q', q, ...
        'quat_norm_error', quat_norm_error, ...
        'attitude_excursion_deg', attitude_excursion_deg, ...
        'omega_time_s', t_omega, ...
        'omega_rad_s', omega, ...
        'omega_norm_rad_s', rate_norm, ...
        'omega_norm_deg_s', rate_norm_deg_s, ...
        'omega_dot_rad_s2', omega_dot, ...
        'omega_dot_norm_deg_s2', omega_dot_norm_deg_s2, ...
        'h_w_time_s', t_h_w, ...
        'h_w_nms', h_w, ...
        'h_w_norm_nms', h_w_norm, ...
        'wheel_speed_rpm', wheel_speed_rpm, ...
        'kinetic_energy_j', kinetic_energy, ...
        'total_ang_momentum_nms', total_ang_momentum, ...
        'total_ang_momentum_norm_nms', total_ang_momentum_norm);

    results.metrics = struct( ...
        'duration_s', max([t_q(end), t_omega(end), t_h_w(end)]), ...
        'sample_count', size(q, 1), ...
        'final_attitude_excursion_deg', attitude_excursion_deg(end), ...
        'peak_attitude_excursion_deg', max(attitude_excursion_deg), ...
        'max_quat_norm_error', max(quat_norm_error), ...
        'final_rate_norm_deg_s', rate_norm_deg_s(end), ...
        'peak_rate_norm_deg_s', max(rate_norm_deg_s), ...
        'rms_rate_norm_deg_s', sqrt(mean(rate_norm_deg_s.^2)), ...
        'peak_rate_axes_deg_s', peak_rate_axes_deg_s, ...
        'rms_rate_axes_deg_s', rms_rate_axes_deg_s, ...
        'peak_angular_accel_deg_s2', max(omega_dot_norm_deg_s2), ...
        'final_kinetic_energy_j', kinetic_energy(end), ...
        'peak_kinetic_energy_j', max(kinetic_energy), ...
        'final_wheel_momentum_norm_nms', h_w_norm(end), ...
        'peak_wheel_momentum_norm_nms', max(h_w_norm), ...
        'peak_wheel_momentum_axes_nms', peak_wheel_momentum_axes, ...
        'peak_wheel_momentum_utilization_pct', 100 * peak_wheel_momentum_util, ...
        'peak_wheel_speed_rpm', peak_wheel_speed_rpm, ...
        'peak_wheel_speed_utilization_pct', 100 * peak_wheel_speed_util, ...
        'peak_total_ang_momentum_norm_nms', max(total_ang_momentum_norm));

    results.parameters = struct( ...
        'dt_s', dt, ...
        't_end_s', t_end, ...
        'q0', q0, ...
        'omega0_rad_s', omega0, ...
        'h_w0_nms', h_w0, ...
        'J_kgm2', J, ...
        'mass_sc_kg', mass_sc, ...
        'orbit_altitude_km', orbit_alt / 1e3, ...
        'orbit_period_min', orbit_period / 60, ...
        'rw_max_momentum_nms', rw_max_momentum, ...
        'rw_max_speed_rpm', rw_max_speed, ...
        'rw_max_torque_nm', rw_max_torque, ...
        'mtq_max_dipole_am2', mtq_max_dipole, ...
        'cmg_max_torque_nm', cmg_max_torque);

    results.metric_rows = {
        'Simulation duration', format_scalar(results.metrics.duration_s), 's';
        'Samples', format_scalar(results.metrics.sample_count), '-';
        'Final attitude excursion', format_scalar(results.metrics.final_attitude_excursion_deg), 'deg';
        'Peak attitude excursion', format_scalar(results.metrics.peak_attitude_excursion_deg), 'deg';
        'Max quaternion norm error', format_scalar(results.metrics.max_quat_norm_error), '-';
        'Final rate norm', format_scalar(results.metrics.final_rate_norm_deg_s), 'deg/s';
        'Peak rate norm', format_scalar(results.metrics.peak_rate_norm_deg_s), 'deg/s';
        'RMS rate norm', format_scalar(results.metrics.rms_rate_norm_deg_s), 'deg/s';
        'Peak angular acceleration', format_scalar(results.metrics.peak_angular_accel_deg_s2), 'deg/s^2';
        'Peak rate by axis', format_vector(results.metrics.peak_rate_axes_deg_s), 'deg/s';
        'RMS rate by axis', format_vector(results.metrics.rms_rate_axes_deg_s), 'deg/s';
        'Final kinetic energy', format_scalar(results.metrics.final_kinetic_energy_j), 'J';
        'Peak kinetic energy', format_scalar(results.metrics.peak_kinetic_energy_j), 'J';
        'Final wheel momentum norm', format_scalar(results.metrics.final_wheel_momentum_norm_nms), 'N*m*s';
        'Peak wheel momentum norm', format_scalar(results.metrics.peak_wheel_momentum_norm_nms), 'N*m*s';
        'Peak wheel momentum by wheel', format_vector(results.metrics.peak_wheel_momentum_axes_nms), 'N*m*s';
        'Peak wheel momentum utilization', format_scalar(results.metrics.peak_wheel_momentum_utilization_pct), '%';
        'Peak estimated wheel speed', format_scalar(results.metrics.peak_wheel_speed_rpm), 'RPM';
        'Peak wheel speed utilization', format_scalar(results.metrics.peak_wheel_speed_utilization_pct), '%';
        'Peak total angular momentum norm', format_scalar(results.metrics.peak_total_ang_momentum_norm_nms), 'N*m*s'
        };

    results.parameter_rows = {
        'Time step dt', format_scalar(results.parameters.dt_s), 's';
        'Simulation horizon t_end', format_scalar(results.parameters.t_end_s), 's';
        'Initial quaternion q0', format_vector(results.parameters.q0), '-';
        'Initial body rate omega0', format_vector(results.parameters.omega0_rad_s), 'rad/s';
        'Initial wheel momentum h_W0', format_vector(results.parameters.h_w0_nms), 'N*m*s';
        'Inertia diagonal', format_vector(diag(results.parameters.J_kgm2)), 'kg*m^2';
        'Spacecraft mass', format_scalar(results.parameters.mass_sc_kg), 'kg';
        'Orbit altitude', format_scalar(results.parameters.orbit_altitude_km), 'km';
        'Orbit period', format_scalar(results.parameters.orbit_period_min), 'min';
        'RW max momentum', format_scalar(results.parameters.rw_max_momentum_nms), 'N*m*s';
        'RW max speed', format_scalar(results.parameters.rw_max_speed_rpm), 'RPM';
        'RW max torque', format_scalar(results.parameters.rw_max_torque_nm), 'N*m';
        'MTQ max dipole', format_scalar(results.parameters.mtq_max_dipole_am2), 'A*m^2';
        'CMG max torque', format_scalar(results.parameters.cmg_max_torque_nm), 'N*m'
        };
end


function ts = require_timeseries(sim_out, name)
    ts = sim_out.get(name);
    assert(isa(ts, 'timeseries'), 'Expected %s timeseries output.', name);
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


function q = normalize_quaternion(q)
    q = q(:);
    n = norm(q);
    assert(n > 0, 'Initial quaternion must be nonzero.');
    q = q / n;
    if q(1) < 0
        q = -q;
    end
end


function rows = normalize_rows(rows)
    scales = sqrt(sum(rows.^2, 2));
    scales(scales < 1e-12) = 1;
    rows = rows ./ scales;
end


function ang_deg = quaternion_separation_deg(q, q_ref)
    dots = abs(q * q_ref(:));
    dots = min(max(dots, -1), 1);
    ang_deg = 2 * acos(dots) * 180 / pi;
end


function deriv = finite_difference(t, samples)
    deriv = zeros(size(samples));
    if numel(t) < 2
        return;
    end

    for col = 1:size(samples, 2)
        deriv(:, col) = gradient(samples(:, col), t);
    end
end


function a_w = reaction_wheel_axes()
    beta = atan(1 / sqrt(2));
    cb = cos(beta);
    sb = sin(beta);
    a_w = [ cb,  0, -cb,  0;
             0, cb,   0, -cb;
            sb, sb,  sb,  sb ];
end


function text = format_scalar(value)
    text = sprintf('%.6g', value);
end


function text = format_vector(values)
    text = sprintf('[%s]', strjoin(arrayfun(@(v) sprintf('%.4g', v), values(:).', ...
        'UniformOutput', false), '  '));
end
