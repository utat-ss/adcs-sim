function report = run_pointing_validation()
%RUN_POINTING_VALIDATION Save bounded truth-based pointing and step-size evidence.
% The near-nadir case has an outward-facing tracker and a small initial error.
% Its convergence does not establish default mission acquisition performance.
% Outward mounting alone does not guarantee tracker availability: Sun/Moon
% exclusion can still invalidate it. The saved report records availability.
    root = fileparts(fileparts(mfilename('fullpath')));
    previous = pwd;
    cleanup = onCleanup(@() cd(previous));
    cd(root);
    out = fullfile(root, 'validation_logs');

    baseline = simulate_adcs_case(struct('t_end', 120));
    save(fullfile(out, 'pointing_default_120s.mat'), 'baseline');
    report.default = assess(baseline);
    fprintf('DEFAULT_CASE=%s\n', jsonencode(report.default));

    q = baseline.signals.q_des(1, :).';
    dq = [cosd(2.5); 0; sind(2.5); 0];
    initial = [q(1)*dq(1)-dot(q(2:4),dq(2:4)); ...
        q(1)*dq(2:4)+dq(1)*q(2:4)+cross(q(2:4),dq(2:4))];
    r = baseline.signals.pos_eci_m(1,:);
    v = baseline.signals.vel_eci_mps(1,:);
    reference_rate = norm(cross(r,v))/dot(r,r);
    options = struct('t_end', 120, 'dt', 0.01, 'q0', initial, ...
        'omega0', [0;-reference_rate;0], ...
        'st_boresight', [0;0;-1]);
    coarse = simulate_adcs_case(options);
    save(fullfile(out, 'pointing_near_nadir_dt010.mat'), 'coarse');
    report.near_nadir = assess(coarse);
    fprintf('NEAR_NADIR_CASE=%s\n', jsonencode(report.near_nadir));
    options.dt = 0.005;
    fine = simulate_adcs_case(options);
    save(fullfile(out, 'pointing_near_nadir_dt005.mat'), 'fine');
    report.half_step = assess(fine);
    qfine = interp1(fine.signals.q_time_s, fine.signals.q, coarse.signals.q_time_s);
    report.max_step_attitude_difference_deg = max(separation(coarse.signals.q, qfine));
    wfine = interp1(fine.signals.omega_time_s, fine.signals.omega_rad_s, coarse.signals.omega_time_s);
    report.max_step_rate_difference_deg_s = max(vecnorm(coarse.signals.omega_rad_s-wfine,2,2))*180/pi;
    report.pointing_check_passed = report.near_nadir.final_truth_error_deg < 1 ...
        && report.half_step.final_truth_error_deg < 1;
    report.step_check_passed = report.max_step_attitude_difference_deg < 0.1 ...
        && report.max_step_rate_difference_deg_s < 0.01;
    save(fullfile(out, 'pointing_validation_20260922.mat'), 'report');
    fid = fopen(fullfile(out, 'pointing_validation_20260922.json'), 'w');
    assert(fid >= 0, 'Could not create pointing validation summary.');
    closer = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', jsonencode(report, PrettyPrint=true));
    fprintf('POINTING_VALIDATION=%s\n', jsonencode(report));
    % Restore the default generated model even if a diagnostic criterion fails.
    evalin('base', 'init_adcs_params; build_adcs_model;');
    close_system('adcs_sim', 0);
    assert(report.pointing_check_passed, 'Near-nadir pointing did not reach 1 degree; inspect saved evidence.');
    assert(report.step_check_passed, 'Step-size differences exceed 0.1 deg / 0.01 deg/s; inspect saved evidence.');
end

function summary = assess(result)
    s = result.signals;
    desired = interp1(s.q_des_time_s, s.q_des, s.q_time_s);
    error = separation(s.q, desired);
    estimate = interp1(s.q_est_time_s, s.q_est, s.q_time_s);
    summary = struct('initial_truth_error_deg', error(1), ...
        'final_truth_error_deg', error(end), ...
        'last_10s_max_truth_error_deg', max(error(s.q_time_s >= s.q_time_s(end)-10)), ...
        'max_estimation_error_deg', max(separation(s.q, estimate)), ...
        'final_rate_deg_s', result.metrics.final_rate_norm_deg_s, ...
        'tracker_valid_fraction', mean(s.tracker_valid > 0.5), ...
        'detumble_fraction', mean(s.mode_id == 1), ...
        'fine_pointing_fraction', mean(s.mode_id == 3));
end

function angle = separation(a, b)
    a = a ./ vecnorm(a,2,2);
    b = b ./ vecnorm(b,2,2);
    angle = 2*acosd(min(1,abs(sum(a.*b,2))));
end
