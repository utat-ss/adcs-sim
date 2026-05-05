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
    for idx = 1:numel(names)
        name = names{idx};
        value = overrides.(name);

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
                assert(rcond(value) > 1e-12, 'J must be invertible.');
            case {'dt', 't_end'}
                validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, mfilename, name);
            otherwise
                validateattributes(value, {'numeric', 'logical', 'char', 'string'}, {'nonempty'}, mfilename, name);
        end

        assignin('base', name, value);
    end
end
