%% Actuator data for Tensor Tech ADCS-MTQ solid core
actuator = struct(          ...
    'dipole',           0.2 ...     % Magnetic dipole moment [A m^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'adcsmtq-core.mat');
save(save_path, 'actuator');