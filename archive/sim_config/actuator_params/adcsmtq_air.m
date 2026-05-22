%% Actuator data for Tensor Tech ADCS-MTQ air core
actuator = struct(          ...
    'dipole',           0.1 ...     % Magnetic dipole moment [A m^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'adcsmtq-air.mat');
save(save_path, 'actuator');