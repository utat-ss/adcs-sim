%% Sensor data for the TensorTech FSS-15M magnetometer
sensor = struct( ...
    'range',        [-800, 800],                ... % Measurement range [uT]
    'resolution',   0.0061,                     ... % Measurement resolution [uT]
    'offset',       1,                          ... % Measurement offset [uT]
    'sensitivity',  0.01,                       ... % Sensitivity error [dec %FS]
    'cov',          diag([41^2, 41^2, 41^2])    ... % Covariance [nT]
);
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'fss-15m.mat');
save(save_path, 'sensor');