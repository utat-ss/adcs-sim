%% Sensor data for the Saggitta Arcsec
sensor = struct(                                            ...
    'fov',          25.2,                                   ... % Field of view (full cone) [deg]
    'exclusion',    80,                                     ... % Exclusion zone (full cone) [deg]
    'cov',          diag([0.0014^2, 0.0014^2, 0.0056^2])    ... % Simple covariance (yaw, pitch, roll) [deg^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'arcsec.mat');
save(save_path, 'sensor');