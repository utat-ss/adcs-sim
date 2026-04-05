%% Sensor data for the TensorTech FSS-15 fine sun sensor
sensor = struct(                    ...
    'fov',    120.00,               ...   % Field of view (full-cone) [deg]
    'cov',    diag([0.2^2, 0.2^2])  ...   % Covariance (x, y) [deg^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'fss-15.mat');
save(save_path, 'sensor');