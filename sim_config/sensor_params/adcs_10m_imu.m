%% Sensor data for the TensorTech ADCS-10m IMU
sensor = struct(                                            ...
    'bias_instability',     [3, 3, 3],                      ... % Bias instability [deg/hr]
    'arw',                  [0.21, 0.21, 0.21],             ... % Angular random walk [deg/sqrt(hr)]
    'cov',                  diag([0.02^2, 0.02^2, 0.02^2])  ... % Gyro covariance [deg^2/sec^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'adcs-10m-imu.mat');
save(save_path, 'sensor');