%% Actuator data for Tensor Tech CMG-10m-N control moment gyroscope
actuator = struct(              ...
    'moi',          12.85,      ... % Rotor moment of inertia [kg mm^2]
    'acc_torque',   1,          ... % Acceleration torque [mN m]
    'gyro_torque',  [15, 100],  ... % Gyroscopic torque [mN m]
    'max_speed',    10500,      ... % Maximum rotor speed [RPM]
    'speed_err',    0.05,       ... % Rotor speed error [dec %]
    'gimbal_err',   0.5         ... % Gimbal angle error [deg]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'cmg-10m-n.mat');
save(save_path, 'actuator');