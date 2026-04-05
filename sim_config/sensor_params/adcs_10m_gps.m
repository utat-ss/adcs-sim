%% Sensor parameters for the Tensor Tech GNSS unit
% Includes receiver and antenna
sensor = struct( ...
    'constellations',       {'gps'},                        ... % Available GNSS constellations
    'hpbw',                 100,                            ... % Half-Power Beamwidth [deg]
    'cov',                  diag([5.0^2, 5.0^2, 5.0^2,      ...
                                  0.15^2, 0.15^2, 0.15^2])  ... % Covariance [m^2 ... m^2/s^2]
    );
[f_dir, ~, ~] = fileparts(mfilename('fullpath'));
save_path = fullfile(f_dir, 'adcs-10m-gps.mat');
save(save_path, 'sensor');