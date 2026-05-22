% Simulator configuration file for UTAT-SS ADCS simulator

%% Constants
G_M3PKGS2       = 6.67408E-11;              % m^3 kg^-1 s^-2
J2_EARTH        = 0.00108263;
J3_EARTH        = -(2.33936E-3) * J2_EARTH;
J4_EARTH        = -(1.49601E-3) * J2_EARTH;
J5_EARTH        = -(0.20995E-3) * J2_EARTH;
EQ_RADIUS_KM    = 6378;                     % km
MASS_EARTH_KG   = 5.972E24;                 % kg
MU_EARTH_KM3PS2 = 398600.435507;            % km^3 s^-2

%% Define simulation parameters
t0 = 0;                                     % Start time
tf = 1000;                                  % End time
dt = 0.01;                                  % Step size

%% Build spacecraft configuration - ground truth
sc = load('finch.mat').spacecraft;
%sc = toSimulinkParams(sc); % Convert to parameters useable in Simulink

%% Initialize orbit
orbit = struct(                         ...
    'semimajor_axis',       6867.5,     ... % km
    'eccentricity',         0.0,        ...
    'inclination',          97.6,       ... % deg
    'arg_periapsis',        0.0,        ... % deg
    'ascending_node',       103.07,     ... % deg
    'true_anomaly',         0.0,        ... % deg
    'epoch',                0           ...
    );

%% Initialize attitude
q0  = [0; 0; 0; 1];                         % Initial attitude quaternion
om0 = [0; 0; 0];                           % Initial body rates deg/s