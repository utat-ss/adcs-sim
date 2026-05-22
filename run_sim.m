clear; clc;

%% User specification
% Specify simulation configuration file
cfg = 'sim_config.m';

% Specify simulation model file
model = 'adcs_sim_finch';

%% Runtime - shouldn't need to change
% Load simulation configuration data
run(cfg);

% Initialize simulation input
load_system(model);
simIn = Simulink.SimulationInput(model);

% Run simulation
simOut = sim(simIn);