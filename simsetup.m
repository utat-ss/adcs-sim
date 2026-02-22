%% Sensor DCMS
dcm_mag = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss1 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss2 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss3 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss4 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss5 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_fss6 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_imu = [1, 0, 0; 0, 1, 0; 0, 0, 1];
dcm_str = [1, 0, 0; 0, 1, 0; 0, 0, 1];

%% Sensor variances
sigma_mag = [0.1, 0.1, 0.1];
sigma_fss1 = [0.1, 0.1];
sigma_fss2 = [0.1, 0.1];
sigma_fss3 = [0.1, 0.1];
sigma_fss4 = [0.1, 0.1];
sigma_fss5 = [0.1, 0.1];
sigma_fss6 = [0.1, 0.1];
sigma_imu = [0.1, 0.1, 0.1];
sigma_str = [0.1, 0.1, 0.1];

%% Sensor refresh rates
f_mag = 1;
f_fss1 = 1;
f_fss2 = 1;
f_fss3 = 1;
f_fss4 = 1;
f_fss5 = 1;
f_fss6 = 1;
f_imu = 1;
f_str = 1;

%% Sensor scale limits
max_mag = [-8e5, 8e5]; % nT

out = sim('adcs-sim',10);