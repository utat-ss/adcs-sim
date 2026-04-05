clear all
close all
clc

%Inertia Matrix
Jxx = 1;
Jxy = 0;
Jxz = 0;
Jyy = 1;
Jyz = 0;
Jzz = 1;
J = [Jxx, Jxy, Jxz; Jxy, Jyy, Jyz; Jxz, Jxy, Jzz];
Jinv = inv(J);
%disp('Inertia matrix J=');
%disp(J);

%initial conditions, these are the values of the state vector at time = 0
%initial tilt
epsilon0 = [0.1; 0.0; 0.0];
%fprintf('epsilon0: %0.2f\n',epsilon0);

% Keep unit quaternion
%this comes from keeping the quarternion constraint e1^2+e2^2+e3^3 +
%eta^2=1. As we need to calculate the length of the vector we are using
%norm function
eta0 = sqrt(1 - norm(epsilon0)^2);
%fprintf('eta0: %0.2f\n',eta0);

%initial spin 
omega0 = [0.01; 0.02; 0.0]; 
%fprintf('omega0: %0.2f\n',omega0);

%state vector
x0 = [epsilon0; eta0; omega0];
%fprintf('x0: %0.2f\n',x0);
%fprintf('\n');

%External torque, i am assuming no external torque on the cubesat
torque_ext = [0; 0; 0];

% Simulation time, i am currently running for 2 minutes
t_start = 0;
t_end = 60*90;
total_time = [t_start, t_end];

%ODE for the simulation
[t, X] = ode45(@(t, x) dynamics(t, x, J, torque_ext), total_time, x0);

return

% Extract values for final result
epsilon_values = X(:, 1:3);
eta_values = X(:, 4);
omega_values = X(:, 5:7);

%Results
fprintf('Time    eps_x    eps_y    eps_z     eta    omg_x    omg_y    omg_z\n');
for i = 1:length(t)
fprintf('%.4f    %.4f    %.4f    %.4f    %.4f    %.4f    %.4f    %.4f\n',t(i),epsilon_values(i,1), epsilon_values(i,2), epsilon_values(i,3),eta_values(i),omega_values(i,1), omega_values(i,2), omega_values(i,3));
end

figure;
hold on;
plot(t, epsilon_values(:,1));
plot(t, epsilon_values(:,2));
plot(t, epsilon_values(:,3));
plot(t, eta_values);
hold off;

figure;
hold on;
plot(t, omega_values(:,1));
plot(t, omega_values(:,2));
plot(t, omega_values(:,3));
hold off;

figure;
hold on;
plot(t, sqrt(epsilon_values(:,1).^2 + epsilon_values(:,2).^2 + epsilon_values(:,3).^2 + eta_values.^2));
hold off;


