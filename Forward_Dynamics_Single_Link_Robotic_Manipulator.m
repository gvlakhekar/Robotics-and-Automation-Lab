%--------------------------------------------------------------------------
% Forward Dynamics of Single Link Robotic Planar Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 

clc;
clear;
close all;

% Parameters
m = 1;      % mass (kg)
a = 1;      % link length (m)
g = 9.81;   % gravity (m/s^2)
tau = 0;    % input torque (Nm)

iner = m * a^2 / 3;      % moment of inertia
grav = m * g * a / 2;    % gravity torque term

% Define dynamics as an inline function
dyn = @(t, y) [y(2); (tau - grav * sin(y(1))) / iner];

% Simulation setup
tspan = [0 10];
y0 = [pi/2; 0];          % initial angle and velocity

% Run simulation
[t, y] = ode45(dyn, tspan, y0);

% Plot results
plot(t, y(:,1), 'r-', t, y(:,2), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('State','fontsize',12,'interpreter','latex');
legend('$\theta$ (rad)', '$\omega$ (rad/s)','fontsize',12,'interpreter','latex');
title('Dynamics of a 1-Link Robotic Arm','fontsize',12,'interpreter','latex');
grid on;
