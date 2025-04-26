%--------------------------------------------------------------------------
% Inverse Dynamics of One-link Planar Arm  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc;
clear;
close all;

% Input parameters
T = 10;              % Total time (s)
thT = pi;            % Final angle (rad)
th0 = 0;             % Initial angle (rad)
m = 1;               % Mass (kg)
a = 1;               % Link length (m)
g = 9.81;            % Gravity (m/s^2)

% Derived parameters
con = 2 * pi / T;            % Angular frequency
delth = thT - th0;           % Change in angle
iner = m * a^2 / 3;          % Moment of inertia (uniform rod about one end)
grav = m * g * a / 2;        % Gravitational torque constant

% Time vector
N = 51;                      % Number of time steps
ti = linspace(0, T, N);      % Time vector

% Preallocate arrays
th = zeros(1, N);
thd = zeros(1, N);
thdd = zeros(1, N);
tau = zeros(1, N);

% Loop to compute trajectory and torque
for i = 1:N
    ang = con * ti(i); % current angular phase
    th(i)   = th0 + (delth / T) * (ti(i) - sin(ang) / con);
    thd(i)  = delth * (1 - cos(ang)) / T;
    thdd(i) = delth * con * sin(ang) / T;
    tau(i)  = iner * thdd(i) + grav * sin(th(i)); % computed torque
end

% Plot joint trajectory
figure;
plot(ti, th,'b-', ti,thd,'r--', ti, thdd,'g-.','LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint States','fontsize',12,'interpreter','latex');
legend ('$\theta (rad)$','$\dot{\theta}(rad/s)$','$\ddot{\theta}(rad/s^2)$','fontsize',12,'interpreter','latex');
grid on
title('Joint Trajectory','fontsize',12,'interpreter','latex');

% Plot torque
figure;
plot(ti, tau, 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Torque (Nm)','fontsize',12,'interpreter','latex');
title('Required Joint Torque','fontsize',12,'interpreter','latex');
grid on;
