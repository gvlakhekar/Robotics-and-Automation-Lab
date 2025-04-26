%--------------------------------------------------------------------------
% Inverse Dynamics of 2R Robotic Planar Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc;
clear;
close all;

% Time and trajectory setup
T = 10;                     % Duration of motion (s)
con = 2*pi/T;               % Angular frequency

% Joint limits
th10 = 0; th1T = pi;        % Joint 1: initial & final
th20 = 0; th2T = pi/2;      % Joint 2: initial & final
delth1 = th1T - th10;
delth2 = th2T - th20;

% Link and mass properties
m1 = 1; m2 = 1;             % Masses
a1 = 1; a2 = 1;             % Link lengths
g = 9.81;                   % Gravity (m/s^2)
iner21 = m2 * a1 * a2;      % Cross inertia term

% Time vector
N = 51;
ti = linspace(0, T, N);

% Preallocate
th1 = zeros(1,N); th2 = zeros(1,N);
th1d = zeros(1,N); th2d = zeros(1,N);
th1dd = zeros(1,N); th2dd = zeros(1,N);
tor1 = zeros(1,N); tor2 = zeros(1,N);

% Loop over time to compute trajectories and torques
for i = 1:N
    ang = con * ti(i);

    % Cycloidal trajectories
    th1(i)   = th10 + (delth1 / T) * (ti(i) - sin(ang)/con);
    th1d(i)  = delth1 * (1 - cos(ang)) / T;
    th1dd(i) = delth1 * con * sin(ang) / T;

    th2(i)   = th20 + (delth2 / T) * (ti(i) - sin(ang)/con);
    th2d(i)  = delth2 * (1 - cos(ang)) / T;
    th2dd(i) = delth2 * con * sin(ang) / T;

    % Dynamics
    thdd = [th1dd(i); th2dd(i)];

    sth2 = sin(th2(i));
    cth2 = cos(th2(i));
    cth1 = cos(th1(i));
    cth12 = cos(th1(i) + th2(i));

    % Inertia matrix
    I22 = m2*a2^2 / 3;
    I21 = I22 + iner21 * cth2 / 2;
    I12 = I21;
    I11 = I22 + m1*a1^2/3 + m2*a1^2 + iner21 * cth2;

    Im = [I11, I12; I21, I22];

    % Coriolis and centrifugal terms
    h1 = -(m2*a1*a2*th1d(i) + iner21/2*th2d(i)) * th2d(i) * sth2;
    h2 = iner21/2 * sth2 * th1d(i)^2;
    Hv = [h1; h2];

    % Gravity terms
    gam1 = m1*g*a1/2*cth1 + m2*g*(a1*cth1 + a2/2*cth12);
    gam2 = m2*g*a2/2*cth12;
    Gv = [gam1; gam2];

    % Joint torque
    tau = Im * thdd + Hv + Gv;
    tor1(i) = tau(1);
    tor2(i) = tau(2);
end

% Plot joint trajectories
figure;
plot(ti, th1, 'r-.', ti, th2, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Angles (rad)','fontsize',12,'interpreter','latex');
legend('$\theta_1$', '$\theta_2$','fontsize',12,'interpreter','latex');
title('2R Manipulator Joint Trajectories','fontsize',12,'interpreter','latex');
grid on;

% Plot joint torques
figure;
plot(ti, tor1, 'r-.', ti, tor2, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Torque (Nm)','fontsize',12,'interpreter','latex');
legend('$\tau_1$', '$\tau_2$','fontsize',12,'interpreter','latex');
title('Required Joint Torques','fontsize',12,'interpreter','latex');
grid on;
