%--------------------------------------------------------------------------
% Inverse Dynamics of 3R Robotic Planar Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc;
clear;
close all;

% Time and trajectory setup
T = 10;
con = 2 * pi / T;

% Joint trajectory limits
th0 = [0; 0; 0];
thT = [pi; pi/2; pi/3];
delth = thT - th0;

% Link and mass parameters
m = [1; 1; 1];              % masses
a = [1; 1; 1];              % link lengths
g = 9.81;

% Time vector
N = 51;
ti = linspace(0, T, N);

% Preallocate
th = zeros(3, N);
thd = zeros(3, N);
thdd = zeros(3, N);
tor = zeros(3, N);

% Loop through time steps
for i = 1:N
    ang = con * ti(i);
    
    % Cycloidal trajectories for each joint
    for j = 1:3
        th(j,i)   = th0(j) + (delth(j)/T)*(ti(i) - sin(ang)/con);
        thd(j,i)  = delth(j)*(1 - cos(ang))/T;
        thdd(j,i) = delth(j)*con*sin(ang)/T;
    end

    % Current joint states
    theta = th(:,i);
    dtheta = thd(:,i);
    ddtheta = thdd(:,i);
    
    % Compute dynamics (manual or using symbolic expressions)
    % For simplicity, we use a basic model where:
    % Inertia matrix: M
    % Coriolis/Centrifugal: H
    % Gravity: G
    % Total torque: τ = M*ddθ + H + G

    % Precompute trigonometric terms
    c12 = cos(theta(1) + theta(2));
    c23 = cos(theta(2) + theta(3));
    c123 = cos(theta(1) + theta(2) + theta(3));
    c1 = cos(theta(1));
    c2 = cos(theta(2));
    c3 = cos(theta(3));
    s2 = sin(theta(2));
    s3 = sin(theta(3));
    s23 = sin(theta(2) + theta(3));
    s123 = sin(theta(1) + theta(2) + theta(3));

    % Inertia matrix M (simplified with constant link inertia)
    I1 = m(1)*a(1)^2/3;
    I2 = m(2)*a(2)^2/3;
    I3 = m(3)*a(3)^2/3;

    M = zeros(3,3);
    M(1,1) = I1 + I2 + I3 + m(2)*(a(1)^2 + a(1)*a(2)*cos(theta(2))) + m(3)*(a(1)^2 + a(2)^2 + a(3)^2 + 2*a(1)*a(2)*cos(theta(2)) + 2*a(2)*a(3)*cos(theta(3)) + 2*a(1)*a(3)*cos(theta(2)+theta(3)));
    M(1,2) = I2 + I3 + m(2)*a(1)*a(2)*cos(theta(2)) + m(3)*(a(2)^2 + a(3)^2 + a(1)*a(2)*cos(theta(2)) + 2*a(2)*a(3)*cos(theta(3)) + a(1)*a(3)*cos(theta(2)+theta(3)));
    M(1,3) = I3 + m(3)*(a(3)^2 + a(2)*a(3)*cos(theta(3)) + a(1)*a(3)*cos(theta(2)+theta(3)));
    M(2,1) = M(1,2);
    M(2,2) = I2 + I3 + m(3)*(a(2)^2 + a(3)^2 + 2*a(2)*a(3)*cos(theta(3)));
    M(2,3) = I3 + m(3)*(a(3)^2 + a(2)*a(3)*cos(theta(3)));
    M(3,1) = M(1,3);
    M(3,2) = M(2,3);
    M(3,3) = I3 + m(3)*a(3)^2;

    % Coriolis and centrifugal (very simplified placeholder)
    H = [0; 0; 0];  % You can expand this with real Coriolis terms

    % Gravity vector
    G = zeros(3,1);
    G(1) = g*(m(1)*a(1)/2*cos(theta(1)) + m(2)*(a(1)*cos(theta(1)) + a(2)/2*cos(theta(1)+theta(2))) + m(3)*(a(1)*cos(theta(1)) + a(2)*cos(theta(1)+theta(2)) + a(3)/2*cos(theta(1)+theta(2)+theta(3))));
    G(2) = g*(m(2)*a(2)/2*cos(theta(1)+theta(2)) + m(3)*(a(2)*cos(theta(1)+theta(2)) + a(3)/2*cos(theta(1)+theta(2)+theta(3))));
    G(3) = g*m(3)*a(3)/2*cos(theta(1)+theta(2)+theta(3));

    % Torque
    tau = M * ddtheta + H + G;

    tor(:,i) = tau;
end

% Plot joint angles
figure;
plot(ti, th(1,:), 'r-', ti, th(2,:), 'g--', ti, th(3,:), 'b-.', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Angles (rad)','fontsize',12,'interpreter','latex');
legend('$\theta_1$', '$\theta_2$', '$\theta_3$','fontsize',12,'interpreter','latex');
title('3R Manipulator Joint Trajectories','fontsize',12,'interpreter','latex');
grid on;

% Plot torques
figure;
plot(ti, tor(1,:), 'r-', ti, tor(2,:), 'g--', ti, tor(3,:), 'b-.', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Torques (Nm)','fontsize',12,'interpreter','latex');
legend('$\tau_1$', '$\tau_2$', '$\tau_3$','fontsize',12,'interpreter','latex');
title('Required Joint Torques','fontsize',12,'interpreter','latex');
grid on;
