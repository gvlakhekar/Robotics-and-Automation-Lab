%--------------------------------------------------------------------------
% Forward Dynamics of Two Link Robotic Planar Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc;
clear;
close all;

%For two-link manipulator 
tspan=[0 10]; y0=[0;0;0;0]; 
[t,y]=ode45('two_link_dynamics',tspan,y0);

% Plot joint angles
figure;
plot(t, y(:,1), 'r-', t, y(:,3), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Angle (rad) and Joint Velocitie (rad/s)','fontsize',12,'interpreter','latex');
legend('$\theta_1$', '$\omega_1$','fontsize',12,'interpreter','latex');
title('2-Link Manipulator Joint Angles','fontsize',12,'interpreter','latex');
grid on;

% Plot joint velocities
figure;
plot(t, y(:,2), 'r-', t, y(:,4), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Angle (rad) and Joint Velocitie (rad/s)','fontsize',12,'interpreter','latex');
legend('$\theta_2$', '$\omega_2$','fontsize',12,'interpreter','latex');
title('2-Link Manipulator Joint Velocities','fontsize',12,'interpreter','latex');
grid on;