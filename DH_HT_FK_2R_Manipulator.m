% DH-Parameter-Homogenous Trasformation_FK

clc;
clear;
close all;

% Link lengths
L1 = 5; % Length of link 1
L2 = 4; % Length of link 2

% Define joint angles at home position (theta1=0, theta2=0)
theta1_home = 0; % radians
theta2_home = 0; % radians

% Calculate end-effector position at home
x_home = L1*cos(theta1_home) + L2*cos(theta1_home + theta2_home);
y_home = L1*sin(theta1_home) + L2*sin(theta1_home + theta2_home);

disp('Home Position:');
disp(['X = ', num2str(x_home)]);
disp(['Y = ', num2str(y_home)]);

% Plot robot at home position
figure;
plot([0, L1*cos(theta1_home), x_home], [0, L1*sin(theta1_home), y_home], '-o', 'LineWidth', 2);
axis equal;
grid on;
xlim([-10 10]);
ylim([-10 10]);
xlabel('X-axis');
ylabel('Y-axis');
title('Robot at Home Position (θ₁ = 0°, θ₂ = 0°)');

% Forward Kinematics Matrix at home
T_home = [cos(theta1_home+theta2_home) -sin(theta1_home+theta2_home) 0 x_home;
          sin(theta1_home+theta2_home)  cos(theta1_home+theta2_home) 0 y_home;
          0                            0                           1 0;
          0                            0                           0 1];

disp('Forward Kinematics Matrix at Home Position:');
disp(T_home);

% -------------------------------------------------------------------------
% Vary θ₁ and θ₂ from 0 to π/2 in 10° steps
theta_vals = deg2rad(0:10:90); % 0° to 90° in radians

for i = 1:length(theta_vals)
    for j = 1:length(theta_vals)
        
        theta1 = theta_vals(i);
        theta2 = theta_vals(j);
        
        % Calculate end-effector position
        x = L1*cos(theta1) + L2*cos(theta1 + theta2);
        y = L1*sin(theta1) + L2*sin(theta1 + theta2);
        
        % Plot robot
        figure;
        plot([0, L1*cos(theta1), x], [0, L1*sin(theta1), y], '-o', 'LineWidth', 2);
        axis equal;
        grid on;
        xlim([-10 10]);
        ylim([-10 10]);
        xlabel('X-axis');
        ylabel('Y-axis');
        title(['Robot at θ₁ = ', num2str(rad2deg(theta1)), '°, θ₂ = ', num2str(rad2deg(theta2)), '°']);
        
        % Display end-effector coordinates
        disp(['At θ₁ = ', num2str(rad2deg(theta1)), '°, θ₂ = ', num2str(rad2deg(theta2)), '°']);
        disp(['X = ', num2str(x), ', Y = ', num2str(y)]);

        % Forward Kinematics Matrix
        T = [cos(theta1+theta2) -sin(theta1+theta2) 0 x;
             sin(theta1+theta2)  cos(theta1+theta2) 0 y;
             0                  0                 1 0;
             0                  0                 0 1];
         
        disp('Forward Kinematics Matrix:');
        disp(T);
    end
end