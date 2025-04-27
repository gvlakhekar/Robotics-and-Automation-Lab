%--------------------------------------------------------------------------
% Jacobian and Singularity 3R Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------

clc;
clear;
close all;

syms theta1 theta2 theta3 l1 l2 l3 real

% Forward kinematics
x = l1*cos(theta1) + l2*cos(theta1 + theta2) + l3*cos(theta1 + theta2 + theta3);
y = l1*sin(theta1) + l2*sin(theta1 + theta2) + l3*sin(theta1 + theta2 + theta3);

% Jacobian
J = [diff(x, theta1), diff(x, theta2), diff(x, theta3);
     diff(y, theta1), diff(y, theta2), diff(y, theta3)];
J = simplify(J);

% Display Jacobian
disp('Jacobian matrix J:')
disp(J)

% Determinant
% Note: For 3R manipulator (2x3 Jacobian), no simple determinant!
% Use rank instead
RankJ = rank(J);
disp('Rank of Jacobian:')
disp(RankJ)

% Define theta ranges
theta1_range = linspace(-pi, pi, 100);
theta2_range = linspace(-pi, pi, 100);
theta3_fixed = 0;  % Fixed theta3 value for visualization (NOT Theta3_fixed!)

% Create meshgrid
[Theta1, Theta2] = meshgrid(theta1_range, theta2_range);

% Compute pseudo-determinant like quantity
l1_val = 1; % assume l1 = 1
l2_val = 1; % assume l2 = 1
l3_val = 1; % assume l3 = 1

Det_like = l1_val * l2_val .* sin(Theta2) + ...
           l1_val * l3_val .* sin(Theta2 + theta3_fixed) + ...
           l2_val * l3_val .* sin(theta3_fixed);

% Plot surface
figure;
surf(Theta1, Theta2, Det_like);
xlabel('\theta_1 (rad)');
ylabel('\theta_2 (rad)');
zlabel('Pseudo-determinant like quantity');
title('Jacobian Pseudo-determinant (3R Manipulator)');
colorbar;
shading interp;
grid on;
hold on;

% Highlight singularities where pseudo-determinant ~ 0
contour3(Theta1, Theta2, Det_like, [0 0], 'LineWidth', 3, 'LineColor', 'r');

legend('Pseudo-det(J)','Singularities')
view(45, 30);
hold off;
