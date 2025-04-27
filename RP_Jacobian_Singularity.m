%--------------------------------------------------------------------------
% Jacobian and Singularity RP Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------

clc;
clear;
close all;

syms theta d l1 real

% Forward kinematics for RP manipulator
x = l1*cos(theta) + d*cos(theta);
y = l1*sin(theta) + d*sin(theta);

% Jacobian matrix
J = [diff(x, theta), diff(x, d);
     diff(y, theta), diff(y, d)];
J = simplify(J);

% Display Jacobian
disp('Jacobian matrix J:')
disp(J)

% Determinant of Jacobian
detJ = simplify(det(J));
disp('Determinant of Jacobian:')
disp(detJ)

% Singularity condition: det(J) = 0
singularities = solve(detJ == 0, d);
disp('Singular configurations (d values):')
disp(singularities)

% Define theta and d range
theta_range = linspace(-pi, pi, 100);
d_range = linspace(0, 2, 100); % d >= 0 (prismatic displacement)

% Create meshgrid
[Theta, D] = meshgrid(theta_range, d_range);

% Assume l1 = 1 for plotting
l1_val = 1;

% Compute determinant across grid
DetJ = l1_val * cos(Theta) + D .* cos(Theta);

% Plot surface
figure;
surf(Theta, D, DetJ);
xlabel('\theta (rad)');
ylabel('d (m)');
zlabel('det(J)');
title('Determinant of Jacobian for RP Manipulator');
colorbar;
shading interp;
grid on;
hold on;

% Highlight singularities where det(J) = 0
contour3(Theta, D, DetJ, [0 0], 'LineWidth', 3, 'LineColor', 'r');

legend('det(J)','Singularities (det(J) = 0)')
view(45, 30);
hold off;
