%--------------------------------------------------------------------------
% Jacobian and Singurity 2R Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
syms theta1 theta2 l1 l2 real

% Forward kinematics
x = l1*cos(theta1) + l2*cos(theta1 + theta2);
y = l1*sin(theta1) + l2*sin(theta1 + theta2);

% Jacobian
J = [diff(x, theta1), diff(x, theta2);
     diff(y, theta1), diff(y, theta2)];
J = simplify(J);

% Determinant
detJ = simplify(det(J));
disp('Determinant of Jacobian:')
disp(detJ)

% Singularity condition
singularities = solve(detJ == 0, theta2);
disp('Singular configurations (theta2 values):')
disp(singularities)
% Define theta1 and theta2 range
theta1_range = linspace(-pi, pi, 100);
theta2_range = linspace(-pi, pi, 100);

% Create meshgrid
[Theta1, Theta2] = meshgrid(theta1_range, theta2_range);

% Compute determinant of Jacobian for each (theta1, theta2)
l1 = 1; % assume l1 = 1
l2 = 1; % assume l2 = 1

DetJ = l1 * l2 .* sin(Theta2);

% Plot determinant surface
figure;
surf(Theta1, Theta2, DetJ);
xlabel('\theta_1 (rad)');
ylabel('\theta_2 (rad)');
zlabel('det(J)');
title('Determinant of Jacobian for 2R Manipulator');
colorbar;
shading interp;
grid on;
hold on;

% Highlight singularities where det(J) = 0
contour3(Theta1, Theta2, DetJ, [0 0], 'LineWidth', 3, 'LineColor', 'r');

legend('det(J)','Singularities (det(J) = 0)')
view(45, 30);
