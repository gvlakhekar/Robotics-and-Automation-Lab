%--------------------------------------------------------------------------
% Forward Kinematics of 3R Robotic Manipulator with Animation   
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc; clear; close all;

% Link lengths
L1 = 1.0;  % Link 1
L2 = 0.7;  % Link 2
L3 = 0.5;  % Link 3

% Define range of joint angles (in degrees)
theta1_deg = linspace(0, 90, 10);
theta2_deg = linspace(0, 90, 10);
theta3_deg = linspace(0, 40, 10);

% Convert to radians
theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);
theta3 = deg2rad(theta3_deg);

% Initialize video frame counter
ct = 1;

% Create figure before loop
figure;

% Loop through joint angles
for i = 1:length(theta1)
    for j = 1:length(theta2)
        for k = 1:length(theta3)

            % Current joint angles
            THETA1 = theta1(i);
            THETA2 = theta2(j);
            THETA3 = theta3(k);

            % Joint positions
            x0 = 0; y0 = 0;
            x1 = L1 * cos(THETA1);
            y1 = L1 * sin(THETA1);
            x2 = x1 + L2 * cos(THETA1 + THETA2);
            y2 = y1 + L2 * sin(THETA1 + THETA2);
            x3 = x2 + L3 * cos(THETA1 + THETA2 + THETA3);
            y3 = y2 + L3 * sin(THETA1 + THETA2 + THETA3);

            % Plot the manipulator
            clf;
            plot([x0 x1 x2 x3], [y0 y1 y2 y3], '-o', 'LineWidth', 2, 'MarkerSize', 8);
            axis equal;
            grid on;
            xlabel('X-axis');
            ylabel('Y-axis');
            title(sprintf('3R Manipulator - θ1 = %.1f°, θ2 = %.1f°, θ3 = %.1f°', ...
                rad2deg(THETA1), rad2deg(THETA2), rad2deg(THETA3)));
            xlim([-2.5 2.5]);
            ylim([-2.5 2.5]);

            pause(0.01);
            M(ct) = getframe(gcf);
            ct = ct + 1;
        end
    end
end

% Save animation as video
videoFile = VideoWriter('forward_kinematics_3R.avi', 'Uncompressed AVI');
open(videoFile);
writeVideo(videoFile, M);
close(videoFile);
