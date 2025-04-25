%--------------------------------------------------------------------------
% Forward Kinematics of 2R Robotic Manipulator with Animation   
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc; clear; close all;

% Link lengths
L1 = 1.0;  % Length of link 1
L2 = 0.5;  % Length of link 2

% Define range of joint angles (in degrees)
theta1_deg = linspace(0, 120, 10);
theta2_deg = linspace(0, 90, 50);

% Convert to radians
theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);

% Initialize video frame counter
ct = 1;

% Create figure before loop
figure;

% Loop through joint angles
for i = 1:length(theta1)
    for j = 1:length(theta2)
        % Use scalar angles from loop
        THETA1 = theta1(i);
        THETA2 = theta2(j);

        % Joint positions
        x0 = 0; y0 = 0;
        x1 = L1 * cos(THETA1);
        y1 = L1 * sin(THETA1);
        x2 = x1 + L2 * cos(THETA1 + THETA2);
        y2 = y1 + L2 * sin(THETA1 + THETA2);

        % Plot the manipulator
        clf;
        plot([x0 x1 x2], [y0 y1 y2], '-o', 'LineWidth', 2, 'MarkerSize', 8);
        axis equal;
        grid on;
        xlabel('X-axis');
        ylabel('Y-axis');
        title(sprintf('2R Manipulator - θ1 = %.1f°, θ2 = %.1f°', rad2deg(THETA1), rad2deg(THETA2)));
        xlim([-2 2]);
        ylim([-2 2]);

        pause(0.01);
        M(ct) = getframe(gcf);
        ct = ct + 1;
    end
end

% Save as video
videoFile = VideoWriter('forward_kinematics.avi', 'Uncompressed AVI');
open(videoFile);
writeVideo(videoFile, M);
close(videoFile);
