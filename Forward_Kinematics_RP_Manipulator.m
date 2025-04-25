%--------------------------------------------------------------------------
% Forward Kinematics of RP Robotic Manipulator with Animation   
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
% 
clc; clear; close all;

% Link length (L1 is fixed, d2 is variable)
L1 = 1.0;         % Fixed length of first link
d2_vals = linspace(0.2, 0.8, 20);  % Range for prismatic joint

% Revolute joint angle in degrees
theta1_deg_vals = linspace(0, 80, 20);  % sweeping from 0 to 90 degrees

% Convert to radians
theta1_vals = deg2rad(theta1_deg_vals);

% Initialize video
ct = 1;
figure;

% Loop through joint angles and prismatic extensions
for i = 1:length(theta1_vals)
    for j = 1:length(d2_vals)
        theta1 = theta1_vals(i);
        d2 = d2_vals(j);

        % Joint positions
        x0 = 0; y0 = 0;
        x1 = L1 * cos(theta1);
        y1 = L1 * sin(theta1);
        x2 = x1 + d2 * cos(theta1);
        y2 = y1 + d2 * sin(theta1);

        % Plot the manipulator
        clf;
        plot([x0 x1 x2], [y0 y1 y2], '-o', 'LineWidth', 2, 'MarkerSize', 8);
        axis equal;
        grid on;
        xlabel('X-axis');
        ylabel('Y-axis');
        title(sprintf('RP Manipulator - θ1 = %.1f°, d2 = %.2f', rad2deg(theta1), d2));
        xlim([-2 2]);
        ylim([-2 2]);

        pause(0.01);
        M(ct) = getframe(gcf);
        ct = ct + 1;
    end
end

% Save as video
videoFile = VideoWriter('forward_kinematics_RP.avi', 'Uncompressed AVI');
open(videoFile);
writeVideo(videoFile, M);
close(videoFile);
