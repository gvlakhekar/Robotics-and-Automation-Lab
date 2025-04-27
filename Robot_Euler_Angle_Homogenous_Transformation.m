%--------------------------------------------------------------------------
% Homogenous Transformation -Analytically
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
clc;
clear;
close all;

%% Part 1a - Main Program

% Example 1
alpha = 10; beta = 20; gamma = 30;
A_PB = [1; 2; 3];
T_AB1 = homogeneous_transformation(alpha, beta, gamma, A_PB)

% Example 2
alpha = 0; beta = 20; gamma = 0;
A_PB = [3; 0; 1];
T_AB2 = homogeneous_transformation(alpha, beta, gamma, A_PB)

%% Part 1b - Interpretation Examples

% Interpretation 1: no additional info needed

% Interpretation 2: Given B{P} and want to calculate A{P}
B_P = [1; 0; 1];
A_P = T_AB2(1:3,1:3) * B_P + T_AB2(1:3,4)

% Interpretation 3: Given A{P1} and want to calculate A{P2}
A_P1 = [1; 0; 1];
B_P1 = T_AB2(1:3,1:3)' * (A_P1 - T_AB2(1:3,4)); % Inverse rotation + translation
A_P2 = T_AB2(1:3,1:3) * B_P1 + T_AB2(1:3,4)

%% Part 1c - Inverse Homogeneous Transformation

% Symbolic Inverse Calculation
T_BA_symbolic = inverse_transformation(T_AB2)

% MATLAB inv() function
T_BA_inv = inv(T_AB2)

% Validation: check if T_AB * T_BA = Identity
Validation1 = T_AB2 * T_BA_symbolic
Validation2 = T_BA_symbolic * T_AB2

% Should be approximately 4x4 identity
disp('Validation 1 Error:')
disp(norm(Validation1 - eye(4)))

disp('Validation 2 Error:')
disp(norm(Validation2 - eye(4)))

%% Local Functions (must be at end of script)

function T_AB = homogeneous_transformation(alpha, beta, gamma, A_PB)
    % Convert degrees to radians
    alpha = deg2rad(alpha);
    beta = deg2rad(beta);
    gamma = deg2rad(gamma);

    % Rotation Matrices
    Rz = [cos(alpha) -sin(alpha) 0;
          sin(alpha) cos(alpha) 0;
          0          0          1];
      
    Ry = [cos(beta)  0 sin(beta);
          0          1 0;
         -sin(beta) 0 cos(beta)];
     
    Rx = [1 0           0;
          0 cos(gamma) -sin(gamma);
          0 sin(gamma) cos(gamma)];

    % Total Rotation Matrix
    R = Rz * Ry * Rx;

    % Homogeneous Transformation Matrix
    T_AB = [R, A_PB;
            0 0 0 1];
end
