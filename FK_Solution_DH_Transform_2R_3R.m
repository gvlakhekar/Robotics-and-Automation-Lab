%--------------------------------------------------------------------------
% Solution of Forward Kinematics of 2R and 3R Manipulator-Analytically
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
a1 = input('Enter the value of angle 1(in degrees) :'); 
a2 = input('Enter the value of angle 2(in degrees) :'); 
a3 = input('Enter the value of angle 3(in degrees) :'); 
L1 = 3; % Link length 1 
L2 = 3; % Link length 2
L3 = 0; % Link length 3 
%% Setting up environment 
z = [-10 10]; 
plot(z,10); 
grid on 
hold on 
O = [0;0;0;1];

% Instantanenious angles 
m = linspace(pi/2,pi/2+a1*pi/180,100); % In radians 
n = linspace(-pi/2,a2*pi/180,100); 
k = linspace(-pi/2,a3*pi/180,100); 
%% Variables to track trajectory of end point 
i=1; 
Cx = zeros(1,1); 
Cy = zeros(1,1);
%% Animation 
for a=1:100 
    [A1,B1,C1] = Transform(m(a),n(a),k(a),L1,L2,L3); % Calling Transform function which is based on DH parameters 
    x = [O(1) A1(1) B1(1)]; % O(1),A1(1),B1(1): X-coordintea of start of link one,two and three 
    y = [O(2) A1(2) B1(2)]; % O(2),A1(2),B1(2): Y-coordintea of start of link one,two and three 
    Cx(i) = B1(1); % Stores x co-ordinates of end point i.e. point C for RRR and B for RR manipulator 
    Cy(i) = B1(2); % Stores y co-ordinates of end point i.e. point C for RRR and B for RR manipulator 
    i = i+1;
    Plot = plot(x,y,'r','Linewidth',1); 
    title('Forward kinematics'); 
    plot(Cx,Cy,'--g','LineWidth',1.5); 
    pause(0.075); % Define speed of Animation 
    delete(Plot); 
    xlabel('X (m)') 
    ylabel('Y (m)') 
end
%% Final Position 
plot(x,y,'r','Linewidth',3); 
xlabel('X (m)') 
ylabel('Y (m)') 
title('RR manipulator following a trajectory (Forward Kinematics)') 
% For 2R manipulator input the third angle as zero