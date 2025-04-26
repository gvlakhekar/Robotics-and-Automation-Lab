%--------------------------------------------------------------------------
% Inverse Kinematics of 2R Robotic Planar Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
%% Desired positions and Link Lengths 
x=6; y=2; L1=4; L2=3; 
%% Case when S2 is negative 
C2=(x^2+y^2-L1^2-L2^2)/(2*L1*L2); 
S2=-sqrt(1-C2^2); theta2_S2_neg=atan2(S2,C2); 
k1=L1+L2*cos(theta2_S2_neg); k2=L2*sin(theta2_S2_neg); 
r=sqrt(k1^2+k2^2); 
theta1_S2_neg=atan2(y/r,x/r)-atan2(k2/r,k1/r); 
solution_1=[theta1_S2_neg*(180/pi),theta2_S2_neg*(180/pi)]; % in degree
%% Case when S2 is positive 
C2=(x^2+y^2-L1^2-L2^2)/(2*L1*L2); 
S2=sqrt(1-C2^2); 
theta2_S2_pos=atan2(S2,C2); 
k1=L1+L2*cos(theta2_S2_pos); 
k2=L2*sin(theta2_S2_pos); 
r=sqrt(k1^2+k2^2); 
theta1_S2_pos=atan2(y/r,x/r)-atan2(k2/r,k1/r); 
solution_2=[theta1_S2_pos*(180/pi),theta2_S2_pos*(180/pi)]; % in degree
syms tabel sol_1 sol_2 theta_1 theta_2 
solution=vpa([0, theta_1, theta_2;sol_1, solution_1;sol_2, solution_2]);
%Inverse Kinematics of 2R Manipulator-Using solve
syms th1 th2 
S = solve(4*cos(th1)+3*cos(th1+th2)==6,4*sin(th1)+3*sin(th1+th2)==2) 
double(S.th1)*(180/pi) % Convert to degree 
double(S.th2)*(180/pi) % Convert to degree 

%Case-II
%First we find the solution for the motor angles in terms of the variable y. 
% Then we evaluate the solution for numerical values of y, and plot the results. 
% The script file is shown below. Note that because there are three symbolic variables in the problem, 
% we must tell the solve function that we want for 𝜃1 and 𝜃2.
syms y th1 th2 
S = solve(4*cos(th1)+3*cos(th1+th2)==6, 4*sin(th1)+3*sin(th1+th2)==y, th1,th2) 
yr = 1:0.1:3.6;
% subs(s,old,new) returns a copy of s, replacing all occurrence of old with new and then evaluate s 
th1r = subs(S.th1,y,yr); 
th2r = subs(S.th2,y,yr); 
th1d = (180/pi)*double(th1r); % Convert to degree 
th2d = (180/pi)*double(th2r); 
subplot(2,1,1) 
plot(yr,th1d,2, -3.2981, 'x',2,40.168,'o','LineWidth', 1.5)
xlabel('y (feet)','fontsize',12,'interpreter','latex'),
ylabel('Theta1 (degrees)','fontsize',12,'interpreter','latex') 
subplot(2,1,2) 
plot(yr,th2d,2, -51.3178,'o',2,51.3178,'x','LineWidth', 1.5)
xlabel('y (feet)','fontsize',12,'interpreter','latex'),
ylabel('Theta2 (degrees)','fontsize',12,'interpreter','latex')