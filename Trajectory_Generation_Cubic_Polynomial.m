%--------------------------------------------------------------------------
% Trajectory Generation for Robotic Manipulator (Cubic Polynomial Method)  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
%% Start
clear all;
clc;
close all;
% Trajectory inputs, both time and others
t_f = 5; % final time for the trajectory
t = 0:0.01:t_f ; % time span
% Trjectory boundary conditions (known)
x_0 = 0;
x_f =5;
x_0_dot=-1;
x_f_dot=1;
%% Coefficient matrix, A
A = [1 , 0 , 0 , 0 ;
    0 , 1 , 0 , 0 ;
    1 , t_f , t_f^2 , t_f ^3;
    0 , 0 , 2*t_f , 3*t_f^2;];
%% known inputs
b = [x_0;x_0_dot;x_f;x_f_dot;];
%% Trajectory coeeficients
tc =inv(A)*b;
%% Trajectory generation starts here
for i = 1:length(t)
    x(i)= [1,t(i),t(i)^2,t(i)^3]*tc;
    v(i)= [0,1,2*t(i),3*t(i)^2]*tc;
    a(i)= [0,0,2,6*t(i)]*tc;
end
%% Plotting the trajectory
plot(t,x,'b-',t,v,'r--',t,a,'g-.','linewidth',1)
legend ('$x(t)$','$\dot{x}(t)$','$\ddot{x}(t)$','fontsize',12,'interpreter','latex');
grid on
title('Trajectory Generation (Cubic Polynomial Method)','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('$\eta(t)$','fontsize',12,'interpreter','latex');
%% End