%--------------------------------------------------------------------------
% Trajectory Generation for Robotic Manipulator (Cubic Polynomial Method) Via Point  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
%% Start
clear all;
clc; 
close all;
% Trajectory inputs, both time and others 
t_v = 2; % time for first segment
t_f = 5; % final time for the trajectory 
t =	0:0.01:t_f ; % time span
% Trjectory boundary conditions (known)
x_0	= 0;
x_v = 2 ;
x_f = 5; 
x_0_dot=0;
x_v_dot=1 ;
x_f_dot=0 ;
%% Coefficient matrix, A
A =[1,0,0,0,0,0,0,0;
    0,1,0,0,0,0,0,0;
    1,t_v,t_v^2,t_v^3,0,0,0,0;
    0,0,2*t_v,3*t_v^2,0,0,0,0;
    0,0,0,0,1,0,0,0;
    0,0,0,0,0,1,0,0;
    0,0,0,0,1,(t_f-t_v),(t_f-t_v)^2,(t_f-t_v)^3;
    0,0,0,0,0,1,2*(t_f-t_v),3*(t_f-t_v)^2;]; 
%% known inputs
 
b = [x_0; x_0_dot;x_v;x_v_dot;x_v;x_v_dot;x_f;x_f_dot];
 
%% Trjectory .coeeficients
 
tc = inv(A)*b;

%%trajectory generation starts here 

for i = 1:length(t)
    if t(i)<t_v
        x(i)=[1,t(i),t(i)^2,t(i)^3]*tc(1:4);
        v(i)=[0,1,2*t(i),3*t(i)^2]*tc(1:4);
        a(i)=[0,0,2,6*t(i)]*tc(1:4);
    else
        x(i)=[1,(t(i)-t_v),(t(i)-t_v)^2,(t(i)-t_v)^3]*tc(5:8);
        v(i)=[0,1,2*(t(i)-t_v),3*(t(i)-t_v)^2]*tc(5:8);
        a(i)=[0,0,2,6*(t(i)-t_v)]*tc(5:8);
    end
end
%% Plotting the trajectory
plot(t,x,'b-',t,v,'r--',t,a,'g-.','linewidth',1)
legend ('$x(t)$','$\dot{x}(t)$','$\ddot{x}(t)$','fontsize',12,'interpreter','latex');
grid on
title('Trajectory Generation (Cubic Polynomial Method)','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('$\eta(t)$','fontsize',12,'interpreter','latex');
%% End
