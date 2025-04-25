%--------------------------------------------------------------------------
% Trajectory Generation for Robotic Manipulator (Parabolic Blend Method)   
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
%start
clear all
close all
clc
vb=1/4;
ab=1/4;
tb=vb/ab;
tf=5;
x0=0 ;xf=5;
t=0:0.01:tf;
%%trajectory generation
for i=1:length(t);
if t(i)<tb
x(i)=1/2*ab*t(i)^2;
v(i)=ab*t(i);a(i)=ab;
elseif t(i)<(tf-tb)
x(i)=1/2*ab*tb^2+vb*(t(i)-tb);
v(i)=vb; a(i)=0;
else
x(i)=1/2*ab*tb^2+vb*(tf-tb*2)+vb*(t(i)-(tf-tb))-1/2*ab*(t(i)-(tf-tb))^2;
v(i)=vb-ab*(t(i)-(tf-tb));a(i)=-ab;
end
end
%% Plotting the trajectory
plot (t,x0+(xf-x0)*x,'b-',t, (xf-x0)*v, 'r--',t, (xf-x0)*a, 'g-.','linewidth',1)
legend ('$x(t)$','$\dot{x}(t)$','$\ddot{x}(t)$','fontsize',12,'interpreter','latex');
grid on
title('Trajectory Generation (Parabolic Blend Method)','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('$\eta(t)$','fontsize',12,'interpreter','latex');
%% End