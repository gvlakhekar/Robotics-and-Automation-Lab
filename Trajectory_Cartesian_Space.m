%--------------------------------------------------------------------------
% Trajectory Generation for Robotic Manipulator (Cartesian Space Trajectory)   
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
%start
r.a1 = 50 ;
r.a2 = 40 ;
p0=[0 10] ;
q0= [pi/2 -pi];
pn=[50 10];
t=4;
dt=0.05;
n=t/dt;
i=1;
pp(i,:)=p0;
qq(i,:)=q0
for k=1:n
i=i+1;
pp(i,:)= p0 + (k/n)*(pn-p0);
q=qq(i-1,:);
r.C1=cos(q(1)); r.S1=sin(q(1));
r.C2=cos(q(2)); r.S2=sin(q(2));
r.C12=cos(q(1)+q(2)); r.S12=sin(q(1)+q(2));
qq(i,:)= q + transpose(IJ(r)* transpose( pp(i,:)-pp(i-1,:)))
end
x=[0 0 0]; y=[0 0 0];
for i=1:length(qq);
xo=x; yo=y;
x(1)=0; y(1)=0; % origin
x(2)= r.a1*cos(qq(i,1)); y(2)=r.a1*sin(qq(i,1));
x(3)= x(2)+r.a2*cos(qq(i,1)+qq(i,2));
y(3)= y(2)+r.a2*sin(qq(i,1)+qq(i,2));
plot(xo,yo,'k-'); hold on;
plot(x,y,'k-'); hold on;
plot(x+0.001,y,'r.'); hold on;
axis equal;
pause(0.05);
end
figure;
P1=plot(0:dt:t,qq(:,1)*57.2958,'-.');hold on;
P2=plot(0:dt:t,qq(:,2)*57.2958,':');hold on;
title('Joints Profile')
legend('Joint1','Joint2')
P1(1).LineWidth = 1.5;
P2(1).LineWidth = 1.5;