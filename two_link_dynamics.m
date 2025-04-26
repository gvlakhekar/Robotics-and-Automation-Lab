% ---- Now define two_link_dynamics inside the same script ----
function ydot=two_link_dynamics(t,y); 
m1 = 3; m2 = 1; a1 = 2; a2 = 1; g = 9.81; iner21 = m2*a1*a2; 
tau1 = 0; tau2 = 0; 
th1=y(1); th2 =y(2); th1d=y(3); th2d=y(4); 

%Inertia matrix 
sth2 = sin(th2); cth2 = cos(th2); 
i22 = m2*a2*a2/3; i21 = i22 + iner21*cth2/2; i12 = i21; 
i11 = i22 + m1*a1*a1/3 + m2*a1*a1 + iner21*cth2; 
im = [i11, i12; i21, i22]; 

%h-vector 
h1 = -(m2*a1*a2*th1d + iner21/2*th2d)*th2d*sth2; 
h2 = iner21/2*sth2*th1d*th1d; 
hv=[h1;h2]; 

%gamma-vector 
cth1 = cos(th1); 
cth12 = cos(th1 + th2); 
gam1 = m1*g*a1/2*cth1 + m2*g*(a1*cth1 + a2/2*cth12); 
gam2 = m1*g*a2/2*cth12; 
gv= [gam1;gam2]; 

% RHS 
tau=[tau1;tau2]; 
phi=tau-hv-gv; 
thdd=im\phi; 
ydot=[y(3);y(4);thdd(1);thdd(2)];
