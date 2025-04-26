%% Denavit–Hartenberg parameters (DH)
function [A,B,C] = Transform(m,n,p,L1,L2,L3)
P = [0 ; 0 ; 0 ; 1]; 
T1 = [cos(m) -sin(m) 0 0; sin(m) cos(m) 0 0; 0 0 1 0; 0 0 0 1]; 
T2 = [cos(n) -sin(n) 0 L1; sin(n) cos(n) 0 0; 0 0 1 0; 0 0 0 1]; 
T3 = [cos(p) -sin(p) 0 L2; sin(p) cos(p) 0 0; 0 0 1 0; 0 0 0 1]; 
T4 = [ 1 0 0 L3; 0 1 0 0; 0 0 1 0; 0 0 0 1]; 
A = T1*T2*P; 
B = T1*T2*T3*P; 
C = T1*T2*T3*T4*P; 
end