% function for finding the inverse jacobian for the given 2 links manipulator
function Jinv = IJ(r);
a1 = 50 ; a2 = 40 ;
J = [-a1*r.S1-a2*r.S12 -a2*r.S12
a1*r.C1+a2*r.C12 a2*r.C12 ];
D = a1*a2*r.S2 ;
Jinv = [ J(2,2)/D -J(1,2)/D ;
-J(2,1)/D J(1,1)/D ];
end