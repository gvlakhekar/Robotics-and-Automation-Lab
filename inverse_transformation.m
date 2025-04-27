function T_BA = inverse_transformation(T_AB)
    % Inverse of homogeneous transformation
    R = T_AB(1:3,1:3);
    p = T_AB(1:3,4);
    
    R_inv = R'; % Transpose of rotation matrix
    p_inv = -R_inv * p;
    
    T_BA = [R_inv, p_inv;
            0 0 0 1];
end