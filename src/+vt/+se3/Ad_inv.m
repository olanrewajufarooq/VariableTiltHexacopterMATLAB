function AdInv = Ad_inv(T)
%AD_INV Inverse adjoint of SE(3) element T
    R = T(1:3,1:3);
    p = T(1:3,4);
    AdInv = [R', zeros(3); -R' * vt.se3.hat3(p), R'];
end
