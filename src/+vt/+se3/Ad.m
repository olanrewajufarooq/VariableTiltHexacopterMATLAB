function AdT = Ad(T)
%AD Adjoint of SE(3) element T
    R = T(1:3,1:3);
    p = T(1:3,4);
    p_hat = vt.se3.hat3(p);
    AdT = [R, zeros(3); p_hat*R, R];
end
