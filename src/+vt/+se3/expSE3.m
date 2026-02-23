function T = expSE3(se3mat)
%EXPSE3 Exponential map from se(3) to SE(3)
    omgmat = se3mat(1:3,1:3);
    v = se3mat(1:3,4);

    theta = sqrt(omgmat(1,2)^2 + omgmat(1,3)^2 + omgmat(2,3)^2);

    if theta == 0
        R = eye(3);
        p = v;
    else
        omgmat_norm = omgmat / theta;
        R = eye(3) + sin(theta) * omgmat_norm + (1 - cos(theta)) * (omgmat_norm * omgmat_norm);
        p = (eye(3)*theta + (1 - cos(theta))*omgmat_norm + (theta - sin(theta))*(omgmat_norm*omgmat_norm)) * v / theta;
    end

    T = [R, p; 0 0 0 1];
end
