function V = vee6(se3mat)
%VEE6 Convert 4x4 se(3) matrix to 6x1 twist
% V = [omega; v]
    V = [se3mat(3,2); se3mat(1,3); se3mat(2,1); se3mat(1:3,4)];
end
