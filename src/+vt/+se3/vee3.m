function w = vee3(S)
%VEE3 Convert 3x3 skew-symmetric matrix to 3x1 vector
    w = [S(3,2); S(1,3); S(2,1)];
end
