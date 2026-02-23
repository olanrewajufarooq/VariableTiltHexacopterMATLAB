function se3mat = hat6(V)
%HAT6 Convert 6x1 twist to 4x4 se(3) matrix
% V = [omega; v]
    w = V(1:3);
    v = V(4:6);
    se3mat = [vt.se3.hat3(w), v; 0 0 0 0];
end
