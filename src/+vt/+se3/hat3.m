function S = hat3(w)
%HAT3 Convert 3x1 vector to 3x3 skew-symmetric matrix
    S = [  0,   -w(3),  w(2);
          w(3),  0,   -w(1);
         -w(2), w(1),  0 ];
end
