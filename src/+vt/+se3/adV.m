function adV = adV(V)
%ADV Lie bracket operator for twist V = [omega; v]
    w_hat = vt.se3.hat3(V(1:3));
    v_hat = vt.se3.hat3(V(4:6));
    adV = [w_hat, zeros(3); v_hat, w_hat];
end
