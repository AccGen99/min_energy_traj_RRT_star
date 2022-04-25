function val = maxAcc(k2, R, A, B, u_max)
    % Max acceleration vector for t = 0
    calc1 = B'*u_max;
    calc2 = A\calc1; %inv(A)*calc1;
    calc3 = R'\calc2;
    x_dd = k2*calc3;
    val = x_dd(1);
end