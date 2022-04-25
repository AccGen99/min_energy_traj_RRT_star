function dydt = vx_ode(t, y, ft, f, C1, C2, alpha_x)
    f = interp1(ft, f, t);
    dydt = zeros(2,1);
    dydt(1) = y(2);
    dydt(2) = (f.^2 + C1).*y(1) + C2*alpha_x;
end