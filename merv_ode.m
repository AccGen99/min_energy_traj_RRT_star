function dydt = merv_ode(t, y, C3, C4, alpha_phi)
    dydt = zeros(2,1);
    dydt(1) = y(2);
    dydt(2) = C3*y(1) + C4*alpha_phi;%(f.^2 + C1).*y(1) + C2*alpha_x;
end