function val = merv_ode_calcs(tf, tau_w)
    %tf = tf(1);
    h2 = sinh(tf/tau_w);
    h1 = 2*(1-cosh(tf/tau_w)) + h2;
    val = [0, 0];
    val(1) = h2;
    val(2) = h1;
end