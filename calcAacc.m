function val = calcAacc(phi_f, t, tf, tau_w, h1)
    val = phi_f*(cosh((t-tf)/tau_w) - cosh(t/tau_w))/((tau_w^2)*h1);
end