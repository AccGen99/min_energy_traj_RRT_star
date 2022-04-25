function val = MERV_old(phi_f, t, t_f, tau_w, h2, h1, alpha)
    val = phi_f*(sinh((t-t_f)/tau_w) - sinh(t/tau_w) + h2) / (tau_w*h1);
end