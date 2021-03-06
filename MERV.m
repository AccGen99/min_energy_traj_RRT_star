function val = MERV(C3, C4, alpha, t, tf)%phi_f, t, tf, tau_w, h2, h1, alpha)
%    val = -(sinh((t-tf)/tau_w) - sinh(t/tau_w) + h2)/(tau_w*h1*phi_f);
    val = -(C4*alpha*exp(-C3^(1/2)*t)*(exp(C3^(1/2)*t) - ...
        exp(2*C3^(1/2)*t) - exp(C3^(1/2)*tf) + ...
        exp(C3^(1/2)*t)*exp(C3^(1/2)*tf)))/(C3*(exp(C3^(1/2)*tf) + 1));
end