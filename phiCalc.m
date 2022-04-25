function val = phiCalc(phi_dot, ang_acc, t_step)
    range = 1:length(phi_dot);
    val = 0*range;
    for i=range
        if i == 1
            val(i) = 0;
        else
            val(i) = val(i-1) + phi_dot(i)*t_step+0.5*ang_acc(i)*t_step^2;
        end
    end
end