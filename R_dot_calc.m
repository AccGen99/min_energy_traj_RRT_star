function val = R_dot_calc(phi)
    val = [-sin(phi), -cos(phi), 0; cos(phi), -sin(phi), 0; 0, 0, 0];
end