function val = cVec(B, R, lamda_dot, alpha, omega, R_dot, lamda, C, A, w2)
    val = 0*lamda_dot;
    val = (B'*inv(R)*(lamda_dot + alpha + omega*R_dot*R'*lamda - C*inv(A)*lamda))/w2;
end

%u = 0*lamda_dot;
%for m = range1
%    l_dot = lamda_dot(m,:);
%    omega = phi_dot(m);
%    p = phi(m);
%    R_d = R_dot_calc(p);
%    l = lamda(m,:);
%    u(m,:) = cVec(B, R, l_dot', alpha', omega, R_d, l', C, A, w2);
%end
%u

