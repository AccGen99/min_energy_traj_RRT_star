function val = calcContVec(w2, B, R, x_dot, D, A, lamda, w1)
    val = (w2*B*R'*x_dot - D'*R'*inv(A')*lamda)/(2*w1);
end