function val = energyCalc(w1,u,w2,x_dot,R,B)
    val = w1*(u'*u)-w2*x_dot'*R*B'*u ;
end