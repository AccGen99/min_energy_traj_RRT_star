syms w(t) C3 C4 alpha tf

D2w = diff(w,t,2);

ode = D2w == C3*w + C4*alpha;

cond1 = w(0) == 0;
cond2 = w(tf) == 0;

conds = [cond1 cond2];

wSol(t) = dsolve(ode, conds);
wSol = simplify(wSol)
