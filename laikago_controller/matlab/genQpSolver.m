import casadi.*

opti = Opti('conic');

x = opti.variable(2, 1);

opti.minimize(x(1)^2 + x(2)^2);
opti.subject_to(x(1)+x(2)>=1);

g = opti.g
A = jacobian(opti.g, x)
A * x
DM(SX(A))



%%
opti.solver('qpoases');
sol = opti.solve();
sol.value(x)
