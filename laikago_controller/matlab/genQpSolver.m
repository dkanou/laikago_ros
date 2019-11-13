import casadi.*

opti = Opti('conic');

x = opti.variable();
y = opti.variable();

opti.minimize(x^2 + y^2);
opti.subject_to(x+y>=1);

opti.solver('osqp');
tic
sol = opti.solve();
sol.value(x)
toc