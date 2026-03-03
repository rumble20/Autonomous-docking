function F = rk4_step_casadi(f,h)
import casadi.*
x = SX.sym('x',6);
u = SX.sym('u',4);

k1 = f(x,u);
k2 = f(x + h/2*k1, u);
k3 = f(x + h/2*k2, u);
k4 = f(x + h*k3, u);

xnext = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
F = Function('F',{x,u},{xnext});
end
