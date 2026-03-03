function f = model_naval3dof_casadi(par)
import casadi.*

x = SX.sym('x',6);      % [N E psi u v r]'
N   = x(1); 
E   = x(2); 
psi = x(3);
u_b = x(4);
v_b = x(5);
r   = x(6);

u = SX.sym('u',4);      % thruster extended forces

% --- thruster mapping (extended) [2] ---
Te = [ 1, 0, 1, 0;
       0, 1, 0, 1;
      -par.ly(1), par.lx(1), -par.ly(2), par.lx(2) ];
tau3 = Te*u;            % [X Y N]'

Xe = tau3(1);
Ye = tau3(2);
Ne = tau3(3);

% --- hydrodynamic terms copied from navalvessel structure (roll removed) ---
eps_abs = 1e-3;
au = sqrt(u_b^2 + eps_abs^2);
av = sqrt(v_b^2 + eps_abs^2);
ar = sqrt(r^2   + eps_abs^2);

% Xh = Xuau*u*|u| + Xvr*v*r
Xh = par.Xuau*u_b*au + par.Xvr*v_b*r;

% Yh = Yauv*|u|*v + Yur*u*r + Yvav*v*|v| + Yvar*v*|r| + Yrav*r*|v|
Yh = par.Yauv*au*v_b + par.Yur*u_b*r + par.Yvav*v_b*av + par.Yvar*v_b*ar + par.Yrav*r*av;

% Nh = Nauv*|u|*v + Naur*|u|*r + Nrar*r*|r| + Nrav*r*|v|
Nh = par.Nauv*au*v_b + par.Naur*au*r + par.Nrar*r*ar + par.Nrav*r*av;

% centripetal terms (from navalvessel, with p=0)
Xc =  par.m*(r*v_b + par.xG*r^2);
Yc = -par.m*u_b*r;
Nc = -par.m*par.xG*u_b*r;

F = [Xh+Xc+Xe;
     Yh+Yc+Ye;
     Nh+Nc+Ne];

% --- 3x3 mass matrix (subset, consistent with navalvessel coefficients) ---
M = [ (par.m - par.Xudot), 0, 0;
      0, (par.m - par.Yvdot), (par.m*par.xG - par.Yrdot);
      0, (par.m*par.xG - par.Nvdot), (par.Izz - par.Nrdot) ];

nu_dot = M\F;   % [u_dot v_dot r_dot]'

% kinematics
Ndot  = u_b*cos(psi) - v_b*sin(psi);
Edot  = u_b*sin(psi) + v_b*cos(psi);
psidot = r;

xdot = [Ndot; Edot; psidot; nu_dot];

f = Function('f',{x,u},{xdot});
end
