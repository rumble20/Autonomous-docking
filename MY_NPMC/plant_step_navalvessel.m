function xplant_next = plant_step_navalvessel(xplant, u, lx, ly, h, freeze_roll)
% xplant = [N E u v p r phi psi]'
% u      = [u1x u1y u2x u2y]'
% freeze_roll: if true, enforce p=phi=0 each step (planar plant)

N   = xplant(1);
E   = xplant(2);
u_b = xplant(3);
v_b = xplant(4);
p   = xplant(5);
r   = xplant(6);
phi = xplant(7);
psi = xplant(8);

if freeze_roll
    p = 0; phi = 0;
end

% thrusters -> tau3 = [X Y N]'
tau3 = thruster_map_2az(u, lx, ly);
Xe = tau3(1); Ye = tau3(2); Ne = tau3(3);
Ke = 0;

tau4 = [Xe; Ye; Ke; Ne];

% navalvessel dynamics state
x_dyn = [u_b; v_b; p; r; phi; psi];

% 1-step Euler (upgrade to RK4 later if needed)
[xdot_dyn,~] = navalvessel(x_dyn, tau4);
x_dyn_next = x_dyn + h*xdot_dyn;

u_b_next = x_dyn_next(1);
v_b_next = x_dyn_next(2);
p_next   = x_dyn_next(3);
r_next   = x_dyn_next(4);
phi_next = x_dyn_next(5);
psi_next = x_dyn_next(6);

if freeze_roll
    p_next = 0; phi_next = 0;
end

% planar kinematics (NED) using yaw only
Ndot = u_b*cos(psi) - v_b*sin(psi);
Edot = u_b*sin(psi) + v_b*cos(psi);

N_next = N + h*Ndot;
E_next = E + h*Edot;

xplant_next = [N_next; E_next; u_b_next; v_b_next; p_next; r_next; phi_next; psi_next];
end
