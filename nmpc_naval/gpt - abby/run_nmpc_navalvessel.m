clear; clc;
import casadi.*

%% USER SETTINGS
h  = 0.2;      % NMPC sample time
Np = 25;       % horizon length

% two azimuth thrusters positions (BODY, meters)
lx = [-20; -20];
ly = [  2;  -2];

% thruster constraints (force magnitude per thruster)
Fmax  = 1.5e5;     % N
dFmax = 2.0e4;     % N per sample step

% keep in low-speed region (DP-like)
vmax = 2.0;        % m/s (DP regime guideline)

freeze_roll = true;  % start TRUE (planar), later set FALSE

%% Build NMPC model params FROM navalvessel constants 
par = struct();
par.lx = lx; par.ly = ly;

const_rho = 1014.0; 
disp_m3   = 357.0;
par.m     = disp_m3*1014.0;

par.Izz   = 47.934e6;
par.xG    = -3.38;

par.Xudot = -17400.0;
par.Xuau  = -1.96e3;
par.Xvr   = 0.33*par.m;

par.Yvdot = -393000;
par.Yrdot = -1400000;
par.Yauv  = -11800;
par.Yur   = 131000;
par.Yvav  = -3700;
par.Yvar  = -794000;
par.Yrav  = -182000;

par.Nvdot = 538000;
par.Nrdot = -38.7e6;
par.Nauv  = -92000;
par.Naur  = -4710000;
par.Nrar  = -202000000;
par.Nrav  = -15600000;

%% CasADi model + integrator
f = model_naval3dof_casadi(par);
F = rk4_step_casadi(f,h);

ssa = @(a) atan2(sin(a),cos(a));

%% NMPC (Opti)
nx=6; nu=4;
opti = Opti();

X = opti.variable(nx,Np+1);
Ubar = opti.variable(nu,Np);
U = Ubar*Fmax;

x0   = opti.parameter(nx,1);
xref = opti.parameter(3,1);      % [N_ref E_ref psi_ref]'
Uprev = opti.parameter(nu,1);

opti.subject_to(X(:,1)==x0);

for k=1:Np
    opti.subject_to(X(:,k+1)==F(X(:,k),U(:,k)));
end

% thruster magnitude constraints
for k=1:Np
    u1x=U(1,k); u1y=U(2,k);
    u2x=U(3,k); u2y=U(4,k);
    opti.subject_to(Ubar(1,k)^2 + Ubar(2,k)^2 <= 1);
    opti.subject_to(Ubar(3,k)^2 + Ubar(4,k)^2 <= 1);
end

dUbar_max = dFmax/Fmax;

% rate constraints
for k=1:Np
    if k==1, dU = U(:,k)-Uprev; else, dU = U(:,k)-U(:,k-1); end
    opti.subject_to(dU <=  dUbar_max);
    opti.subject_to(dU >= -dUbar_max);
end

% low-speed constraint
for k=1:Np+1
    ub=X(4,k); vb=X(5,k);
    opti.subject_to(ub^2+vb^2 <= vmax^2);
end

% cost: pose tracking + small velocities + smooth inputs
Qeta = diag([20,20,200]);
Qnu  = diag([1,1,10]);
Ru   = 1e-10*eye(nu);
Rdu  = 1e-8*eye(nu);

J=0;
for k=1:Np
    eta = X(1:3,k); nuv = X(4:6,k);
    e = [eta(1)-xref(1);
         eta(2)-xref(2);
         ssa(eta(3)-xref(3))];
    J = J + e'*Qeta*e + nuv'*Qnu*nuv + U(:,k)'*Ru*U(:,k);
    if k==1, dU=U(:,k)-Uprev; else, dU=U(:,k)-U(:,k-1); end
    J = J + dU'*Rdu*dU;
end
% terminal
etaN = X(1:3,Np+1);
eN = [etaN(1)-xref(1);
      etaN(2)-xref(2);
      ssa(etaN(3)-xref(3))];
J = J + 30*eN'*Qeta*eN;

opti.minimize(J);

opts.ipopt.print_level = 0;
opts.print_time = 0;
opti.solver('ipopt',opts);

%% Closed-loop simulation on MSS navalvessel plant
Tsim = 120;
Ns = round(Tsim/h);

xplant = [-50; 0;   0;0; 0;0; 0;0];     % [N E u v p r phi psi]'
goal   = [126; 28; 33];                     % [N E psi]'

U_prev = zeros(nu,1);

logPlant = zeros(8,Ns);
logU     = zeros(nu,Ns);

for t=1:Ns
    % NMPC initial state from plant (ignore roll in controller state)
    N=xplant(1); E=xplant(2);
    ub=xplant(3); vb=xplant(4);
    psi=xplant(8); r=xplant(6);

    x0_num = [N; E; psi; ub; vb; r];

    opti.set_value(x0, x0_num);
    opti.set_value(xref, goal);
    opti.set_value(Uprev, U_prev);

    % warm-start (simple)
    opti.set_initial(X, repmat(x0_num,1,Np+1));
    opti.set_initial(U, repmat(U_prev,1,Np));

    sol = opti.solve();
    Xpred = sol.value(X);  % 6 x (Np+1)

    if t==1
        figure; grid on; axis equal; hold on;
        h_pred = plot(nan,nan,'r--','LineWidth',1);
        h_act  = plot(nan,nan,'b-','LineWidth',2);
        legend('NMPC predicted horizon','Actual trajectory');
    end
    
    % update lines: x-axis=E, y-axis=N
    set(h_pred,'XData',Xpred(2,:),'YData',Xpred(1,:));
    set(h_act ,'XData',logPlant(2,1:t),'YData',logPlant(1,1:t));
    drawnow;

    u0 = sol.value(U(:,1));

    % apply to plant
    xplant = plant_step_navalvessel(xplant, u0, lx, ly, h, freeze_roll);

    U_prev = u0;

    logPlant(:,t) = xplant;
    logU(:,t)     = u0;
end

disp('Done.');

%%
t = (0:size(logPlant,2)-1)*h;

% North-East trajectory
figure;
plot(logPlant(2,:), logPlant(1,:));
axis equal; grid on;
xlabel('E [m]'); ylabel('N [m]');
title('Trajectory');

% Heading
figure;
plot(t, logPlant(8,:)); grid on;
xlabel('t [s]'); ylabel('\psi [rad]');
title('Yaw angle');

