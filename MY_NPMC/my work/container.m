function [xdot,U] = container(x,ui)
% Simplified 6-DOF vessel model with 2 azipod thrusters (roll effects ignored)
% Compatible with MATLAB and GNU Octave (www.octave.org).
%
% [xdot,U] = container(x,ui) returns the speed U in m/s and the time derivative 
% of the state vector: x = [ u v r x y psi ]'
% for a container ship L = 175 m with roll dynamics removed.
%
% u     = surge velocity          (m/s)
% v     = sway velocity           (m/s)
% r     = yaw velocity            (rad/s)
% x     = position in x-direction (m)
% y     = position in y-direction (m)
% psi   = yaw angle               (rad)
%
% The input vector is:
%
% ui      = [ alpha1 alpha2 F1 F2 ]'  where
%
% alpha1  = azimuth angle of thruster 1 (rad)
% alpha2  = azimuth angle of thruster 2 (rad)
% F1      = thrust magnitude of thruster 1 (kN)
% F2      = thrust magnitude of thruster 2 (kN)
%
% This is a simplified model focusing on surge, sway, and yaw dynamics.
% Roll effects are ignored for cleaner control.
% The model is based off a simplified version of the Son and Nomoto (1982) container ship model, implemented in Matlab by Fossen T. 

if (length(x) ~= 6), error('x-vector must have dimension 6 (u,v,r,x,y,psi) !'); end
if (length(ui) ~= 4), error('u-vector must have dimension 4 (alpha1,alpha2,F1,F2) !'); end

% Ship dimensions and normalization variables
L = 175;                     % Length of ship (m)
m_total = 41000;             % Total mass (kg), approximate for container ship
U = sqrt(x(1)^2 + x(2)^2);   % Speed magnitude (m/s)

% Check service speed
if U <= 0.1, U = 0.1; end    % Avoid division by zero

% Extract states
u = x(1);       % surge velocity (m/s)
v = x(2);       % sway velocity (m/s)
r = x(3);       % yaw velocity (rad/s)
x_pos = x(4);   % x position (m)
y_pos = x(5);   % y position (m)
psi = x(6);     % yaw angle (rad)

% Extract inputs
alpha1 = ui(1); % azimuth angle thruster 1 (rad)
alpha2 = ui(2); % azimuth angle thruster 2 (rad)
F1 = ui(3);     % thrust magnitude thruster 1 (kN)
F2 = ui(4);     % thrust magnitude thruster 2 (kN)

% Limit thrusts to realistic values
F1_max = 500;   % Max thrust per thruster (kN)
F2_max = 500;
F1 = max(min(F1, F1_max), -F1_max);
F2 = max(min(F2, F2_max), -F2_max);

% ========== SIMPLIFIED HYDRODYNAMIC DERIVATIVES (6-DOF, no roll) ==========
% Nondimensional added mass and hydrodynamic coefficients
% Removed: all roll (K) terms, p and phi related terms, rudder terms

% Mass and added mass parameters (normalized)
m = 20;         % Normalized mass (ship displacement / (0.5*rho*L^3))
Xuu = -0.04;    % Surge resistance coefficient (simplified)
Yv = -0.5;      % Sway damping coefficient
Yr = 0.25;      % Yaw-sway coupling
Nr = -0.15;     % Yaw damping
Nv = -0.1;      % Yaw-sway coupling

% Simplified nonlinear hydrodynamic terms (reduced complexity)
Xvv = -0.1;     % Quadratic sway drag on surge (minimal)
Yvv = -2.0;     % Cubic sway drag
Yrrr = 0.05;    % Cubic yaw effect on sway  
Nvv = -0.05;    % Nonlinear sway-yaw coupling
Nrr = -0.1;     % Quadratic yaw effect on heading

% Inertia parameters
Izz = 0.3;      %#ok<*NASGU> % Normalized yaw moment of inertia

% Thruster geometry: position along ship centerline (normalized by L)
x_thruster_1 = -0.2;  % Thruster 1 position (forward of aft)
x_thruster_2 = -0.2;  % Thruster 2 position (symmetric)


% ========== AZIPOD THRUSTER DYNAMICS AND FORCES ==========

% ========== CORRECTED FORCE CALCULATION ==========

% 1. Hydrodynamic forces (NORMALIZED, dimensionless coefficients)
Xhyd = Xuu * u * abs(u) + Xvv * v * abs(v);
Yhyd = Yv * v + Yr * r + Yvv * v * abs(v) + Yrrr * r^2 * abs(r);
Nhyd = Nv * v + Nr * r + Nvv * v * abs(v) + Nrr * r * abs(r);

% 2. Denormalize ONLY hydrodynamic terms to physical units (kN)
rho = 1025;  % kg/m³
X_hydro_kN = Xhyd * (0.5 * rho * L^2 * U^2) / 1000;  % kN
Y_hydro_kN = Yhyd * (0.5 * rho * L^2 * U^2) / 1000;  % kN
N_hydro_kN = Nhyd * (0.5 * rho * L^3 * U^2) / 1000;  % kN·m

% 3. Azipod thrust forces (ALREADY in kN, NO scaling needed!)
Fx_thrust = F1 * cos(alpha1) + F2 * cos(alpha2);  % kN
Fy_thrust = F1 * sin(alpha1) + F2 * sin(alpha2);  % kN
Mz_thrust = F1 * sin(alpha1) * x_thruster_1 + F2 * sin(alpha2) * x_thruster_2;  % kN·m (dimensionless x L)

% Convert moment to physical units
Mz_thrust_kN = Mz_thrust * L;  % kN·m

% 4. Total forces (both in kN now!)
X_total = X_hydro_kN + Fx_thrust;   % kN
Y_total = Y_hydro_kN + Fy_thrust;   % kN
N_total = N_hydro_kN + Mz_thrust_kN; % kN·m

% ========== STATE DERIVATIVES ==========
m_ship = 41000 * 1000;  % Convert mass to kg (41000 tonnes = 41 million kg)

u_dot = (X_total * 1000 / m_ship) + v * r;  % Convert kN → N, then m/s²
v_dot = (Y_total * 1000 / m_ship) - u * r;
r_dot = (N_total * 1000) / (0.1 * m_ship * L);  % rad/s²


% Kinematic equations (position and orientation)
x_dot = cos(psi) * u - sin(psi) * v;
y_dot = sin(psi) * u + cos(psi) * v;
psi_dot = r;

xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot];
