function [xdot, U] = container(x, ui)
% Simplified 6-DOF container ship model with dual azipod thrusters
% Based on Son & Nomoto (1982) with roll dynamics removed
% Compatible with MATLAB and GNU Octave (www.octave.org)
%
% [xdot, U] = container(x, ui) returns the speed U in m/s and the 
% time derivative of the state vector: x = [ u v r x y psi n1 n2 ]'
%
% States:
%   u     = surge velocity          (m/s)
%   v     = sway velocity           (m/s)
%   r     = yaw velocity            (rad/s)
%   x     = position in x-direction (m)
%   y     = position in y-direction (m)
%   psi   = yaw angle               (rad)
%   n1    = actual shaft velocity thruster 1 (rpm)
%   n2    = actual shaft velocity thruster 2 (rpm)
%
% Inputs:
%   ui = [alpha1 alpha2 n1_c n2_c]' where
%   alpha1 = azimuth angle thruster 1 - aft     (rad)
%   alpha2 = azimuth angle thruster 2 - forward (rad)
%   n1_c   = commanded shaft speed thruster 1   (rpm)
%   n2_c   = commanded shaft speed thruster 2   (rpm)
%
% Reference: Son & Nomoto (1982). On the Coupled Motion of Steering and 
%            Rolling of a High Speed Container Ship, Naval Architect of 
%            Ocean Engineering 20:73-83.
%
% Author: Riccardo Legnini
% Based on T. Fossen's implementation, modified for azipod config
% Date: 2026

%% Input validation
if (length(x) ~= 8), error('x-vector must have dimension 8!'); end
if (length(ui) ~= 4), error('u-vector must have dimension 4!'); end

%% Ship main particulars
L     = 175;                % Length between perpendiculars (m)
B     = 25.40;              % Beam (m)
d     = 8.50;               % Mean draft (m)
nabla = 21222;              % Displacement volume (m^3)
rho   = 1025;               % Water density (kg/m^3)
g     = 9.81;               % Gravity (m/s^2)

% Mass and inertia (dimensional)
m_ship = rho * nabla;       % Ship mass (kg) ≈ 21.75 million kg
Izz_ship = 0.1 * m_ship * L^2;  % Yaw moment of inertia (kg·m^2)

%% Propeller/Azipod parameters
D     = 6.533;              % Propeller diameter (m)
t     = 0.175;              % Thrust deduction factor
wp    = 0.184;              % Wake fraction at propeller

% Azipod positions (m from midship, positive forward)
x_azi1 = -0.45 * L;         % Aft azipod position
x_azi2 =  0.35 * L;         % Forward azipod position (bow thruster style)
y_azi1 =  0;                % Centerline
y_azi2 =  0;                % Centerline

%% Actuator limits
n_max       = 160;          % Max shaft velocity (rpm)
n_min       = -80;          % Min shaft velocity (rpm) - reverse
Dn_max      = 10;           % Max shaft acceleration (rpm/s)
alpha_max   = pi;           % Max azimuth angle (rad)
Dalpha_max  = deg2rad(15);  % Max azimuth rate (rad/s) - typical for azipods

%% Extract states
u   = x(1);                 % Surge velocity (m/s)
v   = x(2);                 % Sway velocity (m/s)
r   = x(3);                 % Yaw velocity (rad/s)
x_pos = x(4);               % X position (m)
y_pos = x(5);               % Y position (m)
psi = x(6);                 % Yaw angle (rad)
n1  = x(7);                 % Shaft speed thruster 1 (rpm)
n2  = x(8);                 % Shaft speed thruster 2 (rpm)

%% Extract inputs
alpha1 = ui(1);             % Azimuth angle thruster 1 (rad)
alpha2 = ui(2);             % Azimuth angle thruster 2 (rad)
n1_c   = ui(3);             % Commanded shaft speed 1 (rpm)
n2_c   = ui(4);             % Commanded shaft speed 2 (rpm)

%% Speed calculation
U = sqrt(u^2 + v^2);        % Total speed (m/s)
if U < 0.1
    U = 0.1;                % Minimum speed to avoid singularities
end

%% Nondimensionalization
% Nondimensional velocities for hydrodynamic calculations
u_nd = u / U;
v_nd = v / U;
r_nd = r * L / U;

%% Nondimensional mass and added mass coefficients (from Son & Nomoto)
m_nd   = 0.00792;           % Nondimensional mass
mx     = 0.000238;          % Added mass in surge
my     = 0.007049;          % Added mass in sway
Jz     = 0.000419;          % Added moment of inertia in yaw
Iz_nd  = 0.000456;          % Nondimensional moment of inertia

%% Hydrodynamic derivatives (Son & Nomoto 1982) - Roll terms removed
% Surge
Xuu      = -0.0004226;
Xvr      = -0.00311;
Xrr      =  0.00020;
Xvv      = -0.00386;

% Sway (roll coupling terms set to zero)
Yv       = -0.0116;
Yr       =  0.00242;
Yvvv     = -0.109;
Yrrr     =  0.00177;
Yvvr     =  0.0214;
Yvrr     = -0.0405;

% Yaw (roll coupling terms set to zero)
Nv       = -0.0038545;
Nr       = -0.00222;
Nvvv     =  0.001492;
Nrrr     = -0.00229;
Nvvr     = -0.0424;
Nvrr     =  0.00156;

%% Nondimensional mass matrix components (simplified without roll)
m11 = m_nd + mx;            % Surge
m22 = m_nd + my;            % Sway
m66 = Iz_nd + Jz;           % Yaw

%% Hydrodynamic forces and moments (nondimensional)
X_hyd = Xuu*u_nd^2 + Xvr*v_nd*r_nd + Xvv*v_nd^2 + Xrr*r_nd^2;
Y_hyd = Yv*v_nd + Yr*r_nd + Yvvv*v_nd^3 + Yrrr*r_nd^3 + Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2;
N_hyd = Nv*v_nd + Nr*r_nd + Nvvv*v_nd^3 + Nrrr*r_nd^3 + Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2;

% Add Coriolis/centripetal terms (nondimensional)
X_hyd = X_hyd + (m_nd + my) * v_nd * r_nd;
Y_hyd = Y_hyd - (m_nd + mx) * u_nd * r_nd;

%% Propeller model for each azipod
% Thruster 1 (aft)
[T1, Q1] = azipod_thrust(n1, u, v, r, x_azi1, alpha1, D, wp, t, rho, U, L);

% Thruster 2 (forward) - reduced wake effect for bow thruster
wp_fwd = wp * 0.3;          % Reduced wake fraction for forward thruster
[T2, Q2] = azipod_thrust(n2, u, v, r, x_azi2, alpha2, D, wp_fwd, t, rho, U, L);

%% Total thrust forces and moments (dimensional, in N and N·m)
% Force components from each thruster
Fx1 = T1 * cos(alpha1);
Fy1 = T1 * sin(alpha1);
Fx2 = T2 * cos(alpha2);
Fy2 = T2 * sin(alpha2);

% Total forces (N)
X_thrust = Fx1 + Fx2;
Y_thrust = Fy1 + Fy2;

% Moments about center of gravity (N·m)
N_thrust = x_azi1 * Fy1 - y_azi1 * Fx1 + x_azi2 * Fy2 - y_azi2 * Fx2;

%% Convert hydrodynamic forces to dimensional (N and N·m)
X_hyd_dim = X_hyd * 0.5 * rho * L^2 * U^2;      % N
Y_hyd_dim = Y_hyd * 0.5 * rho * L^2 * U^2;      % N
N_hyd_dim = N_hyd * 0.5 * rho * L^3 * U^2;      % N·m

%% Total forces (dimensional)
X_total = X_hyd_dim + X_thrust;                 % N
Y_total = Y_hyd_dim + Y_thrust;                 % N
N_total = N_hyd_dim + N_thrust;                 % N·m

%% State derivatives

% Accelerations (dimensional)
u_dot = X_total / (m11 * m_ship / m_nd) + v * r;
v_dot = Y_total / (m22 * m_ship / m_nd) - u * r;
r_dot = N_total / (m66 * Izz_ship / Iz_nd);

% Kinematics
x_dot   = cos(psi) * u - sin(psi) * v;
y_dot   = sin(psi) * u + cos(psi) * v;
psi_dot = r;

%% Actuator dynamics

% Shaft dynamics (first-order lag)
Tm1 = shaft_time_constant(n1);
Tm2 = shaft_time_constant(n2);

% Saturate commanded values
n1_c = max(min(n1_c, n_max), n_min);
n2_c = max(min(n2_c, n_max), n_min);

% Shaft acceleration with rate limiting
n1_dot = (n1_c - n1) / Tm1;
n2_dot = (n2_c - n2) / Tm2;

n1_dot = max(min(n1_dot, Dn_max), -Dn_max);
n2_dot = max(min(n2_dot, Dn_max), -Dn_max);

%% Output
xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot; n1_dot; n2_dot];

end


function [T, Q] = azipod_thrust(n_rpm, u, v, r, x_pos, alpha, D, wp, t, rho, U, L)
% AZIPOD_THRUST Computes thrust and torque for an azipod thruster
%
% Inputs:
%   n_rpm  - Shaft speed (rpm)
%   u, v   - Ship velocities (m/s)
%   r      - Yaw rate (rad/s)
%   x_pos  - Longitudinal position of thruster (m)
%   alpha  - Azimuth angle (rad)
%   D      - Propeller diameter (m)
%   wp     - Wake fraction
%   t      - Thrust deduction factor
%   rho    - Water density (kg/m^3)
%   U      - Ship speed (m/s)
%   L      - Ship length (m)
%
% Outputs:
%   T      - Net thrust (N)
%   Q      - Torque (N·m)

% Convert rpm to rps
n = n_rpm / 60;

% Handle zero/low rpm
if abs(n) < 0.01
    T = 0;
    Q = 0;
    return;
end

% Local velocity at thruster position (accounting for yaw rotation)
u_local = u + r * 0;            % Surge component (no y-offset assumed)
v_local = v + r * x_pos;        % Sway component affected by yaw

% Velocity into the propeller (considering azimuth angle)
u_inflow = u_local * cos(alpha) + v_local * sin(alpha);

% Effective advance velocity with wake
u_a = u_inflow * (1 - wp);

% Advance ratio
J = u_a / (abs(n) * D);

% Limit J to valid range
J = max(min(J, 1.2), -0.5);

% Open water propeller characteristics (Wageningen B-series approximation)
% KT = a0 + a1*J + a2*J^2 (simplified)
KT0 = 0.527;                    % From original model
KT1 = -0.455;
KT = KT0 + KT1 * J;

% Ensure KT doesn't go negative at high J
KT = max(KT, 0.05);

% Thrust (N)
T_gross = rho * D^4 * n * abs(n) * KT;

% Net thrust after thrust deduction
T = (1 - t) * T_gross;

% Torque coefficient (simplified)
KQ = 0.01 * KT + 0.005;
Q = rho * D^5 * n * abs(n) * KQ;

end

function Tm = shaft_time_constant(n_rpm)
% Based on original Son & Nomoto model

if abs(n_rpm) > 18    % 0.3 rps = 18 rpm
    Tm = 5.65 / (abs(n_rpm)/60);
else
    Tm = 18.83;
end

% Limit time constant
Tm = max(min(Tm, 20), 1);

end
