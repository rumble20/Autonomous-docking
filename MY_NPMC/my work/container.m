function [xdot,U] = container(x,ui)
% Compatibel with MATLAB and the free software GNU Octave (www.octave.org).
% [xdot,U] = container(x,ui) returns the speed U in m/s (optionally) and the 
% time derivative of the state vector: x = [ u v r x y psi p phi delta n ]'
% for a container ship L = 175 m  
% 
% Author:    Trygve Lauvdal
% Date:      1994-05-12
%
% The sequent changes have been made to the original code to make it compatible with my 
% "Combined" NMPC approach using 2 Azimuth Thrusters (Bow & Stern):
%
% x  = [ u v r x y psi p phi n_bow n_stern alpha_bow alpha_stern ]' (12x1)
% ui = [ n_c_bow, n_c_stern, alpha_c_bow, alpha_c_stern ]'          (4x1)

if (length(x) ~= 12), error('x-vector must have dimension 12 !'); end
if (length(ui) ~= 4), error('u-vector must have dimension 4 !'); end

L = 175;                     
U = sqrt(x(1)^2 + x(2)^2);   
if U <= 0, U = 0.1; end % Safeguard

% Normalized Velocities
u = x(1)/U;   v = x(2)/U;  
p = x(7)*L/U; r = x(3)*L/U; 
phi = x(8);   psi = x(6); 

% Actuator States
n_bow = x(9); n_stern = x(10);
alpha_bow = x(11); alpha_stern = x(12);

% Commanded Inputs
n_c_bow = ui(1); n_c_stern = ui(2);
alpha_c_bow = ui(3); alpha_c_stern = ui(4);

% Main dimensions and hydrodynamics (Kept identical for bare hull)
m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
ly = 0.0313;     Iz = 0.000456;
Jx = 0.0000034;  Jz = 0.000419;
B = 25.40; dF = 8.00; g = 9.81; nabla = 21222; rho = 1025; D = 6.533; GM = 0.3/L; W = rho*g*nabla/(rho*L^2*U^2/2);
Xuu = -0.0004226; Xvr = -0.00311; Xrr = 0.00020; Xphiphi = -0.00020; Xvv = -0.00386;
Kv = 0.0003026; Kr = -0.000063; Kp = -0.0000075; Kphi = -0.000021; Kvvv = 0.002843; Krrr = -0.0000462; Kvvr = -0.000588; Kvrr = 0.0010565; Kvvphi = -0.0012012; Kvphiphi = -0.0000793; Krrphi = -0.000243; Krphiphi = 0.00003569;
Yv = -0.0116; Yr = 0.00242; Yp = 0; Yphi = -0.000063; Yvvv = -0.109; Yrrr = 0.00177; Yvvr = 0.0214; Yvrr = -0.0405; Yvvphi = 0.04605; Yvphiphi = 0.00304; Yrrphi = 0.009325; Yrphiphi = -0.001368;
Nv = -0.0038545; Nr = -0.00222; Np = 0.000213; Nphi = -0.0001424; Nvvv = 0.001492; Nrrr = -0.00229; Nvvr = -0.0424; Nvrr = 0.00156; Nvvphi = -0.019058; Nvphiphi = -0.0053766; Nrrphi = -0.0038592; Nrphiphi = 0.0024195;
zR = 0.033; t_ded = 0.175;

% Mass matrix elements
m11 = (m+mx); m22 = (m+my); m32 = -my*ly; m42 = my*alphay; m33 = (Ix+Jx); m44 = (Iz+Jz); detM = m22*m33*m44-m32^2*m44-m42^2*m33;

%% --- Azipod Thruster Allocation Model ---
% Moment arms (scaled from the thesis supply ship 32m to the 175m container ship)
lx_bow = 80; lx_stern = -80; 

% Continuous absolute value formulation for thrust [cite: 782]
KT0 = 0.527; eps = 0.001;
n_bow_nd = (n_bow/60) * L/U; 
n_stern_nd = (n_stern/60) * L/U;

T_bow   = 2*rho*D^4 / (U^2*L^2*rho) * KT0 * n_bow_nd * sqrt(n_bow_nd^2 + eps);
T_stern = 2*rho*D^4 / (U^2*L^2*rho) * KT0 * n_stern_nd * sqrt(n_stern_nd^2 + eps);

Thrust_X = T_bow * cos(alpha_bow) + T_stern * cos(alpha_stern);
Thrust_Y = T_bow * sin(alpha_bow) + T_stern * sin(alpha_stern);
Thrust_N = (lx_bow/L) * T_bow * sin(alpha_bow) + (lx_stern/L) * T_stern * sin(alpha_stern);

%% --- Forces and Moments ---
X_f = Xuu*u^2 + (1-t_ded)*Thrust_X + Xvr*v*r + Xvv*v^2 + Xrr*r^2 + Xphiphi*phi^2 + (m + my)*v*r;
Y_f = Yv*v + Yr*r + Yp*p + Yphi*phi + Yvvv*v^3 + Yrrr*r^3 + Yvvr*v^2*r + Yvrr*v*r^2 + Yvvphi*v^2*phi + Yvphiphi*v*phi^2 + Yrrphi*r^2*phi + Yrphiphi*r*phi^2 + Thrust_Y - (m + mx)*u*r;
K_f = Kv*v + Kr*r + Kp*p + Kphi*phi + Kvvv*v^3 + Krrr*r^3 + Kvvr*v^2*r + Kvrr*v*r^2 + Kvvphi*v^2*phi + Kvphiphi*v*phi^2 + Krrphi*r^2*phi + Krphiphi*r*phi^2 - W*GM*phi + zR*Thrust_Y + mx*lx*u*r;
N_f = Nv*v + Nr*r + Np*p + Nphi*phi + Nvvv*v^3 + Nrrr*r^3 + Nvvr*v^2*r + Nvrr*v*r^2 + Nvvphi*v^2*phi + Nvphiphi*v*phi^2 + Nrrphi*r^2*phi + Nrphiphi*r*phi^2 + Thrust_N;

%% --- Actuator Dynamics ---
% First-order lag to represent actuator limits smoothly
n_dot_bow = (n_c_bow - n_bow) / 2.0;
n_dot_stern = (n_c_stern - n_stern) / 2.0;
alpha_dot_bow = (alpha_c_bow - alpha_bow) / 2.0;
alpha_dot_stern = (alpha_c_stern - alpha_stern) / 2.0;

%% --- State Derivatives ---
xdot =[ X_f*(U^2/L)/m11
       -((-m33*m44*Y_f+m32*m44*K_f+m42*m33*N_f)/detM)*(U^2/L)
        ((-m42*m33*Y_f+m32*m42*K_f+N_f*m22*m33-N_f*m32^2)/detM)*(U^2/L^2)
        (cos(psi)*u-sin(psi)*cos(phi)*v)*U
        (sin(psi)*u+cos(psi)*cos(phi)*v)*U 
        cos(phi)*r*(U/L)                
       ((-m32*m44*Y_f+K_f*m22*m44-K_f*m42^2+m32*m42*N_f)/detM)*(U^2/L^2)
        p*(U/L)
        n_dot_bow 
        n_dot_stern
        alpha_dot_bow
        alpha_dot_stern ];
end