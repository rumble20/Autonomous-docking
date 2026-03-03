%% run_mpc_harbour_navigatin.m
% Full-pipeline NMPC harbor navigation test
%
% Architecture:
%   LOS Guidance -> SB-MPC COLAV -> NMPC (nlpsol) -> container() plant
%   PID heading controller as fallback when NMPC fails.
%
% TEST A — Multi-waypoint path following (no extra obstacles)
% TEST B — Multi-waypoint + one static obstacle
% TEST C — Multi-waypoint + one target ship (SB-MPC COLREGs avoidance)
%
% Author: Riccardo Legnini
% Date: 2025 (refactored from Opti-based version)

clear; close all; clc;
clear animateSimResult   % reset rotation cache when re-running the script
fprintf('  NMPC HARBOR NAVIGATION — FULL PIPELINE (LOS + SB-MPC + NMPC)\n');

%% ===== STEP 0: Sanity check on container.m ==============================
x_test = [7; 0; 0; 0; 0; 0; 0; 0; 0; 70];

% u     = x(1)/U;   v   = x(2)/U;  
% p     = x(7)*L/U; r   = x(3)*L/U; 
% phi   = x(8);     psi = x(6); 
% delta = x(9);     n   = x(10)/60*L/U;

u_test = [0; 70];
[xdot_test, U_test] = container(x_test, u_test);
fprintf('  u=%.1f m/s, n=%.0f RPM -> u_dot=%.6f, x_dot=%.4f, psi_dot=%.6f\n', ...
    x_test(1), x_test(10), xdot_test(1), xdot_test(4), xdot_test(6));
fprintf('  (should be near-equilibrium: small derivatives)\n\n');

%% ===== STEP 1: Configure all modules ====================================

% --- Ship animation image -----------------------------------------------
% Path to the top-view vessel image used as the ship icon in animations.
% The image is used as-is: bow must face upward (North) in the file.
shipImgPath = 'c:\Users\SERILEG\OneDrive - ABB\Autonomous-docking\useful pictures\vessel_top.png';

% --- NMPC config --------------------------------------------------------
nmpc_cfg = struct();
nmpc_cfg.N  = 20;
nmpc_cfg.dt = 1.0;
nmpc_cfg.Q  = diag([0, 0, 0.5, 0, 0, 30, 0, 0, 0, 0]); % heading-tracker: only psi(30) + r(0.5)
nmpc_cfg.R  = diag([0.005, 0.001]);   % small rudder penalty (like autobargesim rudderGain=0.005)
nmpc_cfg.use_obstacles     = true;
nmpc_cfg.max_obs           = 15;
nmpc_cfg.use_map_obstacles = true;
nmpc_cfg.max_map_obs       = 10;
nmpc_cfg.V_c    = 0.0;       % Current speed [m/s] (set >0 to test)
nmpc_cfg.beta_c = 0.0;       % Current direction [rad]
nmpc_cfg.model_type = 'full';       % 'full' or 'simplified'
nmpc_cfg.R_rate = diag([0.01, 0.001]); % small rudder-rate smoothing

nmpc = NMPC_Container(nmpc_cfg);

% --- LOS Guidance --------------------------------------------------------
los = LOSGuidance('K_p', 1/150, 'R_a', 60, 'chi_rate_max', pi/20, ...
                   'update_interval', 1);  % tighter lookahead, update every step
fprintf('  LOS Guidance: K_p=%.4f, R_a=%.0f m\n', los.K_p, los.R_a);

% --- SB-MPC COLAV --------------------------------------------------------
colav = SBMPC_COLAV(60, 2, 'D_CLOSE', 400, 'D_SAFE', 200, 'D_INIT', 800);
fprintf('  SB-MPC COLAV: T=%.0fs, D_SAFE=%.0fm, D_CLOSE=%.0fm\n', ...
    colav.T, colav.D_SAFE, colav.D_CLOSE);

% --- PID fallback --------------------------------------------------------
pid = PIDHeadingController(struct('K_p', 400, 'T_d', 50, 'T_i', 10, ...
    'n_default', 70, 'delta_max_deg', 35));
fprintf('  PID fallback: K_p=%d, T_d=%d\n', pid.K_p, pid.T_d);

% --- Actuator model (for simulation plant) --------------------------------
act = ActuatorModel();
fprintf('  Actuator: delta_max=%d deg, rate=%d deg/s\n', act.delta_max, act.Ddelta_max);

% --- Obstacles manager ----------------------------------------------------
harbor = HarborObstacles();

%% ===== STEP 1B: Load Helsinki harbor map =================================
fprintf('\n--- Step 1B: Load harbor map ---\n');
if exist('helsinki_harbour.mat', 'file')
    S = load('helsinki_harbour.mat');
    if isfield(S, 'map')
        harbor.loadPolygonMap(S.map);
        nmpc.setMap(S.map);
    else
        fprintf('  Warning: ''map'' variable not found in .mat\n');
    end
else
    fprintf('  Warning: helsinki_harbour.mat not found. Running without map.\n');
end

%% ===== STEP 2: CasADi dynamics consistency check =========================
fprintf('\n--- Step 2: CasADi-vs-container() consistency ---\n');
import casadi.*
x_sym = SX.sym('x', 10, 1);
u_sym = SX.sym('u', 2, 1);
env_sym = SX.sym('env', 2, 1);
xdot_sym = nmpc.dynamics_full_casadi(x_sym, u_sym, env_sym);
f_check = casadi.Function('f_check', {x_sym, u_sym, env_sym}, {xdot_sym});

xdot_cas = full(f_check(x_test, u_test, [0; 0]));
err = abs(xdot_test - xdot_cas);
max_err = max(err);

labels = {'u_dot','v_dot','r_dot','x_dot','y_dot','psi_dot','p_dot','phi_dot','del_dot','n_dot'};
fprintf('  %-10s  %12s  %12s  %12s\n', 'State', 'container()', 'CasADi', '|error|');
for i = 1:10
    fprintf('  %-10s  %+12.6f  %+12.6f  %12.2e\n', labels{i}, xdot_test(i), xdot_cas(i), err(i));
end
if max_err < 1e-6
    fprintf('  >> PASS (max err = %.2e)\n\n', max_err);
elseif max_err < 1e-2
    fprintf('  >> WARN: small diffs (CasADi safeguards). Should be OK.\n\n');
else
    fprintf('  >> FAIL: Large mismatch — check dynamics!\n\n');
end

%% ===== STEP 3: Build solver ONCE =========================================
fprintf('--- Step 3: Build nlpsol ---\n');
nmpc.buildSolver();

%% ===== Shared simulation parameters =====================================
T_final = 300;          % 5 minutes
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;

%% ========================================================================
%  HELPER: Generate reference trajectory from LOS heading + speed
%% ========================================================================
function x_ref = buildRefTrajectory(x0, chi_d, U_d, N, ~, ~, ~)
    % Build heading-tracker reference for the NMPC.
    %
    % Architecture (like autobargesim): LOS guidance computes chi_d (desired
    % heading). The MPC tracks chi_d as a heading setpoint. Position
    % tracking is handled entirely by LOS, not by the MPC cost.
    %
    % x_ref states:
    %   state 1 (u):   U_d    — desired surge speed
    %   state 3 (r):   r_d    — desired yaw rate (towards psi_d)
    %   state 6 (psi): chi_d  — desired heading
    %   state 10 (n):  70 RPM — nominal
    %   all others:    copy from x0 (don't-care for cost, but needed for
    %                  warm-start and dynamics propagation)
    
    x_ref = repmat(x0(:), 1, N+1);   % start from current state
    
    % Desired yaw rate: steer towards psi_d
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));  % wrapped error
    r_d = psi_err;  % proportional; at dt=1s this is rad/s
    r_d = max(-0.05, min(0.05, r_d));  % limit to ~3 deg/s
    
    for k = 0:N
        x_ref(1, k+1)  = U_d;     % surge speed
        x_ref(3, k+1)  = r_d;     % yaw rate reference
        x_ref(6, k+1)  = chi_d;   % heading reference
        x_ref(10, k+1) = 70;      % RPM nominal
    end
end

%% ========================================================================
%  HELPER: Simulation step (RK4 integration of container model)
%% ========================================================================
function x_next = simStep(x, u_ctrl, dt_s)
    % RK4 integration of container.m
    x(1)  = max(x(1), 0.1);
    x(10) = max(x(10), 1);
    [k1, ~] = container(x, u_ctrl);
    [k2, ~] = container(x + k1*dt_s/2, u_ctrl);
    [k3, ~] = container(x + k2*dt_s/2, u_ctrl);
    [k4, ~] = container(x + k3*dt_s, u_ctrl);
    x_next = x + dt_s/6*(k1 + 2*k2 + 2*k3 + k4);
end

%% ========================================================================
%  TEST A — Multi-waypoint path following (LOS -> NMPC, no COLAV needed)
%% ========================================================================
fprintf('\n==============================================================\n');
fprintf('  TEST A: Multi-waypoint path following (LOS -> NMPC)\n');
fprintf('==============================================================\n');

% Waypoints [x, y] in Helsinki harbour coordinates
wp_A = [-5800, -2800;
         -5500, -2700;
         -5000, -2500;
         -4450, -2200];
wp_speed_A = [7; 7; 7; 5];   % m/s at each wp

x = [7; 0; 0; wp_A(1,1); wp_A(1,2); atan2(wp_A(2,2)-wp_A(1,2), wp_A(2,1)-wp_A(1,1)); 0; 0; 0; 70];
chi_d_prev = x(6);
wp_idx = 1;
nmpc.resetWarmStart();
pid.reset(x(6));
act.reset(x(9), x(10));

traj_A = x;  ctrl_A = [];  solve_ok_A = [];  xte_A = [];
chi_d_log_A = [];  n_fail_consec = 0;

fprintf('  Waypoints: ');
for w = 1:size(wp_A,1)
    fprintf('(%.0f, %.0f) ', wp_A(w,1), wp_A(w,2));
end
fprintf('\n');

for i = 1:length(t)
    % ---- 1) Waypoint management (LOS guidance) ----
    wp_idx = los.findActiveSegment(wp_A, x, wp_idx);
    [chi_d, U_d] = los.computeRef(wp_A, wp_speed_A, x, wp_idx, chi_d_prev, i);
    [xte, ~] = los.computeXTE(wp_A, x, wp_idx);
    chi_d_prev = chi_d;
    
    % ---- 2) Build reference trajectory for NMPC (along path segments) ----
    x_ref = buildRefTrajectory(x, chi_d, U_d, nmpc.N, dt, wp_A, wp_idx);
    
    % ---- 3) Solve NMPC ----
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);
    
    % ---- 4) PID fallback if NMPC fails ----
    if ~info.success
        n_fail_consec = n_fail_consec + 1;
        if n_fail_consec >= 2
            [u_opt, pid] = pid.compute(chi_d, x(3), x(6), dt);
        end
    else
        n_fail_consec = 0;
    end
    
    % ---- 5) Simulate plant (RK4) ----
    x = simStep(x, u_opt, dt);
    
    % ---- 6) Map collision check ----
    [coll, ~] = harbor.checkMapCollision(x(4:5));
    if coll
        fprintf('  >> MAP COLLISION at t=%.1f s, pos=(%.1f, %.1f)\n', t(i), x(4), x(5));
        break;
    end
    
    % ---- 7) Logging ----
    traj_A = [traj_A, x];
    ctrl_A = [ctrl_A, u_opt];
    solve_ok_A = [solve_ok_A, info.success];
    xte_A = [xte_A, xte];
    chi_d_log_A = [chi_d_log_A, chi_d];
    
    % ---- 8) Progress ----
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1f deg wp=%d xte=%+.1fm ok=%d dt=%.3fs\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, info.success, info.solve_time);
    end
    
    % ---- 9) Final waypoint reached? ----
    if norm(x(4:5) - wp_A(end,:)') < los.R_a
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

nA_ok = sum(solve_ok_A);  nA_tot = length(solve_ok_A);
fprintf('  Solver: %d/%d (%.0f%%), mean XTE: %.1f m, max XTE: %.1f m\n', ...
    nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)), max(abs(xte_A)));

figure(1); clf;
% ---- Plot Trajectory ----
subplot(3,1,1);
if ~isempty(harbor.map), harbor.plotMap(); end
hold on;
plot(traj_A(5,:), traj_A(4,:), 'b-', 'LineWidth', 2);
plot(wp_A(:,2), wp_A(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
plot(traj_A(5,1), traj_A(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('A: Path following'); grid on; axis equal;

% ---- Plot Controls ----
subplot(3,1,2);
if ~isempty(ctrl_A)
    t_A = (0:size(ctrl_A,2)-1)*dt;
    yyaxis left;  stairs(t_A, rad2deg(ctrl_A(1,:)), 'b-'); ylabel('Rudder [deg]');
    yyaxis right; stairs(t_A, ctrl_A(2,:), 'r-');           ylabel('RPM');
end
xlabel('Time [s]'); title('A: Controls'); grid on;

% ---- Plot XTE ----
subplot(3,1,3);
if ~isempty(xte_A)
    t_xte_A = (0:length(xte_A)-1)*dt;
    plot(t_xte_A, xte_A, 'b-', 'LineWidth', 1.5);
    yline(0, 'k--');
end
xlabel('Time [s]'); ylabel('XTE [m]'); title('A: Cross-track error'); grid on;

% --- Dynamic animation (post-simulation, does not affect NMPC timing) ----
cfg_anim_A = struct();
cfg_anim_A.figNo       = 10;
cfg_anim_A.testName    = 'A: Path Following';
cfg_anim_A.shipImgFile = shipImgPath;
cfg_anim_A.shipSize    = 0.08;
cfg_anim_A.maxFrames   = 150;
cfg_anim_A.pauseTime   = 0.05;
t_anim_A = (0:size(traj_A,2)-1)*dt;
animateSimResult(traj_A, wp_A, t_anim_A, harbor, cfg_anim_A);

%% ========================================================================
%  TEST B — Multi-waypoint + one static obstacle
%% ========================================================================
fprintf('\n==============================================================\n');
fprintf('  TEST B: Path following + static obstacle avoidance\n');
fprintf('==============================================================\n');

wp_B = [-5800, -2800;
         -5500, -2700;
         -5000, -2400;
         -4400, -2400];
wp_speed_B = [7; 7; 7; 5];

% Place obstacle on the path between wp2 and wp3
obs_pos_B = [-5350; -2600];
obs_rad_B = 40;

harbor_B = HarborObstacles();
if ~isempty(harbor.map), harbor_B.loadPolygonMap(harbor.map); end
harbor_B.addStaticObstacle(obs_pos_B, obs_rad_B, 'Pier-B');

x = [7; 0; 0; wp_B(1,1); wp_B(1,2); atan2(wp_B(2,2)-wp_B(1,2), wp_B(2,1)-wp_B(1,1)); 0; 0; 0; 70];
chi_d_prev = x(6);
wp_idx = 1;
nmpc.resetWarmStart();
pid.reset(x(6));

traj_B = x;  ctrl_B = [];  solve_ok_B = [];  xte_B = [];
n_fail_consec = 0;

fprintf('  Obstacle at (%.0f, %.0f) r=%.0f m\n', obs_pos_B(1), obs_pos_B(2), obs_rad_B);

for i = 1:length(t)
    wp_idx = los.findActiveSegment(wp_B, x, wp_idx);
    [chi_d, U_d] = los.computeRef(wp_B, wp_speed_B, x, wp_idx, chi_d_prev, i);
    [xte, ~] = los.computeXTE(wp_B, x, wp_idx);
    chi_d_prev = chi_d;
    
    x_ref = buildRefTrajectory(x, chi_d, U_d, nmpc.N, dt, wp_B, wp_idx);
    
    % Pass static obstacle to NMPC
    obs_struct = harbor_B.getAllCircularObstacles();
    [u_opt, ~, info] = nmpc.solve(x, x_ref, obs_struct);
    
    if ~info.success
        n_fail_consec = n_fail_consec + 1;
        if n_fail_consec >= 2
            [u_opt, pid] = pid.compute(chi_d, x(3), x(6), dt);
        end
    else
        n_fail_consec = 0;
    end
    
    x = simStep(x, u_opt, dt);
    
    [coll, ~] = harbor_B.checkMapCollision(x(4:5));
    [coll_c, ~] = harbor_B.checkCircularCollision(x(4:5));
    if coll
        fprintf('  >> MAP COLLISION at t=%.1f\n', t(i));
        break;
    end
    if coll_c
        fprintf('  >> OBSTACLE COLLISION at t=%.1f\n', t(i));
        break;
    end
    
    traj_B = [traj_B, x];  ctrl_B = [ctrl_B, u_opt];
    solve_ok_B = [solve_ok_B, info.success];  xte_B = [xte_B, xte];
    
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1fdeg wp=%d ok=%d obs_real=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, info.success, info.n_obs_real);
    end
    
    if norm(x(4:5) - wp_B(end,:)') < los.R_a
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end
nB_ok = sum(solve_ok_B);  nB_tot = length(solve_ok_B);
fprintf('  Solver: %d/%d (%.0f%%), mean XTE: %.1f m\n', ...
    nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), mean(abs(xte_B)));


figure(2); clf;
% ---- Plot Trajectory ----
subplot(3,1,1);
if ~isempty(harbor.map), harbor.plotMap(); end
hold on;
plot(traj_B(5,:), traj_B(4,:), 'b-', 'LineWidth', 2);
plot(wp_B(:,2), wp_B(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
theta_obs = linspace(0, 2*pi, 50);
fill(obs_pos_B(2)+obs_rad_B*cos(theta_obs), obs_pos_B(1)+obs_rad_B*sin(theta_obs), ...
    'r', 'FaceAlpha', 0.3);
plot(traj_B(5,1), traj_B(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('B: Obstacle avoidance'); grid on; axis equal;

% ---- Plot Controls ----
subplot(3,1,2);
if ~isempty(ctrl_B)
    t_B = (0:size(ctrl_B,2)-1)*dt;
    yyaxis left;  stairs(t_B, rad2deg(ctrl_B(1,:)), 'b-'); ylabel('Rudder [deg]');
    yyaxis right; stairs(t_B, ctrl_B(2,:), 'r-');           ylabel('RPM');
end
xlabel('Time [s]'); title('B: Controls'); grid on;

% ---- Plot XTE ----
subplot(3,1,3);
if ~isempty(xte_B)
    t_xte_B = (0:length(xte_B)-1)*dt;
    plot(t_xte_B, xte_B, 'b-', 'LineWidth', 1.5);
    yline(0, 'k--');
end
xlabel('Time [s]'); ylabel('XTE [m]'); title('B: Cross-track error'); grid on;

% --- Dynamic animation ---------------------------------------------------
cfg_anim_B = struct();
cfg_anim_B.figNo       = 11;
cfg_anim_B.testName    = 'B: Obstacle Avoidance';
cfg_anim_B.shipImgFile = shipImgPath;
cfg_anim_B.shipSize    = 0.08;
cfg_anim_B.maxFrames   = 150;
cfg_anim_B.pauseTime   = 0.05;
% Pass the static obstacle so it is drawn in the animation
cfg_anim_B.circObs(1).position = obs_pos_B;
cfg_anim_B.circObs(1).radius   = obs_rad_B;
t_anim_B = (0:size(traj_B,2)-1)*dt;
animateSimResult(traj_B, wp_B, t_anim_B, harbor_B, cfg_anim_B);

%% ========================================================================
%  TEST C — Multi-waypoint + one target ship (SB-MPC COLREGs)
%% ========================================================================
fprintf('\n==============================================================\n');
fprintf('  TEST C: Path following + target ship (SB-MPC COLAV)\n');
fprintf('==============================================================\n');

wp_C = [-5800, -2800;
         -5500, -2700;
         -5200, -2500;
         -4900, -2500];
wp_speed_C = [7; 7; 7; 5];

% Target ship: crossing from port side at ~5 m/s heading ~270 deg
% Start at position that will create a crossing encounter
ts_state = [5; 0; 0; -5400; -2300; deg2rad(180+45); 0; 0; 0; 60];
ts_radius = 30;

harbor_C = HarborObstacles();
if ~isempty(harbor.map), harbor_C.loadPolygonMap(harbor.map); end
harbor_C.addTargetShip(ts_state(1:6), ts_radius, 'TS-1');

x = [7; 0; 0; wp_C(1,1); wp_C(1,2); atan2(wp_C(2,2)-wp_C(1,2), wp_C(2,1)-wp_C(1,1)); 0; 0; 0; 70];
chi_d_prev = x(6);
chi_m_last = 0;  U_m_last = 1.0;
wp_idx = 1;
nmpc.resetWarmStart();
pid.reset(x(6));

traj_C = x;  ctrl_C = [];  solve_ok_C = [];  xte_C = [];
colav_mod_C = [];  % Log SB-MPC modifications
n_fail_consec = 0;

fprintf('  Target ship at (%.0f, %.0f) heading %.0f deg, speed %.1f m/s\n', ...
    ts_state(4), ts_state(5), rad2deg(ts_state(6)), ts_state(1));

for i = 1:length(t)
    % 1) LOS guidance
    wp_idx = los.findActiveSegment(wp_C, x, wp_idx);
    [chi_d, U_d] = los.computeRef(wp_C, wp_speed_C, x, wp_idx, chi_d_prev, i);
    [xte, ~] = los.computeXTE(wp_C, x, wp_idx);
    chi_d_prev = chi_d;
    
    % 2) SB-MPC COLAV: modify course/speed if target ship nearby
    x_ts = harbor_C.getTargetShipStates();
    [chi_c, U_c, chi_m, U_m] = colav.run(x, chi_d, U_d, chi_m_last, U_m_last, x_ts);
    chi_m_last = chi_m;
    U_m_last   = U_m;
    
    % 3) Build reference from COLAV-modified heading (along path)
    x_ref = buildRefTrajectory(x, chi_c, U_c, nmpc.N, dt, wp_C, wp_idx);
    
    % 4) NMPC solve — target ships as circular obstacles (full struct fields required)
    ts_obs = [];
    for ts_k = 1:length(harbor_C.target_ships)
        ts_obs_k.position = harbor_C.target_ships(ts_k).state(4:5);
        ts_obs_k.radius   = harbor_C.target_ships(ts_k).radius;
        ts_obs_k.velocity = [0; 0];
        ts_obs_k.type     = 'target_ship';
        ts_obs_k.name     = harbor_C.target_ships(ts_k).name;
        ts_obs = [ts_obs; ts_obs_k];
    end
    [u_opt, ~, info] = nmpc.solve(x, x_ref, ts_obs);
    
    % 5) PID fallback
    if ~info.success
        n_fail_consec = n_fail_consec + 1;
        if n_fail_consec >= 2
            [u_opt, pid] = pid.compute(chi_c, x(3), x(6), dt);
        end
    else
        n_fail_consec = 0;
    end
    
    % 6) Simulate plant
    x = simStep(x, u_opt, dt);
    
    % 7) Propagate target ships
    harbor_C.updateDynamicObstacles(dt);
    
    % 8) Collision check
    [coll, ~] = harbor_C.checkMapCollision(x(4:5));
    if coll
        fprintf('  >> MAP COLLISION at t=%.1f\n', t(i));
        break;
    end
    % Check target ship proximity
    for ts_k = 1:length(harbor_C.target_ships)
        d_ts = norm(x(4:5) - harbor_C.target_ships(ts_k).state(4:5));
        if d_ts < harbor_C.target_ships(ts_k).radius
            fprintf('  >> TARGET SHIP COLLISION at t=%.1f, d=%.1f m\n', t(i), d_ts);
        end
    end
    
    % 9) Log
    traj_C = [traj_C, x];
    ctrl_C = [ctrl_C, u_opt];
    solve_ok_C = [solve_ok_C, info.success];
    xte_C  = [xte_C, xte];
    colav_mod_C = [colav_mod_C; chi_m, U_m];
    
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1fdeg wp=%d chi_m=%+.0fdeg U_m=%.1f ok=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, rad2deg(chi_m), U_m, info.success);
    end
    
    if norm(x(4:5) - wp_C(end,:)') < los.R_a
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end
nC_ok = sum(solve_ok_C);  nC_tot = length(solve_ok_C);
fprintf('  Solver: %d/%d (%.0f%%), mean XTE: %.1f m\n', ...
    nC_ok, nC_tot, 100*nC_ok/max(nC_tot,1), mean(abs(xte_C)));


figure(3); clf;
% ---- Trajectory + target ship path ----
subplot(3,1,1);
if ~isempty(harbor.map), harbor.plotMap(); end
hold on;
plot(traj_C(5,:), traj_C(4,:), 'b-', 'LineWidth', 2);
plot(wp_C(:,2), wp_C(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
% Plot target ship trajectory (reconstruct from initial state)
ts_start = ts_state;
ts_traj_x = ts_start(4);  ts_traj_y = ts_start(5);
for k = 1:size(traj_C,2)-1
    ts_start(4) = ts_start(4) + (cos(ts_start(6))*ts_start(1) - sin(ts_start(6))*ts_start(2))*dt;
    ts_start(5) = ts_start(5) + (sin(ts_start(6))*ts_start(1) + cos(ts_start(6))*ts_start(2))*dt;
    ts_traj_x = [ts_traj_x, ts_start(4)];
    ts_traj_y = [ts_traj_y, ts_start(5)];
end
plot(ts_traj_y, ts_traj_x, 'm--', 'LineWidth', 1.5);
plot(traj_C(5,1), traj_C(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
legend('Own ship', 'Waypoints', 'Target ship', 'Start', 'Location', 'best');
xlabel('y [m]'); ylabel('x [m]'); title('C: COLAV (SB-MPC)'); grid on; axis equal;

% ---- TEST C: Controls ----
subplot(3,1,2);
if ~isempty(ctrl_C)
    t_C = (0:size(ctrl_C,2)-1)*dt;
    yyaxis left;  stairs(t_C, rad2deg(ctrl_C(1,:)), 'b-'); ylabel('Rudder [deg]');
    yyaxis right; stairs(t_C, ctrl_C(2,:), 'r-');           ylabel('RPM');
end
xlabel('Time [s]'); title('C: Controls'); grid on;

% ---- TEST C: COLAV modifications ----
subplot(3,1,3);
if ~isempty(colav_mod_C)
    t_colav = (0:size(colav_mod_C,1)-1)*dt;
    yyaxis left;  plot(t_colav, rad2deg(colav_mod_C(:,1)), 'b-', 'LineWidth', 1.5);
    ylabel('chi_m [deg]');
    yyaxis right; plot(t_colav, colav_mod_C(:,2), 'r-', 'LineWidth', 1.5);
    ylabel('U_m factor');
end
xlabel('Time [s]'); title('C: SB-MPC modifications'); grid on;

sgtitle('NMPC Container Ship — Full Pipeline (LOS + SB-MPC + NMPC)', 'FontSize', 13);

% --- Dynamic animation (target-ship trajectory is already in ts_traj_x/y) -
cfg_anim_C = struct();
cfg_anim_C.figNo       = 12;
cfg_anim_C.testName    = 'C: COLAV (SB-MPC)';
cfg_anim_C.shipImgFile = shipImgPath;
cfg_anim_C.shipSize    = 0.08;
cfg_anim_C.maxFrames   = 150;
cfg_anim_C.pauseTime   = 0.05;
% Overlay target ship trajectory (reconstructed in the plot block above)
if exist('ts_traj_x', 'var') && exist('ts_traj_y', 'var')
    cfg_anim_C.extraPaths = {{ts_traj_x, ts_traj_y, 'm--', 'Target ship'}};
end
t_anim_C = (0:size(traj_C,2)-1)*dt;
animateSimResult(traj_C, wp_C, t_anim_C, harbor_C, cfg_anim_C);

%% ===== Summary ==========================================================
fprintf('\n=== SUMMARY ===\n');
fprintf('  Test A — %d/%d solves (%.0f%%), mean XTE: %.1f m\n', nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)));
fprintf('  Test B — %d/%d solves (%.0f%%), mean XTE: %.1f m\n', nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), mean(abs(xte_B)));
fprintf('  Test C — %d/%d solves (%.0f%%), mean XTE: %.1f m\n', nC_ok, nC_tot, 100*nC_ok/max(nC_tot,1), mean(abs(xte_C)));
fprintf('  Total solver: OK=%d, Fail=%d\n', nmpc.solve_ok, nmpc.solve_fail);
fprintf('\nDone. Check figures.\n');
