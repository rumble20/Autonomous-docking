%% run_nmpc_simple.m
% NMPC harbor navigation with 8-state azipod container ship model
%
% Architecture:
%   Simple waypoint steering  ->  NMPC_Container_Lite  ->  container() plant
%   PID fallback when NMPC fails.
%
% TEST A — Multi-waypoint path following (no obstacles)
% TEST B — Multi-waypoint + one static obstacle
%
% Dependencies:
%   container.m        - 8-state ship dynamics (Son & Nomoto + azipods)
%   NMPC_Container_Lite.m   - CasADi NMPC solver
%   animateSimResult.m        - Post-simulation animation
%
% Author: Riccardo Legnini 
% Date:   2026-03-10

clear; close all; clc;
clear animateSimResult   
fprintf('  NMPC HARBOR NAVIGATION — 8-State Azipod Model\n\n');

%% ===== Sanity check on container.m ===============================
% Test state: [u=7, v=0, r=0, x=0, y=0, psi=0, n1=100, n2=0] (8-DOF)
% Test input: [alpha1=0, alpha2=0, n1_c=100, n2_c=0] (azipod commands)
x_test = [7; 0; 0; 0; 0; 0; 100; 0];
u_test = [0; 0; 100; 0];
[xdot_test, U_test] = container(x_test, u_test);
fprintf('  container.m check:\n');
fprintf('    u_dot=%.6f m/s², r_dot=%.6f rad/s², n1_dot=%.3f rpm/s\n', ...
    xdot_test(1), xdot_test(3), xdot_test(7));
fprintf('    Speed U=%.2f m/s (should be ~7)\n\n', U_test);

%% ===== NMPC configuration ===============================================
% For 8-DOF azipod model: [u, v, r, x, y, psi, n1, n2]
%                         + [alpha1, alpha2, n1_c, n2_c]
nmpc_cfg = struct();
nmpc_cfg.N  = 20;
nmpc_cfg.dt = 1.0;

% Q weights (8 states): [u, v, r, x, y, psi, n1, n2]
nmpc_cfg.Q = diag([2.0, 0.1, 0.8, 0.03, 0.03, 15, 0.001, 0.001]);

% R weights (4 controls): [alpha1, alpha2, n1_c, n2_c]
nmpc_cfg.R = diag([0.1, 0.1, 0.01, 0.01]);

% R_rate for smooth control transitions
nmpc_cfg.R_rate = diag([0.05, 0.05, 0.005, 0.005]);

nmpc_cfg.max_obs       = 15;
nmpc_cfg.r_safety      = 40;
nmpc_cfg.penalty_slack = 15000;
nmpc_cfg.enable_diagnostics = false;

nmpc = NMPC_Container_Lite(nmpc_cfg);

%% ===== CasADi dynamics consistency check =================================
fprintf('--- CasADi consistency check ---\n');
import casadi.*
x_sym   = SX.sym('x', 8, 1);    % 8-DOF model
u_sym   = SX.sym('u', 4, 1);    % 4 azipod controls
xdot_sym = nmpc.dynamicsCasADi(x_sym, u_sym);
f_check  = casadi.Function('f_check', {x_sym, u_sym}, {xdot_sym});

xdot_cas = full(f_check(x_test, u_test));
max_err  = max(abs(xdot_test - xdot_cas));

labels = {'u_dot','v_dot','r_dot','x_dot','y_dot','psi_dot','n1_dot','n2_dot'};
fprintf('  %-10s  %12s  %12s  %12s\n', 'State', 'container()', 'CasADi', '|error|');
for i = 1:8
    fprintf('  %-10s  %+12.6f  %+12.6f  %12.2e\n', ...
        labels{i}, xdot_test(i), xdot_cas(i), abs(xdot_test(i)-xdot_cas(i)));
end
if max_err < 1e-6
    fprintf('  >> PASS (max err = %.2e)\n\n', max_err);
elseif max_err < 1e-2
    fprintf('  >> WARN: small diffs (CasADi safeguards). OK.\n\n');
else
    fprintf('  >> FAIL: Large mismatch!\n\n');
end

%% ===== Build solver ONCE ================================================
fprintf('--- Building nlpsol ---\n');
nmpc.buildSolver();

%% ===== Ship image path (for animation) ==================================
shipImgPath = 'vessel_top.png';  % Update path as needed

%% ===== Shared simulation parameters =====================================
T_final = 300;          % seconds
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;

% Default shaft speeds for cruising
n1_cruise = 100;        % Aft thruster [rpm]
n2_cruise = 0;          % Forward thruster off during cruise

% Simple PID fallback gains
pid_Kp = 0.8;
pid_Ki = 0.01;
pid_Kd = 5.0;

% Test toggles
if ~exist('run_test_A', 'var'), run_test_A = true;  end
if ~exist('run_test_B', 'var'), run_test_B = true;  end

%% ===== Helsinki harbour map (optional) ==================================
map = [];
if exist('helsinki_harbour.mat', 'file')
    S = load('helsinki_harbour.mat');
    if isfield(S, 'map'), map = S.map; end
end

harbor_anim = [];
if ~isempty(map)
    harbor_anim = HarborAnimHelper(map);
end

%% ========================================================================
%  TEST A — Multi-waypoint path following (no obstacles)
%% ========================================================================
if run_test_A
fprintf('\n==============================================================\n');
fprintf('  TEST A: Multi-waypoint path following (NMPC)\n');
fprintf('==============================================================\n');

wp_A = [-5800, -2800;
         -5500, -2700;
         -5000, -2500;
         -4450, -2200];
wp_speed_A = [7; 7; 7; 5];

% Initial state (8-DOF model)
x0_heading = atan2(wp_A(2,2)-wp_A(1,2), wp_A(2,1)-wp_A(1,1));
% x = [u, v, r, x, y, psi, n1, n2]
x = [7; 0; 0; wp_A(1,1); wp_A(1,2); x0_heading; n1_cruise; n2_cruise];

wp_idx = 1;
R_accept = 35;   % Waypoint acceptance radius [m]

% Preallocate logging (8-DOF states + 4 azipod controls)
traj_A     = zeros(8, length(t)+1);
ctrl_A     = zeros(4, length(t));
solve_ok_A = false(1, length(t));
xte_A      = zeros(1, length(t));
fallback_A = false(1, length(t));
traj_A(:,1) = x;
steps_A = 0;

% PID integral term
psi_err_int = 0;
psi_err_prev = 0;

fprintf('  Waypoints: ');
for w = 1:size(wp_A,1), fprintf('(%.0f, %.0f) ', wp_A(w,1), wp_A(w,2)); end
fprintf('\n');

for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_A, wp_speed_A, wp_idx, R_accept);
    xte = computeXTE(x, wp_A, wp_idx);

    % ---- 2) Build reference trajectory for NMPC (8 states) --------------
    x_ref = buildSimpleRef8(x, chi_d, U_d, nmpc.N, dt, n1_cruise, n2_cruise);

    % ---- 3) Solve NMPC (no obstacles) -----------------------------------
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);

    % ---- 4) PID fallback (for azipods with rpm control) -----------------
    if ~info.success
        psi_err = wrapToPi(chi_d - x(6));
        psi_err_int = psi_err_int + psi_err * dt;
        psi_err_int = max(-1, min(1, psi_err_int));  % Anti-windup
        psi_err_dot = (psi_err - psi_err_prev) / dt;
        psi_err_prev = psi_err;
        
        % Azimuth angle from PID
        alpha = pid_Kp * psi_err + pid_Ki * psi_err_int + pid_Kd * psi_err_dot;
        alpha = max(-pi/4, min(pi/4, alpha));
        
        % Shaft speed proportional to desired speed
        n1_cmd = n1_cruise * (U_d / 7.0);
        n1_cmd = max(0, min(160, n1_cmd));
        
        u_opt = [alpha; 0; n1_cmd; 0];
        fallback_A(i) = true;
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x_old = x;
    x = rk4Step8(x, u_opt, dt);
    
    % Catch position explosion
    if abs(x(4)) > 50000 || abs(x(5)) > 50000 || any(isnan(x)) || any(isinf(x))
        fprintf('\n  ❌ RK4 EXPLOSION AT ITERATION %d\n', i);
        fprintf('     x_before = [u=%.2f, v=%.2f, r=%.4f, x=%.0f, y=%.0f, n1=%.0f]\n', ...
            x_old(1), x_old(2), x_old(3), x_old(4), x_old(5), x_old(7));
        fprintf('     u_opt = [α₁=%.4f, α₂=%.4f, n₁_c=%.1f, n₂_c=%.1f]\n', ...
            u_opt(1), u_opt(2), u_opt(3), u_opt(4));
        break;
    end

    % ---- 6) Logging ------------------------------------------------------
    steps_A = i;
    traj_A(:, i+1)  = x;
    ctrl_A(:, i)    = u_opt;
    solve_ok_A(i)   = info.success;
    xte_A(i)        = xte;

    % ---- 7) Progress -----------------------------------------------------
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1f° wp=%d xte=%+.1fm n1=%.0f ok=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, x(7), info.success);
    end

    % ---- 8) Done? --------------------------------------------------------
    if norm(x(4:5) - wp_A(end,:)') < R_accept
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

% Trim logs
traj_A     = traj_A(:, 1:steps_A+1);
ctrl_A     = ctrl_A(:, 1:steps_A);
solve_ok_A = solve_ok_A(1:steps_A);
xte_A      = xte_A(1:steps_A);
fallback_A = fallback_A(1:steps_A);
t_A        = (0:steps_A) * dt;

nA_ok = sum(solve_ok_A);  nA_tot = length(solve_ok_A);
fprintf('  Solver: %d/%d (%.0f%%), fallback=%d, mean XTE: %.1f m, max XTE: %.1f m\n', ...
    nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), sum(fallback_A), mean(abs(xte_A)), max(abs(xte_A)));

% ------ Test A: Plots -----------------------------------------------------
figure(1); clf;

subplot(3,2,1);
plotMapBackground(map);
hold on;
plot(traj_A(5,:), traj_A(4,:), 'b-', 'LineWidth', 2);
plot(wp_A(:,2), wp_A(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
plot(traj_A(5,1), traj_A(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('A: Path following'); grid on; axis equal;

subplot(3,2,2);
t_ctrl_A = (0:size(ctrl_A,2)-1)*dt;
plot(t_ctrl_A, rad2deg(ctrl_A(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl_A, rad2deg(ctrl_A(2,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title('A: Azimuth Angles'); grid on;
legend('\alpha_1 (aft)', '\alpha_2 (fwd)');

subplot(3,2,3);
plot(t_ctrl_A, ctrl_A(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl_A, ctrl_A(4,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Commanded rpm');
title('A: Shaft Speed Commands'); grid on;
legend('n_{1,c}', 'n_{2,c}');

subplot(3,2,4);
plot(t_A, traj_A(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_A, traj_A(8,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Actual rpm');
title('A: Actual Shaft Speeds'); grid on;
legend('n_1', 'n_2');

subplot(3,2,5);
t_xte_A = (0:length(xte_A)-1)*dt;
plot(t_xte_A, xte_A, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('A: Cross-track error'); grid on;

subplot(3,2,6);
plot(t_A, traj_A(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('A: Surge Velocity'); grid on;

sgtitle('Test A: Path Following (8-State Azipod Model)');

% ------ Test A: Animation ------------------------------------------------
cfg_anim_A = struct();
cfg_anim_A.figNo       = 10;
cfg_anim_A.testName    = 'A: Path Following';
cfg_anim_A.shipImgFile = shipImgPath;
cfg_anim_A.shipSize    = 0.08;
cfg_anim_A.maxFrames   = 150;
cfg_anim_A.pauseTime   = 0.05;

% Convert 8-state to 6-state for animation (uses rows 1-6)
traj_A_anim = traj_A(1:6, :);
animateSimResult(traj_A_anim, wp_A, t_A, harbor_anim, cfg_anim_A);
end


%% ========================================================================
%  TEST B — Multi-waypoint + one static obstacle
%% ========================================================================
if run_test_B
fprintf('\n==============================================================\n');
fprintf('  TEST B: Path following + static obstacle avoidance\n');
fprintf('==============================================================\n');

wp_B = [-3400, -1500;
         -2700, -1300;
         -2400, -1400;
         -1800, -1200];
wp_speed_B = [7; 7; 7; 5];

% One static obstacle on the path
obs_B(1).position = [-3000; -1400];
obs_B(1).radius   = 30;

x0_heading_B = atan2(wp_B(2,2)-wp_B(1,2), wp_B(2,1)-wp_B(1,1));
% 8-DOF model: [u, v, r, x, y, psi, n1, n2]
x = [7; 0; 0; wp_B(1,1); wp_B(1,2); x0_heading_B; n1_cruise; n2_cruise];

wp_idx = 1;
R_accept_B = 50;

% Reset warm-start between tests
nmpc.prev_sol = [];
nmpc.prev_u   = [];

% Reset PID integral
psi_err_int = 0;
psi_err_prev = 0;

% Preallocate (8-DOF states + 4 azipod controls)
traj_B     = zeros(8, length(t)+1);
ctrl_B     = zeros(4, length(t));
solve_ok_B = false(1, length(t));
xte_B      = zeros(1, length(t));
fallback_B = false(1, length(t));
traj_B(:,1) = x;
steps_B = 0;

fprintf('  Obstacle at (%.0f, %.0f) r=%.0f m\n', ...
    obs_B(1).position(1), obs_B(1).position(2), obs_B(1).radius);
fprintf('  Initial pos: (%7.1f, %7.1f), heading: %+6.1f deg\n\n', ...
    x(4), x(5), rad2deg(x(6)));

consecutive_fails = 0;
for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_B, wp_speed_B, wp_idx, R_accept_B);
    xte = computeXTE(x, wp_B, wp_idx);

    % ---- 2) Build reference (8 states) ----------------------------------
    x_ref = buildSimpleRef8(x, chi_d, U_d, nmpc.N, dt, n1_cruise, n2_cruise);

    % ---- 3) Solve NMPC (with obstacle) ----------------------------------
    [u_opt, X_pred, info] = nmpc.solve(x, x_ref, obs_B);

    % ---- 4) PID fallback ------------------------------------------------
    if ~info.success
        psi_err = wrapToPi(chi_d - x(6));
        psi_err_int = psi_err_int + psi_err * dt;
        psi_err_int = max(-1, min(1, psi_err_int));
        psi_err_dot = (psi_err - psi_err_prev) / dt;
        psi_err_prev = psi_err;
        
        alpha = pid_Kp * psi_err + pid_Ki * psi_err_int + pid_Kd * psi_err_dot;
        alpha = max(-pi/4, min(pi/4, alpha));
        
        n1_cmd = n1_cruise * (U_d / 7.0);
        n1_cmd = max(0, min(160, n1_cmd));
        
        u_opt = [alpha; 0; n1_cmd; 0];
        fallback_B(i) = true;
        consecutive_fails = consecutive_fails + 1;
        
        if consecutive_fails == 1
            fprintf('  !! SOLVER FAILED at t=%.1f\n', t(i));
        end
    else
        consecutive_fails = 0;
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x = rk4Step8(x, u_opt, dt);
    
    % ---- 6) Collision check ---------------------------------------------
    d_obs = norm(x(4:5) - obs_B(1).position);
    if d_obs < obs_B(1).radius
        fprintf('  >> COLLISION at t=%.1f s, d=%.1f m\n', t(i), d_obs);
        steps_B = i;
        traj_B(:, i+1) = x;
        ctrl_B(:, i)   = u_opt;
        solve_ok_B(i)  = info.success;
        xte_B(i)       = xte;
        break;
    end

    % ---- 7) Logging ------------------------------------------------------
    steps_B = i;
    traj_B(:, i+1)  = x;
    ctrl_B(:, i)    = u_opt;
    solve_ok_B(i)   = info.success;
    xte_B(i)        = xte;

    % ---- 8) Progress -----------------------------------------------------
    if i == 1 || mod(i, 15) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) ψ=%+6.1f° wp=%d ok=%d obs_d=%.0fm\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, info.success, d_obs);
    end

    % ---- 9) Done? --------------------------------------------------------
    if norm(x(4:5) - wp_B(end,:)') < R_accept_B
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

% Trim logs
traj_B     = traj_B(:, 1:steps_B+1);
ctrl_B     = ctrl_B(:, 1:steps_B);
solve_ok_B = solve_ok_B(1:steps_B);
xte_B      = xte_B(1:steps_B);
fallback_B = fallback_B(1:steps_B);
t_B        = (0:steps_B) * dt;

nB_ok = sum(solve_ok_B);  nB_tot = length(solve_ok_B);
fprintf('  Solver: %d/%d (%.0f%%), fallback=%d, mean XTE: %.1f m\n', ...
    nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), sum(fallback_B), mean(abs(xte_B)));

% ------ Test B: Plots -----------------------------------------------------
figure(2); clf;

subplot(3,2,1);
plotMapBackground(map);
hold on;
plot(traj_B(5,:), traj_B(4,:), 'b-', 'LineWidth', 2);
plot(wp_B(:,2), wp_B(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
theta_circ = linspace(0, 2*pi, 50);
fill(obs_B(1).position(2) + obs_B(1).radius*cos(theta_circ), ...
     obs_B(1).position(1) + obs_B(1).radius*sin(theta_circ), ...
     'r', 'FaceAlpha', 0.3);
plot(traj_B(5,1), traj_B(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('B: Obstacle avoidance'); grid on; axis equal;

subplot(3,2,2);
t_ctrl_B = (0:size(ctrl_B,2)-1)*dt;
plot(t_ctrl_B, rad2deg(ctrl_B(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl_B, rad2deg(ctrl_B(2,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title('B: Azimuth Angles'); grid on;
legend('\alpha_1', '\alpha_2');

subplot(3,2,3);
plot(t_ctrl_B, ctrl_B(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl_B, ctrl_B(4,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Commanded rpm');
title('B: Shaft Speed Commands'); grid on;
legend('n_{1,c}', 'n_{2,c}');

subplot(3,2,4);
plot(t_B, traj_B(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_B, traj_B(8,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Actual rpm');
title('B: Actual Shaft Speeds'); grid on;
legend('n_1', 'n_2');

subplot(3,2,5);
t_xte_B = (0:length(xte_B)-1)*dt;
plot(t_xte_B, xte_B, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('B: Cross-track error'); grid on;

subplot(3,2,6);
plot(t_B, traj_B(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('B: Surge Velocity'); grid on;

sgtitle('Test B: Obstacle Avoidance (8-State Azipod Model)');

% ------ Test B: Animation ------------------------------------------------
cfg_anim_B = struct();
cfg_anim_B.figNo       = 11;
cfg_anim_B.testName    = 'B: Obstacle Avoidance';
cfg_anim_B.shipImgFile = shipImgPath;
cfg_anim_B.shipSize    = 0.08;
cfg_anim_B.maxFrames   = 150;
cfg_anim_B.pauseTime   = 0.05;
cfg_anim_B.circObs(1).position = obs_B(1).position;
cfg_anim_B.circObs(1).radius   = obs_B(1).radius;

traj_B_anim = traj_B(1:6, :);
animateSimResult(traj_B_anim, wp_B, t_B, harbor_anim, cfg_anim_B);
end


%% ===== Summary ==========================================================
fprintf('\n=== SUMMARY ===\n');
if run_test_A && exist('nA_ok', 'var')
    fprintf('  Test A — %d/%d solves (%.0f%%), mean XTE: %.1f m, max XTE: %.1f m\n', ...
        nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)), max(abs(xte_A)));
end
if run_test_B && exist('nB_ok', 'var')
    fprintf('  Test B — %d/%d solves (%.0f%%), fallback=%d, mean XTE: %.1f m\n', ...
        nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), sum(fallback_B), mean(abs(xte_B)));
end
fprintf('  Total solver: OK=%d, Fail=%d\n', nmpc.solve_ok, nmpc.solve_fail);
fprintf('\nDone. Check figures.\n');


%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, wp_speed, wp_idx, R_accept)
% Simple waypoint steering for 8-state model (uses x(4:5) for position)
    n_wps = size(wp, 1);
    pos   = [x(4); x(5)];

    wp_idx = min(max(1, wp_idx), max(1, n_wps - 1));

    while wp_idx < n_wps - 1
        p_from = wp(wp_idx, :)';
        p_to   = wp(wp_idx + 1, :)';
        seg    = p_to - p_from;
        seg_l2 = seg' * seg;
        seg_len = sqrt(seg_l2);

        if seg_l2 < 1e-9
            wp_idx = wp_idx + 1;
            continue;
        end

        proj = dot(pos - p_from, seg) / seg_l2;
        d_to_waypoint = norm(pos - p_to);
        xte_seg = abs(((pos(1)-p_from(1))*seg(2) - (pos(2)-p_from(2))*seg(1)) / max(seg_len, 1e-6));
        near_segment = xte_seg <= max(2*R_accept, 60);

        if d_to_waypoint <= R_accept || (proj >= 1.0 && near_segment)
            wp_idx = wp_idx + 1;
        else
            break;
        end
    end

    p1 = wp(wp_idx, :)';
    p2 = wp(min(wp_idx + 1, n_wps), :)';
    seg = p2 - p1;
    seg_len = norm(seg);

    if seg_len < 1e-6
        target = p2;
    else
        s_proj = dot(pos - p1, seg) / seg_len;
        s_proj = max(0, min(seg_len, s_proj));

        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead = max(30, 4 * U_now);
        s_target = min(seg_len, s_proj + lookahead);

        target = p1 + (s_target / seg_len) * seg;
    end

    dp = target - pos;
    chi_d = atan2(dp(2), dp(1));
    U_d = wp_speed(min(wp_idx + 1, n_wps));

    d_final = norm(pos - wp(end,:)');
    if d_final < 200
        U_d = max(2, U_d * d_final / 200);
    end
end

function xte = computeXTE(x, wp, wp_idx)
% Cross-track error (works with 8-state model)
    n_wps = size(wp, 1);
    idx_from = max(1, wp_idx);
    idx_to   = min(idx_from + 1, n_wps);

    p1 = wp(idx_from, :)';
    p2 = wp(idx_to,   :)';
    pos = [x(4); x(5)];

    seg = p2 - p1;
    seg_len = norm(seg);
    if seg_len < 1
        xte = norm(pos - p1);
        return;
    end

    xte = ((pos(1)-p1(1))*seg(2) - (pos(2)-p1(2))*seg(1)) / seg_len;
end

function x_ref = buildSimpleRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref)
% Build 8-state reference trajectory
%   x_ref: 8 × (N+1) matrix  [u, v, r, x, y, psi, n1, n2]

    x_ref = zeros(8, N+1);
    x_ref(:, 1) = x0;
    
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));
    r_d = 0.5 * psi_err;
    r_d = max(-0.15, min(0.15, r_d));
    
    for k = 2:(N+1)
        x_ref(1, k) = U_d;          % Speed
        x_ref(2, k) = 0;            % Sway
        x_ref(3, k) = r_d;          % Yaw rate
        x_ref(6, k) = chi_d;        % Heading
        x_ref(7, k) = n1_ref;       % Aft shaft speed
        x_ref(8, k) = n2_ref;       % Fwd shaft speed
        
        dx = U_d * dt * cos(chi_d);
        dy = U_d * dt * sin(chi_d);
        x_ref(4, k) = x_ref(4, k-1) + dx;
        x_ref(5, k) = x_ref(5, k-1) + dy;
    end
    
    if any(isnan(x_ref(:))) || any(isinf(x_ref(:)))
        error('[buildSimpleRef8] NaN/Inf detected!');
    end
end

function x_next = rk4Step8(x, u_ctrl, dt_s)
% RK4 integration for 8-state container model
    x(1) = max(x(1), 0.1);
    x(7) = max(x(7), 0);  % n1 >= 0 for forward motion
    
    [k1, ~] = container(x, u_ctrl);
    
    x2 = x + k1*dt_s/2;
    x2(1) = max(x2(1), 0.1);
    x2(7) = max(x2(7), 0);
    [k2, ~] = container(x2, u_ctrl);
    
    x3 = x + k2*dt_s/2;
    x3(1) = max(x3(1), 0.1);
    x3(7) = max(x3(7), 0);
    [k3, ~] = container(x3, u_ctrl);
    
    x4 = x + k3*dt_s;
    x4(1) = max(x4(1), 0.1);
    x4(7) = max(x4(7), 0);
    [k4, ~] = container(x4, u_ctrl);
    
    x_next = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
    x_next(1) = max(x_next(1), 0.1);
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function plotMapBackground(map)
    if isempty(map), return; end
    hold on;
    if isfield(map, 'polygons')
        for kk = 1:length(map.polygons)
            patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
                'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, ...
                'EdgeColor', 'r', 'LineWidth', 1.5);
        end
    end
end
