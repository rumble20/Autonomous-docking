%% run_nmpc_simple.m
% Minimal-viable NMPC harbor navigation — path following + static obstacles
%
% Architecture (simplified):
%   Simple waypoint steering  ->  NMPC_Container_Lite  ->  container() plant
%   PID heading fallback when NMPC fails.
%
% TEST A — Multi-waypoint path following (no obstacles)
% TEST B — Multi-waypoint + one static obstacle
%
% Dependencies:
%   container.m              - Nonlinear ship dynamics (MSS toolbox)
%   NMPC_Container_Lite.m   - Simplified CasADi NMPC solver
%   animateSimResult.m       - Post-simulation animation
%
% Author: Riccardo Legnini
% Date:   2025 (simplified MVP)

clear; close all; clc;
clear animateSimResult   % reset image cache
fprintf('  NMPC HARBOR NAVIGATION — MVP (Simple Guidance + NMPC)\n\n');

%% ===== Sanity check on container.m ======================================
x_test = [7; 0; 0; 0; 0; 0; 0; 0; 0; 70];
u_test = [0; 70];
[xdot_test, ~] = container(x_test, u_test);
fprintf('  container.m check: u_dot=%.6f, psi_dot=%.6f (should be ~0)\n\n', ...
    xdot_test(1), xdot_test(6));

%% ===== NMPC configuration ===============================================
nmpc_cfg = struct();
nmpc_cfg.N  = 20;
nmpc_cfg.dt = 1.0;
nmpc_cfg.Q  = diag([2.0, 0, 0.8, 0.03, 0.03, 15, 0, 0, 0, 0]);
nmpc_cfg.R  = diag([0.005, 0.001]);
nmpc_cfg.R_rate       = diag([0.08, 0.001]);
nmpc_cfg.max_obs      = 15;
nmpc_cfg.r_safety     = 40;
nmpc_cfg.penalty_slack = 15000;

nmpc = NMPC_Container_Lite(nmpc_cfg);

%% ===== CasADi dynamics consistency check =================================
fprintf('--- CasADi consistency check ---\n');
import casadi.*
x_sym   = SX.sym('x', 10, 1);
u_sym   = SX.sym('u', 2, 1);
xdot_sym = nmpc.containerCasADi(x_sym, u_sym);
f_check  = casadi.Function('f_check', {x_sym, u_sym}, {xdot_sym});

xdot_cas = full(f_check(x_test, u_test));
max_err  = max(abs(xdot_test - xdot_cas));

labels = {'u_dot','v_dot','r_dot','x_dot','y_dot','psi_dot','p_dot','phi_dot','del_dot','n_dot'};
fprintf('  %-10s  %12s  %12s  %12s\n', 'State', 'container()', 'CasADi', '|error|');
for i = 1:10
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
shipImgPath = 'c:\Users\SERILEG\OneDrive - ABB\Autonomous-docking\useful pictures\vessel_top.png';

%% ===== Shared simulation parameters =====================================
T_final = 300;          % seconds
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;

% Simple PID fallback gains (used when NMPC fails)
pid_Kp = 1;
pid_Td = 10;

%% ===== Helsinki harbour map (optional, for plotting only) ================
map = [];
if exist('helsinki_harbour.mat', 'file')
    S = load('helsinki_harbour.mat');
    if isfield(S, 'map'), map = S.map; end
end

%% ========================================================================
%  TEST A — Multi-waypoint path following (no obstacles)
%% ========================================================================
fprintf('\n==============================================================\n');
fprintf('  TEST A: Multi-waypoint path following (NMPC)\n');
fprintf('==============================================================\n');

wp_A = [-5800, -2800;
         -5500, -2700;
         -5000, -2500;
         -4450, -2200];
wp_speed_A = [7; 7; 7; 5];

% Initial state
x0_heading = atan2(wp_A(2,2)-wp_A(1,2), wp_A(2,1)-wp_A(1,1));
x = [7; 0; 0; wp_A(1,1); wp_A(1,2); x0_heading; 0; 0; 0; 70];

wp_idx = 1;
R_accept = 35;   % Waypoint acceptance radius [m] - reduced to prevent corner-cutting

% Preallocate logging
traj_A     = zeros(10, length(t)+1);
ctrl_A     = zeros(2,  length(t));
solve_ok_A = false(1,  length(t));
xte_A      = zeros(1,  length(t));
traj_A(:,1) = x;
steps_A = 0;

fprintf('  Waypoints: ');
for w = 1:size(wp_A,1), fprintf('(%.0f, %.0f) ', wp_A(w,1), wp_A(w,2)); end
fprintf('\n');

for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_A, wp_speed_A, wp_idx, R_accept);
    xte = computeXTE(x, wp_A, wp_idx);

    % ---- 2) Build reference trajectory for NMPC ------------------------
    x_ref = buildSimpleRef(x, chi_d, U_d, nmpc.N, dt, wp_A, wp_idx);

    % ---- 3) Solve NMPC (no obstacles) -----------------------------------
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);

    % ---- 4) PID fallback ------------------------------------------------
    if ~info.success
        psi_err = wrapToPi(x(6) - chi_d);
        delta_c = -pid_Kp * (psi_err + pid_Td * x(3));
        delta_c = max(-deg2rad(35), min(deg2rad(35), delta_c));
        u_opt = [delta_c; 70];
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x = rk4Step(x, u_opt, dt);

    % ---- 6) Logging ------------------------------------------------------
    steps_A = i;
    traj_A(:, i+1)  = x;
    ctrl_A(:, i)    = u_opt;
    solve_ok_A(i)   = info.success;
    xte_A(i)        = xte;

    % ---- 7) Progress -----------------------------------------------------
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1fdeg wp=%d xte=%+.1fm ok=%d dt=%.3fs\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, info.success, info.solve_time);
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
t_A        = (0:steps_A) * dt;

nA_ok = sum(solve_ok_A);  nA_tot = length(solve_ok_A);
fprintf('  Solver: %d/%d (%.0f%%), mean XTE: %.1f m, max XTE: %.1f m\n', ...
    nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)), max(abs(xte_A)));

% ------ Test A: Plots -----------------------------------------------------
figure(1); clf;

subplot(3,1,1);
plotMapBackground(map);
hold on;
plot(traj_A(5,:), traj_A(4,:), 'b-', 'LineWidth', 2);
plot(wp_A(:,2), wp_A(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
plot(traj_A(5,1), traj_A(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('A: Path following'); grid on; axis equal;

subplot(3,1,2);
t_ctrl_A = (0:size(ctrl_A,2)-1)*dt;
yyaxis left;  stairs(t_ctrl_A, rad2deg(ctrl_A(1,:)), 'b-'); ylabel('Rudder [deg]');
yyaxis right; stairs(t_ctrl_A, ctrl_A(2,:), 'r-');           ylabel('RPM');
xlabel('Time [s]'); title('A: Controls'); grid on;

subplot(3,1,3);
t_xte_A = (0:length(xte_A)-1)*dt;
plot(t_xte_A, xte_A, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('A: Cross-track error'); grid on;

% ------ Test A: Animation ------------------------------------------------
cfg_anim_A = struct();
cfg_anim_A.figNo       = 10;
cfg_anim_A.testName    = 'A: Path Following';
cfg_anim_A.shipImgFile = shipImgPath;
cfg_anim_A.shipSize    = 0.08;
cfg_anim_A.maxFrames   = 150;
cfg_anim_A.pauseTime   = 0.05;

harbor_anim = [];
if ~isempty(map)
    harbor_anim = HarborAnimHelper(map);
end
animateSimResult(traj_A, wp_A, t_A, harbor_anim, cfg_anim_A);


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

% One static obstacle on the path
obs_B(1).position = [-5350; -2600];
obs_B(1).radius   = 40;

x0_heading_B = atan2(wp_B(2,2)-wp_B(1,2), wp_B(2,1)-wp_B(1,1));
x = [7; 0; 0; wp_B(1,1); wp_B(1,2); x0_heading_B; 0; 0; 0; 70];
wp_idx = 1;
R_accept_B = 25;  % tighter switching for obstacle test (prevents checkpoint skipping)

% Reset warm-start between tests (A -> B) to avoid cross-test bias
nmpc.prev_sol = [];
nmpc.prev_u   = [];

% Preallocate
traj_B     = zeros(10, length(t)+1);
ctrl_B     = zeros(2,  length(t));
solve_ok_B = false(1,  length(t));
xte_B      = zeros(1,  length(t));
traj_B(:,1) = x;
steps_B = 0;

fprintf('  Obstacle at (%.0f, %.0f) r=%.0f m\n', ...
    obs_B(1).position(1), obs_B(1).position(2), obs_B(1).radius);

for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_B, wp_speed_B, wp_idx, R_accept_B);
    xte = computeXTE(x, wp_B, wp_idx);

    % ---- 2) Build reference  --------------------------------------------
    x_ref = buildSimpleRef(x, chi_d, U_d, nmpc.N, dt, wp_B, wp_idx);

    % ---- 3) Solve NMPC (with obstacle) ----------------------------------
    [u_opt, ~, info] = nmpc.solve(x, x_ref, obs_B);

    % ---- 4) PID fallback ------------------------------------------------
    if ~info.success
        psi_err = wrapToPi(x(6) - chi_d);
        delta_c = -pid_Kp * (psi_err + pid_Td * x(3));
        delta_c = max(-deg2rad(35), min(deg2rad(35), delta_c));
        u_opt = [delta_c; 70];
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x = rk4Step(x, u_opt, dt);
    
    % ---- 6) Collision checks (AFTER step) -------------------------------
    % 6a) Circular obstacle collision
    d_obs = norm(x(4:5) - obs_B(1).position);
    if d_obs < obs_B(1).radius
        fprintf('  >> CIRCULAR OBSTACLE COLLISION at t=%.1f s, d=%.1f m\n', t(i), d_obs);
        steps_B = i;
        traj_B(:, i+1) = x;
        ctrl_B(:, i)   = u_opt;
        solve_ok_B(i)  = info.success;
        xte_B(i)       = xte;
        break;
    end

    % 6b) Map polygon collision (island / shoreline)
    map_collision = false;
    if ~isempty(map) && isfield(map, 'polygons')
        xN = x(4);  % North
        yE = x(5);  % East
        for j = 1:length(map.polygons)
            if inpolygon(xN, yE, map.polygons(j).X, map.polygons(j).Y)
                map_collision = true;
                break;
            end
        end
    end

    if map_collision
        fprintf('  >> MAP COLLISION (inside polygon %d) at t=%.1f s, pos=(%.1f, %.1f)\n', ...
            j, t(i), x(4), x(5));
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
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1fdeg wp=%d ok=%d obs_d=%.0fm\n', ...
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
t_B        = (0:steps_B) * dt;

nB_ok = sum(solve_ok_B);  nB_tot = length(solve_ok_B);
fprintf('  Solver: %d/%d (%.0f%%), mean XTE: %.1f m\n', ...
    nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), mean(abs(xte_B)));

% ------ Test B: Plots -----------------------------------------------------
figure(2); clf;

subplot(3,1,1);
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

subplot(3,1,2);
t_ctrl_B = (0:size(ctrl_B,2)-1)*dt;
yyaxis left;  stairs(t_ctrl_B, rad2deg(ctrl_B(1,:)), 'b-'); ylabel('Rudder [deg]');
yyaxis right; stairs(t_ctrl_B, ctrl_B(2,:), 'r-');           ylabel('RPM');
xlabel('Time [s]'); title('B: Controls'); grid on;

subplot(3,1,3);
t_xte_B = (0:length(xte_B)-1)*dt;
plot(t_xte_B, xte_B, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('B: Cross-track error'); grid on;

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

animateSimResult(traj_B, wp_B, t_B, harbor_anim, cfg_anim_B);


%% ===== Summary ==========================================================
fprintf('\n=== SUMMARY ===\n');
fprintf('  Test A — %d/%d solves (%.0f%%), mean XTE: %.1f m, max XTE: %.1f m\n', ...
    nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)), max(abs(xte_A)));
fprintf('  Test B — %d/%d solves (%.0f%%), mean XTE: %.1f m\n', ...
    nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), mean(abs(xte_B)));
fprintf('  Total solver: OK=%d, Fail=%d\n', nmpc.solve_ok, nmpc.solve_fail);
fprintf('\nDone. Check figures.\n');


%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, wp_speed, wp_idx, R_accept)
% Simple waypoint steering: heading to next waypoint, switch when close.
%
%   x:         state vector [u v r x y psi ...]
%   wp:        Nx2 waypoints [x, y]
%   wp_speed:  Nx1 speed at each wp [m/s]
%   wp_idx:    current segment index (1-based)
%   R_accept:  acceptance radius [m]
%
% Returns:
%   chi_d:  desired heading [rad]
%   U_d:    desired speed [m/s]
%   wp_idx: (possibly updated) segment index

    n_wps = size(wp, 1);
    pos   = [x(4); x(5)];

    % Switch waypoint: only when ship has actually reached it
    % (not when it just gets close but is still approaching)
    target_wp = wp(min(wp_idx+1, n_wps), :)';
    d_to_target = norm(pos - target_wp);
    
    % Check if we should switch: either very close, OR we've passed it
    % (use dot product to see if we're past the waypoint)
    should_switch = false;
    if wp_idx < n_wps - 1
        if d_to_target < R_accept
            % Check if we're moving away from the waypoint (passed it)
            vel_vec = [x(1)*cos(x(6)); x(1)*sin(x(6))];  % velocity in NED
            to_wp = target_wp - pos;
            % If dot product is negative, we're moving away
            if norm(vel_vec) > 0.1 && norm(to_wp) > 0.1
                cos_angle = dot(vel_vec, to_wp) / (norm(vel_vec)*norm(to_wp));
                if cos_angle < 0 || d_to_target < R_accept/2  % Passed it or very close
                    should_switch = true;
                end
            elseif d_to_target < R_accept/2
                should_switch = true;
            end
        end
    end
    
    if should_switch
        wp_idx = wp_idx + 1;
    end
    
    % Clamp to valid range
    wp_idx = min(wp_idx, n_wps - 1);

    % Target: next waypoint
    target = wp(min(wp_idx+1, n_wps), :)';
    dp     = target - pos;
    chi_d  = atan2(dp(2), dp(1));

    % Speed: interpolate between current and next wp speed
    U_d = wp_speed(min(wp_idx+1, n_wps));

    % Slow down when approaching final waypoint
    d_final = norm(pos - wp(end,:)');
    if d_final < 200
        U_d = max(2, U_d * d_final / 200);
    end
end

function xte = computeXTE(x, wp, wp_idx)
% Cross-track error: signed distance from vessel to current path segment.
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

    % Cross product gives signed distance
    xte = ((pos(1)-p1(1))*seg(2) - (pos(2)-p1(2))*seg(1)) / seg_len;
end

function x_ref = buildSimpleRef(x0, chi_d, U_d, N, dt, wp, wp_idx)
% Build reference following waypoint path segments (like original buildRefTrajectory)
%
%   x_ref: 10 × (N+1) matrix
%   Tracked states: u (speed), r (yaw rate), x/y (position), psi (heading), n (RPM)

    x_ref = repmat(x0(:), 1, N+1);

    % Desired yaw rate: gentle steering towards chi_d
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));
    r_d = 0.7 * psi_err;
    r_d = max(-0.20, min(0.20, r_d));

    % Position rollout along path segments
    n_wps = size(wp, 1);
    seg_idx = min(max(1, wp_idx), max(1, n_wps-1));
    p_ref = [x0(4); x0(5)];
    
    for k = 1:(N+1)
        x_ref(1, k)  = U_d;        % speed
        x_ref(3, k)  = r_d;        % yaw rate
        x_ref(6, k)  = chi_d;      % heading
        x_ref(10, k) = 70;         % RPM nominal

        % Keep x_ref(:,1) at current position; rollout starts from k=2
        if k > 1
            if seg_idx < n_wps
                p_to = wp(seg_idx+1, :)';
                dp = p_to - p_ref;
                if norm(dp) < max(5, 0.8*U_d*dt) && seg_idx < n_wps-1
                    seg_idx = seg_idx + 1;
                    p_to = wp(seg_idx+1, :)';
                    dp = p_to - p_ref;
                end

                if norm(dp) > 1e-6
                    dir_vec = dp / norm(dp);
                else
                    dir_vec = [cos(chi_d); sin(chi_d)];
                end
                p_ref = p_ref + dir_vec * U_d * dt;
            else
                p_ref = p_ref + [cos(chi_d); sin(chi_d)] * U_d * dt;
            end
        end

        x_ref(4, k) = p_ref(1);
        x_ref(5, k) = p_ref(2);
    end
end

function x_next = rk4Step(x, u_ctrl, dt_s)
% RK4 integration of container.m for one time step.
    x(1)  = max(x(1), 0.1);
    x(10) = max(x(10), 1);
    [k1, ~] = container(x, u_ctrl);
    [k2, ~] = container(x + k1*dt_s/2, u_ctrl);
    [k3, ~] = container(x + k2*dt_s/2, u_ctrl);
    [k4, ~] = container(x + k3*dt_s,   u_ctrl);
    x_next  = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
end

function angle = wrapToPi(angle)
% Wrap angle to [-pi, pi].
    angle = mod(angle + pi, 2*pi) - pi;
end

function plotMapBackground(map)
% Plot harbour map polygons (if loaded).
    if isempty(map), return; end
    hold on;
    if isfield(map, 'polygons')
        for kk = 1:length(map.polygons)
            patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
                'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, ...
                'EdgeColor', 'r', 'LineWidth', 1.5);
        end
    end
    if isfield(map, 'mapPoly')
        for kk = 1:length(map.mapPoly)
            patch(map.mapPoly(kk).Y, map.mapPoly(kk).X, 'c', ...
                'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2);
        end
    end
end
