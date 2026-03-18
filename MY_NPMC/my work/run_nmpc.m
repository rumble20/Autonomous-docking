%% run_nmpc.m
% NMPC harbor navigation with 8-state azipod container ship model
%
% UNIFIED TEST — Single configurable test with:
%   - Multi-waypoint path following
%   - Optional static obstacles
%   - Red-zone map obstacle awareness
%   - PID fallback for robustness
%
% Dependencies:
%   container.m             - 8-state ship dynamics (Son & Nomoto + azipods)
%   NMPC_Container_final.m   - CasADi NMPC solver (see below)
%   NavUtils.m              - Navigation utilities (see below)
%   animateSimResult.m      - Post-simulation animation
%   HarborAnimHelper.m      - Harbor animation helper
%
% Author: Riccardo Legnini 
% Date:   2026-03-18

clear; close all; clc;
clear animateSimResult   
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC HARBOR NAVIGATION — Unified Test (8-State Azipod)\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%  USER CONFIGURATION — EDIT THIS SECTION
%% ═══════════════════════════════════════════════════════════════════════

% ---- WAYPOINTS (rows = [x, y] in meters, NED frame) ----
waypoints = [-3250, -1600;
             -2800, -1600;
             -2400, -1900;
             -2000, -2050];

% ---- WAYPOINT SPEEDS (m/s) — one per segment + final ----
waypoint_speeds = [7; 7; 7; 5];

% ---- STATIC OBSTACLES (set to empty [] for none) ----
% Each obstacle: struct with 'position' [x; y] and 'radius' [m]
static_obstacles = [];
% static_obstacles(1).position = [-3000; -1400];
% static_obstacles(1).radius   = 30;

% ---- SIMULATION PARAMETERS ----
T_final     = 600;      % Total simulation time [s]
R_accept    = 50;       % Waypoint acceptance radius [m]
n1_cruise   = 100;      % Aft thruster cruise speed [rpm]
n2_cruise   = 0;        % Forward thruster (off during cruise)

% ---- MAP OBSTACLE SAMPLING ----
enable_map_obstacles = true;   % Set false to disable red-zone awareness
max_map_obstacles    = 4;      % Max map sample points as obstacles (keep low!)
map_lookahead_m      = 350;    % Forward lookahead distance [m]
map_half_width_m     = 100;    % Corridor half-width [m]
map_sample_radius_m  = 12;     % Virtual obstacle radius for map points

% ---- NMPC TUNING ----
nmpc_N  = 40;           % Prediction horizon steps
nmpc_dt = 1.0;          % Sample time [s]
r_safety = 20;          % Safety margin around obstacles [m]

% Q weights: [u, v, r, x, y, psi, n1, n2]
%   - Higher position weights (x,y) → better obstacle avoidance
%   - Higher heading weight (psi) → tighter path tracking
Q_weights = diag([2.0, 0.1, 0.5, 6.0, 6.0, 4.0, 0.001, 0.001]);

% R weights: [alpha1, alpha2, n1_c, n2_c]
R_weights = diag([0.1, 0.1, 0.01, 0.01]);

% R_rate weights for smooth control transitions
R_rate_weights = diag([0.12, 0.12, 0.006, 0.006]);

% ---- PID FALLBACK GAINS (used when NMPC fails) ----
pid_Kp = 0.8;
pid_Ki = 0.01;
pid_Kd = 5.0;

%% ═══════════════════════════════════════════════════════════════════════
%  INITIALIZATION (DO NOT EDIT BELOW UNLESS DEBUGGING)
%% ═══════════════════════════════════════════════════════════════════════

%% ===== Sanity check on container.m ======================================
x_test = [7; 0; 0; 0; 0; 0; 100; 0];
u_test = [0; 0; 100; 0];
[xdot_test, U_test] = container(x_test, u_test);
fprintf('  container.m check: u_dot=%.4f, r_dot=%.6f, U=%.2f m/s\n\n', ...
    xdot_test(1), xdot_test(3), U_test);

%% ===== Ship image path (for animation) ==================================
scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(scriptDir));
shipImgPath = fullfile(repoRoot, 'useful pictures', 'vessel_top.png');
if ~isfile(shipImgPath)
    warning('Ship image not found. Animation will use fallback shape.');
end

%% ===== Helsinki harbour map =============================================
map = [];
if exist('helsinki_harbour_UPDATED.mat', 'file')
    S = load('helsinki_harbour_UPDATED.mat');
    if isfield(S, 'map'), map = S.map; end
end

harbor_anim = [];
if ~isempty(map)
    harbor_anim = HarborAnimHelper(map);
    fprintf('  Harbor map loaded: %d polygons\n', length(map.polygons));
end

% Pre-sample map polygon edges for fast local queries
map_sample_pts = [];
if enable_map_obstacles && ~isempty(map)
    map_sample_pts = buildMapSamplePoints(map, 100);
    fprintf('  Map sample points: %d\n', size(map_sample_pts, 1));
end

%% ===== NMPC configuration ===============================================
n_static_obs = length(static_obstacles);
if enable_map_obstacles && ~isempty(map_sample_pts)
    max_obs_slots = n_static_obs + max_map_obstacles;
else
    max_obs_slots = n_static_obs;
end

nmpc_cfg = struct();
nmpc_cfg.N  = nmpc_N;
nmpc_cfg.dt = nmpc_dt;
nmpc_cfg.Q  = Q_weights;
nmpc_cfg.R  = R_weights;
nmpc_cfg.R_rate = R_rate_weights;
nmpc_cfg.max_obs = max(1, max_obs_slots);  % At least 1 slot
nmpc_cfg.r_safety = r_safety;
nmpc_cfg.enable_diagnostics = false;

fprintf('\n--- Building NMPC solver (%d obstacle slots) ---\n', nmpc_cfg.max_obs);
nmpc = NMPC_Container_final(nmpc_cfg);
nmpc.buildSolver();

%% ===== Initial state ====================================================
x0_heading = atan2(waypoints(2,2) - waypoints(1,2), ...
                   waypoints(2,1) - waypoints(1,1));
% State: [u, v, r, x, y, psi, n1, n2]
x = [7; 0; 0; waypoints(1,1); waypoints(1,2); x0_heading; n1_cruise; n2_cruise];

% Check if starting inside a map zone (grace period)
in_start_zone = false;
start_zone_type = '';
start_zone_idx = 0;
if ~isempty(map)
    [in_start_zone, start_zone_type, start_zone_idx] = ...
        NavUtils.isInsideAnyMapZone(x(4:5), map);
    if in_start_zone
        fprintf('  [WARN] Start inside map zone (%s #%d) — grace enabled.\n', ...
            start_zone_type, start_zone_idx);
    end
end

%% ===== Simulation setup =================================================
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;
wp_idx = 1;

% Preallocate logging
traj     = zeros(8, length(t)+1);
ctrl     = zeros(4, length(t));
solve_ok = false(1, length(t));
xte_log  = zeros(1, length(t));
fallback = false(1, length(t));
traj(:,1) = x;
steps = 0;

% PID state
psi_err_int = 0;
psi_err_prev = 0;

% Previous control for NMPC rate limiting (NEW!)
u_prev = [0; 0; n1_cruise; n2_cruise];

fprintf('\n  Waypoints: ');
% ... rest of print statements ...

%% ═══════════════════════════════════════════════════════════════════════
%  MAIN SIMULATION LOOP
%% ═══════════════════════════════════════════════════════════════════════

for i = 1:length(t)
    % ---- 1) Waypoint guidance -------------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, waypoints, waypoint_speeds, wp_idx, R_accept);
    xte = computeXTE(x, waypoints, wp_idx);

    % ---- 2) Gather obstacles (static + map samples) ---------------------
    obs_local = static_obstacles;
    
    if enable_map_obstacles && ~isempty(map_sample_pts)
        obs_map = selectMapObstaclesFromSamples( ...
            map_sample_pts, x(4:5), chi_d, max_map_obstacles, ...
            map_lookahead_m, map_half_width_m, map_sample_radius_m);
        obs_local = [obs_local, obs_map];
    end

    % ---- 3) Build reference trajectory ----------------------------------
    x_ref = buildObstacleAwareRef8(x, chi_d, U_d, nmpc.N, dt, ...
                                   n1_cruise, n2_cruise, obs_local);

    % ---- 4) Solve NMPC (MODIFIED - now passes u_prev) -------------------
    [u_opt, ~, info] = nmpc.solve(x, x_ref, obs_local, u_prev);

    % ---- 5) PID fallback if NMPC fails ----------------------------------
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
        fallback(i) = true;
        
        if sum(fallback(1:i)) <= 3
            fprintf('  ⚠ NMPC fail at t=%.0f s, using PID fallback\n', t(i));
        end
    end

    % ---- 6) Simulate plant (RK4) ----------------------------------------
    x_old = x;
    x = rk4Step8(x, u_opt, dt);
    
    % ... collision checks ...

    % ---- 8) Logging -----------------------------------------------------
    steps = i;
    traj(:, i+1)  = x;
    ctrl(:, i)    = u_opt;
    solve_ok(i)   = info.success;
    xte_log(i)    = xte;
    
    % Update previous control for next iteration (NEW!)
    u_prev = u_opt;

    % ---- 9) Progress print ----------------------------------------------
    if i == 1 || mod(i, 20) == 0
        d_nearest_obs = inf;
        for j = 1:length(static_obstacles)
            d_nearest_obs = min(d_nearest_obs, norm(x(4:5) - static_obstacles(j).position));
        end
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) ψ=%+6.1f° wp=%d xte=%+.1fm obs_d=%.0fm ok=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, d_nearest_obs, info.success);
    end

    % ---- 10) Check if done ----------------------------------------------
    if norm(x(4:5) - waypoints(end,:)') < R_accept
        fprintf('\n  ✓ FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

%% ═══════════════════════════════════════════════════════════════════════
%  POST-PROCESSING
%% ═══════════════════════════════════════════════════════════════════════

% Trim logs
traj     = traj(:, 1:steps+1);
ctrl     = ctrl(:, 1:steps);
solve_ok = solve_ok(1:steps);
xte_log  = xte_log(1:steps);
fallback = fallback(1:steps);
t_sim    = (0:steps) * dt;

n_ok = sum(solve_ok);  n_tot = length(solve_ok);
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  SUMMARY\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC solves: %d/%d (%.1f%%)\n', n_ok, n_tot, 100*n_ok/max(n_tot,1));
fprintf('  PID fallback: %d times\n', sum(fallback));
fprintf('  Mean |XTE|: %.1f m, Max |XTE|: %.1f m\n', mean(abs(xte_log)), max(abs(xte_log)));
fprintf('  Final position: (%.1f, %.1f)\n', traj(4,end), traj(5,end));

%% ===== Plots ============================================================
figure(1); clf;

subplot(3,2,1);
plotMapBackground(map);
hold on;
plot(traj(5,:), traj(4,:), 'g-', 'LineWidth', 2);
plot(waypoints(:,2), waypoints(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
theta_circ = linspace(0, 2*pi, 50);
for j = 1:length(static_obstacles)
    fill(static_obstacles(j).position(2) + static_obstacles(j).radius*cos(theta_circ), ...
         static_obstacles(j).position(1) + static_obstacles(j).radius*sin(theta_circ), ...
         'r', 'FaceAlpha', 0.3, 'EdgeColor', 'r', 'LineWidth', 2);
end
plot(traj(5,1), traj(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(traj(5,end), traj(4,end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('East / y [m]'); ylabel('North / x [m]'); 
title('Ship Trajectory'); grid on; axis equal;

subplot(3,2,2);
t_ctrl = (0:size(ctrl,2)-1)*dt;
plot(t_ctrl, rad2deg(ctrl(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl, rad2deg(ctrl(2,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title('Azimuth Angles'); grid on;
legend('\alpha_1 (aft)', '\alpha_2 (fwd)', 'Location', 'best');

subplot(3,2,3);
plot(t_ctrl, ctrl(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl, ctrl(4,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Commanded rpm');
title('Shaft Speed Commands'); grid on;
legend('n_{1,c}', 'n_{2,c}', 'Location', 'best');

subplot(3,2,4);
plot(t_sim, traj(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_sim, traj(8,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Actual rpm');
title('Actual Shaft Speeds'); grid on;
legend('n_1', 'n_2', 'Location', 'best');

subplot(3,2,5);
t_xte = (0:length(xte_log)-1)*dt;
plot(t_xte, xte_log, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('Cross-track Error'); grid on;

subplot(3,2,6);
plot(t_sim, traj(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Surge Velocity'); grid on;

sgtitle('NMPC Harbor Navigation — Unified Test');

%% ===== Animation ========================================================
cfg_anim = struct();
cfg_anim.figNo       = 10;
cfg_anim.testName    = 'NMPC Navigation';
cfg_anim.shipImgFile = shipImgPath;
cfg_anim.shipSize    = 0.08;
cfg_anim.maxFrames   = 200;
cfg_anim.pauseTime   = 0.03;

% Add static obstacles to animation
for j = 1:length(static_obstacles)
    cfg_anim.circObs(j).position = static_obstacles(j).position;
    cfg_anim.circObs(j).radius   = static_obstacles(j).radius;
end

traj_anim = traj(1:6, :);
animateSimResult(traj_anim, waypoints, t_sim, harbor_anim, cfg_anim);

fprintf('\nDone. Check figures.\n');


%% ════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, wp_speed, wp_idx, R_accept)
% Simple waypoint steering for 8-state model
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
        lookahead = max(60, 8 * U_now);
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
% Cross-track error
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
    x_ref = zeros(8, N+1);
    x_ref(:, 1) = x0;
    
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));
    r_d = 0.35 * psi_err;
    r_d = max(-0.10, min(0.10, r_d));
    
    for k = 2:(N+1)
        x_ref(1, k) = U_d;
        x_ref(2, k) = 0;
        x_ref(3, k) = r_d;
        x_ref(6, k) = chi_d;
        x_ref(7, k) = n1_ref;
        x_ref(8, k) = n2_ref;
        
        dx = U_d * dt * cos(chi_d);
        dy = U_d * dt * sin(chi_d);
        x_ref(4, k) = x_ref(4, k-1) + dx;
        x_ref(5, k) = x_ref(5, k-1) + dy;
    end
end     

function x_ref = buildObstacleAwareRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref, obstacles)
% Reference trajectory with obstacle deflection
    safety_margin = 60;
    x_ref = buildSimpleRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref);

    if nargin < 8 || isempty(obstacles)
        return;
    end

    fwd = [cos(chi_d); sin(chi_d)];
    perp = [-sin(chi_d); cos(chi_d)];

    for j = 1:length(obstacles)
        obs_pos = obstacles(j).position(1:2);
        obs_r   = obstacles(j).radius;

        d_vec = obs_pos - x0(4:5);
        along = dot(d_vec, fwd);
        lateral = dot(d_vec, perp);

        horizon_dist = U_d * N * dt;
        if along < 0 || along > horizon_dist + obs_r
            continue;
        end
        if abs(lateral) >= safety_margin
            continue;
        end

        if lateral >= 0
            side_sign = -1;
        else
            side_sign = +1;
        end

        deflect = side_sign * (safety_margin - lateral * side_sign);

        for k = 1:(N+1)
            s = (k-1) / N;
            s_peak = along / horizon_dist;
            s_peak = max(0.05, min(0.95, s_peak));
            w = exp(-((s - s_peak)^2) / (0.22^2));

            x_ref(4, k) = x_ref(4, k) + w * deflect * perp(1);
            x_ref(5, k) = x_ref(5, k) + w * deflect * perp(2);
        end

        if N > 1
            dx_ref = x_ref(4, N+1) - x_ref(4, 1);
            dy_ref = x_ref(5, N+1) - x_ref(5, 1);
            chi_deflected = atan2(dy_ref, dx_ref);
            psi_err2 = atan2(sin(chi_deflected - x0(6)), cos(chi_deflected - x0(6)));
            r_d2 = max(-0.10, min(0.10, 0.35 * psi_err2));
            for k = 2:(N+1)
                x_ref(3, k) = r_d2;
                x_ref(6, k) = chi_deflected;
            end
        end
    end
end

function x_next = rk4Step8(x, u_ctrl, dt_s)
% RK4 integration for 8-state container model
    x(1) = max(x(1), 0.1);
    x(7) = max(x(7), 0);
    
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

function pts = buildMapSamplePoints(map, spacing_m)
% Sample map polygon edges for obstacle queries
    pts = zeros(0,2);
    if nargin < 2 || isempty(spacing_m)
        spacing_m = 100;
    end
    if isempty(map) || ~isfield(map, 'polygons') || isempty(map.polygons)
        return;
    end

    for kk = 1:length(map.polygons)
        px = map.polygons(kk).X(:);
        py = map.polygons(kk).Y(:);

        finite = isfinite(px) & isfinite(py);
        px = px(finite);
        py = py(finite);
        if numel(px) < 3
            continue;
        end

        if px(1) ~= px(end) || py(1) ~= py(end)
            px(end+1) = px(1); %#ok<AGROW>
            py(end+1) = py(1); %#ok<AGROW>
        end

        for ii = 1:(numel(px)-1)
            p1 = [px(ii), py(ii)];
            p2 = [px(ii+1), py(ii+1)];
            seg = p2 - p1;
            seg_len = norm(seg);
            if seg_len < 1e-6
                continue;
            end
            n_samp = max(1, ceil(seg_len / spacing_m));
            a = (0:n_samp)' / n_samp;
            seg_pts = p1 + a .* seg;
            pts = [pts; seg_pts]; %#ok<AGROW>
        end
    end
end

function obs_local = selectMapObstaclesFromSamples(sample_pts, pos_xy, chi_d, max_keep, lookahead_m, half_width_m, radius_m)
% Convert local sampled map points to circle obstacles
    obs_local = struct('position', {}, 'radius', {});

    if isempty(sample_pts) || max_keep <= 0
        return;
    end

    if nargin < 5 || isempty(lookahead_m), lookahead_m = 400; end
    if nargin < 6 || isempty(half_width_m), half_width_m = 150; end
    if nargin < 7 || isempty(radius_m), radius_m = 15; end

    fwd = [cos(chi_d); sin(chi_d)];
    side = [-sin(chi_d); cos(chi_d)];

    rel = sample_pts - pos_xy(:)';
    along = rel * fwd;
    lat = rel * side;

    keep = along >= -40 & along <= lookahead_m & abs(lat) <= half_width_m;
    if ~any(keep)
        return;
    end

    cand = sample_pts(keep, :);
    dc = vecnorm((cand - pos_xy(:)'), 2, 2);
    [~, ord] = sort(dc, 'ascend');
    take = ord(1:min(max_keep, numel(ord)));
    cand = cand(take, :);

    for k = 1:size(cand,1)
        obs_local(k).position = cand(k, :)';
        obs_local(k).radius = radius_m;
    end
end
