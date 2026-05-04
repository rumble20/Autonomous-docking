%% run_nmpc.m
% Pure line-tracking / tube-MPC harbor navigation
%
% Main changes in this clean version:
%   - No point-tracking reference matrix is sent to NMPC.
%   - NMPC receives only the active path segment [wp_start -> wp_end].
%   - Cross-track error is penalized only outside a tube corridor.
%   - No external speed governor. Speed is left to NMPC + obstacle CBFs.
%   - No external reference shaping / XTE recapture logic.

clear; close all; clc;
clear animateSimResult
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC HARBOR NAVIGATION — LINE TRACKING + TUBE MPC\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% USER CONFIGURATION =====================================================
% Waypoints: rows = [x, y, heading_deg].
% If final heading is finite, it is used only as terminal heading target.
waypoints = [-3000, -2600, NaN; -2600, -2700, NaN; -2350, -2600, NaN; -2200, -2650, NaN; -2050, -2600, 0];

% Cruise-speed suggestion for the NMPC soft speed cost.
cruise_speed_mps = 5.0;

% Static obstacles: N-by-3 [x y radius] or struct array.
static_obstacles = [];

% Dynamic obstacles.
dynamic_obs_positions_xy = [-2300, -2800];
dynamic_obs_headings_deg = [135];
dynamic_obs_speeds_mps   = [3];
enable_dynamic_obstacles = false;
dynamic_obs_radius_m     = 25;
dynamic_obs_speed_mps    = 5;
dynamic_obs_nmpc_guard_m = 0;
dynamic_obs_start_mode   = 'proximity';   % immediate | proximity
dynamic_obs_trigger_distance_m = 400;
dynamic_obs_boundary_margin = 300;
dynamic_obs_boundary_policy = 'wrap';     % wrap | bounce | hold

% Dynamic latent awareness (optional planning-only envelopes).
dynamic_latent_awareness = struct();
dynamic_latent_awareness.enabled = false;
dynamic_latent_awareness.n_samples = 0;
dynamic_latent_awareness.dt_s = 2.0;
dynamic_latent_awareness.inflate_radius_m = 0.0;
dynamic_latent_awareness.sample_radius_gain = 0.0;

% Simulation.
T_final = 1000;
R_accept = 90;
R_accept_final = 25;
R_accept_final_soft = 65;
final_capture_speed_mps = 2.5;
final_capture_hold_s = 6;
n1_cruise = 100;
n2_cruise = 100;
n3_cruise = 0;

% Hull footprint.
hull_nominal_length_m = 175;
hull_nominal_beam_m   = 25.4;
hull_scale            = 0.5;

% Animation / logging.
enable_animation_recording = true;
record_fps = 30;
record_output_dir = 'MY_NPMC\my work\plots in development process\recordings';
log_output_dir = 'MY_NPMC\my work\plots in development process\logs';
enable_terminal_log_recording = true;
terminal_log_output_dir = log_output_dir;

% Optional map-aware terminal success gate.
enable_safe_terminal_stop = true;
safe_terminal_radius_m = 50;
safe_terminal_speed_mps = 1.2;
safe_terminal_hold_s = 5;

% Map obstacle detection.
enable_map_obstacles = true;
map_obstacle_model = 'halfplane';  % halfplane | circle
max_map_obstacles = 8;
map_sample_radius_m = 20;
map_edge_spacing_m = 60;
map_include_interior_samples = false;
map_interior_spacing_m = 120;
max_map_halfplanes_transit = 6;
map_halfplane_min_edge_m = 15;
map_lookahead_time_s = 75;
map_lookahead_min_m  = 420;
map_lookahead_max_m  = 900;
map_half_width_min_m = 150;
map_half_width_max_m = 420;

% NMPC core setup.
nmpc_N  = 50;
nmpc_dt = 1.0;
r_safety = 40;
max_brake_rate = 0.4;
actuator_force_weight = 0.015;
forward_incentive_weight = 2.0;
u_min_forward = 0.3;
u_min_final_reverse_mps = -1.20;

% Tube-MPC path cost.
path_cost_cfg = struct();
path_cost_cfg.W_xte_heavy = 12.0;
path_cost_cfg.W_along = 1.2;
path_cost_cfg.W_tube_m = 20.0;
path_cost_cfg.soft_speed_cap_weight = 0.20;
path_cost_cfg.soft_speed_cap_mps = cruise_speed_mps;

% Final waypoint terminal cost.
terminal_goal_cfg = struct();
terminal_goal_cfg.pos_weight = 140.0;
terminal_goal_cfg.heading_weight = 60.0;
terminal_goal_cfg.stop_u_weight = 45.0;
terminal_goal_cfg.stop_v_weight = 28.0;
terminal_goal_cfg.stop_r_weight = 28.0;
terminal_goal_cfg.term_pose_eps_xy_m = [max(1.5, R_accept_final); max(1.5, R_accept_final)];
terminal_goal_cfg.term_pose_eps_psi_deg = 5.0;
terminal_goal_cfg.term_vel_max_u_mps = max(0.6, final_capture_speed_mps);
terminal_goal_cfg.term_vel_max_v_mps = 0.8;
terminal_goal_cfg.term_vel_max_r_radps = deg2rad(6);
terminal_goal_cfg.term_pose_slack_max = 2.0;

% Soft obstacle slack.
soft_obstacle_cfg = struct();
soft_obstacle_cfg.enabled = true;
soft_obstacle_cfg.max_slack_m = 10.0;
soft_obstacle_cfg.penalty_weight = 2.5e5;

% Twin-stern synchrony.
azipod_sync_cfg = struct();
azipod_sync_cfg.transit_alpha_split_rad = deg2rad(60);
azipod_sync_cfg.transit_stern_split_rpm = 60;
azipod_sync_cfg.final_alpha_split_rad   = deg2rad(20);
azipod_sync_cfg.final_stern_split_rpm   = 20;

% PID fallback.
pid_Kp = 1.8;
pid_Ki = 0.02;
pid_Kd = 1.2;

%% NORMALIZE / SETUP ======================================================
static_obstacles = normalizeStaticObstacles(static_obstacles);
scriptDir = fileparts(mfilename('fullpath'));
repoRoot = pwd;
if ~isfolder(fullfile(repoRoot, 'useful pictures'))
    repoRoot = fileparts(fileparts(scriptDir));
end

if ~isfolder(log_output_dir)
    log_output_dir = fullfile(repoRoot, log_output_dir);
end
if ~isfolder(record_output_dir)
    record_output_dir = fullfile(repoRoot, record_output_dir);
end
if ~isfolder(terminal_log_output_dir)
    terminal_log_output_dir = fullfile(repoRoot, terminal_log_output_dir);
end

if enable_terminal_log_recording
    if ~exist(terminal_log_output_dir, 'dir')
        mkdir(terminal_log_output_dir);
    end
    terminal_log_file = fullfile(terminal_log_output_dir, ...
        ['run_nmpc_line_tube_' datestr(now, 'yyyymmdd_HHMMSS') '.txt']);
    diary(terminal_log_file);
    diary on;
end

fprintf('  Waypoints: ');
for iwp = 1:size(waypoints, 1)
    fprintf('(%d, %d) ', waypoints(iwp,1), waypoints(iwp,2));
end
fprintf('\n');

shipImgPath = fullfile(repoRoot, 'useful pictures', 'vessel_top.png');
if ~isfile(shipImgPath)
    warning('Ship image not found. Animation will use fallback shape.');
end

%% MAP LOAD ===============================================================
map = [];
mapCandidates = {
    fullfile(repoRoot, 'MY_NPMC', 'helsinki_harbour_UPDATED.mat'), ...
    fullfile(repoRoot, 'MY_NPMC', 'helsinki_harbour.mat'), ...
    fullfile(scriptDir, 'helsinki_harbour_UPDATED.mat'), ...
    fullfile(scriptDir, 'helsinki_harbour.mat'), ...
    'helsinki_harbour_UPDATED.mat', ...
    'helsinki_harbour.mat'
};
for kMap = 1:numel(mapCandidates)
    if isfile(mapCandidates{kMap})
        S = load(mapCandidates{kMap});
        if isfield(S, 'map')
            map = S.map;
            break;
        end
    end
end

harbor_anim = [];
if ~isempty(map)
    harbor_anim = HarborAnimHelper(map);
    fprintf('  Harbor map loaded: %d polygons\n', length(map.polygons));
end

map_sample_pts = [];
map_edge_set = [];
final_is_map_constrained = false;
if enable_map_obstacles && ~isempty(map)
    map_edge_set = buildMapHalfPlaneEdgeSet(map, map_halfplane_min_edge_m);
    fprintf('  Map edges (half-plane candidates): %d\n', length(map_edge_set));

    if strcmpi(strtrim(map_obstacle_model), 'circle')
        map_sample_pts = buildMapSamplePoints(map, map_edge_spacing_m, ...
            map_include_interior_samples, map_interior_spacing_m);
        fprintf('  Map sample points (circle mode): %d\n', size(map_sample_pts, 1));
    end

    if isfield(NavUtils, 'isInsideAnyMapZone')
        [in_final_zone, ~, ~] = NavUtils.isInsideAnyMapZone(waypoints(end,1:2)', map);
        final_is_map_constrained = in_final_zone;
    end
end

%% DYNAMIC OBSTACLES ======================================================
map_bounds = estimateMapBounds(map, waypoints, dynamic_obs_boundary_margin);
dynamic_obstacles = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {}, 'active', {}, 'enabled', {}, 'id', {});
if enable_dynamic_obstacles
    dynamic_obstacles = buildDynamicObstaclesFromConfig( ...
        dynamic_obs_positions_xy, dynamic_obs_headings_deg, dynamic_obs_speeds_mps, ...
        dynamic_obs_radius_m, dynamic_obs_speed_mps);
    dynamic_obstacles = configureDynamicStartMode( ...
        dynamic_obstacles, dynamic_obs_start_mode, dynamic_obs_trigger_distance_m);
    fprintf('  Dynamic obstacles enabled: %d\n', length(dynamic_obstacles));
else
    fprintf('  Dynamic obstacles disabled\n');
end

%% NMPC CONFIGURATION =====================================================
hull_cfg = buildHullFootprintConfig(hull_nominal_length_m, hull_nominal_beam_m, hull_scale, r_safety);

n_static_obs = length(static_obstacles);
n_dynamic_obs = 0;
n_dynamic_obs_virtual = 0;
if enable_dynamic_obstacles
    n_dynamic_obs = length(dynamic_obstacles);
    if isfield(dynamic_latent_awareness, 'enabled') && dynamic_latent_awareness.enabled
        n_samp = max(0, round(getOr(dynamic_latent_awareness, 'n_samples', 0)));
        n_dynamic_obs_virtual = n_dynamic_obs * n_samp;
    end
end
use_map_circles = enable_map_obstacles && strcmpi(strtrim(map_obstacle_model), 'circle') && ~isempty(map_sample_pts);
use_map_halfplanes = enable_map_obstacles && strcmpi(strtrim(map_obstacle_model), 'halfplane') && ~isempty(map_edge_set);

if use_map_circles
    max_obs_slots = n_static_obs + max_map_obstacles + n_dynamic_obs + n_dynamic_obs_virtual;
else
    max_obs_slots = n_static_obs + n_dynamic_obs + n_dynamic_obs_virtual;
end
if use_map_halfplanes
    max_hp_slots = max(0, round(max_map_halfplanes_transit));
else
    max_hp_slots = 0;
end

nmpc_cfg = struct();
nmpc_cfg.N  = nmpc_N;
nmpc_cfg.dt = nmpc_dt;
nmpc_cfg.Q  = diag([0.0, 0.04, 0.08, 0, 0, 0, 0.001, 0.001, 0.001]);
nmpc_cfg.R  = diag([0.10, 0.10, 0.006, 0.006, 0.010]);
nmpc_cfg.R_rate = diag([0.12, 0.12, 0.004, 0.004, 0.010]);
nmpc_cfg.max_obs = max(1, max_obs_slots);
nmpc_cfg.max_halfplanes = max_hp_slots;
nmpc_cfg.r_safety = r_safety;
nmpc_cfg.collision_model = 'oriented-rectangle';
nmpc_cfg.hull_length_m = hull_cfg.length_m;
nmpc_cfg.hull_beam_m = hull_cfg.beam_m;
nmpc_cfg.hull_clearance_m = hull_cfg.nmpc_clearance_m;
nmpc_cfg.enable_diagnostics = false;
nmpc_cfg.actuator_force_weight = actuator_force_weight;
nmpc_cfg.forward_incentive_weight = forward_incentive_weight;
nmpc_cfg.u_min_forward = u_min_forward;
nmpc_cfg.max_brake_rate = max_brake_rate;
nmpc_cfg.soft_obs_weight = soft_obstacle_cfg.penalty_weight;
nmpc_cfg.soft_obs_default_max_m = 0.0;
nmpc_cfg.path_xte_weight_default = path_cost_cfg.W_xte_heavy;
nmpc_cfg.path_along_weight_default = path_cost_cfg.W_along;
nmpc_cfg.path_tube_half_width_default = path_cost_cfg.W_tube_m;
nmpc_cfg.soft_speed_cap_default_mps = path_cost_cfg.soft_speed_cap_mps;
nmpc_cfg.soft_speed_cap_weight_default = path_cost_cfg.soft_speed_cap_weight;
nmpc_cfg.terminal_goal_pos_weight_default = terminal_goal_cfg.pos_weight;
nmpc_cfg.terminal_goal_heading_weight_default = terminal_goal_cfg.heading_weight;
nmpc_cfg.terminal_stop_u_weight_default = terminal_goal_cfg.stop_u_weight;
nmpc_cfg.terminal_stop_v_weight_default = terminal_goal_cfg.stop_v_weight;
nmpc_cfg.terminal_stop_r_weight_default = terminal_goal_cfg.stop_r_weight;

fprintf('\n--- Building clean line-tracking tube-NMPC (%d obstacle slots, %d half-plane slots) ---\n', ...
    nmpc_cfg.max_obs, nmpc_cfg.max_halfplanes);
fprintf('  Collision model: oriented rectangle %.1f m x %.1f m\n', hull_cfg.length_m, hull_cfg.beam_m);
fprintf('  Path cost: XTE tube %.1f m | W_xte %.2f | W_along %.2f\n', ...
    path_cost_cfg.W_tube_m, path_cost_cfg.W_xte_heavy, path_cost_cfg.W_along);
fprintf('  External speed governor: OFF\n');
nmpc = NMPC_Container_final(nmpc_cfg);
nmpc.buildSolver();

%% INITIAL STATE / LOGGING ================================================
x0_heading = atan2(waypoints(2,2) - waypoints(1,2), ...
                   waypoints(2,1) - waypoints(1,1));
x = [7; 0; 0; waypoints(1,1); waypoints(1,2); x0_heading; n1_cruise; n2_cruise; n3_cruise];

dt = nmpc_cfg.dt;
t  = 0:dt:T_final;
wp_idx = 1;
final_capture_count = 0;
final_capture_steps_needed = max(1, ceil(final_capture_hold_s / dt));
safe_terminal_count = 0;
safe_terminal_steps_needed = max(1, ceil(safe_terminal_hold_s / dt));

traj     = zeros(9, length(t)+1);
ctrl     = zeros(5, length(t));
solve_ok = false(1, length(t));
xte_log  = zeros(1, length(t));
fallback = false(1, length(t));
X_pred_hist = cell(1, length(t));
step_time_log     = nan(1, length(t));
guide_time_log    = nan(1, length(t));
obs_time_log      = nan(1, length(t));
ref_time_log      = nan(1, length(t));
solve_call_log    = nan(1, length(t));
solve_time_log    = nan(1, length(t));
integr_time_log   = nan(1, length(t));
rt_ratio_log      = nan(1, length(t));
n_obs_log         = nan(1, length(t));
cost_log          = nan(1, length(t));
obs_pack_drift_log = nan(1, length(t));
collision_log     = false(1, length(t));
brake_margin_log  = nan(1, length(t));
psi_ref_log       = nan(1, length(t));
heading_err_log   = nan(1, length(t));
wp_idx_log        = nan(1, length(t));
soft_slack_max_log = nan(1, length(t));
soft_slack_sum_log = nan(1, length(t));
terminal_pose_slack_max_log = nan(1, length(t));
az_split_limit_log = nan(1, length(t));
stern_split_limit_log = nan(1, length(t));
traj(:,1) = x;
steps = 0;

n_dyn = length(dynamic_obstacles);
dyn_obs_hist = nan(n_dyn, 2, length(t)+1);
if n_dyn > 0
    for jj = 1:n_dyn
        dyn_obs_hist(jj, :, 1) = dynamic_obstacles(jj).position(1:2)';
    end
end

psi_err_int = 0;
psi_err_prev = 0;
u_prev = [0; 0; n1_cruise; n2_cruise; n3_cruise];
u_prev_ship = x(1);

%% MAIN SIMULATION LOOP ===================================================
for i = 1:length(t)
    t_step = tic;

    % 1) Update active segment index only, then use plain segment heading
    t_seg = tic;
    wp_idx = updateWaypointIndexOnly(x, waypoints, wp_idx, R_accept);

    xte = computeXTE(x, waypoints, wp_idx);
    n_wps = size(waypoints, 1);

    seg_start_idx = min(max(1, wp_idx), n_wps-1);
    seg_end_idx   = seg_start_idx + 1;

    wp_start_xy = waypoints(seg_start_idx, 1:2)';
    wp_end_xy   = waypoints(seg_end_idx,   1:2)';

    seg_vec = wp_end_xy - wp_start_xy;
    if norm(seg_vec) > 1e-9
        chi_seg = atan2(seg_vec(2), seg_vec(1));
    else
        chi_seg = x(6);
    end

    on_final_waypoint = (seg_end_idx == n_wps);

    goal_heading_enable = false;
    goal_heading_rad = chi_seg;
    if on_final_waypoint && size(waypoints,2) >= 3 && isfinite(waypoints(end,3))
        goal_heading_enable = true;
        goal_heading_rad = deg2rad(waypoints(end,3));
    end

    path_ref = struct();
    path_ref.wp_start = wp_start_xy;
    path_ref.wp_end = wp_end_xy;
    path_ref.goal_heading_rad = goal_heading_rad;
    path_ref.goal_heading_enable = goal_heading_enable;

    guide_time_log(i) = toc(t_seg);

    % 2) Dynamic obstacle propagation
    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        if strcmpi(strtrim(dynamic_obs_start_mode), 'proximity')
            dynamic_obstacles = activateDynamicObstaclesByProximity(dynamic_obstacles, x(4:5));
        end
        if i > 1
            dynamic_obstacles = propagateDynamicObstacles(dynamic_obstacles, dt, map_bounds, dynamic_obs_boundary_policy);
        end
    end
    if ~isempty(dynamic_obstacles)
        for jj = 1:length(dynamic_obstacles)
            dyn_obs_hist(jj, :, i) = dynamic_obstacles(jj).position(1:2)';
        end
    end

    % 3) Gather obstacles / half-planes
    t_seg = tic;
    obs_local = static_obstacles;
    obs_map = struct('position', {}, 'radius', {});
    map_halfplanes = struct('normal', {}, 'offset', {});

    if enable_map_obstacles && (~isempty(map_sample_pts) || ~isempty(map_edge_set))
        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead_now = min(map_lookahead_max_m, max(map_lookahead_min_m, U_now * map_lookahead_time_s));
        half_width_now = min(map_half_width_max_m, max(map_half_width_min_m, 0.45 * lookahead_now));

        if use_map_halfplanes
            map_halfplanes = selectMapHalfPlanesFromEdges( ...
                map_edge_set, x(4:5), chi_seg, max_hp_slots, lookahead_now, half_width_now, [], 0, 1.0);
        elseif use_map_circles
            obs_map = selectMapObstaclesFromSamples( ...
                map_sample_pts, x(4:5), chi_seg, max_map_obstacles, ...
                lookahead_now, half_width_now, map_sample_radius_m);
            obs_local = [obs_local, obs_map];
        end
    end

    obs_dyn = struct('position', {}, 'radius', {}, 'velocity', {});
    obs_dyn_latent = struct('position', {}, 'radius', {}, 'velocity', {});
    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        obs_dyn = dynamicToCircleObstacles(dynamic_obstacles, dynamic_obs_nmpc_guard_m);
        if isfield(dynamic_latent_awareness, 'enabled') && dynamic_latent_awareness.enabled
            obs_dyn_latent = buildLatentDynamicAwarenessObstacles(dynamic_obstacles, x(4:5), dynamic_latent_awareness);
        end
        for kk = 1:length(obs_dyn)
            if isfield(dynamic_obstacles(kk), 'speed') && isfield(dynamic_obstacles(kk), 'heading')
                obs_dyn(kk).velocity = dynamic_obstacles(kk).speed * ...
                    [cos(dynamic_obstacles(kk).heading); sin(dynamic_obstacles(kk).heading)];
            else
                obs_dyn(kk).velocity = [0; 0];
            end
        end
        for kk = 1:length(obs_dyn_latent)
            obs_dyn_latent(kk).velocity = [0; 0];
        end
        obs_local = [obs_local, obs_dyn_latent, obs_dyn];
        obs_pack_drift_log(i) = computeDynamicPackagingDrift(dynamic_obstacles, obs_local);
    end
    obs_time_log(i) = toc(t_seg);

    % 4) Solve options: pure segment cost + tube cost + soft cruise cap
    t_seg = tic;
    solve_opts = struct();
    solve_opts.state_weights_diag = diag(nmpc_cfg.Q);
    solve_opts.input_weights_diag = diag(nmpc_cfg.R);
    solve_opts.rate_weights_diag = diag(nmpc_cfg.R_rate);
    solve_opts.stage_state_cost_scale = 1.0;
    solve_opts.stage_input_tracking_scale = 1.0;
    solve_opts.terminal_cost_scale = 1.0;
    solve_opts.terminal_actuator_cost_scale = 1.0;
    solve_opts.terminal_forward_cost_scale = 1.0;
    solve_opts.collision_clearance_m = hull_cfg.nmpc_clearance_m;
    solve_opts.path_xte_weight = path_cost_cfg.W_xte_heavy;
    solve_opts.path_along_weight = path_cost_cfg.W_along;
    solve_opts.path_tube_half_width_m = path_cost_cfg.W_tube_m;
    solve_opts.soft_speed_cap_weight = path_cost_cfg.soft_speed_cap_weight;
    solve_opts.soft_speed_cap_mps = path_cost_cfg.soft_speed_cap_mps;
    solve_opts.goal_heading_enable = goal_heading_enable;
    solve_opts.goal_heading_rad = goal_heading_rad;
    solve_opts.is_final_waypoint = on_final_waypoint;
    solve_opts.enable_soft_obstacles = soft_obstacle_cfg.enabled;
    solve_opts.soft_obs_max_m = soft_obstacle_cfg.max_slack_m;
    solve_opts.map_halfplanes = map_halfplanes;

    if on_final_waypoint
        solve_opts.terminal_goal_pos_weight = terminal_goal_cfg.pos_weight;
        solve_opts.terminal_goal_heading_weight = terminal_goal_cfg.heading_weight * double(goal_heading_enable);
        solve_opts.terminal_stop_u_weight = terminal_goal_cfg.stop_u_weight;
        solve_opts.terminal_stop_v_weight = terminal_goal_cfg.stop_v_weight;
        solve_opts.terminal_stop_r_weight = terminal_goal_cfg.stop_r_weight;
        solve_opts.enable_terminal_pose = true;
        solve_opts.term_pose_eps_xy_m = terminal_goal_cfg.term_pose_eps_xy_m;
        if goal_heading_enable
            solve_opts.term_pose_eps_psi_rad = deg2rad(terminal_goal_cfg.term_pose_eps_psi_deg);
        else
            solve_opts.term_pose_eps_psi_rad = pi;
        end
        solve_opts.term_vel_max_u_mps = terminal_goal_cfg.term_vel_max_u_mps;
        solve_opts.term_vel_max_v_mps = terminal_goal_cfg.term_vel_max_v_mps;
        solve_opts.term_vel_max_r_radps = terminal_goal_cfg.term_vel_max_r_radps;
        solve_opts.term_pose_slack_max = terminal_goal_cfg.term_pose_slack_max;
        solve_opts.n3_max = nmpc.n_bow_max;
        solve_opts.max_azimuth_split = azipod_sync_cfg.final_alpha_split_rad;
        solve_opts.max_stern_cmd_split = azipod_sync_cfg.final_stern_split_rpm;
        desired_u_min_forward = u_min_final_reverse_mps;
    else
        solve_opts.enable_terminal_pose = false;
        solve_opts.terminal_goal_pos_weight = 30.0;
        solve_opts.terminal_goal_heading_weight = 0.0;
        solve_opts.terminal_stop_u_weight = 0.0;
        solve_opts.terminal_stop_v_weight = 0.0;
        solve_opts.terminal_stop_r_weight = 0.0;
        solve_opts.n3_max = 0;
        solve_opts.max_azimuth_split = azipod_sync_cfg.transit_alpha_split_rad;
        solve_opts.max_stern_cmd_split = azipod_sync_cfg.transit_stern_split_rpm;
        desired_u_min_forward = u_min_forward;
    end
    az_split_limit_log(i) = solve_opts.max_azimuth_split;
    stern_split_limit_log(i) = solve_opts.max_stern_cmd_split;
    ref_time_log(i) = toc(t_seg);

    % 5) Solve NMPC
    t_seg = tic;
    [u_opt, X_pred, info] = nmpc.solve(x, path_ref, obs_local, u_prev, desired_u_min_forward, solve_opts);
    solve_call_log(i) = toc(t_seg);
    solve_time_log(i) = getOr(info, 'solve_time', solve_call_log(i));
    n_obs_log(i) = getOr(info, 'n_obs_real', length(obs_local));
    if isfield(info, 'cost'), cost_log(i) = info.cost; end
    if isfield(info, 'max_soft_slack_m'), soft_slack_max_log(i) = info.max_soft_slack_m; end
    if isfield(info, 'sum_soft_slack_m'), soft_slack_sum_log(i) = info.sum_soft_slack_m; end
    if isfield(info, 'max_terminal_slack'), terminal_pose_slack_max_log(i) = info.max_terminal_slack; end
    X_pred_hist{i} = X_pred;

    % 6) PID fallback if solver fails
    if ~info.success
        psi_err = wrapToPi(chi_seg - x(6));
        psi_err_int = psi_err_int + psi_err * dt;
        psi_err_int = max(-1, min(1, psi_err_int));
        psi_err_dot = (psi_err - psi_err_prev) / dt;
        psi_err_prev = psi_err;
        alpha = pid_Kp * psi_err + pid_Ki * psi_err_int + pid_Kd * psi_err_dot;
        alpha = max(-pi/4, min(pi/4, alpha));
        n1_cmd = n1_cruise;
        u_opt = [alpha; alpha; n1_cmd; n1_cmd; 0];
        fallback(i) = true;
        if sum(fallback(1:i)) <= 3
            fprintf('  ⚠ NMPC fail at t=%.0f s, using PID fallback\n', t(i));
        end
    end

    % 7) Plant integration
    t_seg = tic;
    x = rk4Step9(x, u_opt, dt);
    integr_time_log(i) = toc(t_seg);
    du_surge = x(1) - u_prev_ship;
    brake_margin_log(i) = du_surge + max_brake_rate * dt;
    u_prev_ship = x(1);
    step_time_log(i) = toc(t_step);
    rt_ratio_log(i) = step_time_log(i) / max(dt, 1e-9);

    % 8) Collision checks
    [hit_static, ~, ~] = detectHullCircleHit(x, static_obstacles, hull_cfg, 0.0);
    [hit_map_samples, ~, ~] = detectHullCircleHit(x, obs_map, hull_cfg, 0.0);
    [hit_dyn_latent, ~, ~] = detectHullCircleHit(x, obs_dyn_latent, hull_cfg, 0.0);
    [hit_dyn_real, ~, ~] = detectHullCircleHit(x, obs_dyn, hull_cfg, 0.0);
    hit_obs = hit_static || hit_dyn_real;
    [hit_map, ~, ~] = detectHullMapHit(x, hull_cfg, map);
    if hit_obs || hit_map
        collision_log(i) = true;
        fprintf(['  [COLLISION] t=%.1f s hit_obs=%d hit_map=%d ', ...
                 '(static=%d map_samples=%d dyn_latent=%d dyn_real=%d)\n'], ...
            t(i), hit_obs, hit_map, hit_static, hit_map_samples, hit_dyn_latent, hit_dyn_real);
        steps = i;
        traj(:, i+1) = x;
        ctrl(:, i) = u_opt;
        solve_ok(i) = info.success;
        xte_log(i) = xte;
        u_prev = u_opt;
        break;
    end

    % 9) Logging
    steps = i;
    traj(:, i+1) = x;
    ctrl(:, i) = u_opt;
    solve_ok(i) = info.success;
    xte_log(i) = xte;
    heading_err_log(i) = wrapToPi(chi_seg - x(6));
    wp_idx_log(i) = wp_idx;
    psi_ref_log(i) = chi_seg;
    u_prev = u_opt;

    % 10) Progress print
    if i == 1 || mod(i, 20) == 0 || ~info.success || rt_ratio_log(i) > 1.0
        d_nearest_obs = inf;
        for j = 1:length(obs_local)
            d_nearest_obs = min(d_nearest_obs, norm(x(4:5) - obs_local(j).position));
        end
        mode_str = 'TRANSIT';
        if on_final_waypoint, mode_str = 'FINAL'; end
        fprintf(['  [t=%5.1f] mode=%s pos=(%7.1f,%6.1f) psi=%+6.1fdeg wp=%d xte=%+.1fm ', ...
                 'obs_d=%.0fm comp=%.1fms RT=%.2f obs=%d\n'], ...
            t(i), mode_str, x(4), x(5), rad2deg(x(6)), wp_idx, xte, d_nearest_obs, ...
            1e3*step_time_log(i), rt_ratio_log(i), round(n_obs_log(i)));
    end

    % 11) Mission complete
    d_final_now = norm(x(4:5) - waypoints(end,1:2)');
    U_now_ship = hypot(x(1), x(2));
    hard_final_hit = (d_final_now < R_accept_final);
    soft_final_hit = (d_final_now < R_accept_final_soft) && (U_now_ship <= final_capture_speed_mps);
    safe_terminal_hit = enable_safe_terminal_stop && final_is_map_constrained && ...
        (d_final_now < safe_terminal_radius_m) && (U_now_ship <= safe_terminal_speed_mps);

    if hard_final_hit || soft_final_hit
        final_capture_count = final_capture_count + 1;
    else
        final_capture_count = 0;
    end
    if safe_terminal_hit
        safe_terminal_count = safe_terminal_count + 1;
    else
        safe_terminal_count = 0;
    end

    if hard_final_hit || final_capture_count >= final_capture_steps_needed || ...
            safe_terminal_count >= safe_terminal_steps_needed
        if hard_final_hit
            fprintf('\n  ✓ FINAL WAYPOINT REACHED (hard gate %.1f m) at t=%.1f s!\n', R_accept_final, t(i));
        elseif safe_terminal_count >= safe_terminal_steps_needed
            fprintf(['\n  ✓ FINAL WAYPOINT SAFELY CAPTURED (map-constrained goal): ', ...
                     'radius %.1f m, U<=%.1f m/s for %.1f s at t=%.1f s!\n'], ...
                safe_terminal_radius_m, safe_terminal_speed_mps, safe_terminal_hold_s, t(i));
        else
            fprintf('\n  ✓ FINAL WAYPOINT CAPTURED (soft gate %.1f m, U<=%.1f m/s for %.1f s) at t=%.1f s!\n', ...
                R_accept_final_soft, final_capture_speed_mps, final_capture_hold_s, t(i));
        end
        break;
    end
end

%% TRIM LOGS ==============================================================
if ~exist('steps', 'var') || isempty(steps) || steps < 1
    fprintf('⚠ No simulation steps completed. Skipping output generation.\n');
    if enable_terminal_log_recording
        diary off;
    end
    return;
end

traj     = traj(:, 1:steps+1);
ctrl     = ctrl(:, 1:steps);
solve_ok = solve_ok(1:steps);
xte_log  = xte_log(1:steps);
fallback = fallback(1:steps);
t_sim    = (0:steps) * dt;
X_pred_hist = X_pred_hist(1:steps);
step_time_log   = step_time_log(1:steps);
guide_time_log  = guide_time_log(1:steps);
obs_time_log    = obs_time_log(1:steps);
ref_time_log    = ref_time_log(1:steps);
solve_call_log  = solve_call_log(1:steps);
solve_time_log  = solve_time_log(1:steps);
integr_time_log = integr_time_log(1:steps);
rt_ratio_log    = rt_ratio_log(1:steps);
n_obs_log       = n_obs_log(1:steps);
cost_log        = cost_log(1:steps);
obs_pack_drift_log = obs_pack_drift_log(1:steps);
collision_log   = collision_log(1:steps);
brake_margin_log = brake_margin_log(1:steps);
psi_ref_log = psi_ref_log(1:steps);
heading_err_log = heading_err_log(1:steps);
wp_idx_log = wp_idx_log(1:steps);
soft_slack_max_log = soft_slack_max_log(1:steps);
soft_slack_sum_log = soft_slack_sum_log(1:steps);
terminal_pose_slack_max_log = terminal_pose_slack_max_log(1:steps);
az_split_limit_log = az_split_limit_log(1:steps);
stern_split_limit_log = stern_split_limit_log(1:steps);

%% SUMMARY / OUTPUT =======================================================
output_gen_error = false;
try
    n_ok = sum(solve_ok);
    n_tot = length(solve_ok);
    fprintf('\n══════════════════════════════════════════════════════════════\n');
    fprintf('  SUMMARY\n');
    fprintf('══════════════════════════════════════════════════════════════\n');
    fprintf('  NMPC solves: %d/%d (%.1f%%)\n', n_ok, n_tot, 100*n_ok/max(n_tot,1));
    fprintf('  PID fallback: %d times\n', sum(fallback));
    fprintf('  Mean |XTE|: %.1f m, Max |XTE|: %.1f m\n', mean(abs(xte_log)), max(abs(xte_log)));
    fprintf('  Final position: (%.1f, %.1f)\n', traj(4,end), traj(5,end));
    fprintf('  Final heading: %.1f deg\n', rad2deg(traj(6,end)));
    fprintf('  Collisions detected: %d\n', sum(collision_log));
    if any(isfinite(obs_pack_drift_log))
        fprintf('  Dynamic packaging drift [m]: max=%.3f\n', max(obs_pack_drift_log(isfinite(obs_pack_drift_log))));
    end
    if any(isfinite(soft_slack_max_log))
        fprintf('  Soft obstacle slack [m]: max=%.3f, cumulative=%.3f\n', ...
            max(soft_slack_max_log(isfinite(soft_slack_max_log))), ...
            sum(soft_slack_sum_log(isfinite(soft_slack_sum_log))));
    end
    if any(isfinite(terminal_pose_slack_max_log))
        fprintf('  Terminal pose slack [m]: max=%.3f\n', max(terminal_pose_slack_max_log(isfinite(terminal_pose_slack_max_log))));
    end
    if any(isfinite(brake_margin_log))
        fprintf('  Brake-rate margin [m/s]: min=%.4f, violations=%d\n', ...
            min(brake_margin_log(isfinite(brake_margin_log))), sum(brake_margin_log < -1e-6));
    end

    valid_step = isfinite(step_time_log);
    valid_solve = isfinite(solve_time_log);
    n_overrun = sum(step_time_log(valid_step) > dt);
    fprintf('\n  REAL-TIME FEASIBILITY (dt = %.3f s)\n', dt);
    fprintf('  Step compute [ms]: mean=%.2f, p95=%.2f, max=%.2f\n', ...
        1e3*mean(step_time_log(valid_step)), ...
        1e3*safePercentile(step_time_log(valid_step), 95), ...
        1e3*max(step_time_log(valid_step)));
    fprintf('  NMPC solve  [ms]: mean=%.2f, p95=%.2f, max=%.2f\n', ...
        1e3*mean(solve_time_log(valid_solve)), ...
        1e3*safePercentile(solve_time_log(valid_solve), 95), ...
        1e3*max(solve_time_log(valid_solve)));
    fprintf('  RT overruns (step_time > dt): %d/%d (%.2f%%)\n', ...
        n_overrun, steps, 100*n_overrun/max(steps,1));

    if exist('animateSimResult', 'file') == 2
        try
            anim_cfg = struct();
            anim_cfg.testName = 'Line Tracking + Tube MPC';
            anim_cfg.shipImgFile = shipImgPath;
            anim_cfg.hullCfg = hull_cfg;
            anim_cfg.plannedRoutes = X_pred_hist;
            anim_cfg.thrusterHistory = ctrl;
            anim_cfg.dynamicObsHistory = dyn_obs_hist(:, :, 1:steps+1);
            anim_cfg.dynamicObsRadius = dynamic_obs_radius_m;
            anim_cfg.circObs = static_obstacles;

            if enable_animation_recording
                if ~exist(record_output_dir, 'dir')
                    mkdir(record_output_dir);
                end
                ts = datestr(now, 'yyyymmdd_HHMMSS');
                anim_cfg.recordVideo = true;
                anim_cfg.recordFps = record_fps;
                anim_cfg.videoFile = fullfile(record_output_dir, ['run_nmpc' ts '.mp4']);
            end

            animateSimResult(traj, waypoints(:,1:2), t_sim, harbor_anim, anim_cfg);
        catch ME_anim
            warning(ME_anim.identifier, ': Animation failed: %s', ME_anim.message);
        end
    end
catch ME
    output_gen_error = true;
    warning(ME.identifier, ': Output generation failed: %s', ME.message);
end

if enable_terminal_log_recording
    diary off;
end

if output_gen_error
    fprintf('  Output generation had errors, but simulation completed.\n');
end

%% LOCAL FUNCTIONS ========================================================


function wp_idx = updateWaypointIndexOnly(x, wp, wp_idx, R_accept)
% Update only the active segment index.
% No heading generation. No speed shaping. No recapture logic.

    n_wps = size(wp, 1);
    if n_wps <= 1
        wp_idx = 1;
        return;
    end

    last_seg_idx = n_wps - 1;
    wp_idx = min(max(1, wp_idx), last_seg_idx);

    pos = [x(4); x(5)];

    % Allow at most one advance per control step.
    while wp_idx < last_seg_idx
        p_from = wp(wp_idx, 1:2)';
        p_to   = wp(wp_idx + 1, 1:2)';
        seg    = p_to - p_from;
        seg_l2 = seg' * seg;
        seg_len = sqrt(seg_l2);

        if seg_l2 < 1e-9
            wp_idx = wp_idx + 1;
            return;
        end

        proj = dot(pos - p_from, seg) / seg_l2;
        d_to_waypoint = norm(pos - p_to);

        xte_seg = abs(((pos(1)-p_from(1))*seg(2) - (pos(2)-p_from(2))*seg(1)) / max(seg_len, 1e-6));

        % Corner detection for the waypoint we are approaching.
        turn_angle_deg = 0;
        if (wp_idx + 2) <= n_wps
            p_next = wp(wp_idx + 2, 1:2)';
            seg_next = p_next - p_to;
            seg_next_len = norm(seg_next);
            if seg_len > 1e-6 && seg_next_len > 1e-6
                c_turn = dot(seg, seg_next) / max(seg_len * seg_next_len, 1e-6);
                c_turn = max(-1.0, min(1.0, c_turn));
                turn_angle_deg = rad2deg(acos(c_turn));
            end
        end

        sharp_turn = turn_angle_deg >= 28;

        if sharp_turn
            % Require true proximity before handing off at corners.
            advance_now = ...
                (d_to_waypoint <= max(25, 0.35 * R_accept)) && ...
                (proj >= 0.85) && ...
                (xte_seg <= max(25, 0.60 * R_accept));
        else
            % For non-sharp corners, allow either proximity or clear overshoot.
            near_gate = d_to_waypoint <= max(45, 0.80 * R_accept);
            passed_gate = (proj >= 1.0) && (xte_seg <= max(50, 0.90 * R_accept));
            advance_now = near_gate || passed_gate;
        end

        if advance_now
            old_wp_idx = wp_idx;
            wp_idx = wp_idx + 1;
            fprintf('  [wp-advance] %d -> %d (proj=%.2f, d_wp=%.1f m, xte=%.1f m, turn=%.1f deg)\n', ...
                old_wp_idx, wp_idx, proj, d_to_waypoint, xte_seg, turn_angle_deg);
            return;
        else
            return;
        end
    end
end



function xte = computeXTE(x, wp, wp_idx)
% Cross-track error
    n_wps = size(wp, 1);
    idx_from = max(1, wp_idx);
    idx_to   = min(idx_from + 1, n_wps);

    p1 = wp(idx_from, 1:2)';
    p2 = wp(idx_to,   1:2)';
    pos = [x(4); x(5)];

    seg = p2 - p1;
    seg_len = norm(seg);
    if seg_len < 1
        xte = norm(pos - p1);
        return;
    end
    xte = ((pos(1)-p1(1))*seg(2) - (pos(2)-p1(2))*seg(1)) / seg_len;
end



function x_next = rk4Step9(x, u_ctrl, dt_s)
% RK4 integration for 9-state container model
    [k1, ~] = container(x, u_ctrl);
    
    x2 = x + k1*dt_s/2;
    [k2, ~] = container(x2, u_ctrl);
    
    x3 = x + k2*dt_s/2;
    [k3, ~] = container(x3, u_ctrl);
    
    x4 = x + k3*dt_s;
    [k4, ~] = container(x4, u_ctrl);
    
    x_next = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end


function pts = buildMapSamplePoints(map, spacing_m, include_interior, interior_spacing_m)
% Sample map polygon edges (and optional interiors) for obstacle queries
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

            if include_interior
                xmin = min(px); xmax = max(px);
                ymin = min(py); ymax = max(py);
                xv = xmin:interior_spacing_m:xmax;
                yv = ymin:interior_spacing_m:ymax;
                if ~isempty(xv) && ~isempty(yv)
                    [XX, YY] = meshgrid(xv, yv);
                    in = inpolygon(XX(:), YY(:), px, py);
                    if any(in)
                        pts = [pts; [XX(in), YY(in)]]; %#ok<AGROW>
                    end
                end
            end
    end

    if ~isempty(pts)
        % Remove near-duplicates created by edge/interior overlap.
        pts = unique(round(pts, 1), 'rows');
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
    cand_ord = cand(ord, :);

    % Keep nearest points but enforce minimum spacing so selected
    % obstacles represent different map regions instead of one cluster.
    min_sep = max(2*radius_m, 25);
    cand = zeros(0,2);
    for ii = 1:size(cand_ord,1)
        p = cand_ord(ii, :);
        if isempty(cand)
            cand = p; %#ok<AGROW>
        else
            dmin = min(vecnorm(cand - p, 2, 2));
            if dmin >= min_sep
                cand = [cand; p]; %#ok<AGROW>
            end
        end
        if size(cand,1) >= max_keep
            break;
        end
    end

    for k = 1:size(cand,1)
        obs_local(k).position = cand(k, :)';
        obs_local(k).radius = radius_m;
    end
end

function edge_set = buildMapHalfPlaneEdgeSet(map, min_edge_len_m)
% Build compact map edge set for local half-plane extraction.
    edge_set = struct('p1', {}, 'p2', {}, 'mid', {}, 't', {}, 'n_left', {}, 'len', {});
    if nargin < 2 || isempty(min_edge_len_m)
        min_edge_len_m = 10;
    end
    if isempty(map) || ~isfield(map, 'polygons') || isempty(map.polygons)
        return;
    end

    out_idx = 0;
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
            p1 = [px(ii); py(ii)];
            p2 = [px(ii+1); py(ii+1)];
            seg = p2 - p1;
            seg_len = norm(seg);
            if seg_len < min_edge_len_m
                continue;
            end

            t_hat = seg / seg_len;
            n_left = [-t_hat(2); t_hat(1)];

            out_idx = out_idx + 1;
            edge_set(out_idx).p1 = p1;
            edge_set(out_idx).p2 = p2;
            edge_set(out_idx).mid = 0.5 * (p1 + p2);
            edge_set(out_idx).t = t_hat;
            edge_set(out_idx).n_left = n_left;
            edge_set(out_idx).len = seg_len;
        end
    end
end

function d_min = nearestDistanceToMapEdges(p_xy, edge_set)
% Nearest Euclidean distance from point to map edge set.
    d_min = inf;
    if isempty(edge_set)
        return;
    end

    p = p_xy(:);
    for ii = 1:length(edge_set)
        d_i = pointToSegmentDistance(p, edge_set(ii).p1, edge_set(ii).p2);
        d_min = min(d_min, d_i);
    end
end

function hp_local = selectMapHalfPlanesFromEdges(edge_set, pos_xy, chi_d, max_keep, lookahead_m, half_width_m, goal_xy, goal_exclusion_m, relax_ratio)
% Select nearby map edges and convert them to local half-planes.
% Each half-plane is oriented to keep the vessel on its current side.
    hp_local = struct('normal', {}, 'offset', {});

    if isempty(edge_set) || max_keep <= 0
        return;
    end
    if nargin < 5 || isempty(lookahead_m), lookahead_m = 400; end
    if nargin < 6 || isempty(half_width_m), half_width_m = 150; end
    if nargin < 7, goal_xy = []; end
    if nargin < 8 || isempty(goal_exclusion_m), goal_exclusion_m = 0; end
    if nargin < 9 || isempty(relax_ratio), relax_ratio = 1.0; end

    relax_ratio = max(0.35, min(1.0, relax_ratio));
    pos = pos_xy(:);
    fwd = [cos(chi_d); sin(chi_d)];
    side = [-sin(chi_d); cos(chi_d)];

    n_e = length(edge_set);
    keep = false(1, n_e);
    score = inf(1, n_e);

    for ii = 1:n_e
        mid_i = edge_set(ii).mid;
        rel_i = mid_i - pos;
        along_i = dot(rel_i, fwd);
        lat_i = dot(rel_i, side);

        if along_i < -50 || along_i > lookahead_m
            continue;
        end
        if abs(lat_i) > half_width_m
            continue;
        end

        if ~isempty(goal_xy)
            if pointToSegmentDistance(goal_xy(:), edge_set(ii).p1, edge_set(ii).p2) < goal_exclusion_m
                continue;
            end
        end

        keep(ii) = true;
        d_seg = pointToSegmentDistance(pos, edge_set(ii).p1, edge_set(ii).p2);
        score(ii) = d_seg + 0.30 * abs(lat_i) + 0.06 * max(along_i, 0);
    end

    idx = find(keep);
    if isempty(idx)
        return;
    end

    [~, ord_local] = sort(score(idx), 'ascend');
    idx = idx(ord_local);

    min_line_sep = max(12, 30 * relax_ratio);
    out_idx = 0;
    used_n = zeros(2, 0);
    used_b = zeros(0, 1);

    for kk = 1:length(idx)
        e = edge_set(idx(kk));
        n_l = e.n_left;

        % Pick normal that points toward current vessel side.
        if dot(n_l, pos - e.p1) >= 0
            n_use = n_l;
        else
            n_use = -n_l;
        end
        b_use = dot(n_use, e.p1);

        too_close = false;
        for jj = 1:size(used_n, 2)
            if dot(used_n(:,jj), n_use) > 0.97 && abs(used_b(jj) - b_use) < min_line_sep
                too_close = true;
                break;
            end
        end
        if too_close
            continue;
        end

        out_idx = out_idx + 1;
        hp_local(out_idx).normal = n_use;
        hp_local(out_idx).offset = b_use;
        used_n(:, end+1) = n_use; %#ok<AGROW>
        used_b(end+1, 1) = b_use; %#ok<AGROW>

        if out_idx >= max_keep
            break;
        end
    end
end

function d = pointToSegmentDistance(p, a, b)
% Euclidean distance from point p to segment [a,b].
    ab = b - a;
    ab2 = dot(ab, ab);
    if ab2 < 1e-12
        d = norm(p - a);
        return;
    end
    t = dot(p - a, ab) / ab2;
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(p - proj);
end

function dynamic_obstacles = buildDynamicObstaclesFromConfig(pos_xy, heading_deg, speeds_mps, radius_default, speed_default)
% Build moving obstacles strictly from user configuration.
    if nargin < 4 || isempty(radius_default), radius_default = 20; end
    if nargin < 5 || isempty(speed_default), speed_default = 3.0; end

    if isempty(pos_xy)
        error(['Dynamic obstacles are enabled but dynamic_obs_positions_xy is empty. ', ...
               'Provide rows [x y] in the USER CONFIGURATION section.']);
    end
    if size(pos_xy,2) ~= 2
        error('dynamic_obs_positions_xy must be an N-by-2 matrix with rows [x y].');
    end

    n_obs = size(pos_xy,1);
    if isempty(heading_deg)
        error(['Dynamic obstacles are enabled but dynamic_obs_headings_deg is empty. ', ...
               'Provide one heading per obstacle (or one scalar to replicate).']);
    end

    heading_deg = heading_deg(:);
    if numel(heading_deg) == 1 && n_obs > 1
        heading_deg = repmat(heading_deg, n_obs, 1);
    end
    if numel(heading_deg) ~= n_obs
        error('dynamic_obs_headings_deg must have length 1 or match number of obstacle rows.');
    end

    if isempty(speeds_mps)
        speed_vec = speed_default * ones(n_obs,1);
    else
        speed_vec = speeds_mps(:);
        if numel(speed_vec) == 1 && n_obs > 1
            speed_vec = repmat(speed_vec, n_obs, 1);
        end
        if numel(speed_vec) ~= n_obs
            error('dynamic_obs_speeds_mps must have length 1 or match number of obstacle rows.');
        end
    end

    dynamic_obstacles = repmat(struct('position', [0;0], 'radius', radius_default, ...
        'speed', speed_default, 'heading', 0, 'active', true, 'enabled', true, ...
        'moving', true, 'trigger_distance', inf, 'id', 1), 1, n_obs);

    for k = 1:n_obs
        dynamic_obstacles(k).position = pos_xy(k,:).';
        dynamic_obstacles(k).radius = radius_default;
        dynamic_obstacles(k).speed = speed_vec(k);
        dynamic_obstacles(k).heading = deg2rad(heading_deg(k));
        dynamic_obstacles(k).active = true;
        dynamic_obstacles(k).enabled = true;
        dynamic_obstacles(k).moving = true;
        dynamic_obstacles(k).trigger_distance = inf;
        dynamic_obstacles(k).id = k;
    end
end

function dynamic_obstacles = configureDynamicStartMode(dynamic_obstacles, start_mode, trigger_distance_m)
% Configure whether obstacles move immediately or on proximity trigger.
    if isempty(dynamic_obstacles)
        return;
    end
    if nargin < 2 || isempty(start_mode)
        start_mode = 'immediate';
    end

    n_obs = length(dynamic_obstacles);
    trig = trigger_distance_m;
    if nargin < 3 || isempty(trig)
        trig = inf;
    end
    trig = trig(:);
    if numel(trig) == 1 && n_obs > 1
        trig = repmat(trig, n_obs, 1);
    end
    if numel(trig) ~= n_obs
        error('dynamic_obs_trigger_distance_m must have length 1 or match number of obstacle rows.');
    end

    is_proximity = strcmpi(strtrim(start_mode), 'proximity');
    for k = 1:n_obs
        dynamic_obstacles(k).trigger_distance = trig(k);
        dynamic_obstacles(k).moving = ~is_proximity;
    end
end

function dynamic_obstacles = activateDynamicObstaclesByProximity(dynamic_obstacles, ship_pos)
% Start obstacle motion when ship is within trigger distance.
    if isempty(dynamic_obstacles)
        return;
    end
    for k = 1:length(dynamic_obstacles)
        if ~isfield(dynamic_obstacles(k), 'enabled') || ~dynamic_obstacles(k).enabled
            continue;
        end
        if ~isfield(dynamic_obstacles(k), 'active') || ~dynamic_obstacles(k).active
            continue;
        end
        if isfield(dynamic_obstacles(k), 'moving') && dynamic_obstacles(k).moving
            continue;
        end

        d = norm(ship_pos(:) - dynamic_obstacles(k).position(1:2));
        trig = inf;
        if isfield(dynamic_obstacles(k), 'trigger_distance') && ~isempty(dynamic_obstacles(k).trigger_distance)
            trig = dynamic_obstacles(k).trigger_distance;
        end
        if d <= trig
            dynamic_obstacles(k).moving = true;
        end
    end
end

function dynamic_obstacles = propagateDynamicObstacles(dynamic_obstacles, dt, bounds, boundary_policy)
% Forward Euler propagation with deterministic boundary handling
    if isempty(dynamic_obstacles)
        return;
    end
    if nargin < 4 || isempty(boundary_policy)
        boundary_policy = 'deactivate';
    end

    for k = 1:length(dynamic_obstacles)
        if ~isfield(dynamic_obstacles(k), 'enabled') || ~dynamic_obstacles(k).enabled
            continue;
        end
        if ~isfield(dynamic_obstacles(k), 'active') || ~dynamic_obstacles(k).active
            continue;
        end
        if isfield(dynamic_obstacles(k), 'moving') && ~dynamic_obstacles(k).moving
            continue;
        end

        speed_k = dynamic_obstacles(k).speed;
        hdg_k = dynamic_obstacles(k).heading;
        dx = dt * speed_k * cos(hdg_k);
        dy = dt * speed_k * sin(hdg_k);
        dynamic_obstacles(k).position = dynamic_obstacles(k).position + [dx; dy];

        if isPositionOutsideBounds(dynamic_obstacles(k).position, bounds)
            switch lower(strtrim(boundary_policy))
                case 'clip'
                    dynamic_obstacles(k).position(1) = min(max(dynamic_obstacles(k).position(1), bounds.xmin), bounds.xmax);
                    dynamic_obstacles(k).position(2) = min(max(dynamic_obstacles(k).position(2), bounds.ymin), bounds.ymax);
                case 'wrap'
                    dynamic_obstacles(k).position(1) = wrapLinear(dynamic_obstacles(k).position(1), bounds.xmin, bounds.xmax);
                    dynamic_obstacles(k).position(2) = wrapLinear(dynamic_obstacles(k).position(2), bounds.ymin, bounds.ymax);
                otherwise
                    dynamic_obstacles(k).active = false;
            end
        end
    end
end

function obs_dyn = dynamicToCircleObstacles(dynamic_obstacles, radius_guard_m)
% Convert active dynamic obstacle states to NMPC-compatible circle obstacles
    obs_dyn = struct('position', {}, 'radius', {});
    if isempty(dynamic_obstacles)
        return;
    end
    if nargin < 2 || isempty(radius_guard_m)
        radius_guard_m = 0;
    end

    out_idx = 0;
    for k = 1:length(dynamic_obstacles)
        is_enabled = isfield(dynamic_obstacles(k), 'enabled') && dynamic_obstacles(k).enabled;
        is_active = isfield(dynamic_obstacles(k), 'active') && dynamic_obstacles(k).active;
        if ~(is_enabled && is_active)
            continue;
        end
        out_idx = out_idx + 1;
        obs_dyn(out_idx).position = dynamic_obstacles(k).position(1:2);
        obs_dyn(out_idx).radius = dynamic_obstacles(k).radius + radius_guard_m;
    end
end

function obs_virtual = buildLatentDynamicAwarenessObstacles(dynamic_obstacles, ship_pos, cfg)
% Build virtual future-occupancy circles for dynamic obstacles.
    obs_virtual = struct('position', {}, 'radius', {});
    if isempty(dynamic_obstacles) || nargin < 3 || isempty(cfg)
        return;
    end
    if ~getOr(cfg, 'enabled', false)
        return;
    end

    horizon_s = max(0, getOr(cfg, 'horizon_s', 0));
    n_samples = max(0, round(getOr(cfg, 'n_samples', 0)));
    if horizon_s <= 0 || n_samples <= 0
        return;
    end

    radius_scale = max(1.0, getOr(cfg, 'radius_scale', 1.0));
    awareness_distance_m = max(0, getOr(cfg, 'awareness_distance_m', inf));
    only_when_not_moving = getOr(cfg, 'only_when_not_moving', true);

    tau = linspace(horizon_s / n_samples, horizon_s, n_samples);
    out_idx = 0;

    for k = 1:length(dynamic_obstacles)
        is_enabled = isfield(dynamic_obstacles(k), 'enabled') && dynamic_obstacles(k).enabled;
        is_active = isfield(dynamic_obstacles(k), 'active') && dynamic_obstacles(k).active;
        if ~(is_enabled && is_active)
            continue;
        end

        if only_when_not_moving
            is_moving = isfield(dynamic_obstacles(k), 'moving') && dynamic_obstacles(k).moving;
            if is_moving
                continue;
            end
        end

        if nargin >= 2 && ~isempty(ship_pos)
            if norm(dynamic_obstacles(k).position(1:2) - ship_pos(:)) > awareness_distance_m
                continue;
            end
        end

        p0 = dynamic_obstacles(k).position(1:2);
        speed_k = dynamic_obstacles(k).speed;
        hdg_k = dynamic_obstacles(k).heading;
        vel_k = speed_k * [cos(hdg_k); sin(hdg_k)];
        rad_k = dynamic_obstacles(k).radius * radius_scale;

        for j = 1:length(tau)
            out_idx = out_idx + 1;
            obs_virtual(out_idx).position = p0 + tau(j) * vel_k;
            obs_virtual(out_idx).radius = rad_k;
        end
    end
end

function [hit, hit_idx, min_sep] = detectHullCircleHit(x_state, obstacles, hull_cfg, safety_buffer)
% Oriented-rectangle hull vs circle obstacles.
    hit = false;
    hit_idx = 0;
    min_sep = inf;
    if nargin < 4 || isempty(safety_buffer)
        safety_buffer = 0;
    end
    if isempty(obstacles)
        return;
    end

    pos = x_state(4:5);
    psi = x_state(6);
    c = cos(psi);
    s = sin(psi);
    hx = hull_cfg.half_length_m;
    hy = hull_cfg.half_beam_m;

    for k = 1:length(obstacles)
        rel = obstacles(k).position(1:2) - pos;

        % Obstacle center in ship body frame.
        qx = c * rel(1) + s * rel(2);
        qy = -s * rel(1) + c * rel(2);

        dx = max(abs(qx) - hx, 0);
        dy = max(abs(qy) - hy, 0);
        d_rect = hypot(dx, dy);

        sep = d_rect - (obstacles(k).radius + safety_buffer);
        min_sep = min(min_sep, sep);
        if sep <= 0
            hit = true;
            hit_idx = k;
            return;
        end
    end
end

function [hit, zone_type, zone_idx] = detectHullMapHit(x_state, hull_cfg, map)
% Oriented-rectangle hull collision against map polygon zones.
    hit = false;
    zone_type = '';
    zone_idx = 0;
    if isempty(map)
        return;
    end

    hull_poly = buildHullPolygon(x_state(4:5), x_state(6), hull_cfg);

    if isfield(map, 'polygons') && ~isempty(map.polygons)
        [hit_poly, idx_poly] = hullHitsPolygonSet(hull_poly, map.polygons);
        if hit_poly
            hit = true;
            zone_type = 'polygons';
            zone_idx = idx_poly;
            return;
        end
    end

    if isfield(map, 'mapPoly') && ~isempty(map.mapPoly)
        [hit_poly, idx_poly] = hullHitsPolygonSet(hull_poly, map.mapPoly);
        if hit_poly
            hit = true;
            zone_type = 'mapPoly';
            zone_idx = idx_poly;
            return;
        end
    end
end

function [hit, idx] = hullHitsPolygonSet(hull_poly, polygon_set)
% Test hull polygon against all polygons in a map polygon set.
    hit = false;
    idx = 0;
    if isempty(polygon_set)
        return;
    end

    for j = 1:length(polygon_set)
        px = polygon_set(j).X(:);
        py = polygon_set(j).Y(:);
        rings = splitPolygonRings(px, py);
        for rr = 1:length(rings)
            ring = rings{rr};
            if size(ring,1) < 3
                continue;
            end
            if polygonsIntersectOrContain(hull_poly, ring)
                hit = true;
                idx = j;
                return;
            end
        end
    end
end

function hull_poly = buildHullPolygon(pos_xy, psi, hull_cfg)
% Return 4x2 hull corner coordinates in world frame.
    hx = hull_cfg.half_length_m;
    hy = hull_cfg.half_beam_m;

    local = [ hx,  hy;
              hx, -hy;
             -hx, -hy;
             -hx,  hy];

    c = cos(psi);
    s = sin(psi);
    R = [c, -s; s, c];
    hull_poly = (R * local')' + pos_xy(:)';
end

function rings = splitPolygonRings(px, py)
% Split NaN-separated polygon arrays into finite rings.
    rings = {};
    if isempty(px) || isempty(py)
        return;
    end

    finite = isfinite(px) & isfinite(py);
    keep = finite | (isnan(px) & isnan(py));
    px = px(keep);
    py = py(keep);
    if isempty(px)
        return;
    end

    sep = isnan(px) | isnan(py);
    idx_sep = find(sep);
    starts = [1; idx_sep + 1];
    ends = [idx_sep - 1; numel(px)];

    for k = 1:numel(starts)
        s = starts(k);
        e = ends(k);
        if s > e
            continue;
        end
        rx = px(s:e);
        ry = py(s:e);
        ring_ok = isfinite(rx) & isfinite(ry);
        ring = [rx(ring_ok), ry(ring_ok)];
        if size(ring,1) >= 3
            rings{end+1} = ring; %#ok<AGROW>
        end
    end
end

function hit = polygonsIntersectOrContain(polyA, polyB)
% Polygon intersection for simple polygons (with containment fallback).
    hit = false;

    nA = size(polyA, 1);
    nB = size(polyB, 1);
    if nA < 3 || nB < 3
        return;
    end

    % Edge-edge intersection test.
    for ia = 1:nA
        a1 = polyA(ia, :);
        a2 = polyA(mod(ia, nA) + 1, :);
        for ib = 1:nB
            b1 = polyB(ib, :);
            b2 = polyB(mod(ib, nB) + 1, :);
            if segmentsIntersect2D(a1, a2, b1, b2)
                hit = true;
                return;
            end
        end
    end

    % Containment checks.
    [in1, on1] = inpolygon(polyA(1,1), polyA(1,2), polyB(:,1), polyB(:,2));
    if in1 || on1
        hit = true;
        return;
    end
    [in2, on2] = inpolygon(polyB(1,1), polyB(1,2), polyA(:,1), polyA(:,2));
    if in2 || on2
        hit = true;
    end
end



function tf = segmentsIntersect2D(p1, p2, q1, q2)
% Robust 2D segment intersection including collinear overlap.
    eps_v = 1e-9;
    o1 = orient2d(p1, p2, q1);
    o2 = orient2d(p1, p2, q2);
    o3 = orient2d(q1, q2, p1);
    o4 = orient2d(q1, q2, p2);

    if ((o1 > eps_v && o2 < -eps_v) || (o1 < -eps_v && o2 > eps_v)) && ...
       ((o3 > eps_v && o4 < -eps_v) || (o3 < -eps_v && o4 > eps_v))
        tf = true;
        return;
    end

    tf = (abs(o1) <= eps_v && onSegment2D(p1, q1, p2, eps_v)) || ...
         (abs(o2) <= eps_v && onSegment2D(p1, q2, p2, eps_v)) || ...
         (abs(o3) <= eps_v && onSegment2D(q1, p1, q2, eps_v)) || ...
         (abs(o4) <= eps_v && onSegment2D(q1, p2, q2, eps_v));
end



function o = orient2d(a, b, c)
    o = (b(1)-a(1))*(c(2)-a(2)) - (b(2)-a(2))*(c(1)-a(1));
end



function tf = onSegment2D(a, p, b, eps_v)
    tf = p(1) <= max(a(1), b(1)) + eps_v && p(1) >= min(a(1), b(1)) - eps_v && ...
         p(2) <= max(a(2), b(2)) + eps_v && p(2) >= min(a(2), b(2)) - eps_v;
end



function hull_cfg = buildHullFootprintConfig(length_nominal_m, beam_nominal_m, hull_scale, r_safety_point)
% Build oriented-rectangle hull config and compatibility clearance.
    if nargin < 1 || isempty(length_nominal_m)
        length_nominal_m = 72;
    end
    if nargin < 2 || isempty(beam_nominal_m)
        beam_nominal_m = 24;
    end
    if nargin < 3 || isempty(hull_scale)
        hull_scale = 0.5;
    end
    if nargin < 4 || isempty(r_safety_point)
        r_safety_point = 34;
    end

    hull_cfg = struct();
    hull_cfg.length_m = max(4, hull_scale * length_nominal_m);
    hull_cfg.beam_m = max(2, hull_scale * beam_nominal_m);
    hull_cfg.half_length_m = 0.5 * hull_cfg.length_m;
    hull_cfg.half_beam_m = 0.5 * hull_cfg.beam_m;

    % Map old point-safety radius to footprint + clearance margin.
    hull_cfg.circ_radius_m = hypot(hull_cfg.half_length_m, hull_cfg.half_beam_m);
    hull_cfg.nmpc_clearance_m = max(2.0, r_safety_point - hull_cfg.circ_radius_m);
end



function drift = computeDynamicPackagingDrift(dynamic_obstacles, obs_local)
% Verifies dynamic obstacles are passed to solver in same position/radius schema
    obs_dyn = dynamicToCircleObstacles(dynamic_obstacles);
    if isempty(obs_dyn)
        drift = 0;
        return;
    end
    if length(obs_local) < length(obs_dyn)
        drift = inf;
        return;
    end

    tail_idx = (length(obs_local) - length(obs_dyn) + 1):length(obs_local);
    e = zeros(1, length(obs_dyn));
    for k = 1:length(obs_dyn)
        e(k) = norm(obs_local(tail_idx(k)).position(1:2) - obs_dyn(k).position(1:2));
    end
    drift = max(e);
end



function bounds = estimateMapBounds(map, waypoints, margin)
% Build a finite bounding box used by dynamic obstacle lifecycle policy
    if nargin < 3 || isempty(margin)
        margin = 100;
    end

    xs = waypoints(:,1);
    ys = waypoints(:,2);
    if ~isempty(map)
        if isfield(map, 'polygons') && ~isempty(map.polygons)
            for k = 1:length(map.polygons)
                xs = [xs; map.polygons(k).X(:)]; %#ok<AGROW>
                ys = [ys; map.polygons(k).Y(:)]; %#ok<AGROW>
            end
        end
        if isfield(map, 'mapPoly') && ~isempty(map.mapPoly)
            for k = 1:length(map.mapPoly)
                xs = [xs; map.mapPoly(k).X(:)]; %#ok<AGROW>
                ys = [ys; map.mapPoly(k).Y(:)]; %#ok<AGROW>
            end
        end
    end

    xs = xs(isfinite(xs));
    ys = ys(isfinite(ys));
    if isempty(xs) || isempty(ys)
        xs = [0; 1000];
        ys = [0; 1000];
    end
    bounds.xmin = min(xs) - margin;
    bounds.xmax = max(xs) + margin;
    bounds.ymin = min(ys) - margin;
    bounds.ymax = max(ys) + margin;
end



function tf = isPositionOutsideBounds(pos_xy, bounds)
    tf = pos_xy(1) < bounds.xmin || pos_xy(1) > bounds.xmax || ...
         pos_xy(2) < bounds.ymin || pos_xy(2) > bounds.ymax;
end



function y = wrapLinear(x, xmin, xmax)
    rng = xmax - xmin;
    if rng <= 0
        y = x;
        return;
    end
    y = xmin + mod(x - xmin, rng);
end



function ok = runDynamicReplayCheck(dynamic_obstacles, n_steps, dt, bounds, policy)
% Deterministic replay check for dynamic obstacle propagation
    if nargin < 2 || isempty(n_steps)
        n_steps = 20;
    end
    a = dynamic_obstacles;
    b = dynamic_obstacles;
    for i = 1:n_steps
        a = propagateDynamicObstacles(a, dt, bounds, policy);
        b = propagateDynamicObstacles(b, dt, bounds, policy);
    end
    da = dynamicToCircleObstacles(a);
    db = dynamicToCircleObstacles(b);
    if length(da) ~= length(db)
        ok = false;
        return;
    end
    err = 0;
    for k = 1:length(da)
        err = max(err, norm(da(k).position - db(k).position));
    end
    ok = err <= 1e-12;
end



function v = getOr(s, field_name, default_value)
% Safe struct field getter with default fallback.
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        v = s.(field_name);
    else
        v = default_value;
    end
end



function p = safePercentile(x, prc)
% Percentile with compatibility fallback
    if isempty(x)
        p = NaN;
        return;
    end
    x = x(isfinite(x));
    if isempty(x)
        p = NaN;
        return;
    end
    try
        p = prctile(x, prc);
    catch
        xs = sort(x(:));
        idx = max(1, min(numel(xs), round(prc/100 * numel(xs))));
        p = xs(idx);
    end
end



function printTimingHistogram(label, vec, ref_value)
% Print a compact ASCII histogram against a real-time reference
    if isempty(vec)
        fprintf('    %s: no data\n', label);
        return;
    end

    vals = vec(isfinite(vec));
    if isempty(vals)
        fprintf('    %s: no finite data\n', label);
        return;
    end

    edges = [0, 0.25, 0.5, 0.75, 1.0, 1.25, inf] * ref_value;
    counts = histcounts(vals, edges);
    tags = {'0-25%', '25-50%', '50-75%', '75-100%', '100-125%', '>125%'};
    max_count = max(counts);

    fprintf('    %s (ref=%.3f)\n', label, ref_value);
    for ii = 1:numel(counts)
        n_hash = 0;
        if max_count > 0
            n_hash = round(32 * counts(ii) / max_count);
        end
        bar = repmat('#', 1, n_hash);
        fprintf('      %-8s | %-32s %4d\n', tags{ii}, bar, counts(ii));
    end
end



function obs_struct = normalizeStaticObstacles(obs_in)
% Normalize static obstacle input to struct array with position/radius.
    if isempty(obs_in)
        obs_struct = struct('position', {}, 'radius', {});
        return;
    end

    if isstruct(obs_in)
        obs_struct = obs_in;
        return;
    end

    if isnumeric(obs_in)
        if size(obs_in,2) ~= 3
            error(['static_obstacles numeric format must be N-by-3 rows [x y radius]. ', ...
                   'Use [] for no obstacles.']);
        end
        n_obs = size(obs_in, 1);
        obs_struct = repmat(struct('position', [0;0], 'radius', 0), 1, n_obs);
        for k = 1:n_obs
            obs_struct(k).position = obs_in(k, 1:2)';
            obs_struct(k).radius = obs_in(k, 3);
        end
        return;
    end

    error('static_obstacles must be either N-by-3 numeric or struct array.');
end
