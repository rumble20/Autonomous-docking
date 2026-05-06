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

%% USER CONFIGURATION 

% Waypoints: rows = [x, y].
% No heading is stored in the waypoint list anymore.
waypoints = [-3000, -2600; -2800, -2700; -2600, -2700; -2350, -2650; -2200, -2650; -1940, -2450];

% BERTHING MODE ==========================================================
% This block fully replaces the old waypoint third-column final heading.
% Use it to tell the ship how to approach and park at the final target.
berth_cfg = struct();
berth_cfg.enabled = false;                 % true = use precise berthing mode
berth_cfg.target_xy = [-1940; -2450];     % final parking position [x; y]
berth_cfg.heading_deg = 30;                % desired final ship heading
berth_cfg.prepare_last_n_segments = 3;    % start preparing on last route segments
berth_cfg.activate_dist_m = 260;          % force berth mode when this close to berth target
berth_cfg.preview_heading_weight = 18.0;  % mild early heading preparation before full berth mode
berth_cfg.preview_goal_weight_gain = 1.35;
berth_cfg.capture_radius_m = 12;          % hard completion radius in berth mode
berth_cfg.capture_speed_mps = 0.35;       % completion speed in berth mode

% Final approach corridor (optional but recommended for parking style)
berth_cfg.use_corridor = true;
berth_cfg.corridor_origin_xy = [-2210; -2600];
berth_cfg.corridor_heading_deg = 0;
berth_cfg.corridor_half_width_m = 12;
berth_cfg.corridor_along_min_m = -220;
berth_cfg.corridor_along_max_m = 25;

% Tight final pose / speed envelope
berth_cfg.pose_eps_xy_m = [4; 4];
berth_cfg.pose_eps_psi_deg = 3.0;
berth_cfg.vel_max_u_mps = 0.30;
berth_cfg.vel_max_v_mps = 0.20;
berth_cfg.vel_max_r_radps = deg2rad(2.5);
berth_cfg.pose_slack_max = 0.5;

% Final docking authority / style
berth_cfg.u_min_final_mps = -0.8;         % allow slight reverse during docking if needed
berth_cfg.n3_max = 110;
berth_cfg.max_azimuth_split_rad = deg2rad(10);
berth_cfg.max_stern_cmd_split_rpm = 8;

% Route-following restore: keep waypoint order softly, still driven by NMPC
route_follow_cfg = struct();
route_follow_cfg.tighten_near_gate_dist_m = 180;
route_follow_cfg.tighten_tube_m = 15;
route_follow_cfg.tighten_xte_weight = 16.0;
route_follow_cfg.transit_goal_pos_weight_far  = 30.0;
route_follow_cfg.transit_goal_pos_weight_mid  = 50.0;
route_follow_cfg.transit_goal_pos_weight_near = 90.0;
route_follow_cfg.transit_goal_dist_mid_m  = 220;
route_follow_cfg.transit_goal_dist_near_m = 120;
route_follow_cfg.sharp_turn_deg = 28;
route_follow_cfg.sharp_turn_goal_weight_gain = 1.4;
route_follow_cfg.sharp_turn_heading_weight = 8.0;
route_follow_cfg.sharp_turn_heading_enable_dist_m = 160;

% Cruise-speed suggestion for the NMPC soft speed cost.
cruise_speed_mps = 5.0;

% Static obstacles: N-by-3 [x y radius] or struct array.
static_obstacles = [];

% Dynamic obstacles.
dynamic_obs_positions_xy = [-2300, -2800; -2300, -3000];
dynamic_obs_headings_deg = [135; 45];
dynamic_obs_speeds_mps   = [3; 3];
enable_dynamic_obstacles = true;
dynamic_obs_radius_m     = 25;
dynamic_obs_speed_mps    = 5;
dynamic_obs_nmpc_guard_m = 0;
dynamic_obs_start_mode   = 'immediate';   % immediate | proximity
dynamic_obs_trigger_distance_m = 300;
dynamic_obs_boundary_margin = 200;
dynamic_obs_boundary_policy = 'wrap';     % wrap | bounce | hold

% Dynamic latent awareness (optional planning-only envelopes).
dynamic_latent_awareness = struct();
dynamic_latent_awareness.enabled = true;
dynamic_latent_awareness.n_samples = 4;
dynamic_latent_awareness.dt_s = 2.0;
dynamic_latent_awareness.inflate_radius_m = 8.0;
dynamic_latent_awareness.sample_radius_gain = 0.15;

% Simulation.
T_final = 30;
R_accept = 90;
R_accept_final = 10;
R_accept_final_soft = 50;
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
max_map_halfplanes_transit = 10;
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
max_brake_rate = 0.8;
actuator_force_weight = 0.015;
forward_incentive_weight = 1;
u_min_forward = 0.1;

% Tube-MPC path cost.
path_cost_cfg = struct();
path_cost_cfg.W_xte_heavy = 12.0;
path_cost_cfg.W_along = 1.2;
path_cost_cfg.W_tube_m = 20.0;
path_cost_cfg.soft_speed_cap_weight = 0.05;
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

% CONTINUOUS CAUTION / BERTHING SCHEDULER 
sched_cfg = struct();

% Speed scheduling
sched_cfg.u_cruise_mps = cruise_speed_mps;      % normal open-water preference
sched_cfg.u_tight_mps  = 1.8;                   % preferred speed in tight geometry
sched_cfg.u_berth_mps  = 0.45;                  % preferred speed near final berth

sched_cfg.soft_speed_weight_far  = 0.03;
sched_cfg.soft_speed_weight_near = 0.70;

% Forward lower bound scheduling
sched_cfg.u_min_forward_far  = u_min_forward;
sched_cfg.u_min_forward_near = 0.02;

% Tightness from map clearance
sched_cfg.clearance_lo_m = 45;   % full caution if nearer than this
sched_cfg.clearance_hi_m = 140;  % no caution if farther than this

% Optional channel width awareness
sched_cfg.channel_width_lo_m = 70;
sched_cfg.channel_width_hi_m = 180;

% Stopping-distance awareness
sched_cfg.brake_eff_mps2 = 0.55;   % effective deceleration for caution logic
sched_cfg.stop_margin_m  = 25;     % extra margin
sched_cfg.stop_buffer_m  = 35;     % extra comparison slack

% Turn severity awareness
sched_cfg.turn_angle_lo_deg = 15;
sched_cfg.turn_angle_hi_deg = 55;
sched_cfg.turn_dist_far_m   = 240;
sched_cfg.turn_dist_near_m  = 60;

% Berth ramp
sched_cfg.berth_d_activate_m = berth_cfg.activate_dist_m;
sched_cfg.berth_d_full_m     = 35;

% Weight scheduling
sched_cfg.xte_weight_far  = path_cost_cfg.W_xte_heavy;
sched_cfg.xte_weight_near = 24.0;

sched_cfg.tube_far_m  = path_cost_cfg.W_tube_m;
sched_cfg.tube_near_m = 10.0;

sched_cfg.heading_weight_far   = 0.0;
sched_cfg.heading_weight_turn  = 12.0;
sched_cfg.heading_weight_berth = 140.0;
sched_cfg.heading_weight_return = 35.0;

sched_cfg.stop_u_weight_far   = 0.0;
sched_cfg.stop_u_weight_berth = 120.0;
sched_cfg.stop_v_weight_berth = 80.0;
sched_cfg.stop_r_weight_berth = 80.0;

sched_cfg.term_pos_weight_far   = route_follow_cfg.transit_goal_pos_weight_far;
sched_cfg.term_pos_weight_berth = 220.0;

% Terminal envelope scheduling near berth
sched_cfg.term_xy_far_m   = [10; 10];
sched_cfg.term_xy_near_m  = berth_cfg.pose_eps_xy_m(:);
sched_cfg.term_psi_far_rad  = deg2rad(12);
sched_cfg.term_psi_near_rad = deg2rad(berth_cfg.pose_eps_psi_deg);

sched_cfg.term_u_far_mps = 1.2;
sched_cfg.term_u_near_mps = berth_cfg.vel_max_u_mps;
sched_cfg.term_v_far_mps = 0.5;
sched_cfg.term_v_near_mps = berth_cfg.vel_max_v_mps;
sched_cfg.term_r_far_radps = deg2rad(5);
sched_cfg.term_r_near_radps = berth_cfg.vel_max_r_radps;

% Berth corridor scheduling
sched_cfg.corridor_half_width_far_m  = 22;
sched_cfg.corridor_half_width_near_m = berth_cfg.corridor_half_width_m;

% Blend rule
sched_cfg.use_max_blend = true;   % if false, use weighted sum below
sched_cfg.w_tight = 0.45;
sched_cfg.w_stop  = 0.25;
sched_cfg.w_turn  = 0.35;
sched_cfg.w_berth = 0.80;

% DYNAMIC OBSTACLE INTERACTION SCHEDULER
sched_cfg.T_cpa_far     = 20.0;    % s: near-zero TTC activation at/above this
sched_cfg.T_cpa_near    = 5.0;     % s: strong TTC activation at/below this
sched_cfg.d_cpa_th      = 60.0;    % m: CPA threshold for caution
sched_cfg.v_close_ref   = 3.0;     % m/s: closing rate reference
sched_cfg.cos_beta_max  = cos(deg2rad(120)); % 120deg forward sector
sched_cfg.d_dot_ref     = 2.0;     % m/s: separation rate threshold
sched_cfg.u_yield       = 0.6;    % m/s: crawl speed during yield
sched_cfg.w_along_min   = 0.35;    % minimal progress reward
sched_cfg.R_rate_scale_yield = 2;% control smoothness multiplier
sched_cfg.map_barrier_w_base = 0.0;
sched_cfg.map_barrier_w_near = 18.0;
sched_cfg.map_soft_margin_m = 18;


%% NORMALIZE / SETUP ======================================================
static_obstacles = normalizeStaticObstacles(static_obstacles);
berth_cfg = normalizeBerthCfg(berth_cfg, waypoints);
repoRoot = pwd;

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
if exist('helsinki_harbour_UPDATED.mat', 'file')
    S = load('helsinki_harbour_UPDATED.mat');
    if isfield(S, 'map'), map = S.map; end
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
        map_goal_xy = waypoints(end,1:2)';
        if berth_cfg.enabled
            map_goal_xy = berth_cfg.target_xy(:);
        end
        [in_final_zone, ~, ~] = NavUtils.isInsideAnyMapZone(map_goal_xy, map);
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
% Lambda scheduler logs
lambda_tight_log = nan(1, length(t));
lambda_stop_log  = nan(1, length(t));
lambda_turn_log  = nan(1, length(t));
lambda_berth_log = nan(1, length(t));
lambda_yield_log = nan(1, length(t));
lambda_return_log= nan(1, length(t));
lambda_ttc_log   = nan(1, length(t));
lambda_total_log = nan(1, length(t));
u_cap_log        = nan(1, length(t));
d_edge_log       = nan(1, length(t));
d_stop_log       = nan(1, length(t));
traj(:,1) = x;
steps = 0;

n_dyn = length(dynamic_obstacles);
dyn_obs_hist = nan(n_dyn, 2, length(t)+1);
if n_dyn > 0
    for jj = 1:n_dyn
        dyn_obs_hist(jj, :, 1) = dynamic_obstacles(jj).position(1:2)';
    end
end
u_prev = [0; 0; n1_cruise; n2_cruise; n3_cruise];
u_prev_ship = x(1);

%% MAIN SIMULATION LOOP ===================================================
for i = 1:length(t)
    t_step = tic;
% 1) Update active segment only, then define segment / berth references
t_seg = tic;
wp_idx = updateWaypointIndexManaged(x, waypoints, wp_idx, R_accept);

xte = computeXTE(x, waypoints, wp_idx);
n_wps = size(waypoints, 1);
last_seg_idx = max(1, n_wps - 1);

seg_start_idx = min(max(1, wp_idx), last_seg_idx);
seg_end_idx   = min(seg_start_idx + 1, n_wps);

wp_start_xy_route = waypoints(seg_start_idx, 1:2)';
wp_end_xy_route   = waypoints(seg_end_idx,   1:2)';

wp_start_xy = wp_start_xy_route;
wp_end_xy   = wp_end_xy_route;

seg_vec_route = wp_end_xy_route - wp_start_xy_route;
if norm(seg_vec_route) > 1e-9
    chi_seg = atan2(seg_vec_route(2), seg_vec_route(1));
else
    chi_seg = x(6);
end

on_final_waypoint = (seg_end_idx == n_wps);
d_to_berth = inf;
berth_preview_active = false;
berth_mode_active = false;

goal_heading_enable = false;
goal_heading_rad = chi_seg;
chi_ctrl = chi_seg;

if berth_cfg.enabled
    d_to_berth = norm(x(4:5) - berth_cfg.target_xy(:));
    preview_first_seg = max(1, last_seg_idx - max(1, berth_cfg.prepare_last_n_segments) + 1);
    berth_preview_active = (seg_start_idx >= preview_first_seg) || (d_to_berth <= 1.35 * berth_cfg.activate_dist_m);
    berth_mode_active = on_final_waypoint || (d_to_berth <= berth_cfg.activate_dist_m);

    if berth_preview_active
        goal_heading_enable = true;
        goal_heading_rad = deg2rad(berth_cfg.heading_deg);
    end

    if berth_mode_active
        wp_end_xy = berth_cfg.target_xy(:);
        goal_heading_enable = true;
        goal_heading_rad = deg2rad(berth_cfg.heading_deg);
        if berth_cfg.use_corridor
            chi_ctrl = deg2rad(berth_cfg.corridor_heading_deg);
        else
            chi_ctrl = goal_heading_rad;
        end
    end
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
                map_edge_set, x(4:5), chi_ctrl, max_hp_slots, lookahead_now, half_width_now, [], 0, 1.0);
        elseif use_map_circles
            obs_map = selectMapObstaclesFromSamples( ...
                map_sample_pts, x(4:5), chi_ctrl, max_map_obstacles, ...
                lookahead_now, half_width_now, map_sample_radius_m);
            obs_local = [obs_local, obs_map];
        end
    end

    obs_dyn = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {});
    obs_dyn_latent = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {});

    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        obs_dyn = dynamicToCircleObstacles(dynamic_obstacles, dynamic_obs_nmpc_guard_m);

        if isfield(dynamic_latent_awareness, 'enabled') && dynamic_latent_awareness.enabled
            obs_dyn_latent = buildLatentDynamicAwarenessObstacles( ...
                dynamic_obstacles, x(4:5), dynamic_latent_awareness);
        end

        % Force same field schema and row shape before concatenation
        obs_local      = normalizeObstacleSchema(obs_local);
        obs_dyn_latent = normalizeObstacleSchema(obs_dyn_latent);
        obs_dyn        = normalizeObstacleSchema(obs_dyn);

        obs_local = [obs_local, obs_dyn_latent, obs_dyn];
        obs_pack_drift_log(i) = computeDynamicPackagingDrift(dynamic_obstacles, obs_local);
    end

obs_time_log(i) = toc(t_seg);

% 4) Solve options: segment cost + tube cost + endpoint pull + optional berth mode
t_seg = tic;
solve_opts = struct();

solve_opts.state_weights_diag = diag(nmpc_cfg.Q);
solve_opts.input_weights_diag = diag(nmpc_cfg.R);
solve_opts.rate_weights_diag  = diag(nmpc_cfg.R_rate);

solve_opts.stage_state_cost_scale         = 1.0;
solve_opts.stage_input_tracking_scale     = 1.0;
solve_opts.terminal_cost_scale            = 1.0;
solve_opts.terminal_actuator_cost_scale   = 1.0;
solve_opts.terminal_forward_cost_scale    = 1.0;

solve_opts.collision_clearance_m = hull_cfg.nmpc_clearance_m;
solve_opts.path_xte_weight       = path_cost_cfg.W_xte_heavy;
solve_opts.path_along_weight     = path_cost_cfg.W_along;
solve_opts.path_tube_half_width_m= path_cost_cfg.W_tube_m;
solve_opts.soft_speed_cap_weight = path_cost_cfg.soft_speed_cap_weight;
solve_opts.soft_speed_cap_mps    = path_cost_cfg.soft_speed_cap_mps;

solve_opts.goal_heading_enable = goal_heading_enable;
solve_opts.goal_heading_rad    = goal_heading_rad;
solve_opts.is_final_waypoint   = on_final_waypoint;

solve_opts.enable_soft_obstacles = soft_obstacle_cfg.enabled;
solve_opts.soft_obs_max_m        = soft_obstacle_cfg.max_slack_m;
solve_opts.map_halfplanes        = map_halfplanes;

% Safe defaults for optional scheduled fields
solve_opts.terminal_goal_pos_weight     = terminal_goal_cfg.pos_weight;
solve_opts.terminal_goal_heading_weight = terminal_goal_cfg.heading_weight;
solve_opts.terminal_stop_u_weight       = terminal_goal_cfg.stop_u_weight;
solve_opts.terminal_stop_v_weight       = terminal_goal_cfg.stop_v_weight;
solve_opts.terminal_stop_r_weight       = terminal_goal_cfg.stop_r_weight;

solve_opts.enable_terminal_pose   = false;
solve_opts.term_pose_eps_xy_m     = terminal_goal_cfg.term_pose_eps_xy_m(:);
solve_opts.term_pose_eps_psi_rad  = deg2rad(terminal_goal_cfg.term_pose_eps_psi_deg);
solve_opts.term_vel_max_u_mps     = terminal_goal_cfg.term_vel_max_u_mps;
solve_opts.term_vel_max_v_mps     = terminal_goal_cfg.term_vel_max_v_mps;
solve_opts.term_vel_max_r_radps   = terminal_goal_cfg.term_vel_max_r_radps;
solve_opts.term_pose_slack_max    = terminal_goal_cfg.term_pose_slack_max;

solve_opts.enable_berth_corridor = false;
if isfield(berth_cfg, 'corridor_origin_xy') && ~isempty(berth_cfg.corridor_origin_xy)
    solve_opts.berth_corridor_origin_xy = berth_cfg.corridor_origin_xy(:);
else
    solve_opts.berth_corridor_origin_xy = [0; 0];
end
if isfield(berth_cfg, 'corridor_heading_deg') && ~isempty(berth_cfg.corridor_heading_deg)
    solve_opts.berth_corridor_heading_rad = deg2rad(berth_cfg.corridor_heading_deg);
else
    solve_opts.berth_corridor_heading_rad = 0;
end
if isfield(berth_cfg, 'corridor_half_width_m') && ~isempty(berth_cfg.corridor_half_width_m)
    solve_opts.berth_corridor_half_width_m = berth_cfg.corridor_half_width_m;
else
    solve_opts.berth_corridor_half_width_m = inf;
end
if isfield(berth_cfg, 'corridor_along_min_m') && ~isempty(berth_cfg.corridor_along_min_m)
    solve_opts.berth_corridor_along_min_m = berth_cfg.corridor_along_min_m;
else
    solve_opts.berth_corridor_along_min_m = -inf;
end
if isfield(berth_cfg, 'corridor_along_max_m') && ~isempty(berth_cfg.corridor_along_max_m)
    solve_opts.berth_corridor_along_max_m = berth_cfg.corridor_along_max_m;
else
    solve_opts.berth_corridor_along_max_m = inf;
end

solve_opts.R_rate_scale_obs  = 1.0;
solve_opts.map_barrier_weight = sched_cfg.map_barrier_w_base;
solve_opts.u_min_forward      = nmpc_cfg.u_min_forward;

solve_opts.n3_max              = 0;
solve_opts.max_azimuth_split   = azipod_sync_cfg.transit_alpha_split_rad;
solve_opts.max_stern_cmd_split = azipod_sync_cfg.transit_stern_split_rpm;

solve_opts.map_soft_margin_m = sched_cfg.map_soft_margin_m;

% CONTINUOUS GEOMETRY-AWARE SCHEDULER
U_now = sqrt(x(1)^2 + x(2)^2);
U_ship_world = [cos(x(6))*x(1) - sin(x(6))*x(2); sin(x(6))*x(1) + cos(x(6))*x(2)];
d_to_active_end = norm(x(4:5) - wp_end_xy);


% --- 1) Clearance-aware tightness ------------------------------------
d_edge_now = inf;
if enable_map_obstacles && ~isempty(map_edge_set)
    d_edge_now = nearestDistanceToMapEdges(x(4:5), map_edge_set);
end
lambda_clearance = revRamp01(d_edge_now, sched_cfg.clearance_lo_m, sched_cfg.clearance_hi_m);
lambda_channel   = revRamp01(2.0*d_edge_now, sched_cfg.channel_width_lo_m, sched_cfg.channel_width_hi_m);
lambda_tight = max(lambda_clearance, lambda_channel);

% --- 2) Stopping-distance awareness ----------------------------------
a_eff = max(0.05, sched_cfg.brake_eff_mps2);
d_stop = U_now^2 / (2 * a_eff) + sched_cfg.stop_margin_m;
d_stop_log(i) = d_stop;
d_free_candidates = []; d_free_candidates(end+1) = max(0, d_to_active_end);
if berth_cfg.enabled
    d_entry = distanceToBerthCorridorEntry(x(4:5), berth_cfg);
    d_endcorr = distanceToBerthCorridorEnd(x(4:5), berth_cfg);
    if isfinite(d_entry),   d_free_candidates(end+1) = max(0, d_entry);   end
    if isfinite(d_endcorr), d_free_candidates(end+1) = max(0, d_endcorr); end
end
if isfinite(d_edge_now), d_free_candidates(end+1) = max(0, d_edge_now); end
d_free_ref = min(d_free_candidates);
lambda_stop = revRamp01(d_free_ref - d_stop, 0, sched_cfg.stop_buffer_m);

% --- 3) Turn severity awareness --------------------------------------
turn_next_deg = getNextTurnAngleDeg(waypoints, seg_start_idx);
lambda_turn_angle = ramp01(turn_next_deg, sched_cfg.turn_angle_lo_deg, sched_cfg.turn_angle_hi_deg);
lambda_turn_dist  = revRamp01(d_to_active_end, sched_cfg.turn_dist_near_m, sched_cfg.turn_dist_far_m);
lambda_turn = lambda_turn_angle * lambda_turn_dist;

% --- 4) Continuous berth activation ----------------------------------
if berth_cfg.enabled
    d_berth = norm(x(4:5) - berth_cfg.target_xy(:));
    lambda_berth = revRamp01(d_berth, sched_cfg.berth_d_full_m, sched_cfg.berth_d_activate_m);
else
    lambda_berth = 0;
end

% --- 5) Dynamic Obstacle TTC / Yield / Return Lambdas ----------------
lambda_ttc   = 0;
lambda_yield = 0;
lambda_return= 0;
small_eps = 1e-6;

if enable_dynamic_obstacles && ~isempty(obs_dyn)
    for j = 1:length(obs_dyn)
        r      = obs_dyn(j).position(:) - x(4:5);
        v_obs  = obs_dyn(j).speed * [cos(obs_dyn(j).heading); sin(obs_dyn(j).heading)];
        v_rel  = v_obs - U_ship_world;
        d_rel  = norm(r) + eps;
        d_dot  = (r' * v_rel) / d_rel;
        v_close= max(0, -d_dot);
        
        % CPA
        v_rel_sq = max(v_rel'*v_rel, eps);
        t_cpa    = max(0, -(r'*v_rel) / v_rel_sq);
        d_cpa    = norm(r + t_cpa * v_rel);
        
        % Sector filter (cosine of relative bearing)
        cos_beta = (r'*[cos(x(6)); sin(x(6))]) / d_rel;
        lam_sec  = sat01((cos_beta - sched_cfg.cos_beta_max) / (1 - sched_cfg.cos_beta_max + eps));
        
        % Base TTC activation (exact math from prompt)
        l_ttc = sat01((sched_cfg.T_cpa_far - t_cpa)/(sched_cfg.T_cpa_far - sched_cfg.T_cpa_near + eps)) * ...
                sat01((sched_cfg.d_cpa_th - d_cpa)/(sched_cfg.d_cpa_th + eps)) * ...
                sat01(v_close / (sched_cfg.v_close_ref + eps));
        
        lambda_ttc   = max(lambda_ttc, l_ttc * lam_sec);
        lambda_yield = max(lambda_yield, l_ttc * lam_sec * sat01((-d_dot) / (sched_cfg.d_dot_ref + eps)));
        l_return = sat01((sched_cfg.d_cpa_th - d_cpa) / (sched_cfg.d_cpa_th + small_eps)) * sat01(d_dot / (sched_cfg.d_dot_ref + small_eps));
        lambda_return = max(lambda_return, l_return * lam_sec);

    end
end


% --- 6) Master caution blend -----------------------------------------
lambda_total = max([lambda_tight, lambda_stop, lambda_turn, lambda_berth, lambda_yield]);

% APPLY CONTINUOUS SCHEDULING TO NMPC SOLVE OPTIONS
% Speed shaping (geom + dynamic yield)
U_cap_tight = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_tight_mps, lambda_tight);
U_cap_stop  = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_tight_mps, lambda_stop);
U_cap_turn  = lerp(sched_cfg.u_cruise_mps, max(1.2, sched_cfg.u_tight_mps), lambda_turn);
U_cap_berth = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_berth_mps, lambda_berth);
U_cap_dyn   = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_yield, lambda_yield);
solve_opts.soft_speed_cap_mps = min([U_cap_tight, U_cap_stop, U_cap_turn, U_cap_berth, U_cap_dyn]);
solve_opts.soft_speed_cap_weight = lerp(sched_cfg.soft_speed_weight_far, sched_cfg.soft_speed_weight_near, lambda_total);

% Along-track reward scheduling (yield reduces progress urgency)
solve_opts.path_along_weight = lerp(path_cost_cfg.W_along, sched_cfg.w_along_min, lambda_yield);

% Tube & XTE scheduling (return tightens path after CPA)
geom_caution = max([lambda_tight, lambda_turn, lambda_berth]);
lambda_path_return = max(geom_caution, lambda_return);

solve_opts.path_tube_half_width_m = ...
    lerp(sched_cfg.tube_far_m, sched_cfg.tube_near_m, lambda_path_return);

solve_opts.path_xte_weight = ...
    lerp(sched_cfg.xte_weight_far, sched_cfg.xte_weight_near, lambda_path_return);

solve_opts.path_heading_weight = max([ ...
    0.35 * lambda_turn  * sched_cfg.heading_weight_turn, ...
    0.10 * lambda_berth * sched_cfg.heading_weight_berth, ...
    lambda_return       * sched_cfg.heading_weight_return]);


% ROUTE-ORDER RESTORE / GATE TIGHTENING
if lambda_berth < 0.05 && lambda_yield < 0.10 && lambda_return < 0.15
    if d_to_active_end <= route_follow_cfg.transit_goal_dist_near_m
        transit_goal_w = route_follow_cfg.transit_goal_pos_weight_near;
    elseif d_to_active_end <= route_follow_cfg.transit_goal_dist_mid_m
        transit_goal_w = route_follow_cfg.transit_goal_pos_weight_mid;
    else
        transit_goal_w = route_follow_cfg.transit_goal_pos_weight_far;
    end

    near_gate_lam = revRamp01(d_to_active_end, 40, route_follow_cfg.tighten_near_gate_dist_m);

    solve_opts.path_tube_half_width_m = min( ...
        solve_opts.path_tube_half_width_m, ...
        lerp(path_cost_cfg.W_tube_m, route_follow_cfg.tighten_tube_m, near_gate_lam));

    solve_opts.path_xte_weight = max( ...
        solve_opts.path_xte_weight, ...
        lerp(path_cost_cfg.W_xte_heavy, route_follow_cfg.tighten_xte_weight, near_gate_lam));

    solve_opts.terminal_goal_pos_weight = max( ...
        solve_opts.terminal_goal_pos_weight, transit_goal_w);

    if turn_next_deg >= route_follow_cfg.sharp_turn_deg
        sharp_turn_lam = revRamp01(d_to_active_end, 40, route_follow_cfg.sharp_turn_heading_enable_dist_m);

        solve_opts.path_heading_weight = max( ...
            solve_opts.path_heading_weight, ...
            sharp_turn_lam * route_follow_cfg.sharp_turn_heading_weight);

        solve_opts.terminal_goal_pos_weight = max( ...
            solve_opts.terminal_goal_pos_weight, ...
            transit_goal_w * lerp(1.0, route_follow_cfg.sharp_turn_goal_weight_gain, sharp_turn_lam));
    end
end

% Surge lower bound scheduling (yield allows near-stop)
desired_u_min_forward = lerp(sched_cfg.u_min_forward_far, sched_cfg.u_yield, lambda_yield);
solve_opts.u_min_forward = desired_u_min_forward;

% Heading & stop scheduling (bert   h/turn vs return)
solve_opts.terminal_goal_heading_weight = max(lambda_turn * sched_cfg.heading_weight_turn, ...
    lambda_berth * sched_cfg.heading_weight_berth) + lambda_return * 15.0;

if lambda_berth >= max(0.20, lambda_turn)
    solve_opts.goal_heading_enable = true; solve_opts.goal_heading_rad = deg2rad(berth_cfg.heading_deg);
elseif lambda_turn > 0.05 || lambda_return > 0.1
    solve_opts.goal_heading_enable = true; solve_opts.goal_heading_rad = chi_seg;
else
    solve_opts.goal_heading_enable = false;
end

solve_opts.terminal_goal_pos_weight = max( ...
    solve_opts.terminal_goal_pos_weight, ...
    lerp(sched_cfg.term_pos_weight_far, sched_cfg.term_pos_weight_berth, lambda_berth));
solve_opts.terminal_stop_u_weight = lerp(sched_cfg.stop_u_weight_far, sched_cfg.stop_u_weight_berth, lambda_berth);
solve_opts.terminal_stop_v_weight = lerp(0.0, sched_cfg.stop_v_weight_berth, lambda_berth);
solve_opts.terminal_stop_r_weight = lerp(0.0, sched_cfg.stop_r_weight_berth, lambda_berth);

% EARLY BERTH PREVIEW
if berth_preview_active && ~berth_mode_active
    solve_opts.goal_heading_enable = true;
    solve_opts.goal_heading_rad = deg2rad(berth_cfg.heading_deg);

    solve_opts.terminal_goal_heading_weight = max( ...
        solve_opts.terminal_goal_heading_weight, ...
        berth_cfg.preview_heading_weight);

    solve_opts.terminal_goal_pos_weight = max( ...
        solve_opts.terminal_goal_pos_weight, ...
        berth_cfg.preview_goal_weight_gain * solve_opts.terminal_goal_pos_weight);
end

% Terminal envelope
if berth_cfg.enabled && (lambda_berth > 0.05)
    solve_opts.enable_terminal_pose = true;
    solve_opts.term_pose_eps_xy_m = lerp(sched_cfg.term_xy_far_m, sched_cfg.term_xy_near_m, lambda_berth);
    solve_opts.term_pose_eps_psi_rad = lerp(sched_cfg.term_psi_far_rad, sched_cfg.term_psi_near_rad, lambda_berth);
    solve_opts.term_vel_max_u_mps = lerp(sched_cfg.term_u_far_mps, sched_cfg.term_u_near_mps, lambda_berth);
    solve_opts.term_vel_max_v_mps = lerp(sched_cfg.term_v_far_mps, sched_cfg.term_v_near_mps, lambda_berth);
    solve_opts.term_vel_max_r_radps = lerp(sched_cfg.term_r_far_radps, sched_cfg.term_r_near_radps, lambda_berth);
    solve_opts.term_pose_slack_max = lerp(2.0, berth_cfg.pose_slack_max, lambda_berth);
end

% Map corridor & relevance
if berth_cfg.enabled && berth_cfg.use_corridor && (lambda_berth > 0.10)
    solve_opts.enable_berth_corridor = true;
    solve_opts.berth_corridor_origin_xy = berth_cfg.corridor_origin_xy(:);
    solve_opts.berth_corridor_heading_rad = deg2rad(berth_cfg.corridor_heading_deg);
    solve_opts.berth_corridor_half_width_m = lerp(sched_cfg.corridor_half_width_far_m, sched_cfg.corridor_half_width_near_m, lambda_berth);
    solve_opts.berth_corridor_along_min_m = berth_cfg.corridor_along_min_m;
    solve_opts.berth_corridor_along_max_m = berth_cfg.corridor_along_max_m;
end

% Smoothness scaling (yield penalizes jerky avoidance arcs)
solve_opts.R_rate_scale_obs = 1.0 + lambda_yield * (sched_cfg.R_rate_scale_yield - 1.0);

% Soft map barrier weight (tight + return phases)
solve_opts.map_barrier_weight = sched_cfg.map_barrier_w_base + ...
    max(lambda_tight, lambda_return) * (sched_cfg.map_barrier_w_near - sched_cfg.map_barrier_w_base);

% Hard structural limits only
if berth_mode_active
    solve_opts.n3_max = berth_cfg.n3_max;
    solve_opts.max_azimuth_split = berth_cfg.max_azimuth_split_rad;
    solve_opts.max_stern_cmd_split = berth_cfg.max_stern_cmd_split_rpm;
    desired_u_min_forward = min(desired_u_min_forward, berth_cfg.u_min_final_mps);
else
    solve_opts.n3_max = 0;
    solve_opts.max_azimuth_split = azipod_sync_cfg.transit_alpha_split_rad;
    solve_opts.max_stern_cmd_split = azipod_sync_cfg.transit_stern_split_rpm;
end

solve_opts.u_min_forward = desired_u_min_forward;

if on_final_waypoint, solve_opts.enable_terminal_pose = true; end

% Update logs
lambda_tight_log(i) = lambda_tight; lambda_stop_log(i) = lambda_stop;
lambda_turn_log(i)  = lambda_turn;  lambda_berth_log(i)= lambda_berth;
lambda_total_log(i) = lambda_total; lambda_yield_log(i)= lambda_yield;
lambda_return_log(i)= lambda_return; lambda_ttc_log(i)  = lambda_ttc;
u_cap_log(i)        = solve_opts.soft_speed_cap_mps; d_edge_log(i) = d_edge_now;
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

    % 6) Plant integration
    t_seg = tic;
    x = rk4Step9(x, u_opt, dt);
    integr_time_log(i) = toc(t_seg);
    du_surge = x(1) - u_prev_ship;
    brake_margin_log(i) = du_surge + max_brake_rate * dt;
    u_prev_ship = x(1);
    step_time_log(i) = toc(t_step);
    rt_ratio_log(i) = step_time_log(i) / max(dt, 1e-9);

    % 7) Collision checks
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

    % 8) Logging
    steps = i;
    traj(:, i+1) = x;
    ctrl(:, i) = u_opt;
    solve_ok(i) = info.success;
    xte_log(i) = xte;
    heading_err_log(i) = wrapToPi(chi_ctrl - x(6));
    wp_idx_log(i) = wp_idx;
    psi_ref_log(i) = chi_ctrl;
    u_prev = u_opt;

    % 9) Progress print
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
    % 10) Mission complete
    mission_goal_xy = waypoints(end,1:2)';
    mission_capture_radius = R_accept_final;
    mission_capture_radius_soft = R_accept_final_soft;
    mission_capture_speed = final_capture_speed_mps;
    mission_hold_steps = final_capture_steps_needed;
    mission_ready = on_final_waypoint || berth_mode_active;

    if berth_cfg.enabled
        mission_goal_xy = berth_cfg.target_xy(:);
        mission_capture_radius = berth_cfg.capture_radius_m;
        mission_capture_radius_soft = max(berth_cfg.capture_radius_m, safe_terminal_radius_m);
        mission_capture_speed = berth_cfg.capture_speed_mps;
        mission_hold_steps = max(1, ceil(final_capture_hold_s / dt));
    end

    d_final_now = norm(x(4:5) - mission_goal_xy);
    U_now_ship = hypot(x(1), x(2));

    hard_final_hit = mission_ready && (d_final_now < mission_capture_radius);
    soft_final_hit = mission_ready && ...
        (d_final_now < mission_capture_radius_soft) && ...
        (U_now_ship <= mission_capture_speed);

    safe_terminal_hit = mission_ready && enable_safe_terminal_stop && final_is_map_constrained && ...
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

    if hard_final_hit || final_capture_count >= mission_hold_steps || ...
            safe_terminal_count >= safe_terminal_steps_needed
        if hard_final_hit
            fprintf('✓ FINAL TARGET REACHED at t=%.1f s!', t(i));
        elseif safe_terminal_count >= safe_terminal_steps_needed
            fprintf(['✓ FINAL TARGET SAFELY CAPTURED: ', 'radius %.1f m, U<=%.1f m/s for %.1f s at t=%.1f s!'],...
                safe_terminal_radius_m, safe_terminal_speed_mps, safe_terminal_hold_s, t(i));
        else
            fprintf('✓ FINAL TARGET CAPTURED at t=%.1f s!', t(i));
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
t_sim    = (0:steps) * dt;
X_pred_hist = X_pred_hist(1:steps);
step_time_log   = step_time_log(1:steps);
guide_time_log  = guide_time_log(1:steps);
obs_time_log = obs_time_log(1:steps);
lambda_tight_log = lambda_tight_log(1:steps);
lambda_stop_log  = lambda_stop_log(1:steps);
lambda_turn_log  = lambda_turn_log(1:steps);
lambda_berth_log = lambda_berth_log(1:steps);
lambda_yield_log = lambda_yield_log(1:steps);
lambda_return_log= lambda_return_log(1:steps);
lambda_ttc_log   = lambda_ttc_log(1:steps);
lambda_total_log = lambda_total_log(1:steps);
u_cap_log        = u_cap_log(1:steps);
d_edge_log       = d_edge_log(1:steps);
d_stop_log       = d_stop_log(1:steps);
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
            thr_cfg = struct();
            thr_cfg.pos_body_m = [
                -0.45 * hull_cfg.length_m, -0.5 * hull_cfg.beam_m;
                -0.45 * hull_cfg.length_m,  0.5 * hull_cfg.beam_m;
                 0.17 * hull_cfg.length_m,  0.0 * hull_cfg.beam_m
            ];
            thr_cfg.types = {'azipod', 'azipod', 'bow'};
            thr_cfg.n_max = [160 160 140];
            thr_cfg.alpha_idx = [1 2 0];
            thr_cfg.rpm_idx = [3 4 5];
            thr_cfg.alpha_fixed = [NaN NaN pi/2];
            anim_cfg.thrusterCfg = thr_cfg;
            anim_cfg.showControlPlot = true;
            anim_cfg.showSpeedPlot = false;
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
                anim_cfg.videoFile = fullfile(record_output_dir, ['run_nmpc_' ts '.mp4']);
            end

            animateSimResult(traj, waypoints, t_sim, harbor_anim, anim_cfg);
        catch ME_anim
            warning(ME_anim.identifier, 'Animation failed: %s', ME_anim.message);
        end
    end
catch ME
    output_gen_error = true;
    warning(ME.identifier, 'Output generation failed: %s', ME.message);
end

if enable_terminal_log_recording
    diary off;
end

if output_gen_error
    fprintf('  Output generation had errors, but simulation completed.\n');
end

%% LOCAL FUNCTIONS ========================================================



function wp_idx = updateWaypointIndexManaged(x, wp, wp_idx, R_accept)
% Robust waypoint progression.
% Only decides segment handoff.
% No heading generation. No speed shaping.

    n_wps = size(wp, 1);
    if n_wps <= 1
        wp_idx = 1;
        return;
    end

    last_seg_idx = n_wps - 1;
    wp_idx = min(max(1, wp_idx), last_seg_idx);
    pos = [x(4); x(5)];

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

        sharp_turn = (turn_angle_deg >= 28);
        approaching_final = (wp_idx >= n_wps - 2);

        if sharp_turn || approaching_final
            advance_now = ...
                (d_to_waypoint <= max(30, 0.45 * R_accept)) && ...
                (proj >= 0.85) && ...
                (xte_seg <= max(30, 0.70 * R_accept));
        else
            near_gate = (d_to_waypoint <= max(45, 0.80 * R_accept));
            passed_gate = (proj >= 1.00) && (xte_seg <= max(55, 0.90 * R_accept));
            missed_gate = (proj >= 0.96) && (d_to_waypoint <= max(120, 1.70 * R_accept));

            better_next = false;
            if (wp_idx + 2) <= n_wps
                p_next = wp(wp_idx + 2, 1:2)';
                d_to_next = norm(pos - p_next);
                better_next = (proj >= 0.92) && ...
                              (d_to_next < d_to_waypoint) && ...
                              (xte_seg <= max(90, 1.50 * R_accept));
            end

            advance_now = near_gate || passed_gate || missed_gate || better_next;
        end

        if advance_now
            old_wp_idx = wp_idx;
            wp_idx = wp_idx + 1;
            fprintf('  [wp-advance] %d -> %d (proj=%.2f, d_wp=%.1f m, xte=%.1f m, turn=%.1f deg)', ...
                old_wp_idx, wp_idx, proj, d_to_waypoint, xte_seg, turn_angle_deg);
            return;
        else
            return;
        end
    end
end


function ang_deg = getWaypointTurnAngleDeg(wp, seg_start_idx)
% Turn angle between active segment and next segment
    n_wps = size(wp, 1);
    ang_deg = 0;

    i1 = seg_start_idx;
    i2 = min(seg_start_idx + 1, n_wps);
    i3 = min(seg_start_idx + 2, n_wps);
    if i3 <= i2
        return;
    end

    s1 = wp(i2,1:2)' - wp(i1,1:2)';
    s2 = wp(i3,1:2)' - wp(i2,1:2)';
    n1 = norm(s1);
    n2 = norm(s2);
    if n1 < 1e-9 || n2 < 1e-9
        return;
    end

    cang = dot(s1, s2) / max(n1*n2, 1e-9);
    cang = max(-1.0, min(1.0, cang));
    ang_deg = rad2deg(acos(cang));
end


function cfg = normalizeBerthCfg(cfg, waypoints)
% Fill berth config defaults and shapes
    if nargin < 1 || isempty(cfg)
        cfg = struct();
    end

    cfg.enabled = logical(getOr(cfg, 'enabled', false));
    final_xy = waypoints(end,1:2)';
    cfg.target_xy = getOr(cfg, 'target_xy', final_xy);
    cfg.target_xy = cfg.target_xy(:);
    if numel(cfg.target_xy) < 2
        cfg.target_xy = final_xy;
    else
        cfg.target_xy = cfg.target_xy(1:2);
    end

    if size(waypoints,1) >= 2
        v = waypoints(end,1:2)' - waypoints(end-1,1:2)';
        heading_deg_default = rad2deg(atan2(v(2), v(1)));
    else
        heading_deg_default = 0;
    end

    cfg.heading_deg = getOr(cfg, 'heading_deg', heading_deg_default);
    cfg.prepare_last_n_segments = max(1, round(getOr(cfg, 'prepare_last_n_segments', 2)));
    cfg.activate_dist_m = getOr(cfg, 'activate_dist_m', 260);
    cfg.preview_heading_weight = getOr(cfg, 'preview_heading_weight', 18.0);
    cfg.preview_goal_weight_gain = getOr(cfg, 'preview_goal_weight_gain', 1.35);
    cfg.capture_radius_m = getOr(cfg, 'capture_radius_m', 12);
    cfg.capture_speed_mps = getOr(cfg, 'capture_speed_mps', 0.35);

    cfg.use_corridor = logical(getOr(cfg, 'use_corridor', false));
    cfg.corridor_origin_xy = getOr(cfg, 'corridor_origin_xy', final_xy);
    cfg.corridor_origin_xy = cfg.corridor_origin_xy(:);
    if numel(cfg.corridor_origin_xy) < 2
        cfg.corridor_origin_xy = final_xy;
    else
        cfg.corridor_origin_xy = cfg.corridor_origin_xy(1:2);
    end
    cfg.corridor_heading_deg = getOr(cfg, 'corridor_heading_deg', cfg.heading_deg);
    cfg.corridor_half_width_m = getOr(cfg, 'corridor_half_width_m', 12);
    cfg.corridor_along_min_m = getOr(cfg, 'corridor_along_min_m', -200);
    cfg.corridor_along_max_m = getOr(cfg, 'corridor_along_max_m', 20);

    cfg.pose_eps_xy_m = getOr(cfg, 'pose_eps_xy_m', [4; 4]);
    cfg.pose_eps_xy_m = cfg.pose_eps_xy_m(:);
    if numel(cfg.pose_eps_xy_m) == 1
        cfg.pose_eps_xy_m = [cfg.pose_eps_xy_m; cfg.pose_eps_xy_m];
    else
        cfg.pose_eps_xy_m = cfg.pose_eps_xy_m(1:2);
    end

    cfg.pose_eps_psi_deg = getOr(cfg, 'pose_eps_psi_deg', 3.0);
    cfg.vel_max_u_mps = getOr(cfg, 'vel_max_u_mps', 0.30);
    cfg.vel_max_v_mps = getOr(cfg, 'vel_max_v_mps', 0.20);
    cfg.vel_max_r_radps = getOr(cfg, 'vel_max_r_radps', deg2rad(2.5));
    cfg.pose_slack_max = getOr(cfg, 'pose_slack_max', 0.5);
    cfg.u_min_final_mps = getOr(cfg, 'u_min_final_mps', -0.8);
    cfg.n3_max = getOr(cfg, 'n3_max', 110);
    cfg.max_azimuth_split_rad = getOr(cfg, 'max_azimuth_split_rad', deg2rad(10));
    cfg.max_stern_cmd_split_rpm = getOr(cfg, 'max_stern_cmd_split_rpm', 8);
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
% Convert active dynamic obstacle states to NMPC-compatible circle obstacles.
% Carries speed & heading for scheduler/container use.

    obs_dyn = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {});

    if isempty(dynamic_obstacles)
        return;
    end

    dynamic_obstacles = dynamic_obstacles(:).';   % force row orientation

    if nargin < 2 || isempty(radius_guard_m)
        radius_guard_m = 0;
    end

    out_idx = 0;
    for k = 1:length(dynamic_obstacles)
        is_enabled = isfield(dynamic_obstacles(k), 'enabled') && dynamic_obstacles(k).enabled;
        is_active  = isfield(dynamic_obstacles(k), 'active')  && dynamic_obstacles(k).active;
        if ~(is_enabled && is_active)
            continue;
        end

        if isfield(dynamic_obstacles(k), 'moving') && ~dynamic_obstacles(k).moving
            continue;
        end

        out_idx = out_idx + 1;

        pos_k = dynamic_obstacles(k).position(1:2);
        obs_dyn(out_idx).position = pos_k(:);
        obs_dyn(out_idx).radius   = dynamic_obstacles(k).radius + radius_guard_m;

        if isfield(dynamic_obstacles(k), 'speed') && ~isempty(dynamic_obstacles(k).speed)
            obs_dyn(out_idx).speed = dynamic_obstacles(k).speed;
        else
            obs_dyn(out_idx).speed = 0;
        end

        if isfield(dynamic_obstacles(k), 'heading') && ~isempty(dynamic_obstacles(k).heading)
            obs_dyn(out_idx).heading = dynamic_obstacles(k).heading;
        else
            obs_dyn(out_idx).heading = 0;
        end
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
        obs_struct = obs_in(:).';
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



function y = sat01(x)
    y = max(0, min(1, x));
end

function y = ramp01(x, x0, x1)
    if x1 <= x0, y = double(x >= x1);
    else, y = sat01((x - x0) / (x1 - x0)); end
end

function y = revRamp01(x, x_lo, x_hi)
    if x_hi <= x_lo, y = double(x <= x_lo);
    else, y = sat01((x_hi - x) / (x_hi - x_lo)); end
end

function y = lerp(a, b, lam)
    y = (1 - lam) * a + lam * b;
end

function ang_deg = wrapTo180Deg(ang_deg)
    ang_deg = mod(ang_deg + 180, 360) - 180;
end

function d = distanceToBerthCorridorEnd(pos_xy, berth_cfg)
    if ~isfield(berth_cfg, 'corridor_origin_xy') || isempty(berth_cfg.corridor_origin_xy)
        d = inf; return; end
    p = pos_xy(:) - berth_cfg.corridor_origin_xy(:);
    psi = deg2rad(berth_cfg.corridor_heading_deg); t_hat = [cos(psi); sin(psi)];
    d = berth_cfg.corridor_along_max_m - dot(p, t_hat);
end

function d = distanceToBerthCorridorEntry(pos_xy, berth_cfg)
    if ~isfield(berth_cfg, 'corridor_origin_xy') || isempty(berth_cfg.corridor_origin_xy)
        d = inf; return; end
    p = pos_xy(:) - berth_cfg.corridor_origin_xy(:);
    psi = deg2rad(berth_cfg.corridor_heading_deg); t_hat = [cos(psi); sin(psi)];
    d = dot(p, t_hat) - berth_cfg.corridor_along_min_m;
end

function turn_deg = getNextTurnAngleDeg(waypoints, seg_start_idx)
    n_wps = size(waypoints,1);
    if seg_start_idx < 1 || seg_start_idx+2 > n_wps, turn_deg = 0; return; end
    v1 = waypoints(seg_start_idx+1,:)' - waypoints(seg_start_idx,:)';
    v2 = waypoints(seg_start_idx+2,:)' - waypoints(seg_start_idx+1,:)';
    if norm(v1)<1e-9 || norm(v2)<1e-9, turn_deg = 0; return; end
    turn_deg = abs(wrapTo180Deg(rad2deg(atan2(v2(2),v2(1)) - atan2(v1(2),v1(1)))));
end

function obs_out = normalizeObstacleSchema(obs_in)
% Force common obstacle schema and row orientation before concatenation.
    obs_out = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {});
    if isempty(obs_in)
        return;
    end

    obs_in = obs_in(:).';   % always row-shaped
    n = numel(obs_in);

    obs_out = repmat(struct('position', [0;0], 'radius', 0, ...
                            'speed', 0, 'heading', 0), 1, n);

    for k = 1:n
        if ~isfield(obs_in(k), 'position') || ~isfield(obs_in(k), 'radius')
            error('Obstacle schema must contain at least position and radius.');
        end

        obs_out(k).position = obs_in(k).position(:);
        obs_out(k).radius   = obs_in(k).radius;

        if isfield(obs_in(k), 'speed') && ~isempty(obs_in(k).speed)
            obs_out(k).speed = obs_in(k).speed;
        end
        if isfield(obs_in(k), 'heading') && ~isempty(obs_in(k).heading)
            obs_out(k).heading = obs_in(k).heading;
        end
    end
end
