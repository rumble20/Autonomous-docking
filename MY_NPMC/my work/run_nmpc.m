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
% The penultimate point is a staging point north of the berth so the last
% segment becomes a real final docking leg instead of a shallow transit leg.
waypoints = [-2600, -1400; -2300, -1300; -2100, -1300; -1750, -1250; -1900, -1250;];

% Static obstacles: N-by-3 [x y radius] or struct array.
static_obstacles = [];

% Dynamic obstacles.
dynamic_obs_positions_xy = [-2400, -2400];
dynamic_obs_headings_deg = [90];
dynamic_obs_speeds_mps   = [3];
enable_dynamic_obstacles = false;
dynamic_obs_radius_m     = 25;
dynamic_obs_speed_mps    = 5;
dynamic_obs_nmpc_guard_m = 0;
dynamic_obs_start_mode   = 'proximity';   % immediate | proximity
dynamic_obs_trigger_distance_m = 300;
dynamic_obs_boundary_margin = 200;
dynamic_obs_boundary_policy = 'wrap';    

% BERTHING MODE ==========================================================
% This block fully replaces the old waypoint third-column final heading.
% Use it to tell the ship how to approach and park at the final target.
berth_cfg = struct();
berth_cfg.enabled = true;                 % true = use precise berthing mode
berth_cfg.target_xy = [waypoints(end, 1); waypoints(end, 2)];     % final parking position [x; y]
berth_cfg.heading_deg = 180;                % desired final ship heading
berth_cfg.prepare_last_n_segments = 3;    % start preparing on last route segments
berth_cfg.activate_dist_m = 400;          % force berth mode when this close to berth target
berth_cfg.final_leg_only = false;          % allow berth mode by proximity, even if a waypoint was skipped
berth_cfg.preview_heading_weight = 18.0;  % mild early heading preparation before full berth mode
berth_cfg.preview_goal_weight_gain = 1.35;
berth_cfg.capture_radius_m = 12;          % hard completion radius in berth mode
berth_cfg.capture_speed_mps = 0.35;       % completion speed in berth mode

% Final approach corridor (dependent on berthing target and heading)
% ===== Derive corridor from waypoints and berthing config =====
last_wp = waypoints(end, :)';             % final waypoint [x; y]
if size(waypoints, 1) > 1
    second_last_wp = waypoints(end-1, :)';  % approach reference point
    approach_vec = last_wp - second_last_wp;
    approach_dist = norm(approach_vec);
    if approach_dist > eps
        approach_unit = approach_vec / approach_dist;
    else
        approach_unit = [1; 0];  % fallback if waypoints are identical
    end
else
    approach_unit = [1; 0];  % default approach direction if only one waypoint
end

% Corridor starts from a point ahead of last waypoint along approach vector
corridor_approach_dist_m = 300;  % distance to start corridor from target
berth_cfg.corridor_origin_xy = last_wp + approach_unit * corridor_approach_dist_m;
berth_cfg.corridor_heading_deg = berth_cfg.heading_deg;  % inherit berthing heading
berth_cfg.use_corridor = true;
berth_cfg.corridor_half_width_m = 12;
berth_cfg.corridor_along_min_m = -corridor_approach_dist_m;  % from corridor origin to target
berth_cfg.corridor_along_max_m = 25;  % small overshoot tolerance

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
route_follow_cfg.sharp_turn_deg = 10;
route_follow_cfg.sharp_turn_goal_weight_gain = 2.0;
route_follow_cfg.sharp_turn_heading_weight = 30.0;
route_follow_cfg.sharp_turn_heading_enable_dist_m = 280;

% Cruise-speed suggestion for the NMPC soft speed cost.
cruise_speed_mps = 5.0;

% Dynamic latent awareness (optional planning-only envelopes).
dynamic_latent_awareness = struct();
dynamic_latent_awareness.enabled = true;
dynamic_latent_awareness.horizon_s = 8.0;          % e.g. 4 samples * 2 s
dynamic_latent_awareness.n_samples = 4;
dynamic_latent_awareness.radius_scale = 1.15;
dynamic_latent_awareness.awareness_distance_m = inf;
dynamic_latent_awareness.only_when_not_moving = false;


% Simulation.
T_final = 500;
R_accept = 90;
R_accept_final = 10;
R_accept_final_soft = 50;

wp_switch_cfg = struct();
wp_switch_cfg.allow_multi_skip = true;
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
use_light_theme = true;   % true = print-friendly light mode, false = current dark mode

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
path_cost_cfg.W_along = 6.0;
path_cost_cfg.W_tube_m = 40.0;
path_cost_cfg.soft_speed_cap_weight = 4.0;
path_cost_cfg.soft_speed_cap_mps = cruise_speed_mps;
path_cost_cfg.soft_speed_floor_weight = 1.0;
path_cost_cfg.soft_speed_floor_mps = max(0.0, cruise_speed_mps - 0.5);

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
terminal_goal_cfg.term_pose_slack_max = 5.0;


% Soft obstacle slack.
soft_obstacle_cfg = struct();
soft_obstacle_cfg.enabled = true;
soft_obstacle_cfg.max_slack_m = 15.0;
soft_obstacle_cfg.penalty_weight = 5e4;

% Twin-stern synchrony.
azipod_sync_cfg = struct();
azipod_sync_cfg.enabled = false;
azipod_sync_cfg.transit_alpha_split_rad = deg2rad(60);
azipod_sync_cfg.transit_stern_split_rpm = 60;
azipod_sync_cfg.final_alpha_split_rad   = deg2rad(20);
azipod_sync_cfg.final_stern_split_rpm   = 20;

% CONTINUOUS CAUTION / BERTHING SCHEDULER 
sched_cfg = struct();

% Speed scheduling
sched_cfg.u_cruise_mps = cruise_speed_mps;      % normal open-water preference
sched_cfg.u_tight_mps  = 1.4;                   % preferred speed in tight geometry
sched_cfg.u_turn_mps   = 0.35;                  % speed cap during sharp turns (180 berth rotation)
sched_cfg.u_berth_mps  = 0.45;                  % preferred speed near final berth

sched_cfg.soft_speed_weight_far  = 0.08;
sched_cfg.soft_speed_weight_near = 1.50;
sched_cfg.soft_speed_floor_ratio = 0.90;
sched_cfg.soft_speed_floor_weight_far = path_cost_cfg.soft_speed_floor_weight;
sched_cfg.soft_speed_floor_weight_near = 0.10;

% Forward lower bound scheduling
sched_cfg.u_min_forward_far  = u_min_forward;
sched_cfg.u_min_forward_near = 0.12;

% Tightness from map clearance
sched_cfg.clearance_lo_m = 45;   % full caution if nearer than this
sched_cfg.clearance_hi_m = 110;  % no caution if farther than this

% Optional channel width awareness
sched_cfg.channel_width_lo_m = 70;
sched_cfg.channel_width_hi_m = 140;

% Stopping-distance awareness
sched_cfg.brake_eff_mps2 = 0.55;   % effective deceleration for caution logic
sched_cfg.stop_margin_m  = 18;     % extra margin
sched_cfg.stop_buffer_m  = 60;     % extra comparison slack

% Turn severity awareness
sched_cfg.turn_angle_lo_deg = 15;
sched_cfg.turn_angle_hi_deg = 32;
sched_cfg.turn_dist_far_m   = 180;
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
sched_cfg.heading_weight_turn  = 24.0;
sched_cfg.heading_weight_berth = 900.0;
sched_cfg.heading_weight_return = 35.0;
sched_cfg.heading_weight_stop = 18.0;

sched_cfg.stop_u_weight_far   = 0.0;
sched_cfg.stop_u_weight_berth = 25.0;
sched_cfg.stop_v_weight_berth = 18.0;
sched_cfg.stop_r_weight_berth = 18.0;

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
sched_cfg.u_yield       = 1;    % m/s: crawl speed during yield
sched_cfg.w_along_min   = 0.05;    % minimal progress reward
sched_cfg.R_rate_scale_yield = 2;% control smoothness multiplier
sched_cfg.map_barrier_w_base = 0.0;
sched_cfg.map_barrier_w_near = 80.0;
sched_cfg.map_soft_margin_m = 18;

%% HELPER FUNCTION ALIASES ================================================
% Local helper functions were refactored to RunNmpcHelpers.m.
normalizeStaticObstacles = @RunNmpcHelpers.normalizeStaticObstacles;
normalizeBerthCfg = @RunNmpcHelpers.normalizeBerthCfg;
getOr = @RunNmpcHelpers.getOr;
buildMapHalfPlaneEdgeSet = @RunNmpcHelpers.buildMapHalfPlaneEdgeSet;
buildMapSamplePoints = @RunNmpcHelpers.buildMapSamplePoints;
estimateMapBounds = @RunNmpcHelpers.estimateMapBounds;
buildDynamicObstaclesFromConfig = @RunNmpcHelpers.buildDynamicObstaclesFromConfig;
configureDynamicStartMode = @RunNmpcHelpers.configureDynamicStartMode;
buildHullFootprintConfig = @RunNmpcHelpers.buildHullFootprintConfig;
updateWaypointIndexManaged = @RunNmpcHelpers.updateWaypointIndexManaged;
computeXTE = @RunNmpcHelpers.computeXTE;
activateDynamicObstaclesByProximity = @RunNmpcHelpers.activateDynamicObstaclesByProximity;
propagateDynamicObstacles = @RunNmpcHelpers.propagateDynamicObstacles;
selectMapHalfPlanesFromEdges = @RunNmpcHelpers.selectMapHalfPlanesFromEdges;
selectMapObstaclesFromSamples = @RunNmpcHelpers.selectMapObstaclesFromSamples;
dynamicToCircleObstacles = @RunNmpcHelpers.dynamicToCircleObstacles;
buildLatentDynamicAwarenessObstacles = @RunNmpcHelpers.buildLatentDynamicAwarenessObstacles;
normalizeObstacleSchema = @RunNmpcHelpers.normalizeObstacleSchema;
computeDynamicPackagingDrift = @RunNmpcHelpers.computeDynamicPackagingDrift;
nearestDistanceToMapEdges = @RunNmpcHelpers.nearestDistanceToMapEdges;
revRamp01 = @RunNmpcHelpers.revRamp01;
ramp01 = @RunNmpcHelpers.ramp01;
sat01 = @RunNmpcHelpers.sat01;
lerp = @RunNmpcHelpers.lerp;
wrapTo180Deg = @RunNmpcHelpers.wrapTo180Deg;
wrapToPi = @RunNmpcHelpers.wrapToPi;
distanceToBerthCorridorEntry = @RunNmpcHelpers.distanceToBerthCorridorEntry;
distanceToBerthCorridorEnd = @RunNmpcHelpers.distanceToBerthCorridorEnd;
getNextTurnAngleDeg = @RunNmpcHelpers.getNextTurnAngleDeg;
rk4Step9 = @RunNmpcHelpers.rk4Step9;
detectHullCircleHit = @RunNmpcHelpers.detectHullCircleHit;
detectHullMapHit = @RunNmpcHelpers.detectHullMapHit;
safePercentile = @RunNmpcHelpers.safePercentile;
plotMapBackground = @RunNmpcHelpers.plotMapBackground;


%% NORMALIZE / SETUP ======================================================
static_obstacles = normalizeStaticObstacles(static_obstacles);
berth_cfg = normalizeBerthCfg(berth_cfg, waypoints);
repoRoot = pwd;
run_dir = fileparts(which('run_nmpc'));
if ~isempty(run_dir)
    addpath(run_dir, '-begin');
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
nmpc_cfg.soft_speed_floor_default_mps = path_cost_cfg.soft_speed_floor_mps;
nmpc_cfg.soft_speed_floor_weight_default = path_cost_cfg.soft_speed_floor_weight;
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
path_heading_weight_log = nan(1, length(t));
terminal_heading_weight_log = nan(1, length(t));
soft_speed_cap_weight_log = nan(1, length(t));
soft_speed_floor_weight_log = nan(1, length(t));
path_xte_weight_log = nan(1, length(t));
path_tube_half_width_log = nan(1, length(t));
terminal_pos_weight_log = nan(1, length(t));
terminal_stop_u_weight_log = nan(1, length(t));
terminal_stop_v_weight_log = nan(1, length(t));
terminal_stop_r_weight_log = nan(1, length(t));
goal_heading_enable_log = nan(1, length(t));
goal_heading_rad_log = nan(1, length(t));
map_barrier_weight_log = nan(1, length(t));
u_min_forward_log = nan(1, length(t));
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
% Heading smoothing state for gradual goal changes
goal_heading_smooth = x(6);
goal_heading_target = x(6);
goal_smooth_steps = 1; % number of steps to ramp heading reference
goal_smooth_count = 0;

%% MAIN SIMULATION LOOP ===================================================
for i = 1:length(t)
    t_step = tic;
    % 1) Update active segment only, then define segment / berth references
    t_seg = tic;
    wp_idx = updateWaypointIndexManaged(x, waypoints, wp_idx, R_accept, wp_switch_cfg);

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
    desired_goal_heading = chi_seg;
    desired_goal_enable = false;

    if berth_cfg.enabled
        d_to_berth = norm(x(4:5) - berth_cfg.target_xy(:));
        preview_first_seg = max(1, last_seg_idx - max(1, berth_cfg.prepare_last_n_segments) + 1);
        preview_seg_ok = (seg_start_idx >= preview_first_seg);
        mode_seg_ok = true;
        if getOr(berth_cfg, 'final_leg_only', true)
            % Keep strict waypoint-following until the true final leg.
            preview_seg_ok = (seg_start_idx >= last_seg_idx);
            mode_seg_ok = (seg_start_idx >= last_seg_idx);
        end
        berth_preview_active = preview_seg_ok && (d_to_berth <= 1.35 * berth_cfg.activate_dist_m);
        berth_mode_active = on_final_waypoint || (mode_seg_ok && (d_to_berth <= berth_cfg.activate_dist_m));

        if berth_preview_active
            goal_heading_enable = true;
            preview_w = revRamp01(d_to_berth, 0.9 * berth_cfg.activate_dist_m, 1.8 * berth_cfg.activate_dist_m);
            goal_heading_rad = wrapToPi((1 - preview_w) * chi_seg + preview_w * deg2rad(berth_cfg.heading_deg));
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
            dyn_obs_hist(jj, :, i+1) = dynamic_obstacles(jj).position(1:2)';
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
    solve_opts.soft_speed_floor_weight = path_cost_cfg.soft_speed_floor_weight;
    solve_opts.soft_speed_floor_mps    = path_cost_cfg.soft_speed_floor_mps;

    solve_opts.goal_heading_enable = goal_heading_enable;
    solve_opts.goal_heading_rad    = goal_heading_rad;
    solve_opts.is_final_waypoint   = on_final_waypoint;

    solve_opts.enable_soft_obstacles = soft_obstacle_cfg.enabled;
    solve_opts.soft_obs_max_m        = soft_obstacle_cfg.max_slack_m;
    solve_opts.map_halfplanes        = map_halfplanes;

    % Safe defaults for optional scheduled fields
    solve_opts.terminal_goal_pos_weight     = 0.0;
    solve_opts.terminal_goal_heading_weight = 0.0;
    solve_opts.terminal_stop_u_weight       = 0.0;
    solve_opts.terminal_stop_v_weight       = 0.0;
    solve_opts.terminal_stop_r_weight       = 0.0;

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
    if getOr(azipod_sync_cfg, 'enabled', true)
        solve_opts.max_azimuth_split   = azipod_sync_cfg.transit_alpha_split_rad;
        solve_opts.max_stern_cmd_split = azipod_sync_cfg.transit_stern_split_rpm;
    else
        solve_opts.max_azimuth_split   = inf;
        solve_opts.max_stern_cmd_split = inf;
    end

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
    if berth_cfg.enabled && berth_mode_active
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

    % Preview strictness: starts on the last N segments or by berth distance,
    % but ramps smoothly rather than only adding mild heading prep.
    lambda_berth_preview = 0;
    if berth_cfg.enabled && berth_preview_active
        d_preview_on = 1.8 * berth_cfg.activate_dist_m;
        d_preview_full = 0.9 * berth_cfg.activate_dist_m;
        lambda_berth_preview = revRamp01(d_to_berth, d_preview_full, d_preview_on);
    end

    % Unified berth preparation intensity
    lambda_berth_strict = max(lambda_berth, 0.75 * lambda_berth_preview);


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
    U_cap_turn  = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_turn_mps, lambda_turn);
    U_cap_berth = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_berth_mps, lambda_berth_strict);
    U_cap_dyn   = lerp(sched_cfg.u_cruise_mps, sched_cfg.u_yield, lambda_yield);
    turn_berth_intensity = max(lambda_turn, lambda_berth_strict);
    solve_opts.soft_speed_cap_mps = min([U_cap_tight, U_cap_stop, U_cap_turn, U_cap_berth, U_cap_dyn]);
    solve_opts.soft_speed_cap_weight = lerp(sched_cfg.soft_speed_weight_far, sched_cfg.soft_speed_weight_near, lambda_total);
    speed_floor_ratio_now = lerp(sched_cfg.soft_speed_floor_ratio, 0.0, turn_berth_intensity);
    solve_opts.soft_speed_floor_mps = max(0.0, speed_floor_ratio_now * solve_opts.soft_speed_cap_mps);
    solve_opts.soft_speed_floor_weight = lerp(sched_cfg.soft_speed_floor_weight_far, sched_cfg.soft_speed_floor_weight_near, lambda_total) * (1.0 - 0.9 * turn_berth_intensity);

    % Along-track reward scheduling (yield reduces progress urgency)
    along_floor_now = lerp(sched_cfg.w_along_min, 0.0, turn_berth_intensity);
    solve_opts.path_along_weight = lerp(path_cost_cfg.W_along, along_floor_now, max(lambda_yield, turn_berth_intensity));

    % Tube & XTE scheduling (return tightens path after CPA)
    geom_caution = max([lambda_tight, lambda_turn, lambda_berth]);
    lambda_path_return = max(geom_caution, lambda_return);
    lambda_recover = ramp01(abs(xte), 18, 70) * max(0.0, 1.0 - max([lambda_tight, lambda_turn, lambda_berth, lambda_yield]));

    solve_opts.path_tube_half_width_m = ...
        lerp(sched_cfg.tube_far_m, sched_cfg.tube_near_m, lambda_path_return);

    solve_opts.path_xte_weight = ...
        lerp(sched_cfg.xte_weight_far, sched_cfg.xte_weight_near, lambda_path_return);

    solve_opts.path_heading_weight = max([ ...
    2.5, ...
    0.75 * lambda_turn  * sched_cfg.heading_weight_turn, ...
    0.10 * lambda_berth * sched_cfg.heading_weight_berth, ...
    lambda_return       * sched_cfg.heading_weight_return]);

    stop_heading_weight = lerp(0.0, sched_cfg.heading_weight_stop, max(lambda_stop, lambda_tight));
    solve_opts.path_heading_weight = max(solve_opts.path_heading_weight, stop_heading_weight);

    % Recovery recapture: once obstacle pressure is gone, force the ship back
    % onto the segment instead of letting large lateral error linger.
    if lambda_recover > 0
        solve_opts.path_tube_half_width_m = min(solve_opts.path_tube_half_width_m, ...
            lerp(sched_cfg.tube_far_m, max(8.0, 0.75 * sched_cfg.tube_near_m), lambda_recover));
        solve_opts.path_xte_weight = max(solve_opts.path_xte_weight, ...
            lerp(sched_cfg.xte_weight_far, 60.0, lambda_recover));
        solve_opts.path_heading_weight = max(solve_opts.path_heading_weight, ...
            lerp(10.0, 32.0, lambda_recover));
        solve_opts.terminal_goal_heading_weight = max(solve_opts.terminal_goal_heading_weight, ...
            lerp(10.0, 40.0, lambda_recover));
        solve_opts.soft_obs_max_m = min(solve_opts.soft_obs_max_m, ...
            lerp(soft_obstacle_cfg.max_slack_m, max(1.5, 0.35 * soft_obstacle_cfg.max_slack_m), lambda_recover));
        if ~berth_preview_active && ~berth_mode_active
            goal_heading_enable = true;
            goal_heading_rad = chi_seg;
            desired_goal_heading = chi_seg;
            desired_goal_enable = true;
            goal_heading_smooth = chi_seg;
            goal_heading_target = chi_seg;
            goal_smooth_count = 0;
        end
    end

    
    % Early berth strictness: tighten line tracking already during preview
    solve_opts.path_tube_half_width_m = min( ...
        solve_opts.path_tube_half_width_m, ...
        lerp(path_cost_cfg.W_tube_m, 4.0, lambda_berth_strict));

    solve_opts.path_xte_weight = max( ...
        solve_opts.path_xte_weight, ...
        lerp(path_cost_cfg.W_xte_heavy, 36.0, lambda_berth_strict));

    solve_opts.path_heading_weight = max( ...
        solve_opts.path_heading_weight, ...
        lerp(10.0, 48.0, lambda_berth_strict));


    % Surge lower bound scheduling (yield allows near-stop)
    desired_u_min_forward = lerp(sched_cfg.u_min_forward_far, sched_cfg.u_min_forward_near, lambda_yield);
    solve_opts.u_min_forward = desired_u_min_forward;

    % Heading & stop scheduling (bert   h/turn vs return)
    solve_opts.terminal_goal_heading_weight = max( ...
        lambda_turn * sched_cfg.heading_weight_turn, ...
        lambda_berth_strict * sched_cfg.heading_weight_berth) + ...
        lambda_return * 15.0;

    if lambda_berth_strict >= max(0.20, lambda_turn)
        solve_opts.goal_heading_enable = true; solve_opts.goal_heading_rad = deg2rad(berth_cfg.heading_deg);
    elseif lambda_turn > 0.05 || lambda_return > 0.1
        solve_opts.goal_heading_enable = true; solve_opts.goal_heading_rad = chi_seg;
    elseif on_final_waypoint
        solve_opts.goal_heading_enable = true; solve_opts.goal_heading_rad = chi_seg;
    else
        solve_opts.goal_heading_enable = false;
    end

    % Pre-emptive heading control for sharp upcoming turns:
    % If the next segment after the active one makes a large turn, ask NMPC
    % to start tracking the next-segment heading early so references stay
    % continuous during the handoff.
    sharp_turn_active = false;
    if seg_end_idx < n_wps
        next_idx = min(seg_end_idx + 1, n_wps);
        p_next = waypoints(next_idx, 1:2)';
        chi_next_seg = atan2(p_next(2) - wp_end_xy(2), p_next(1) - wp_end_xy(1));
        heading_diff_deg = abs(wrapTo180Deg(rad2deg(chi_next_seg - chi_seg)));
        % Begin blending toward the next segment before the handoff point.
        % This gives the controller a geometric preview instead of waiting
        % until the segment switch to discover the turn.
        if heading_diff_deg >= 5.0
            turn_preview_w = ramp01(d_to_active_end, 0.55 * route_follow_cfg.sharp_turn_heading_enable_dist_m, ...
                                           route_follow_cfg.sharp_turn_heading_enable_dist_m);
            if turn_preview_w > 0
                desired_goal_heading = wrapToPi((1 - turn_preview_w) * chi_seg + turn_preview_w * chi_next_seg);
                desired_goal_enable = true;
            end
        end
        if heading_diff_deg >= route_follow_cfg.sharp_turn_deg && d_to_active_end <= route_follow_cfg.sharp_turn_heading_enable_dist_m
            % For sharp turns, increase the heading pressure as soon as we are
            % within the lookahead band.
            sharp_turn_active = true;
            if getOr(solve_opts, 'terminal_goal_heading_weight', 0) <= 0
                solve_opts.terminal_goal_heading_weight = route_follow_cfg.sharp_turn_heading_weight;
            else
                solve_opts.terminal_goal_heading_weight = solve_opts.terminal_goal_heading_weight * route_follow_cfg.sharp_turn_goal_weight_gain;
            end
            solve_opts.path_heading_weight = max(getOr(solve_opts, 'path_heading_weight', 0), route_follow_cfg.sharp_turn_heading_weight);
        end
    end

    % If no special heading target is active, use the current scheduled goal.
    if ~desired_goal_enable
        desired_goal_heading = getOr(solve_opts, 'goal_heading_rad', chi_ctrl);
        desired_goal_enable = getOr(solve_opts, 'goal_heading_enable', goal_heading_enable);
    end

    % Smooth large heading changes over several steps to avoid discontinuities
    delta = wrapToPi(desired_goal_heading - goal_heading_smooth);
    if desired_goal_enable && abs(rad2deg(delta)) > 2.0
        % Start (or continue) smoothing towards new target
        goal_heading_target = desired_goal_heading;
        goal_smooth_count = max(goal_smooth_count, goal_smooth_steps);
    end
    if goal_smooth_count > 0
        % Incremental step toward target
        step_delta = wrapToPi(goal_heading_target - goal_heading_smooth) / max(1, goal_smooth_count);
        goal_heading_smooth = wrapToPi(goal_heading_smooth + step_delta);
        goal_smooth_count = goal_smooth_count - 1;
        heading_weight_gain = lerp(0.35, 1.0, 1.0 - goal_smooth_count / max(1, goal_smooth_steps));
        solve_opts.goal_heading_enable = true;
        solve_opts.goal_heading_rad = goal_heading_smooth;
        if isfield(solve_opts, 'terminal_goal_heading_weight') && ~isempty(solve_opts.terminal_goal_heading_weight)
            solve_opts.terminal_goal_heading_weight = solve_opts.terminal_goal_heading_weight * heading_weight_gain;
        end
        if isfield(solve_opts, 'path_heading_weight') && ~isempty(solve_opts.path_heading_weight)
            solve_opts.path_heading_weight = solve_opts.path_heading_weight * heading_weight_gain;
        end
    else
        solve_opts.goal_heading_enable = desired_goal_enable;
        solve_opts.goal_heading_rad = desired_goal_heading;
    end

    % Re-assert sharp-turn heading pressure after smoothing to avoid drift
    if sharp_turn_active
        solve_opts.path_heading_weight = max(getOr(solve_opts, 'path_heading_weight', 0), 1.5 * route_follow_cfg.sharp_turn_heading_weight);
        solve_opts.terminal_goal_heading_weight = max(getOr(solve_opts, 'terminal_goal_heading_weight', 0), 1.5 * route_follow_cfg.sharp_turn_heading_weight);
    end

    % When the stop constraint is fully active, keep heading anchored to the segment
    if lambda_stop > 0.6 && ~lambda_yield && ~berth_preview_active && ~berth_mode_active
        solve_opts.goal_heading_enable = true;
        solve_opts.goal_heading_rad = chi_seg;
        solve_opts.path_heading_weight = max(getOr(solve_opts, 'path_heading_weight', 0), route_follow_cfg.sharp_turn_heading_weight);
    end

    solve_opts.terminal_goal_pos_weight = max( ...
        solve_opts.terminal_goal_pos_weight, ...
        lerp(sched_cfg.term_pos_weight_far, sched_cfg.term_pos_weight_berth, lambda_berth_strict));
    solve_opts.terminal_stop_u_weight = lerp(sched_cfg.stop_u_weight_far, sched_cfg.stop_u_weight_berth, lambda_berth_strict);
    solve_opts.terminal_stop_v_weight = lerp(0.0, sched_cfg.stop_v_weight_berth, lambda_berth_strict);
    solve_opts.terminal_stop_r_weight = lerp(0.0, sched_cfg.stop_r_weight_berth, lambda_berth_strict);

    % EARLY BERTH PREVIEW
    if berth_preview_active && ~berth_mode_active
    solve_opts.goal_heading_enable = true;
    solve_opts.goal_heading_rad = wrapToPi((1 - lambda_berth_strict) * chi_seg + lambda_berth_strict * deg2rad(berth_cfg.heading_deg));

        solve_opts.terminal_goal_heading_weight = max( ...
            solve_opts.terminal_goal_heading_weight, ...
            berth_cfg.preview_heading_weight);

        solve_opts.terminal_goal_pos_weight = max( ...
            solve_opts.terminal_goal_pos_weight, ...
            berth_cfg.preview_goal_weight_gain * solve_opts.terminal_goal_pos_weight);
    end

    % Terminal envelope
    if berth_cfg.enabled && (lambda_berth_strict > 0.10)
    solve_opts.enable_terminal_pose = true;
        solve_opts.term_pose_eps_xy_m = lerp(sched_cfg.term_xy_far_m, sched_cfg.term_xy_near_m, lambda_berth_strict);
        solve_opts.term_pose_eps_psi_rad = lerp(sched_cfg.term_psi_far_rad, sched_cfg.term_psi_near_rad, lambda_berth_strict);
        solve_opts.term_vel_max_u_mps = lerp(sched_cfg.term_u_far_mps, sched_cfg.term_u_near_mps, lambda_berth_strict);
        solve_opts.term_vel_max_v_mps = lerp(sched_cfg.term_v_far_mps, sched_cfg.term_v_near_mps, lambda_berth_strict);
        solve_opts.term_vel_max_r_radps = lerp(sched_cfg.term_r_far_radps, sched_cfg.term_r_near_radps, lambda_berth_strict);
        solve_opts.term_pose_slack_max = lerp(2.0, berth_cfg.pose_slack_max, lambda_berth_strict);
    end

    % Map corridor & relevance
    if berth_cfg.enabled && berth_cfg.use_corridor && (lambda_berth_strict > 0.15)
        solve_opts.enable_berth_corridor = true;
    solve_opts.berth_corridor_origin_xy = berth_cfg.corridor_origin_xy(:);
    solve_opts.berth_corridor_heading_rad = deg2rad(berth_cfg.corridor_heading_deg);
        % Base scheduled half-width (near/far blend)
        base_half = lerp(sched_cfg.corridor_half_width_far_m, sched_cfg.corridor_half_width_near_m, lambda_berth_strict);
        solve_opts.berth_corridor_half_width_m = base_half;
    solve_opts.berth_corridor_along_min_m = berth_cfg.corridor_along_min_m;
    solve_opts.berth_corridor_along_max_m = berth_cfg.corridor_along_max_m;
    end

    % Progressive relaxation is lambda-driven and only expands when the vessel
    % is already near the berth corridor geometry.
    corr_heading_rad = deg2rad(berth_cfg.corridor_heading_deg);
    heading_err = abs(wrapToPi(corr_heading_rad - x(6)));
    base_frac = min(1.0, heading_err / pi);
    relax_frac = base_frac * max(0.0, lambda_berth_strict);
    tube_w = getOr(solve_opts, 'path_tube_half_width_m', path_cost_cfg.W_tube_m);
    relax_gate = double((abs(xte) <= 0.5 * tube_w) && (d_to_berth <= 1.2 * getOr(berth_cfg, 'activate_dist_m', 400)));
    relax_frac = relax_frac * relax_gate;

    relax_scale = 1.0 + relax_frac * (getOr(berth_cfg, 'corridor_relax_max_scale', 3.0) - 1.0);
    solve_opts.berth_corridor_half_width_m = berth_cfg.corridor_half_width_m * relax_scale;

    along_extra = getOr(berth_cfg, 'corridor_relax_along_extra_m', 150) * relax_frac;
    solve_opts.berth_corridor_along_min_m = berth_cfg.corridor_along_min_m - along_extra;
    solve_opts.berth_corridor_along_max_m = berth_cfg.corridor_along_max_m + along_extra;

    % Smoothness scaling (yield penalizes jerky avoidance arcs)
    solve_opts.R_rate_scale_obs = 1.0 + lambda_yield * (sched_cfg.R_rate_scale_yield - 1.0);

    % Soft map barrier weight (tight + return phases)
    solve_opts.map_barrier_weight = sched_cfg.map_barrier_w_base + ...
        max(lambda_tight, lambda_return) * (sched_cfg.map_barrier_w_near - sched_cfg.map_barrier_w_base);

    % Keep structural limits mode-aware, but let the lambda schedule drive behavior.
    berth_struct_lambda = max(lambda_berth_strict, double(berth_mode_active));
    solve_opts.n3_max = round(lerp(0, berth_cfg.n3_max, berth_struct_lambda));
    if getOr(azipod_sync_cfg, 'enabled', true)
        solve_opts.max_azimuth_split = lerp(azipod_sync_cfg.transit_alpha_split_rad, berth_cfg.max_azimuth_split_rad, berth_struct_lambda);
        solve_opts.max_stern_cmd_split = lerp(azipod_sync_cfg.transit_stern_split_rpm, berth_cfg.max_stern_cmd_split_rpm, berth_struct_lambda);
    else
        solve_opts.max_azimuth_split = inf;
        solve_opts.max_stern_cmd_split = inf;
    end

    solve_opts.u_min_forward = desired_u_min_forward;

    % Online forward-preference scheduling (single NMPC formulation):
    % transit => strong forward preference; final berth => allow reverse freely.
    solve_opts.forward_preference_scale = max(0.0, 1.0 - max(lambda_berth_strict, 0.6 * lambda_turn));

    % Keep reverse allowed near the berth via the same lambda rather than mode-specific logic.
    solve_opts.u_min_forward = min(desired_u_min_forward, ...
        lerp(sched_cfg.u_min_forward_far, berth_cfg.u_min_final_mps, lambda_berth_strict));
    solve_opts.u_min_forward = min(solve_opts.u_min_forward, lerp(sched_cfg.u_min_forward_far, -0.05, lambda_turn));

    % Terminal guidance is lambda-driven, not branch-driven.
    solve_opts.enable_terminal_pose = (lambda_berth_strict > 0.10) || berth_mode_active || berth_preview_active || on_final_waypoint;
    lambda_gate_final = revRamp01(d_to_active_end, 25, 160);
    lambda_terminal_conn = max([lambda_gate_final, lambda_return, 0.5*lambda_tight, lambda_berth_strict]);

    solve_opts.goal_heading_enable = true;
    solve_opts.goal_heading_rad = wrapToPi((1 - lambda_berth_strict) * chi_seg + lambda_berth_strict * deg2rad(berth_cfg.heading_deg));

    solve_opts.terminal_line_xte_weight = max( ...
        getOr(solve_opts, 'terminal_line_xte_weight', 0), ...
        lerp(0.0, 260.0, lambda_terminal_conn));
    solve_opts.terminal_line_heading_weight = max( ...
        getOr(solve_opts, 'terminal_line_heading_weight', 0), ...
        lerp(0.0, 110.0, lambda_terminal_conn));

    solve_opts.path_xte_weight = max( ...
        solve_opts.path_xte_weight, ...
        lerp(16.0, 34.0, lambda_gate_final));

    solve_opts.path_tube_half_width_m = min( ...
        solve_opts.path_tube_half_width_m, ...
        lerp(12.0, 7.0, lambda_gate_final));

    solve_opts.terminal_goal_pos_weight = max( ...
        solve_opts.terminal_goal_pos_weight, ...
        lerp(90.0, 180.0, lambda_terminal_conn));

              

    % Update logs
    lambda_tight_log(i) = lambda_tight; lambda_stop_log(i) = lambda_stop;
    lambda_turn_log(i)  = lambda_turn;  lambda_berth_log(i)= lambda_berth;
    lambda_total_log(i) = lambda_total; lambda_yield_log(i)= lambda_yield;
    lambda_return_log(i)= lambda_return; lambda_ttc_log(i)  = lambda_ttc;
    u_cap_log(i)        = solve_opts.soft_speed_cap_mps; d_edge_log(i) = d_edge_now;
    soft_speed_cap_weight_log(i) = solve_opts.soft_speed_cap_weight;
    soft_speed_floor_weight_log(i) = solve_opts.soft_speed_floor_weight;
    path_xte_weight_log(i) = solve_opts.path_xte_weight;
    path_tube_half_width_log(i) = solve_opts.path_tube_half_width_m;
    terminal_pos_weight_log(i) = solve_opts.terminal_goal_pos_weight;
    terminal_stop_u_weight_log(i) = solve_opts.terminal_stop_u_weight;
    terminal_stop_v_weight_log(i) = solve_opts.terminal_stop_v_weight;
    terminal_stop_r_weight_log(i) = solve_opts.terminal_stop_r_weight;
    path_heading_weight_log(i) = getOr(solve_opts, 'path_heading_weight', NaN);
    terminal_heading_weight_log(i) = getOr(solve_opts, 'terminal_goal_heading_weight', NaN);
    goal_heading_enable_log(i) = double(getOr(solve_opts, 'goal_heading_enable', false));
    goal_heading_rad_log(i) = getOr(solve_opts, 'goal_heading_rad', NaN);
    map_barrier_weight_log(i) = getOr(solve_opts, 'map_barrier_weight', NaN);
    u_min_forward_log(i) = getOr(solve_opts, 'u_min_forward', NaN);
    az_split_limit_log(i) = solve_opts.max_azimuth_split;
    stern_split_limit_log(i) = solve_opts.max_stern_cmd_split;
    ref_time_log(i) = toc(t_seg);

    % 5) Solve NMPC
    t_seg = tic;
    [u_opt, X_pred, info] = nmpc.solve(x, path_ref, obs_local, u_prev, solve_opts.u_min_forward, solve_opts);
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
        if berth_preview_active, mode_str = 'BERTH_PREVIEW'; end
        if on_final_waypoint, mode_str = 'FINAL'; end
        if berth_mode_active, mode_str = 'BERTH'; end
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

    % Heading gate for berth capture (only enforced when berth mode enabled)
    heading_ok = true;
    if berth_cfg.enabled && isfield(berth_cfg, 'capture_heading_deg')
        desired_heading_rad = deg2rad(berth_cfg.heading_deg);
        heading_ok = abs(wrapToPi(x(6) - desired_heading_rad)) <= deg2rad(berth_cfg.capture_heading_deg);
        % Above uses configured capture_heading_deg (deg) as threshold; default set in normalizeBerthCfg
    end

    % Require also a sufficiently precise final pose (within configured pose eps + slack)
    pos_ok = true;
    if berth_cfg.enabled && isfield(berth_cfg, 'pose_eps_xy_m')
        try
            pose_eps = berth_cfg.pose_eps_xy_m(:);
            pose_slack = getOr(berth_cfg, 'pose_slack_max', 0.0);
            pos_err = abs(x(4:5) - mission_goal_xy);
            pos_ok = all(pos_err <= (pose_eps + pose_slack));
        catch
            pos_ok = true;
        end
    end

    hard_final_hit = mission_ready && (d_final_now < mission_capture_radius) && heading_ok && pos_ok;
    soft_final_hit = mission_ready && ...
        (d_final_now < mission_capture_radius_soft) && ...
        (U_now_ship <= mission_capture_speed) && heading_ok && pos_ok;

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
path_heading_weight_log = path_heading_weight_log(1:steps);
terminal_heading_weight_log = terminal_heading_weight_log(1:steps);
soft_speed_cap_weight_log = soft_speed_cap_weight_log(1:steps);
soft_speed_floor_weight_log = soft_speed_floor_weight_log(1:steps);
path_xte_weight_log = path_xte_weight_log(1:steps);
path_tube_half_width_log = path_tube_half_width_log(1:steps);
terminal_pos_weight_log = terminal_pos_weight_log(1:steps);
terminal_stop_u_weight_log = terminal_stop_u_weight_log(1:steps);
terminal_stop_v_weight_log = terminal_stop_v_weight_log(1:steps);
terminal_stop_r_weight_log = terminal_stop_r_weight_log(1:steps);
goal_heading_enable_log = goal_heading_enable_log(1:steps);
goal_heading_rad_log = goal_heading_rad_log(1:steps);
map_barrier_weight_log = map_barrier_weight_log(1:steps);
u_min_forward_log = u_min_forward_log(1:steps);

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
            anim_cfg.useLightTheme = use_light_theme;
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

%% PLOTTING

figure(1); clf;

subplot(3,3,1);
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
if ~isempty(dynamic_obstacles)
    dyn_cols = lines(max(1, size(dyn_obs_hist,1)));
    for j = 1:size(dyn_obs_hist,1)
        hx = squeeze(dyn_obs_hist(j,2,1:steps+1));
        hy = squeeze(dyn_obs_hist(j,1,1:steps+1));
        valid = isfinite(hx) & isfinite(hy);
        if nnz(valid) >= 2
            plot(hx(valid), hy(valid), '--', 'Color', dyn_cols(j,:), 'LineWidth', 1.2, ...
                'HandleVisibility', 'off');
        end
        if any(valid)
            last_idx = find(valid, 1, 'last');
            fill(hx(last_idx) + dynamic_obs_radius_m*cos(theta_circ), ...
                 hy(last_idx) + dynamic_obs_radius_m*sin(theta_circ), ...
                 dyn_cols(j,:), 'FaceAlpha', 0.12, 'EdgeColor', dyn_cols(j,:), ...
                 'LineWidth', 1.3, 'HandleVisibility', 'off');
        end
    end
end
plot(traj(5,1), traj(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(traj(5,end), traj(4,end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('East / y [m]'); ylabel('North / x [m]'); 
title('Ship Trajectory'); grid on; axis equal;

subplot(3,3,2);
t_ctrl = (0:size(ctrl,2)-1)*dt;
plot(t_ctrl, rad2deg(ctrl(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl, rad2deg(ctrl(2,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Azimuth [deg]');
title('Azimuth Angles'); grid on;
legend('\alpha_1 (port stern)', '\alpha_2 (starboard stern)', 'Location', 'best');

subplot(3,3,3);
plot(t_ctrl, ctrl(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl, ctrl(4,:), 'r--', 'LineWidth', 1.5);
plot(t_ctrl, ctrl(5,:), 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Commanded rpm');
title('Shaft Speed Commands'); grid on;
legend('n_{1,c}', 'n_{2,c}', 'n_{3,c}', 'Location', 'best');

subplot(3,3,4);
plot(t_sim, traj(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_sim, traj(8,:), 'r--', 'LineWidth', 1.5);
plot(t_sim, traj(9,:), 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Actual rpm');
title('Actual Shaft Speeds'); grid on;
legend('n_1', 'n_2', 'n_3', 'Location', 'best');

subplot(3,3,5);
t_xte = (0:length(xte_log)-1)*dt;
plot(t_xte, xte_log, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('Cross-track Error'); grid on;

subplot(3,3,6);
plot(t_sim, traj(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Surge Velocity'); grid on;

subplot(3,3,7);
plot(t_xte, rad2deg(heading_err_log), 'm-', 'LineWidth', 1.5); hold on;
yline(0, 'k--');
xlabel('Time [s]'); ylabel('\psi_{ref}-\psi [deg]');
title('Heading Error vs Time'); grid on;

subplot(3,3,8);
yyaxis left;
plot(t_ctrl, rad2deg(ctrl(1,:)), 'b-', 'LineWidth', 1.2); hold on;
plot(t_ctrl, rad2deg(ctrl(2,:)), 'r--', 'LineWidth', 1.2);
ylabel('Rudder cmd [deg]');
yyaxis right;
plot(t_ctrl, ctrl(3,:), 'c-', 'LineWidth', 1.0);
plot(t_ctrl, ctrl(4,:), 'g--', 'LineWidth', 1.0);
plot(t_ctrl, ctrl(5,:), 'k-', 'LineWidth', 1.0);
ylabel('Thrust cmd [rpm]');
xlabel('Time [s]');
title('Commanded Rudder / Thrust vs Time'); grid on;
legend('\alpha_1', '\alpha_2', 'n_{1,c}', 'n_{2,c}', 'n_{3,c}', 'Location', 'best');

sgtitle('NMPC Harbor Navigation — Unified Test');

figure(3); clf;
set(gcf, 'Position', [120 80 1550 980]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

phase_names = {'Tight', 'Stop', 'Turn', 'Berth', 'Yield', 'Return', 'TTC'};
phase_colors = [
    0.00 0.45 0.74;
    0.85 0.33 0.10;
    0.93 0.69 0.13;
    0.49 0.18 0.56;
    0.47 0.67 0.19;
    0.30 0.75 0.93;
    0.64 0.08 0.18];

lambda_stack = [lambda_tight_log(:), lambda_stop_log(:), lambda_turn_log(:), ...
                lambda_berth_log(:), lambda_yield_log(:), lambda_return_log(:), lambda_ttc_log(:)];
[~, dominant_idx] = max(lambda_stack, [], 2);
switch_points = [1; find(diff(dominant_idx) ~= 0) + 1; numel(t_sim)];

nexttile;
hold on;
for k = 1:numel(phase_names)
    plot(t_sim(1:steps), lambda_stack(:,k), 'LineWidth', 1.7, 'Color', phase_colors(k,:));
end
plot(t_sim(1:steps), lambda_total_log, 'k-', 'LineWidth', 2.6);
ylim([0 1.05]);
xlim([t_sim(1) t_sim(end)]);
grid on;
ylabel('Lambda');
title('Scheduler Lambdas');
legend([phase_names, {'Total'}], 'Location', 'eastoutside');

nexttile;
hold on;
plot(t_sim(1:steps), path_xte_weight_log, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.7);
plot(t_sim(1:steps), path_tube_half_width_log, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.7);
plot(t_sim(1:steps), path_heading_weight_log, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.7);
plot(t_sim(1:steps), terminal_heading_weight_log, 'Color', [0.93 0.69 0.13], 'LineWidth', 1.7);
plot(t_sim(1:steps), soft_speed_cap_weight_log, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.7);
plot(t_sim(1:steps), soft_speed_floor_weight_log, 'Color', [0.30 0.75 0.93], 'LineWidth', 1.5, 'LineStyle', '--');
plot(t_sim(1:steps), terminal_pos_weight_log, 'Color', [0.64 0.08 0.18], 'LineWidth', 1.7);
plot(t_sim(1:steps), map_barrier_weight_log, 'Color', [0.15 0.15 0.15], 'LineWidth', 1.5, 'LineStyle', ':');
grid on;
xlim([t_sim(1) t_sim(end)]);
ylabel('Weight / width');
title('Scheduled Cost Terms');
legend({'XTE weight', 'Tube half-width', 'Path heading weight', 'Terminal heading weight', ...
        'Speed cap weight', 'Speed floor weight', 'Terminal position weight', 'Map barrier weight'}, ...
        'Location', 'eastoutside');

nexttile;
hold on;
cost_plot = max(cost_log, eps);
cost_plot(~isfinite(cost_plot)) = NaN;
semilogy(t_sim(1:steps), cost_plot, 'k-', 'LineWidth', 2.0);
cost_y = cost_plot(isfinite(cost_plot));
if isempty(cost_y)
    cost_y = eps;
end
cost_ylim = [max(eps, min(cost_y) * 0.8), max(cost_y) * 1.2];
ylim(cost_ylim);
for s = 1:numel(switch_points)-1
    idx0 = switch_points(s);
    idx1 = switch_points(s+1);
    if idx1 > idx0
        patch([t_sim(idx0) t_sim(idx1) t_sim(idx1) t_sim(idx0)], ...
            [cost_ylim(1) cost_ylim(1) cost_ylim(2) cost_ylim(2)], ...
            phase_colors(dominant_idx(idx0),:), 'FaceAlpha', 0.08, ...
            'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
end
for s = 2:numel(switch_points)-1
    xline(t_sim(switch_points(s)), '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
yyaxis right;
stairs(t_sim(1:steps), dominant_idx, 'LineWidth', 1.2, 'Color', [0.25 0.25 0.25]);
yticks(1:numel(phase_names));
yticklabels(phase_names);
ylabel('Dominant lambda');
ylim([0.5 numel(phase_names)+0.5]);
yyaxis left;
xlim([t_sim(1) t_sim(end)]);
ylabel('Cost');
xlabel('Time [s]');
grid on;
title('Cost Switching Behaviour');

sgtitle('NMPC Scheduler Diagnostics — Lambdas, Scheduled Weights, and Cost Switching');
