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

%  USER CONFIGURATION — EDIT THIS SECTION

% ---- WAYPOINTS (rows = [x, y] in meters, NED frame) ----
waypoints = [-3400, -1550;
             -2800, -1600;
             -2400, -1900;
             -2050, -2150;];

% ---- WAYPOINT SPEEDS (m/s) — one per segment + final ----
waypoint_speeds = [4.5; 4.5; 5; 4];

% ---- STATIC OBSTACLES (set to empty [] for none) ----
% Each obstacle: struct with 'position' [x; y] and 'radius' [m]
static_obstacles = [];
% static_obstacles(1).position = [-3000; -1400];
% static_obstacles(1).radius   = 30;

% ---- SIMULATION PARAMETERS ----
T_final     = 600;      % Total simulation time [s]
R_accept    = 85;       % Intermediate waypoint acceptance radius [m] (moderately permissive)
R_accept_final = 15;    % Final waypoint acceptance radius [m]
R_accept_final_soft = 32;      % Soft terminal capture radius [m] (used with low-speed hold)
final_capture_speed_mps = 1.5; % Max speed for soft terminal capture [m/s]
final_capture_hold_s = 6;      % Time inside soft capture gate before declaring success [s]
n1_cruise   = 70;       % Aft thruster cruise speed [rpm]
n2_cruise   = 0;        % Forward thruster (off during cruise)

% ---- ANIMATION RECORDING ----
enable_animation_recording = true;
record_fps = 30;
record_output_dir = 'C:\Users\SERILEG\OneDrive - ABB\Autonomous-docking\MY_NPMC\my work\plots in development process\recordings';

% ---- TERMINAL CONSTRAINT RELAXATION ----
% These only apply near the final waypoint to avoid local minima/circling.
enable_terminal_map_relax = true;
terminal_relax_dist_m = 260;
terminal_map_exclusion_m = 120;
terminal_min_avoid_scale = 0.55;
terminal_disable_map_samples = true;  % disable virtual map sample obstacles near final approach

% ---- MAP OBSTACLE SAMPLING ----
enable_map_obstacles = true;   % Set false to disable red-zone awareness
max_map_obstacles    = 8;     % Max map sample points as obstacles
map_lookahead_m      = 400;    % Base forward lookahead distance [m]
map_half_width_m     = 200;    % Base corridor half-width [m]
map_sample_radius_m  = 4;      % Virtual obstacle radius for map points

% Balanced guidance/avoidance coupling (global, not map-specific)
map_lookahead_time_s = 75;     % Forward preview time [s]
map_lookahead_min_m  = 420;    % Clamp lower bound for lookahead [m]
map_lookahead_max_m  = 900;    % Clamp upper bound for lookahead [m]
map_half_width_min_m = 220;    % Clamp lower bound for corridor width [m]
map_half_width_max_m = 420;    % Clamp upper bound for corridor width [m]

% ---- DYNAMIC OBSTACLES (forward motion, no turning) ----
enable_dynamic_obstacles = true;      % Master switch for moving obstacles
dynamic_obs_speed_mps    = 10;       % Constant speed [m/s]
dynamic_obs_radius_m     = 25;        % Circular obstacle radius [m]
dynamic_obs_boundary_policy = 'deactivate'; % deactivate | clip | wrap
dynamic_obs_boundary_margin = 120;    % Margin around map bounds [m]
enable_dynamic_replay_check = true;   % Determinism self-check

% Manual dynamic obstacle definition (required when enabled):
% - Positions are rows [x y] in meters.
% - Headings are in degrees (0=+x/North, 90=+y/East).
% - Speeds are in m/s (scalar or one per obstacle).
dynamic_obs_positions_xy = [-2600 -1600];  % Example: [-3000 -1700; -2920 -1740]
dynamic_obs_headings_deg = [230];           % Example: [90; 110]
dynamic_obs_speeds_mps   = [5];             % [] uses dynamic_obs_speed_mps for all

% Dynamic obstacle motion trigger mode:
% - 'immediate': obstacles move from t=0
% - 'proximity': obstacles stay stationary until ship is close
dynamic_obs_start_mode = 'proximity';       % immediate | proximity
dynamic_obs_trigger_distance_m = 250;       % scalar or one per obstacle

% ---- NMPC TUNING ----
nmpc_N  = 40;           % Prediction horizon steps
nmpc_dt = 1.0;          % Sample time [s]
r_safety = 14;          % Safety margin around obstacles [m]

% Hard guidance-heading corridor (forces bow to stay near checkpoint direction)
guidance_heading_hard_cfg = struct();
guidance_heading_hard_cfg.enable = true;
guidance_heading_hard_cfg.max_err_deg = 30;  % Tighter corridor to reduce lateral sway
guidance_heading_hard_cfg.use_nominal_waypoint_heading = true;

% Q weights: [u, v, r, x, y, psi, n1, n2]
%   - Higher position weights (x,y) → better obstacle avoidance
%   - Higher heading weight (psi) → tighter path tracking
Q_weights = diag([2.0, 0.1, 0.6, 5.0, 5.0, 3.2, 0.001, 0.001]);

% R weights: [alpha1, alpha2, n1_c, n2_c]
R_weights = diag([0.1, 0.1, 0.01, 0.01]);

% R_rate weights for smooth control transitions
R_rate_weights = diag([0.08, 0.08, 0.006, 0.006]);

% Obstacle-aware reference shaping (inertia/lookahead balance)
avoid_ref_cfg = struct();
avoid_ref_cfg.base_margin_m   = 22;    % baseline lateral keep-out in reference shaping
avoid_ref_cfg.speed_gain_s    = 0.7;   % extra margin = speed_gain * U_d
avoid_ref_cfg.obs_radius_gain = 0.25;  % how much obstacle radius increases lateral deflection
avoid_ref_cfg.deflect_sigma   = 0.19;  % smaller -> sharper local avoidance bend
avoid_ref_cfg.r_ref_max       = 0.14;  % max |r_ref| sent to NMPC [rad/s]

% ---- CONTROLLER SAFETY SETTINGS ----
enforce_command_limits = true;

% ---- RUNTIME BUDGET / PERFORMANCE SETTINGS ----
runtime_cfg = struct();
runtime_cfg.budget_s = 2.50;          % Laptop-relaxed per-step budget target
runtime_cfg.warn_s = 2.20;            % Warning threshold (laptop profile)
runtime_cfg.hard_limit_s = 3.20;      % Deterministic hard-overrun fallback threshold
runtime_cfg.acceptance_target_s = 1.00;
runtime_cfg.acceptance_slack = 2.50;  % laptop slack multiplier for acceptance checks

runtime_opt_cfg = struct();
runtime_opt_cfg.enable_warm_start = true;
runtime_opt_cfg.enable_control_hold = true;
runtime_opt_cfg.control_hold_base = 1;
runtime_opt_cfg.control_hold_max = 3;
runtime_opt_cfg.control_hold_clearance_m = 120;
runtime_opt_cfg.enable_adaptive_ref_horizon = true;
runtime_opt_cfg.ref_horizon_min_ratio = 0.40;
runtime_opt_cfg.ref_horizon_warn_ratio = 0.70;
runtime_opt_cfg.enable_adaptive_map_slots = true;
runtime_opt_cfg.map_slots_min = 3;
runtime_opt_cfg.map_slots_warn = 6;

% ---- SHIP HITBOX (rectangle) ----
enable_rectangular_hitbox = true;
ship_geometry_scale = 0.50;  % Geometry-only scaling: keeps hydrodynamic model untouched.
hitbox_cfg = struct();
hitbox_cfg.length_m = 175 * ship_geometry_scale;
hitbox_cfg.beam_m = 25.4 * ship_geometry_scale;
hitbox_cfg.safety_margin_m = 2;
hitbox_cfg.sample_points = 9;  % footprint samples for map collision checks
hitbox_cfg.enable = enable_rectangular_hitbox;
hitbox_cfg.use_point_fallback = ~enable_rectangular_hitbox;

% ---- BERTHING DIRECTION / TERMINAL HEADING GUIDANCE ----
berth_guidance_cfg = struct();
berth_guidance_cfg.enable = true;
berth_guidance_cfg.approach_switch_dist_m = 650;
berth_guidance_cfg.approach_offset_m = 240;
berth_guidance_cfg.heading_blend_dist_m = 180;
berth_guidance_cfg.direction_change_hysteresis = 0.15;
berth_guidance_cfg.requested_terminal_heading_deg = 320;  % If NaN, terminal heading is along final approach path; otherwise use this fixed heading.

% ---- PID FALLBACK GAINS (used when NMPC fails) ----
pid_Kp = 0.8;
pid_Ki = 0.01;
pid_Kd = 5.0;

% ---- REAL-TIME DIAGNOSTICS ----
enable_rt_terminal_plots = true;

% ---- OPTIONAL ENV OVERRIDES (for batch verification) ----
env_dyn = getenv('NMPC_ENABLE_DYNAMIC_OBS');
if ~isempty(env_dyn)
    enable_dynamic_obstacles = strcmpi(strtrim(env_dyn), '1') || strcmpi(strtrim(env_dyn), 'true');
end
env_tfinal = getenv('NMPC_TFINAL');
if ~isempty(env_tfinal)
    tf_num = str2double(env_tfinal);
    if isfinite(tf_num) && tf_num > 0
        T_final = tf_num;
    end
end

env_budget = getenv('NMPC_RUNTIME_BUDGET_S');
if ~isempty(env_budget)
    v = str2double(env_budget);
    if isfinite(v) && v > 0
        runtime_cfg.budget_s = v;
    end
end
env_warn = getenv('NMPC_RUNTIME_WARN_S');
if ~isempty(env_warn)
    v = str2double(env_warn);
    if isfinite(v) && v > 0
        runtime_cfg.warn_s = v;
    end
end
env_hard = getenv('NMPC_RUNTIME_HARD_S');
if ~isempty(env_hard)
    v = str2double(env_hard);
    if isfinite(v) && v > 0
        runtime_cfg.hard_limit_s = v;
    end
end
env_hold = getenv('NMPC_ENABLE_CONTROL_HOLD');
if ~isempty(env_hold)
    runtime_opt_cfg.enable_control_hold = strcmpi(strtrim(env_hold), '1') || strcmpi(strtrim(env_hold), 'true');
end
env_hold_max = getenv('NMPC_CONTROL_HOLD_MAX');
if ~isempty(env_hold_max)
    v = str2double(env_hold_max);
    if isfinite(v) && v >= 1
        runtime_opt_cfg.control_hold_max = round(v);
    end
end
env_hold_clearance = getenv('NMPC_CONTROL_HOLD_CLEARANCE_M');
if ~isempty(env_hold_clearance)
    v = str2double(env_hold_clearance);
    if isfinite(v) && v >= 0
        runtime_opt_cfg.control_hold_clearance_m = v;
    end
end
env_map_slots = getenv('NMPC_MAX_MAP_OBS');
if ~isempty(env_map_slots)
    v = str2double(env_map_slots);
    if isfinite(v) && v >= 1
        max_map_obstacles = round(v);
    end
end

%  INITIALIZATION (DO NOT EDIT BELOW UNLESS DEBUGGING)

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

    [in_final_zone, final_zone_type, final_zone_idx] = NavUtils.isInsideAnyMapZone(waypoints(end,:)', map);
    if in_final_zone
        warning('Final waypoint is inside map zone (%s #%d). This can block terminal convergence.', ...
            final_zone_type, final_zone_idx);
    else
        fprintf('  Final waypoint is outside mapped forbidden zones\n');
    end
end

% Pre-sample map polygon edges for fast local queries
map_sample_pts = [];
if enable_map_obstacles && ~isempty(map)
    map_sample_pts = buildMapSamplePoints(map, 100);
    fprintf('  Map sample points: %d\n', size(map_sample_pts, 1));

    d_final_map = min(sqrt(sum((map_sample_pts - waypoints(end,:)).^2, 2)));
    nominal_keepout = r_safety + map_sample_radius_m + avoid_ref_cfg.base_margin_m;
    fprintf('  Final waypoint -> nearest map sample: %.1f m (nominal keepout %.1f m)\n', ...
        d_final_map, nominal_keepout);
    if d_final_map < nominal_keepout
        warning(['Final waypoint is close to virtual map keep-out. ', ...
            'Terminal relaxation may be required to avoid circling.']);
    end
end

% Dynamic obstacle initialization (manual-only from user config)
map_bounds = estimateMapBounds(map, waypoints, dynamic_obs_boundary_margin);
dynamic_obstacles = struct('position', {}, 'radius', {}, 'speed', {}, 'heading', {}, 'active', {}, 'enabled', {}, 'id', {});
if enable_dynamic_obstacles
    dynamic_obstacles = buildDynamicObstaclesFromConfig( ...
        dynamic_obs_positions_xy, dynamic_obs_headings_deg, dynamic_obs_speeds_mps, ...
        dynamic_obs_radius_m, dynamic_obs_speed_mps);
    dynamic_obstacles = configureDynamicStartMode( ...
        dynamic_obstacles, dynamic_obs_start_mode, dynamic_obs_trigger_distance_m);
    fprintf('  Dynamic obstacles enabled: %d\n', length(dynamic_obstacles));
    if enable_dynamic_replay_check
        replay_ok = runDynamicReplayCheck(dynamic_obstacles, 40, nmpc_dt, map_bounds, dynamic_obs_boundary_policy);
        if replay_ok
            fprintf('  Dynamic replay determinism check: PASS\n');
        else
            warning('Dynamic replay determinism check failed.');
        end
    end
else
    fprintf('  Dynamic obstacles disabled\n');
end

%% ===== NMPC configuration ===============================================
n_static_obs = length(static_obstacles);
n_dynamic_obs = 0;
if enable_dynamic_obstacles
    n_dynamic_obs = length(dynamic_obstacles);
end
if enable_map_obstacles && ~isempty(map_sample_pts)
    max_obs_slots = n_static_obs + max_map_obstacles + n_dynamic_obs;
else
    max_obs_slots = n_static_obs + n_dynamic_obs;
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
nmpc_cfg.enforce_output_limits = enforce_command_limits;
nmpc_cfg.enable_warm_start = runtime_opt_cfg.enable_warm_start;
nmpc_cfg.enable_heading_hard_constraint = guidance_heading_hard_cfg.enable;
nmpc_cfg.heading_hard_max_err_deg = guidance_heading_hard_cfg.max_err_deg;
nmpc_cfg.enable_corner_obstacle_constraints = hitbox_cfg.enable;
nmpc_cfg.hitbox_length_m = hitbox_cfg.length_m;
nmpc_cfg.hitbox_beam_m = hitbox_cfg.beam_m;
nmpc_cfg.hitbox_safety_margin_m = hitbox_cfg.safety_margin_m;

[runtime_cfg, runtime_opt_cfg] = validateRuntimeOptimizationConfig(runtime_cfg, runtime_opt_cfg, nmpc_cfg, max_map_obstacles);

fprintf('\n--- Building NMPC solver (%d obstacle slots) ---\n', nmpc_cfg.max_obs);
nmpc = NMPC_Container_final(nmpc_cfg);
nmpc.buildSolver();

fprintf('  Hydrodynamic model hull: L=%.1f m (container.m / NMPC model)\n', nmpc.L);
fprintf('  Collision/animation footprint: L=%.1f m, B=%.1f m, margin=%.1f m\n', ...
    hitbox_cfg.length_m, hitbox_cfg.beam_m, hitbox_cfg.safety_margin_m);

[ship_hitbox_half_diag, ~] = getShipHitboxGeometry(hitbox_cfg);
avoid_ref_cfg.extra_clearance_m = ship_hitbox_half_diag;

%% ===== Initial state ====================================================
x0_heading = atan2(waypoints(2,2) - waypoints(1,2), ...
                   waypoints(2,1) - waypoints(1,1));
% State: [u, v, r, x, y, psi, n1, n2]
x = [7; 0; 0; waypoints(1,1); waypoints(1,2); x0_heading; n1_cruise; n2_cruise];

%% ===== Simulation setup =================================================
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;
wp_idx = 1;
final_capture_count = 0;
final_capture_steps_needed = max(1, ceil(final_capture_hold_s / dt));
terminal_mode_announced = false;

% Preallocate logging
traj     = zeros(8, length(t)+1);
ctrl     = zeros(4, length(t));
solve_ok = false(1, length(t));
xte_log  = zeros(1, length(t));
fallback = false(1, length(t));
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
runtime_event_class = strings(1, length(t));
runtime_warning_log = false(1, length(t));
runtime_overrun_log = false(1, length(t));
runtime_stage_keys = {'guidance', 'obstacle_pack', 'reference_build', 'solve_dispatch', 'plant_integrate', 'step_total'};
runtime_stage_ms = nan(length(runtime_stage_keys), length(t));
traj(:,1) = x;
steps = 0;

n_dyn = length(dynamic_obstacles);
dyn_obs_hist = nan(n_dyn, 2, length(t)+1);
if n_dyn > 0
    for jj = 1:n_dyn
        dyn_obs_hist(jj, :, 1) = dynamic_obstacles(jj).position(1:2)';
    end
end

% PID state
psi_err_int = 0;
psi_err_prev = 0;

% Previous control for NMPC rate limiting 
u_prev = [0; 0; n1_cruise; n2_cruise];
u_hold = u_prev;
hold_countdown = 0;
berth_state = struct('selected_dir', [NaN; NaN], 'terminal_heading', NaN, 'mode', 'nominal');
limit_events_total = 0;
runtime_diag = struct();
runtime_diag.on_time_count = 0;
runtime_diag.warning_count = 0;
runtime_diag.overrun_count = 0;
runtime_diag.event_log = struct('k', {}, 't', {}, 'class', {}, 'step_s', {}, 'solve_s', {}, 'rt_ratio', {}, 'n_obs', {}, 'fallback', {});

fprintf('\n  Waypoints: ');
for i = 1:size(waypoints, 1)
    fprintf('(%d, %d) ', waypoints(i,1), waypoints(i,2));
end

%  MAIN SIMULATION LOOP

for i = 1:length(t)
    t_step = tic;

    % ---- 1) Waypoint guidance -------------------------------------------
    t_seg = tic;
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, waypoints, waypoint_speeds, wp_idx, R_accept);
    chi_wp_nominal = chi_d;
    xte = computeXTE(x, waypoints, wp_idx);
    d_final_now = norm(x(4:5) - waypoints(end,:)');
    guide_time_log(i) = toc(t_seg);
    runtime_stage_ms(1, i) = 1e3 * guide_time_log(i);

    % ---- 1.5) Activate/propagate moving obstacles -----------------------
    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        if strcmpi(strtrim(dynamic_obs_start_mode), 'proximity')
            dynamic_obstacles = activateDynamicObstaclesByProximity(dynamic_obstacles, x(4:5));
        end
        if i > 1
            dynamic_obstacles = propagateDynamicObstacles( ...
                dynamic_obstacles, dt, map_bounds, dynamic_obs_boundary_policy);
        end
    end
    if ~isempty(dynamic_obstacles)
        for jj = 1:length(dynamic_obstacles)
            dyn_obs_hist(jj, :, i) = dynamic_obstacles(jj).position(1:2)';
        end
    end

    % ---- 2) Gather obstacles (static + map samples) ---------------------
    t_seg = tic;
    obs_local = static_obstacles;
    obs_source_tags = repmat({'static'}, 1, length(static_obstacles));
    
    if enable_map_obstacles && ~isempty(map_sample_pts)
        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead_now = min(map_lookahead_max_m, max(map_lookahead_min_m, U_now * map_lookahead_time_s));
        half_width_now = min(map_half_width_max_m, max(map_half_width_min_m, 0.45 * lookahead_now));

        on_final_leg = wp_idx >= max(1, size(waypoints,1)-1);
        terminal_mode_active = enable_terminal_map_relax && on_final_leg && (d_final_now < terminal_relax_dist_m);
        if terminal_mode_active && ~terminal_mode_announced
            fprintf('  [terminal-mode] final-leg map-sample relaxation active (d_final=%.1f m)\n', d_final_now);
            terminal_mode_announced = true;
        end

        % Near the terminal waypoint, reduce virtual-map corridor aggressiveness.
        if terminal_mode_active
            relax_ratio = max(0.25, d_final_now / max(terminal_relax_dist_m, 1));
            lookahead_now = max(120, relax_ratio * lookahead_now);
            half_width_now = max(80, relax_ratio * half_width_now);
        end

        max_map_slots_now = max_map_obstacles;
        if runtime_opt_cfg.enable_adaptive_map_slots && i > 1
            if rt_ratio_log(i-1) > runtime_cfg.warn_s / max(dt, 1e-9)
                max_map_slots_now = runtime_opt_cfg.map_slots_warn;
            else
                max_map_slots_now = max_map_obstacles;
            end
        end
        max_map_slots_now = min(max_map_obstacles, max(runtime_opt_cfg.map_slots_min, max_map_slots_now));

        obs_map = selectMapObstaclesFromSamples( ...
            map_sample_pts, x(4:5), chi_d, max_map_slots_now, ...
            lookahead_now, half_width_now, map_sample_radius_m);

        % Deterministic terminal fix:
        % remove virtual map sample obstacles on final approach to avoid
        % artificial local minima around a valid final waypoint.
        if terminal_mode_active && terminal_disable_map_samples
            obs_map = struct('position', {}, 'radius', {});
        end

        % Avoid building an artificial obstacle wall around the terminal point.
        if terminal_mode_active && ~isempty(obs_map)
            keep_obs = true(1, length(obs_map));
            p_goal = waypoints(end,:)';
            for jj = 1:length(obs_map)
                if norm(obs_map(jj).position(1:2) - p_goal) < terminal_map_exclusion_m
                    keep_obs(jj) = false;
                end
            end
            obs_map = obs_map(keep_obs);
        end

        obs_local = [obs_local, obs_map];
        if ~isempty(obs_map)
            obs_source_tags = [obs_source_tags, repmat({'map-sample'}, 1, length(obs_map))];
        end
    end
    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        obs_dyn = dynamicToCircleObstacles(dynamic_obstacles);
        obs_local = [obs_local, obs_dyn];
        if ~isempty(obs_dyn)
            obs_source_tags = [obs_source_tags, repmat({'dynamic'}, 1, length(obs_dyn))];
        end
        obs_pack_drift_log(i) = computeDynamicPackagingDrift(dynamic_obstacles, obs_local);
    end

    [chi_d, berth_cmd, berth_state] = selectBerthApproachGuidance( ...
        x, waypoints, wp_idx, chi_d, obs_local, berth_guidance_cfg, berth_state);
    obs_time_log(i) = toc(t_seg);
    runtime_stage_ms(2, i) = 1e3 * obs_time_log(i);

    % ---- 3) Build reference trajectory ----------------------------------
    t_seg = tic;
    avoid_ref_step = avoid_ref_cfg;
    if enable_terminal_map_relax && wp_idx >= max(1, size(waypoints,1)-1) && d_final_now < terminal_relax_dist_m
        avoid_scale = max(terminal_min_avoid_scale, d_final_now / max(terminal_relax_dist_m, 1));
        avoid_ref_step.base_margin_m = avoid_ref_cfg.base_margin_m * avoid_scale;
        avoid_ref_step.speed_gain_s = avoid_ref_cfg.speed_gain_s * avoid_scale;
    end
    x_ref = buildObstacleAwareRef8(x, chi_d, U_d, nmpc.N, dt, ...
                                   n1_cruise, n2_cruise, obs_local, avoid_ref_step, berth_cmd);

    if runtime_opt_cfg.enable_adaptive_ref_horizon
        ref_ratio = 1.0;
        if i > 1 && rt_ratio_log(i-1) > runtime_cfg.warn_s / max(dt, 1e-9)
            ref_ratio = runtime_opt_cfg.ref_horizon_warn_ratio;
        end
        ref_ratio = max(runtime_opt_cfg.ref_horizon_min_ratio, min(1.0, ref_ratio));
        N_eff = max(2, min(nmpc.N, round(ref_ratio * nmpc.N)));
        if N_eff < nmpc.N
            x_ref(:, N_eff+2:end) = repmat(x_ref(:, N_eff+1), 1, nmpc.N - N_eff);
        end
    end

    if guidance_heading_hard_cfg.use_nominal_waypoint_heading
        psi_hard_ref = buildHeadingGuideProfile(chi_wp_nominal, nmpc.N);
    else
        psi_hard_ref = buildHeadingGuideProfile(chi_d, nmpc.N);
    end
    ref_time_log(i) = toc(t_seg);
    runtime_stage_ms(3, i) = 1e3 * ref_time_log(i);

    % ---- 4) Solve NMPC (MODIFIED - now passes u_prev) -------------------
    t_seg = tic;
    do_solve = true;
    hold_steps_now = runtime_opt_cfg.control_hold_base;
    min_guard_clearance = inf;
    if ~isempty(obs_local)
        for jj = 1:length(obs_local)
            d_center = norm(x(4:5) - obs_local(jj).position(1:2));
            guard_clear = d_center - (obs_local(jj).radius + r_safety + ship_hitbox_half_diag);
            min_guard_clearance = min(min_guard_clearance, guard_clear);
        end
    end
    near_obstacle_guard = min_guard_clearance < runtime_opt_cfg.control_hold_clearance_m;
    if runtime_opt_cfg.enable_control_hold && i > 1
        if near_obstacle_guard
            hold_steps_now = 1;
        elseif rt_ratio_log(i-1) > runtime_cfg.warn_s / max(dt, 1e-9)
            hold_steps_now = runtime_opt_cfg.control_hold_max;
        end
        hold_steps_now = max(1, min(runtime_opt_cfg.control_hold_max, hold_steps_now));
    end

    if hold_countdown > 0
        do_solve = false;
        hold_countdown = hold_countdown - 1;
    end

    if do_solve
        [u_opt, ~, info] = nmpc.solve(x, x_ref, obs_local, u_prev, psi_hard_ref);
        u_hold = u_opt;
        hold_countdown = hold_steps_now - 1;
    else
        u_opt = u_hold;
        info = struct();
        info.success = true;
        info.solve_time = 0;
        info.n_obs_real = length(obs_local);
        info.cost = NaN;
        info.limit_events = struct('step', {}, 'channel', {}, 'requested', {}, 'bounded', {}, 'bound', {}, 'timestamp', {});
        info.used_hold = true;
    end
    solve_call_log(i) = toc(t_seg);
    runtime_stage_ms(4, i) = 1e3 * solve_call_log(i);
    if isfield(info, 'solve_time')
        solve_time_log(i) = info.solve_time;
    else
        solve_time_log(i) = solve_call_log(i);
    end
    if isfield(info, 'n_obs_real')
        n_obs_log(i) = info.n_obs_real;
    else
        n_obs_log(i) = length(obs_local);
    end
    if isfield(info, 'cost')
        cost_log(i) = info.cost;
    end
    if isfield(info, 'limit_events') && ~isempty(info.limit_events)
        limit_events_total = limit_events_total + numel(info.limit_events);
    end

    % ---- 5) PID fallback if NMPC fails ----------------------------------
    fallback_reason = "none";
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
        
        u_pid = [alpha; 0; n1_cmd; 0];
        [u_opt, lim_events_pid] = nmpc.enforceControlLimits(u_pid, u_prev);
        limit_events_total = limit_events_total + numel(lim_events_pid);
        fallback(i) = true;
        fallback_reason = "nmpc-fail";
        
        if sum(fallback(1:i)) <= 3
            fprintf('  ⚠ NMPC fail at t=%.0f s, using PID fallback\n', t(i));
        end
    end

    % ---- 6) Simulate plant (RK4) ----------------------------------------
    elapsed_pre_integr = toc(t_step);
    if elapsed_pre_integr > runtime_cfg.hard_limit_s
        u_overrun = buildRuntimeFallbackCommand(u_prev, n1_cruise, n2_cruise);
        [u_opt, lim_events_rt] = nmpc.enforceControlLimits(u_overrun, u_prev);
        limit_events_total = limit_events_total + numel(lim_events_rt);
        fallback(i) = true;
        fallback_reason = "runtime-hard-limit";
    end

    t_seg = tic;
    x_old = x;
    x = rk4Step8(x, u_opt, dt);
    integr_time_log(i) = toc(t_seg);
    runtime_stage_ms(5, i) = 1e3 * integr_time_log(i);
    step_time_log(i) = toc(t_step);
    runtime_stage_ms(6, i) = 1e3 * step_time_log(i);
    rt_ratio_log(i) = step_time_log(i) / max(dt, 1e-9);

    [runtime_class, is_warn, is_overrun] = classifyRuntimeStep(step_time_log(i), runtime_cfg);
    runtime_event_class(i) = runtime_class;
    runtime_warning_log(i) = is_warn;
    runtime_overrun_log(i) = is_overrun;
    if runtime_class == "on-time"
        runtime_diag.on_time_count = runtime_diag.on_time_count + 1;
    elseif runtime_class == "warning"
        runtime_diag.warning_count = runtime_diag.warning_count + 1;
    else
        runtime_diag.overrun_count = runtime_diag.overrun_count + 1;
    end
    evt = struct();
    evt.k = i;
    evt.t = t(i);
    evt.class = char(runtime_class);
    evt.step_s = step_time_log(i);
    evt.solve_s = solve_time_log(i);
    evt.rt_ratio = rt_ratio_log(i);
    evt.n_obs = n_obs_log(i);
    evt.fallback = char(fallback_reason);
    runtime_diag.event_log(end+1) = evt;

    % ---- 7) Collision checks (map + circle obstacles) ------------------
    % Hard-collision stop uses only physical obstacles here.
    % Virtual map-sample obstacles remain in NMPC optimization, but they should
    % not trigger simulation termination; true map contact is handled by hit_map.
    obs_hard = obs_local;
    if ~isempty(obs_hard) && ~isempty(obs_source_tags)
        keep_hard = true(1, length(obs_hard));
        n_tag = min(length(obs_source_tags), length(obs_hard));
        for kk = 1:n_tag
            if strcmpi(obs_source_tags{kk}, 'map-sample')
                keep_hard(kk) = false;
            end
        end
        obs_hard = obs_hard(keep_hard);
    end

    if hitbox_cfg.enable
        footprint = buildShipFootprint(x(4:5), x(6), hitbox_cfg);
        [hit_obs, hit_obs_idx, min_sep_obs] = detectFootprintObstacleHit(footprint, obs_hard);
        [hit_map, hit_map_type, hit_map_idx] = detectFootprintMapHit(footprint, map, hitbox_cfg.sample_points);
    else
        [hit_obs, hit_obs_idx, min_sep_obs] = detectCircleObstacleHit(x(4:5), obs_hard, 0.0);
        [hit_map, hit_map_type, hit_map_idx] = NavUtils.isInsideAnyMapZone(x(4:5), map);
    end
    if hit_obs || hit_map
        collision_log(i) = true;
        fprintf('  [COLLISION] t=%.1f s hit_obs=%d hit_map=%d\n', t(i), hit_obs, hit_map);
        if hit_obs && hit_obs_idx >= 1 && hit_obs_idx <= length(obs_hard)
            src = 'hard-obstacle';
            c_hit = obs_hard(hit_obs_idx).position(1:2);
            d_center = norm(x(4:5) - c_hit);
            fprintf('    [COLLISION-DETAIL] source=%s idx=%d center_dist=%.2f m radius=%.2f m sep=%.2f m\n', ...
                src, hit_obs_idx, d_center, obs_hard(hit_obs_idx).radius, min_sep_obs);
        elseif isfinite(min_sep_obs)
            fprintf('    [COLLISION-DETAIL] obstacle sep=%.2f m\n', min_sep_obs);
        end
        if hit_map
            fprintf('    [COLLISION-DETAIL] map_zone=%s idx=%d\n', hit_map_type, hit_map_idx);
        end
        steps = i;
        traj(:, i+1)  = x;
        ctrl(:, i)    = u_opt;
        solve_ok(i)   = info.success;
        xte_log(i)    = xte;
        u_prev = u_opt;
        break;
    end

    % ---- 8) Logging -----------------------------------------------------
    steps = i;
    traj(:, i+1)  = x;
    ctrl(:, i)    = u_opt;
    solve_ok(i)   = info.success;
    xte_log(i)    = xte;
    
    % Update previous control for next iteration
    u_prev = u_opt;

    % ---- 9) Progress print ----------------------------------------------
    if i == 1 || mod(i, 20) == 0 || ~info.success || rt_ratio_log(i) > 1.0
        d_nearest_obs = inf;
        for j = 1:length(obs_local)
            d_nearest_obs = min(d_nearest_obs, norm(x(4:5) - obs_local(j).position));
        end
        fprintf(['  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+6.1fdeg wp=%d xte=%+.1fm ', ...
                 'obs_d=%.0fm ok=%d comp=%.1fms solve=%.1fms RT=%.2f obs=%d fb=%d\n'], ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, d_nearest_obs, info.success, ...
            1e3*step_time_log(i), 1e3*solve_time_log(i), rt_ratio_log(i), ...
            round(n_obs_log(i)), fallback(i));
    end

    % ---- 10) Check if done ----------------------------------------------
    d_final_now = norm(x(4:5) - waypoints(end,:)');
    U_now_ship = hypot(x(1), x(2));
    hard_final_hit = (d_final_now < R_accept_final);
    soft_final_hit = (d_final_now < R_accept_final_soft) && (U_now_ship <= final_capture_speed_mps);

    if hard_final_hit || soft_final_hit
        final_capture_count = final_capture_count + 1;
    else
        final_capture_count = 0;
    end

    if hard_final_hit || final_capture_count >= final_capture_steps_needed
        if hard_final_hit
            fprintf('\n  ✓ FINAL WAYPOINT REACHED (hard gate %.1f m) at t=%.1f s!\n', R_accept_final, t(i));
        else
            fprintf('\n  ✓ FINAL WAYPOINT CAPTURED (soft gate %.1f m, U<=%.1f m/s for %.1f s) at t=%.1f s!\n', ...
                R_accept_final_soft, final_capture_speed_mps, final_capture_hold_s, t(i));
        end
        break;
    end
end

%  POST-PROCESSING

% Trim logs
traj     = traj(:, 1:steps+1);
ctrl     = ctrl(:, 1:steps);
solve_ok = solve_ok(1:steps);
xte_log  = xte_log(1:steps);
fallback = fallback(1:steps);
t_sim    = (0:steps) * dt;
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
runtime_event_class = runtime_event_class(1:steps);
runtime_warning_log = runtime_warning_log(1:steps);
runtime_overrun_log = runtime_overrun_log(1:steps);
runtime_stage_ms = runtime_stage_ms(:, 1:steps);

n_ok = sum(solve_ok);  n_tot = length(solve_ok);
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  SUMMARY\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC solves: %d/%d (%.1f%%)\n', n_ok, n_tot, 100*n_ok/max(n_tot,1));
fprintf('  PID fallback: %d times\n', sum(fallback));
fprintf('  Mean |XTE|: %.1f m, Max |XTE|: %.1f m\n', mean(abs(xte_log)), max(abs(xte_log)));
fprintf('  Final position: (%.1f, %.1f)\n', traj(4,end), traj(5,end));
if any(isfinite(obs_pack_drift_log))
    fprintf('  Dynamic packaging drift [m]: max=%.3f\n', max(obs_pack_drift_log(isfinite(obs_pack_drift_log))));
end
fprintf('  Collisions detected: %d\n', sum(collision_log));
fprintf('  Limit events (post-solve bounded): %d\n', limit_events_total);

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
fprintf('  Worst RT ratio: %.3f\n', max(rt_ratio_log(valid_step)));
fprintf('  Runtime budget [s]: target=%.2f warn=%.2f hard=%.2f\n', ...
    runtime_cfg.budget_s, runtime_cfg.warn_s, runtime_cfg.hard_limit_s);
fprintf('  Runtime class counts: on-time=%d warning=%d overrun=%d\n', ...
    runtime_diag.on_time_count, runtime_diag.warning_count, runtime_diag.overrun_count);

stage_mean_ms = mean(runtime_stage_ms(:, valid_step), 2, 'omitnan');
stage_p95_ms = nan(size(stage_mean_ms));
for kk = 1:numel(stage_mean_ms)
    stage_p95_ms(kk) = 1e3 * safePercentile(runtime_stage_ms(kk, valid_step) / 1e3, 95);
end
fprintf('  Stage timings [ms] (mean / p95):\n');
for kk = 1:numel(runtime_stage_keys)
    fprintf('    %-16s %8.2f / %8.2f\n', runtime_stage_keys{kk}, stage_mean_ms(kk), stage_p95_ms(kk));
end

acc_threshold_s = runtime_cfg.acceptance_target_s * runtime_cfg.acceptance_slack;
rt_p95_s = safePercentile(step_time_log(valid_step), 95);
rt_max_s = max(step_time_log(valid_step));
rt_accept_ok = (rt_p95_s <= acc_threshold_s) && (runtime_diag.overrun_count == 0);
fprintf('  Runtime acceptance: p95<=%.2f s and 0 hard overruns -> %s\n', ...
    acc_threshold_s, ternary(rt_accept_ok, 'PASS', 'FAIL'));

if any(valid_step)
    [~, worst_idx] = sort(step_time_log, 'descend');
    n_worst = min(5, numel(worst_idx));
    fprintf('\n  TOP-%d SLOWEST STEPS\n', n_worst);
    fprintf('    k    t[s]   step[ms]  solve[ms]  guide[ms]  obs[ms]  ref[ms]  int[ms]  ok\n');
    for kk = 1:n_worst
        k = worst_idx(kk);
        fprintf('  %4d  %6.1f   %7.2f    %7.2f    %7.2f   %6.2f  %7.2f  %7.2f   %d\n', ...
            k, t(k), 1e3*step_time_log(k), 1e3*solve_time_log(k), 1e3*guide_time_log(k), ...
            1e3*obs_time_log(k), 1e3*ref_time_log(k), 1e3*integr_time_log(k), solve_ok(k));
    end
end

if enable_rt_terminal_plots
    fprintf('\n  TERMINAL TIMING PLOTS (ASCII)\n');
    printTimingHistogram('Total step time', step_time_log, dt);
    printTimingHistogram('NMPC solve time', solve_time_log, dt);
    printTimingHistogram('RT ratio (step/dt)', rt_ratio_log, 1.0);
end

% Optional metrics export for benchmark automation
metrics_out_path = strtrim(getenv('NMPC_METRICS_OUT'));
if ~isempty(metrics_out_path)
    runtime_metrics = struct();
    runtime_metrics.scenario = strtrim(getenv('NMPC_SCENARIO_NAME'));
    runtime_metrics.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
    runtime_metrics.dt = dt;
    runtime_metrics.steps = steps;
    runtime_metrics.solve_success_rate = n_ok / max(n_tot, 1);
    runtime_metrics.fallback_count = sum(fallback);
    runtime_metrics.collisions = sum(collision_log);
    runtime_metrics.final_distance_m = norm(traj(4:5,end) - waypoints(end,:)');
    runtime_metrics.final_reached = runtime_metrics.final_distance_m <= max(R_accept_final, R_accept_final_soft);
    runtime_metrics.step_mean_s = mean(step_time_log(valid_step));
    runtime_metrics.step_p95_s = rt_p95_s;
    runtime_metrics.step_max_s = rt_max_s;
    runtime_metrics.deadline_miss_rate = n_overrun / max(steps, 1);
    runtime_metrics.runtime_budget_s = runtime_cfg.budget_s;
    runtime_metrics.runtime_warn_s = runtime_cfg.warn_s;
    runtime_metrics.runtime_hard_limit_s = runtime_cfg.hard_limit_s;
    runtime_metrics.runtime_acceptance_threshold_s = acc_threshold_s;
    runtime_metrics.runtime_accept_ok = rt_accept_ok;
    runtime_metrics.runtime_class_counts = struct('on_time', runtime_diag.on_time_count, ...
                                                 'warning', runtime_diag.warning_count, ...
                                                 'overrun', runtime_diag.overrun_count);
    runtime_metrics.stage_keys = runtime_stage_keys;
    runtime_metrics.stage_mean_ms = stage_mean_ms;
    runtime_metrics.stage_p95_ms = stage_p95_ms;

    [out_dir, ~, ~] = fileparts(metrics_out_path);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    save(metrics_out_path, 'runtime_metrics');
    fprintf('  Runtime metrics exported: %s\n', metrics_out_path);
end

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

% Recording options (passed to animateSimResult)
cfg_anim.recordVideo = enable_animation_recording;
cfg_anim.recordFps   = record_fps;
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
record_dir = record_output_dir;
if cfg_anim.recordVideo && ~exist(record_dir, 'dir')
    mkdir(record_dir);
end
cfg_anim.videoFile = fullfile(record_dir, sprintf('nmpc_run_%s.mp4', timestamp));

% Visualize ship hitbox in animation as a light-blue rectangle.
cfg_anim.hitbox = struct();
cfg_anim.hitbox.enable = hitbox_cfg.enable;
cfg_anim.hitbox.length_m = hitbox_cfg.length_m;
cfg_anim.hitbox.beam_m = hitbox_cfg.beam_m;
cfg_anim.hitbox.safety_margin_m = hitbox_cfg.safety_margin_m;
cfg_anim.hitbox.edge_color = [0.45 0.82 1.00];
cfg_anim.hitbox.face_color = [0.45 0.82 1.00];
cfg_anim.hitbox.face_alpha = 0.10;
cfg_anim.hitbox.line_width = 1.4;

% Add static obstacles to animation
for j = 1:length(static_obstacles)
    cfg_anim.circObs(j).position = static_obstacles(j).position;
    cfg_anim.circObs(j).radius   = static_obstacles(j).radius;
end
if ~isempty(dynamic_obstacles)
    cfg_anim.dynamicObsHistory = dyn_obs_hist(:, :, 1:steps+1);
    cfg_anim.dynamicObsRadius = dynamic_obs_radius_m;
end

traj_anim = traj(1:6, :);
fprintf('\n  Launching animation...\n');
if cfg_anim.recordVideo
    fprintf('  Recording video to: %s\n', cfg_anim.videoFile);
else
    fprintf('  Recording disabled by configuration.\n');
end
try
    animateSimResult(traj_anim, waypoints, t_sim, harbor_anim, cfg_anim);
catch ME
    warning('run_nmpc:AnimationRecordFailed', '%s', ME.message);
    warning('run_nmpc:AnimationRetryNoRecord', 'Retrying animation without recording.');
    cfg_anim.recordVideo = false;
    animateSimResult(traj_anim, waypoints, t_sim, harbor_anim, cfg_anim);
end

if enable_animation_recording
    if isfile(cfg_anim.videoFile)
        fprintf('  Video saved: %s\n', cfg_anim.videoFile);
    else
        [vp, vn, ~] = fileparts(cfg_anim.videoFile);
        avi_fallback = fullfile(vp, [vn '.avi']);
        if isfile(avi_fallback)
            fprintf('  Video saved (AVI fallback): %s\n', avi_fallback);
        else
            fprintf('  WARNING: expected video file not found: %s\n', cfg_anim.videoFile);
        end
    end
end

fprintf('\nDone. Check figures.\n');


%  LOCAL FUNCTIONS

function [runtime_cfg, runtime_opt_cfg] = validateRuntimeOptimizationConfig(runtime_cfg, runtime_opt_cfg, nmpc_cfg, max_map_obstacles)
% Validate and clamp runtime/performance settings to safe deterministic bounds.
    runtime_cfg.budget_s = max(0.05, runtime_cfg.budget_s);
    runtime_cfg.warn_s = max(0.01, runtime_cfg.warn_s);
    runtime_cfg.hard_limit_s = max(runtime_cfg.warn_s + 0.01, runtime_cfg.hard_limit_s);
    runtime_cfg.acceptance_target_s = max(0.05, runtime_cfg.acceptance_target_s);
    runtime_cfg.acceptance_slack = min(4.0, max(1.0, runtime_cfg.acceptance_slack));

    runtime_opt_cfg.control_hold_base = max(1, round(runtime_opt_cfg.control_hold_base));
    runtime_opt_cfg.control_hold_max = max(runtime_opt_cfg.control_hold_base, round(runtime_opt_cfg.control_hold_max));
    runtime_opt_cfg.control_hold_max = min(5, runtime_opt_cfg.control_hold_max);
    runtime_opt_cfg.control_hold_clearance_m = max(0, runtime_opt_cfg.control_hold_clearance_m);

    runtime_opt_cfg.ref_horizon_min_ratio = min(1.0, max(0.2, runtime_opt_cfg.ref_horizon_min_ratio));
    runtime_opt_cfg.ref_horizon_warn_ratio = min(1.0, max(runtime_opt_cfg.ref_horizon_min_ratio, runtime_opt_cfg.ref_horizon_warn_ratio));

    runtime_opt_cfg.map_slots_min = max(1, min(max_map_obstacles, round(runtime_opt_cfg.map_slots_min)));
    runtime_opt_cfg.map_slots_warn = max(runtime_opt_cfg.map_slots_min, min(max_map_obstacles, round(runtime_opt_cfg.map_slots_warn)));

    if nmpc_cfg.N < 10 && runtime_opt_cfg.enable_adaptive_ref_horizon
        warning('Adaptive reference horizon enabled with short N=%d; disabling adaptation for stability.', nmpc_cfg.N);
        runtime_opt_cfg.enable_adaptive_ref_horizon = false;
    end
end

function [runtime_class, is_warn, is_overrun] = classifyRuntimeStep(step_s, runtime_cfg)
    is_overrun = step_s > runtime_cfg.hard_limit_s;
    is_warn = (~is_overrun) && (step_s > runtime_cfg.warn_s);
    if is_overrun
        runtime_class = "overrun";
    elseif is_warn
        runtime_class = "warning";
    else
        runtime_class = "on-time";
    end
end

function u_fb = buildRuntimeFallbackCommand(u_prev, n1_cruise, n2_cruise)
% Deterministic hard-overrun policy: hold azimuth, decay shaft commands toward cruise.
    u_fb = u_prev(:);
    if numel(u_fb) ~= 4
        u_fb = [0; 0; n1_cruise; n2_cruise];
        return;
    end
    u_fb(3) = 0.85 * u_prev(3) + 0.15 * n1_cruise;
    u_fb(4) = 0.85 * u_prev(4) + 0.15 * n2_cruise;
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, wp_speed, wp_idx, R_accept)
% Simple waypoint steering for 8-state model
    n_wps = size(wp, 1);
    pos   = [x(4); x(5)];
    d_final = norm(pos - wp(end,:)');
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
        p_next = wp(min(wp_idx + 2, n_wps), :)';
        d_to_next_waypoint = norm(pos - p_next);
        xte_seg = abs(((pos(1)-p_from(1))*seg(2) - (pos(2)-p_from(2))*seg(1)) / max(seg_len, 1e-6));
        near_segment = xte_seg <= max(3*R_accept, 120);

        % Intermediate checkpoints are intentionally skippable when
        % obstacle/dynamic constraints push the vessel off the nominal line.
        skip_due_to_progress = (proj >= 0.95) && near_segment;
        skip_due_to_better_next = (proj >= 0.75) && near_segment && (d_to_next_waypoint < d_to_waypoint);
        if d_to_waypoint <= R_accept || skip_due_to_progress || skip_due_to_better_next || (proj >= 1.0 && near_segment)
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
        lookahead_nominal = max(60, 8 * U_now);
        lookahead_terminal = 30 + 0.55 * d_final;
        lookahead = max(20, min(lookahead_nominal, lookahead_terminal));
        s_target = min(seg_len, s_proj + lookahead);
        target = p1 + (s_target / seg_len) * seg;
    end

    dp = target - pos;
    chi_d = atan2(dp(2), dp(1));
    U_d = wp_speed(min(wp_idx + 1, length(wp_speed)));

    if d_final < 220
        U_d = min(U_d, 1.2 + 0.03 * d_final);
    end
    if d_final < 70
        U_d = min(U_d, 1.0);
    end
    U_d = max(0.8, U_d);
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

function psi_ref = buildHeadingGuideProfile(chi_cmd, N)
% Constant heading guide profile for hard heading corridor constraints.
    psi_ref = repmat(chi_cmd, N+1, 1);
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

function x_ref = buildObstacleAwareRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref, obstacles, avoid_cfg, berth_cmd)
% Reference trajectory with obstacle deflection
    if nargin < 9 || isempty(avoid_cfg)
        avoid_cfg = struct('base_margin_m', 80, 'speed_gain_s', 0.0, ...
            'obs_radius_gain', 0.5, 'deflect_sigma', 0.22, 'r_ref_max', 0.10);
    end
    if nargin < 10 || isempty(berth_cmd)
        berth_cmd = struct('active', false, 'terminal_heading', NaN, 'distance_to_goal', inf, 'heading_blend_dist', 180);
    end
    if ~isfield(avoid_cfg, 'extra_clearance_m')
        avoid_cfg.extra_clearance_m = 0;
    end
    safety_margin = avoid_cfg.base_margin_m + avoid_cfg.speed_gain_s * max(0, U_d);
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
        if along < 0 || along > horizon_dist + obs_r + avoid_cfg.extra_clearance_m
            continue;
        end
        if abs(lateral) >= (safety_margin + obs_r + avoid_cfg.extra_clearance_m)
            continue;
        end

        if lateral >= 0
            side_sign = -1;
        else
            side_sign = +1;
        end

        deflect = side_sign * (safety_margin + avoid_cfg.extra_clearance_m + avoid_cfg.obs_radius_gain * obs_r - lateral * side_sign);

        for k = 1:(N+1)
            s = (k-1) / N;
            s_peak = along / horizon_dist;
            s_peak = max(0.05, min(0.95, s_peak));
            w = exp(-((s - s_peak)^2) / (avoid_cfg.deflect_sigma^2));

            x_ref(4, k) = x_ref(4, k) + w * deflect * perp(1);
            x_ref(5, k) = x_ref(5, k) + w * deflect * perp(2);
        end

        if N > 1
            dx_ref = x_ref(4, N+1) - x_ref(4, 1);
            dy_ref = x_ref(5, N+1) - x_ref(5, 1);
            chi_deflected = atan2(dy_ref, dx_ref);
            psi_err2 = atan2(sin(chi_deflected - x0(6)), cos(chi_deflected - x0(6)));
            r_d2 = max(-avoid_cfg.r_ref_max, min(avoid_cfg.r_ref_max, 0.35 * psi_err2));
            for k = 2:(N+1)
                x_ref(3, k) = r_d2;
                x_ref(6, k) = chi_deflected;
            end
        end
    end

    if berth_cmd.active && isfinite(berth_cmd.terminal_heading)
        for k = 2:(N+1)
            s = (k-1) / max(N,1);
            blend_gate = max(0, min(1, 1 - berth_cmd.distance_to_goal / max(berth_cmd.heading_blend_dist, 1)));
            w = min(1, blend_gate + s^2 * (1 - blend_gate));
            x_ref(6, k) = blendAngles(x_ref(6, k), berth_cmd.terminal_heading, w);
            psi_err = atan2(sin(x_ref(6, k) - x0(6)), cos(x_ref(6, k) - x0(6)));
            x_ref(3, k) = max(-avoid_cfg.r_ref_max, min(avoid_cfg.r_ref_max, 0.35 * psi_err));
        end
    end
end

function [chi_cmd, berth_cmd, state] = selectBerthApproachGuidance(x, waypoints, ~, chi_nom, obstacles, cfg, state)
% Chooses an obstacle-aware approach direction and terminal heading near berth.
    chi_cmd = chi_nom;
    berth_cmd = struct('active', false, 'terminal_heading', NaN, 'distance_to_goal', inf, 'heading_blend_dist', 180);

    if nargin < 7 || isempty(state)
        state = struct('selected_dir', [NaN; NaN], 'terminal_heading', NaN, 'mode', 'nominal');
    end
    if nargin < 6 || isempty(cfg) || ~isfield(cfg, 'enable') || ~cfg.enable
        return;
    end

    p_ship = x(4:5);
    p_goal = waypoints(end, :)';
    d_goal = norm(p_goal - p_ship);
    berth_cmd.distance_to_goal = d_goal;
    berth_cmd.heading_blend_dist = cfg.heading_blend_dist_m;

    if d_goal > cfg.approach_switch_dist_m
        state.mode = 'nominal';
        return;
    end

    if isfield(cfg, 'requested_terminal_heading_deg') && isfinite(cfg.requested_terminal_heading_deg)
        psi_term = deg2rad(cfg.requested_terminal_heading_deg);
    else
        if size(waypoints,1) >= 2
            seg = waypoints(end,:) - waypoints(end-1,:);
            psi_term = atan2(seg(2), seg(1));
        else
            psi_term = x(6);
        end
    end

    headings = [psi_term, wrapToPi(psi_term + pi)];
    best_cost = inf;
    best_dir = [cos(psi_term); sin(psi_term)];
    for c = 1:2
        dir_c = [cos(headings(c)); sin(headings(c))];
        p_app = p_goal - cfg.approach_offset_m * dir_c;
        chi_c = atan2(p_app(2) - p_ship(2), p_app(1) - p_ship(1));
        dchi = atan2(sin(chi_c - x(6)), cos(chi_c - x(6)));
        hchi = atan2(sin(chi_c - chi_nom), cos(chi_c - chi_nom));
        obs_pen = corridorObstaclePenalty(p_ship, p_app, obstacles);
        cost = 0.9*abs(dchi) + 0.5*abs(hchi) + obs_pen;

        if all(isfinite(state.selected_dir))
            same_sign = sign(dot(state.selected_dir, dir_c));
            if same_sign < 0
                cost = cost + cfg.direction_change_hysteresis;
            end
        end

        if cost < best_cost
            best_cost = cost;
            chi_cmd = chi_c;
            best_dir = dir_c;
        end
    end

    if d_goal < cfg.heading_blend_dist_m
        chi_cmd = blendAngles(chi_cmd, psi_term, 1 - d_goal / max(cfg.heading_blend_dist_m, 1));
    end

    state.selected_dir = best_dir;
    state.terminal_heading = psi_term;
    state.mode = 'berthing';
    berth_cmd.active = true;
    berth_cmd.terminal_heading = psi_term;
end

function p = corridorObstaclePenalty(p0, p1, obstacles)
% Lightweight obstacle penalty along an approach corridor.
    p = 0;
    if isempty(obstacles)
        return;
    end
    for k = 1:length(obstacles)
        [d, ~] = NavUtils.pointToSegment(obstacles(k).position(1), obstacles(k).position(2), ...
            p0(1), p0(2), p1(1), p1(2));
        clear_margin = d - obstacles(k).radius;
        if clear_margin < 0
            p = p + 6.0;
        elseif clear_margin < 80
            p = p + 0.015 * (80 - clear_margin);
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

function obs_dyn = dynamicToCircleObstacles(dynamic_obstacles)
% Convert active dynamic obstacle states to NMPC-compatible circle obstacles
    obs_dyn = struct('position', {}, 'radius', {});
    if isempty(dynamic_obstacles)
        return;
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
        obs_dyn(out_idx).radius = dynamic_obstacles(k).radius;
    end
end

function [hit, hit_idx, min_sep] = detectCircleObstacleHit(pos_xy, obstacles, safety_buffer)
% Circle collision check with configurable safety buffer
    hit = false;
    hit_idx = 0;
    min_sep = inf;
    if nargin < 3 || isempty(safety_buffer)
        safety_buffer = 0;
    end
    if isempty(obstacles)
        return;
    end

    for k = 1:length(obstacles)
        d = norm(pos_xy(:) - obstacles(k).position(1:2));
        sep = d - (obstacles(k).radius + safety_buffer);
        min_sep = min(min_sep, sep);
        if sep <= 0
            hit = true;
            hit_idx = k;
            return;
        end
    end
end

function footprint = buildShipFootprint(pos_xy, psi, cfg)
% Rectangle footprint in world frame (4x2 vertices in [x y]).
    L = cfg.length_m + 2*cfg.safety_margin_m;
    B = cfg.beam_m + 2*cfg.safety_margin_m;
    half = [L/2, B/2];
    local = [ half(1),  half(2);
              half(1), -half(2);
             -half(1), -half(2);
             -half(1),  half(2)];
    R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
    world = (R * local')';
    world(:,1) = world(:,1) + pos_xy(1);
    world(:,2) = world(:,2) + pos_xy(2);
    footprint = world;
end

function [hit, hit_idx, min_sep] = detectFootprintObstacleHit(footprint, obstacles)
% Collision between rectangle footprint and circle obstacles.
    hit = false;
    hit_idx = 0;
    min_sep = inf;
    if isempty(obstacles)
        return;
    end

    poly_x = footprint(:,1);
    poly_y = footprint(:,2);
    n = size(footprint,1);
    for k = 1:length(obstacles)
        c = obstacles(k).position(1:2);
        r = obstacles(k).radius;

        [in_poly, on_poly] = inpolygon(c(1), c(2), poly_x, poly_y);
        if in_poly || on_poly
            hit = true;
            hit_idx = k;
            min_sep = -r;
            return;
        end

        d_min = inf;
        for i = 1:n
            j = mod(i, n) + 1;
            [d, ~] = NavUtils.pointToSegment(c(1), c(2), footprint(i,1), footprint(i,2), footprint(j,1), footprint(j,2));
            d_min = min(d_min, d);
        end

        sep = d_min - r;
        min_sep = min(min_sep, sep);
        if sep <= 0
            hit = true;
            hit_idx = k;
            return;
        end
    end
end

function [hit, zone_type, zone_idx] = detectFootprintMapHit(footprint, map, n_samples)
% Map collision check for rectangle footprint (vertices + sampled edges).
    hit = false;
    zone_type = '';
    zone_idx = 0;
    if isempty(map)
        return;
    end
    if nargin < 3 || isempty(n_samples)
        n_samples = 7;
    end

    % Vertices first
    for i = 1:size(footprint,1)
        [in_zone, zt, zi] = NavUtils.isInsideAnyMapZone(footprint(i,:)', map);
        if in_zone
            hit = true;
            zone_type = zt;
            zone_idx = zi;
            return;
        end
    end

    % Sample edge points to catch edge intersection with map zones.
    n = size(footprint,1);
    for i = 1:n
        j = mod(i, n) + 1;
        p1 = footprint(i,:);
        p2 = footprint(j,:);
        for s = 1:n_samples
            a = s / (n_samples + 1);
            ps = (1-a)*p1 + a*p2;
            [in_zone, zt, zi] = NavUtils.isInsideAnyMapZone(ps(:), map);
            if in_zone
                hit = true;
                zone_type = zt;
                zone_idx = zi;
                return;
            end
        end
    end
end

function [half_diag, area] = getShipHitboxGeometry(cfg)
    L = cfg.length_m + 2*cfg.safety_margin_m;
    B = cfg.beam_m + 2*cfg.safety_margin_m;
    half_diag = 0.5 * sqrt(L^2 + B^2);
    area = L * B;
end

function ang = blendAngles(a0, a1, w)
    w = max(0, min(1, w));
    da = atan2(sin(a1 - a0), cos(a1 - a0));
    ang = a0 + w * da;
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
