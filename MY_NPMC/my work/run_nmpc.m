%% run_nmpc.m
% NMPC harbor navigation with 9-state stern-azipod + bow-thruster container ship model
%
% UNIFIED TEST — Single configurable test with:
%   - Multi-waypoint path following
%   - Optional static obstacles
%   - Red-zone map obstacle awareness
%   - PID fallback for robustness
%
% Dependencies:
%   container.m             - 9-state ship dynamics (Son & Nomoto + twin stern azipods + bow thruster)
%   NMPC_Container_final.m   - CasADi NMPC solver (see below)
%   NavUtils.m              - Navigation utilities (see below)
%   animateSimResult.m      - Post-simulation animation
%   HarborAnimHelper.m      - Harbor animation helper
%
% Author: Riccardo Legnini 
% Date:   2026-02 to 2026-06

clear; close all; clc;
clear animateSimResult   
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC HARBOR NAVIGATION — Unified Test (9-State Azipod + Bow Thruster)\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%  USER CONFIGURATION — EDIT THIS SECTION

% ---- WAYPOINTS (rows = [x, y] in meters, NED frame) ----
waypoints = [-3000 -2600; -2600 -2700; -2400, -2500; -2350 -2200; -2350, -2050; -2500, -1850];

% ---- CRUISE SPEED TARGET (m/s) ----
% Guidance now uses one global cruise speed and lets the speed governor +
% NMPC handle local slowdowns near obstacles/final approach.
cruise_speed_mps = 5;

% ---- STATIC OBSTACLES (set to empty [] for none) ----
% Each obstacle: struct with 'position' [x; y] and 'radius' [m]
static_obstacles = [];

% Manual dynamic obstacle definition (required when enabled):
% - Positions are rows [x y] in meters.
% - Headings are in degrees (0=+x/North, 90=+y/East).
% - Speeds are in m/s (scalar or one per obstacle).
dynamic_obs_positions_xy = [-2400, -2500];  % Example: [-3000 -1700; -2920 -1740]
dynamic_obs_headings_deg = [270];           % Example: [90; 110]
dynamic_obs_speeds_mps   = [3];             % [] uses dynamic_obs_speed_mps for all

% ---- SIMULATION PARAMETERS ----
T_final     = 1000;      % Total simulation time [s]
R_accept    = 90;       % Intermediate waypoint acceptance radius [m] (keeps waypoint switching honest)
R_accept_final = 25;    % Final waypoint acceptance radius [m]
R_accept_final_soft = 65;      % Soft terminal capture radius [m] (used with low-speed hold)
final_capture_speed_mps = 2.5; % Max speed for soft terminal capture [m/s]
final_capture_hold_s = 6;      % Time inside soft capture gate before declaring success [s]
n1_cruise   = 100;      % Port stern azipod cruise speed [rpm]
n2_cruise   = 100;      % Starboard stern azipod cruise speed [rpm]
n3_cruise   = 0;        % Bow tunnel thruster (off during cruise)

% ---- HULL FOOTPRINT MODEL (oriented rectangle) ----
% Full-ship collision model uses a yawed rectangle centered at [x,y].
% Real ship: 175m L × 25.4m B (from container.m).
% Active hitbox is exactly 50% of real dimensions.
hull_nominal_length_m = 175;
hull_nominal_beam_m   = 25.4;
hull_scale            = 0.5;    % Active hitbox = 87.5m x 12.7m

% ---- ANIMATION RECORDING ----
enable_animation_recording = true;
record_fps = 30;
record_output_dir = 'MY_NPMC\my work\plots in development process\recordings';
log_output_dir = 'MY_NPMC\my work\plots in development process\logs';

% ---- TERMINAL LOG RECORDING ----
% Saves Command Window/terminal prints to a timestamped .txt file.
enable_terminal_log_recording = true;
terminal_log_output_dir = log_output_dir;

% ---- TERMINAL CONSTRAINT RELAXATION ----
% These only apply near the final waypoint to avoid local minima/circling.
enable_terminal_map_relax = true;
terminal_relax_dist_m = 420;
terminal_relax_activation_dist_m = 220;  % activate map-relax only when actually near final corridor
terminal_map_exclusion_m = 110;
terminal_min_avoid_scale = 0.90;
terminal_disable_map_samples = false; % keep map samples in terminal corridor to preserve map-awareness

% ---- TERMINAL PRECISION MODE ----
% Keep strong forward-bias during normal transit, but allow near-stop
% recapture and tight turning on final waypoint approach.
enable_terminal_precision_mode = true;
terminal_precision_dist_m = 160;              % Activate precision behavior only close to terminal [m]
terminal_precision_u_min_forward_mps = 0.35;  % Keep enough forward authority during recapture [m/s]
terminal_precision_guidance_min_mps = 0.35;   % Guidance speed floor during terminal precision mode [m/s]
terminal_precision_guidance_max_mps = 1.4;    % Guidance cap during terminal precision mode [m/s]
terminal_precision_close_dist_m = 20;         % Extra-slow zone for tight final recapture [m]
terminal_precision_close_speed_mps = 0.50;    % Max speed in extra-slow zone [m/s]
final_leg_cruise_floor_mps = 1.3;             % keep sufficient forward pace before terminal precision mode
terminal_heading_hold_radius_m = 12;          % Freeze noisy dp-heading when very close to target [m]
terminal_heading_velocity_switch_mps = 0.35;  % If moving enough, align with velocity heading [m/s]

% Optional map-aware terminal stop capture:
% If final waypoint sits close to map keep-out, allow mission success inside
% a safe radius around the final point at low speed.
enable_safe_terminal_stop = true;
safe_terminal_radius_m = 50;
safe_terminal_speed_mps = 1.2;
safe_terminal_hold_s = 5;

% ---- MAP OBSTACLE SAMPLING ----
enable_map_obstacles = true;   % Set false to disable red-zone awareness
max_map_obstacles    = 3;     % Max map sample points as obstacles (runtime-safe)
map_sample_radius_m  = 13;     % Virtual obstacle radius for map points
map_edge_spacing_m = 80; % Spacing for sampling map polygon edges (larger = fewer virtual obstacles)
map_include_interior_samples = false; % Whether to fill large polygon interiors with random samples (increases obstacle count, can improve avoidance in wide zones)
map_interior_spacing_m = 120; % Spacing for interior map samples; Larger = fewer obstacles, less realism, lower computation.

% Balanced guidance/avoidance coupling (global, not map-specific)
map_lookahead_time_s = 75;     % Forward preview time [s]
map_lookahead_min_m  = 420;    % Clamp lower bound for lookahead [m]
map_lookahead_max_m  = 900;    % Clamp upper bound for lookahead [m]
map_half_width_min_m = 220;    % Clamp lower bound for corridor width [m]
map_half_width_max_m = 420;    % Clamp upper bound for corridor width [m]

% ---- DYNAMIC OBSTACLES (forward motion, no turning) ----
enable_dynamic_obstacles = false;      % Master switch for moving obstacles
dynamic_obs_speed_mps    = 10;       % Constant speed [m/s]
dynamic_obs_radius_m     = 25;        % Circular obstacle radius [m]
dynamic_obs_boundary_policy = 'deactivate'; % deactivate | clip | wrap
dynamic_obs_boundary_margin = 120;    % Margin around map bounds [m]
enable_dynamic_replay_check = true;   % Determinism self-check

% Dynamic obstacle motion trigger mode:
% - 'immediate': obstacles move from t=0
% - 'proximity': obstacles stay stationary until ship is close
dynamic_obs_start_mode = 'proximity';       % immediate | proximity
dynamic_obs_trigger_distance_m = 420;       % scalar or one per obstacle (earlier activation)

% Latent awareness for proximity-triggered dynamic obstacles:
% Adds a lightweight predicted occupancy tube so NMPC can anticipate
% obstacle motion before and during activation.
dynamic_latent_awareness = struct();
dynamic_latent_awareness.enabled = true;
dynamic_latent_awareness.horizon_s = 18;
dynamic_latent_awareness.n_samples = 2;
dynamic_latent_awareness.awareness_distance_m = 700;
dynamic_latent_awareness.radius_scale = 1.15;
dynamic_latent_awareness.only_when_not_moving = false;

% ---- SPEED GOVERNOR (reference speed capping) ----
% Combines:
%   1) Distance-based obstacle speed cap (all obstacle types)
%   2) TCPA/DCPA risk cap (dynamic obstacles)
% Final commanded U_d is min(base guidance speed, governor caps).
speed_governor = struct();
speed_governor.enabled = true;
speed_governor.cruise_speed_mps = cruise_speed_mps;
speed_governor.min_speed_mps = 0.5;            % Keep >= NMPC lower bound (0.1 m/s)
speed_governor.dist_trigger_m = 100;           % Begin slowing when clearance below this (more responsive)
speed_governor.dist_stop_m = 30;               % Strong slowdown close to obstacle (earlier & deeper)
speed_governor.clearance_buffer_m = 15;        % Extra standoff added to obstacle radii (more conservative)
speed_governor.use_map_samples_for_dist_cap = false; % Avoid map-point-induced crawl; NMPC still sees map obstacles
speed_governor.tcpa_horizon_s = 60;            % Lookahead for collision-time risk (longer preview)
speed_governor.dcpa_trigger_m = 130;           % DCPA threshold for slowdown (earlier trigger)
speed_governor.tcpa_risk_gain = 1.60;          % >1 to counter strong speed-tracking weight

% ---- NMPC TUNING ----
nmpc_N  = 35;           % Prediction horizon steps (runtime/accuracy compromise)
nmpc_dt = 1.0;          % Sample time [s]
r_safety = 40;          % Safety margin around obstacles [m]

% === BALANCED TUNING (default for open water & cluttered zones) ===
Q_weights_balanced = diag([2.0, 0.18, 0.95, 8, 8, 4.4, 0.001, 0.001, 0.001]);
R_weights_balanced = diag([0.27, 0.27, 0.010, 0.010, 0.080]);
R_rate_weights_balanced = diag([0.300, 0.300, 0.012, 0.012, 0.080]);

% === PHASE-B (precision berth) tightening ===
Q_weights_berth = diag([2.0, 0.20, 1.35, 9.0, 9.0, 8.4, 0.001, 0.001, 0.001]);
R_weights_berth = diag([0.30, 0.30, 0.012, 0.012, 0.100]);
R_rate_weights_berth = diag([0.360, 0.360, 0.015, 0.015, 0.090]);

% === AGGRESSIVE TUNING (activated in tight corridor mode) ===
Q_weights_aggressive = diag([2.0, 0.22, 1.10, 5.7, 5.7, 4.9, 0.001, 0.001, 0.001]);
R_weights_aggressive = diag([0.135, 0.135, 0.005, 0.005, 0.070]);
R_rate_weights_aggressive = diag([0.105, 0.105, 0.003, 0.003, 0.030]);

% Start with balanced tuning (will switch dynamically)
Q_weights = Q_weights_balanced;
R_weights = R_weights_balanced;
R_rate_weights = R_rate_weights_balanced;

% ---- EXPLICIT TRANSIT/PRECISION PHASE SWITCHING ----
phase_switch_cfg = struct();
phase_switch_cfg.enabled = true;
phase_switch_cfg.T_hor_s = nmpc_N * nmpc_dt;
phase_switch_cfg.u_max_mps = cruise_speed_mps;
phase_switch_cfg.v_max_mps = 0.9;
phase_switch_cfg.hysteresis_m = 20;
phase_switch_cfg.R_s_m = phase_switch_cfg.T_hor_s * ...
    sqrt(phase_switch_cfg.u_max_mps^2 + phase_switch_cfg.v_max_mps^2);
phase_switch_cfg.prefinal_entry_radius_m = max(1.2 * R_accept, 90);

% Phase-specific map sampling load and collision strictness.
max_map_obstacles_transit = min(max_map_obstacles, 2);
max_map_obstacles_berth = max(max_map_obstacles, 6);
r_safety_transit = 0.70 * r_safety;
r_safety_berth = r_safety;
hull_clearance_scale_transit = 0.75;
hull_clearance_scale_berth = 1.00;

% Selective soft obstacle constraints (Phase B only or fail-streak trigger).
soft_obstacle_cfg = struct();
soft_obstacle_cfg.enabled = true;
soft_obstacle_cfg.activation_dist_m = 0.85 * phase_switch_cfg.R_s_m;
soft_obstacle_cfg.fail_streak_trigger = 2;
soft_obstacle_cfg.max_slack_m = 10.0;
soft_obstacle_cfg.penalty_weight = 2.5e5;

% Phase-B minimum surge floor to avoid ultra-slow dithering near berth.
phase_berth_u_min_floor_mps = 0.30;

% ---- TIGHT CORRIDOR MODE (auto-detection) ----
enable_tight_corridor_mode = false;     % Keep disabled by default; fixed-build solver cannot retune weights online safely
tight_corridor_threshold_m = 60;        % Trigger when available maneuver space < this [m]
tight_corridor_hysteresis_m = 80;       % Hysteresis to prevent oscillation [m]
tight_corridor_density_radius_m = 220;  % Radius to count nearby obstacles for crowding trigger [m]
tight_corridor_min_obs_count = 4;       % Trigger when >= this many nearby obstacles create constrained passage
tight_corridor_mode_active = false;     % Runtime flag (updated in simulation loop)
tight_corridor_speed_cap_mps = 3.6;     % Global speed cap in constrained passages
tight_corridor_avoid_scale = 0.78;      % Reduce reference keep-out inflation in narrow channels
tight_corridor_r_ref_max = 0.18;        % Allow stronger heading correction when corridor is tight

% NEW: Actuator and forward motion penalties (ADDRESSING BACKWARD MOTION ISSUE)
actuator_force_weight = 0.015;       % Penalty on RPM magnitude (kept lower to preserve recoverability)
forward_incentive_weight = 3.0;      % Reduced forward-motion bias to improve turning authority during recapture
waypoint_heading_weight = 0.0;       % Extra heading penalty at waypoints (0=disabled)
u_min_forward = 0.5;                % Minimum forward speed [m/s] (reduced to avoid over-constraining turns)

% ---- BRAKING CONSTRAINT (fuel cost optimization) ----
% Limits the rate of deceleration (surge acceleration) to minimize braking fuel costs.
% In real operations, rapid deceleration followed by re-acceleration burns excessive fuel.
% This constraint enforces a maximum deceleration rate (soft limit on negative surge acceleration).
max_brake_rate = 0.4;                % Max deceleration rate [m/s^2] (slightly looser for turn-in)
u_min_forward_nav = u_min_forward;   % Keep nominal forward-only behavior in normal navigation

terminal_precision_cfg = struct();
terminal_precision_cfg.enabled = enable_terminal_precision_mode;
terminal_precision_cfg.activation_dist_m = terminal_precision_dist_m;
terminal_precision_cfg.standard_min_speed_mps = 0.8;
terminal_precision_cfg.terminal_min_speed_mps = terminal_precision_guidance_min_mps;
terminal_precision_cfg.terminal_max_speed_mps = terminal_precision_guidance_max_mps;
terminal_precision_cfg.close_dist_m = terminal_precision_close_dist_m;
terminal_precision_cfg.close_speed_mps = terminal_precision_close_speed_mps;
terminal_precision_cfg.heading_hold_radius_m = terminal_heading_hold_radius_m;
terminal_precision_cfg.velocity_heading_switch_mps = terminal_heading_velocity_switch_mps;
terminal_precision_cfg.direct_target_mode = true;

% Obstacle-aware reference shaping (inertia/lookahead balance)
avoid_ref_cfg = struct();
avoid_ref_cfg.base_margin_m   = 35;    % baseline lateral keep-out in reference shaping
avoid_ref_cfg.speed_gain_s    = 1   ;   % extra margin = speed_gain * U_d
avoid_ref_cfg.obs_radius_gain = 0.45;  % how much obstacle radius increases lateral deflection
avoid_ref_cfg.deflect_sigma   = 0.24;  % larger -> smoother local avoidance bend
avoid_ref_cfg.r_ref_max       = 0.14;  % max |r_ref| sent to NMPC [rad/s]
avoid_ref_cfg.include_map_samples = false; % avoid map-point-induced global drift in reference shaping
avoid_ref_cfg.include_dynamic_obstacles = false; % keep dynamic obstacles in hard NMPC constraints, not in deflected reference

% Cross-track recovery policy (general, map/obstacle agnostic)
% When |XTE| grows, reduce avoidance inflation + speed and allow more
% yaw-rate authority in the reference so the vessel recenters faster.
xte_recovery_cfg = struct();
xte_recovery_cfg.enabled = true;
xte_recovery_cfg.trigger_m = 30;                 % start recovery shaping earlier when off-track
xte_recovery_cfg.full_m = 70;                    % full recovery gain at/above this |XTE|
xte_recovery_cfg.min_avoid_scale = 0.50;         % minimum scale on avoid_ref base margin
xte_recovery_cfg.min_speed_gain_scale = 0.50;    % minimum scale on avoid_ref speed gain
xte_recovery_cfg.recapture_r_ref_max = 0.20;     % stronger heading-rate authority during recapture
xte_recovery_cfg.speed_cap_full_mps = cruise_speed_mps; % do not slow the vessel during recapture
xte_recovery_cfg.force_wp_mode_enabled = true;   % keep enabled to avoid long off-segment drift after bypass
xte_recovery_cfg.force_wp_xte_m = 75;            % activate emergency recapture earlier on large cross-track error
xte_recovery_cfg.force_wp_clearance_m = 75;      % require only moderate local clearance before recapture
xte_recovery_cfg.force_wp_speed_cap_mps = 3.5;   % moderate speed during recapture to prioritize heading convergence
xte_recovery_cfg.force_wp_avoid_scale = 1.05;    % preserve/strengthen obstacle-avoidance inflation in recapture
xte_recovery_cfg.force_wp_r_ref_max = 0.22;      % strong yaw-rate authority for segment rejoin
xte_recovery_cfg.force_wp_release_m = 22;        % hold emergency recapture until |XTE| is back inside this band
xte_recovery_cfg.force_wp_disable_with_dynamic = true; % keep dynamic-vessel guard active
xte_recovery_cfg.force_wp_dynamic_clearance_m = 130;   % allow recapture once dynamic obstacle has moderate separation
xte_recovery_cfg.force_wp_segment_rejoin = true; % steer to segment rejoin point before chasing waypoint
xte_recovery_cfg.force_wp_rejoin_lookahead_m = 18;
xte_recovery_cfg.force_wp_rejoin_xte_gain = 0.20;
xte_recovery_cfg.force_wp_wp_blend_min = 0.10;
xte_recovery_cfg.force_wp_wp_blend_max = 0.32;
xte_recovery_cfg.force_wp_target_blend = 0.82;

% Final-approach watchdog: detect sustained loss of progress toward final
% waypoint and force a temporary direct-to-goal recovery mode.
missed_approach_cfg = struct();
missed_approach_cfg.enabled = true;
missed_approach_cfg.monitor_dist_m = 280;          % only monitor when reasonably close to final
missed_approach_cfg.increase_tol_m = 0.6;          % meaningful step-wise increase threshold
missed_approach_cfg.decrease_tol_m = 0.6;          % meaningful progress threshold
missed_approach_cfg.trigger_steps = 8;             % sustained regressions before latching recovery
missed_approach_cfg.release_steps = 6;             % sustained improvements before releasing recovery
missed_approach_cfg.speed_cap_mps = 2.0;           % keep recovery controllable
missed_approach_cfg.speed_floor_mps = 0.9;         % prevent stall during recovery
missed_approach_cfg.avoid_scale = 0.72;            % lighten ref-inflation while re-acquiring goal line
missed_approach_cfg.recapture_r_ref_max = 0.30;    % increase yaw authority in recovery

% Dynamic-threat safety override (active moving obstacles)
dynamic_threat_cfg = struct();
dynamic_threat_cfg.enabled = true;
dynamic_threat_cfg.clearance_m = 300;         % trigger when ship-to-dynamic clearance below this
dynamic_threat_cfg.speed_cap_mps = 1.2;       % cap surge command under dynamic threat
dynamic_threat_cfg.avoid_margin_scale = 1.15; % inflate ref keep-out under dynamic threat

% ---- PID FALLBACK GAINS (used when NMPC fails) ----
pid_Kp = 0.8;
pid_Ki = 0.01;
pid_Kd = 5.0;

% ---- REAL-TIME DIAGNOSTICS ----
enable_rt_terminal_plots = true;
enable_final_leg_debug = false;  % Set true for per-step final-leg debug prints
env_final_dbg = getenv('NMPC_FINAL_LEG_DEBUG');
if ~isempty(env_final_dbg)
    enable_final_leg_debug = strcmpi(strtrim(env_final_dbg), '1') || strcmpi(strtrim(env_final_dbg), 'true');
end

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
env_tight = getenv('NMPC_ENABLE_TIGHT_CORRIDOR_MODE');
if ~isempty(env_tight)
    enable_tight_corridor_mode = strcmpi(strtrim(env_tight), '1') || strcmpi(strtrim(env_tight), 'true');
end

%  INITIALIZATION (DO NOT EDIT BELOW UNLESS DEBUGGING)

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(scriptDir));

% Prefer the local model/solver folder so the new 9-state container
% implementation is used even when another container.m exists elsewhere.
if isempty(strfind(path, scriptDir))
    addpath(scriptDir, '-begin');
end

% Resolve relative output folders against repo root so logging is robust
% even if other functions change the current working directory.
if isempty(regexp(record_output_dir, '^[A-Za-z]:[\\/]|^\\\\', 'once'))
    record_output_dir = fullfile(repoRoot, record_output_dir);
end
if isempty(regexp(terminal_log_output_dir, '^[A-Za-z]:[\\/]|^\\\\', 'once'))
    terminal_log_output_dir = fullfile(repoRoot, terminal_log_output_dir);
end

run_timestamp = datestr(now, 'yyyymmdd_HHMMSS');
terminal_log_file = ''; %#ok<NASGU>
if enable_terminal_log_recording
    if ~exist(terminal_log_output_dir, 'dir')
        mkdir(terminal_log_output_dir);
    end
    terminal_log_file = fullfile(terminal_log_output_dir, sprintf('nmpc_run_%s_log.txt', run_timestamp));
    diary off;
    diary(terminal_log_file);
    terminal_log_cleanup = onCleanup(@() diary('off'));
    fprintf('  Terminal log recording to: %s\n', terminal_log_file);
end

%% ===== Sanity check on container.m ======================================
x_test = [7; 0; 0; 0; 0; 0; 100; 100; 0];
u_test = [0; 0; 100; 100; 0];
[xdot_test, U_test] = container(x_test, u_test);
fprintf('  container.m check: u_dot=%.4f, r_dot=%.6f, U=%.2f m/s\n\n', ...
    xdot_test(1), xdot_test(3), U_test);

%% ===== Ship image path (for animation) ==================================
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
final_is_map_constrained = false;
if enable_map_obstacles && ~isempty(map)
    map_sample_pts = buildMapSamplePoints(map, map_edge_spacing_m, ...
        map_include_interior_samples, map_interior_spacing_m);
    fprintf('  Map sample points: %d\n', size(map_sample_pts, 1));

    d_final_map = min(sqrt(sum((map_sample_pts - waypoints(end,:)).^2, 2)));
    nominal_keepout = r_safety + map_sample_radius_m + avoid_ref_cfg.base_margin_m;
    fprintf('  Final waypoint -> nearest map sample: %.1f m (nominal keepout %.1f m)\n', ...
        d_final_map, nominal_keepout);
    if d_final_map < nominal_keepout
        final_is_map_constrained = true;
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
if enable_map_obstacles && ~isempty(map_sample_pts)
    max_obs_slots_transit = n_static_obs + max_map_obstacles_transit + n_dynamic_obs + n_dynamic_obs_virtual;
    max_obs_slots_berth = n_static_obs + max_map_obstacles_berth + n_dynamic_obs + n_dynamic_obs_virtual;
else
    max_obs_slots_transit = n_static_obs + n_dynamic_obs + n_dynamic_obs_virtual;
    max_obs_slots_berth = n_static_obs + n_dynamic_obs + n_dynamic_obs_virtual;
end

nmpc_cfg_transit = struct();
nmpc_cfg_transit.N  = nmpc_N;
nmpc_cfg_transit.dt = nmpc_dt;
nmpc_cfg_transit.Q  = Q_weights;
nmpc_cfg_transit.R  = R_weights;
nmpc_cfg_transit.R_rate = R_rate_weights;
nmpc_cfg_transit.max_obs = max(1, max_obs_slots_transit);
nmpc_cfg_transit.r_safety = r_safety_transit;
nmpc_cfg_transit.collision_model = 'point';
nmpc_cfg_transit.hull_length_m = hull_cfg.length_m;
nmpc_cfg_transit.hull_beam_m = hull_cfg.beam_m;
nmpc_cfg_transit.hull_clearance_m = hull_cfg.nmpc_clearance_m * hull_clearance_scale_transit;
nmpc_cfg_transit.enable_diagnostics = false;
nmpc_cfg_transit.actuator_force_weight = actuator_force_weight;
nmpc_cfg_transit.forward_incentive_weight = forward_incentive_weight;
nmpc_cfg_transit.waypoint_heading_weight = waypoint_heading_weight;
nmpc_cfg_transit.u_min_forward = u_min_forward;
nmpc_cfg_transit.max_brake_rate = max_brake_rate;
nmpc_cfg_transit.soft_obs_weight = soft_obstacle_cfg.penalty_weight;
nmpc_cfg_transit.soft_obs_default_max_m = 0.0;

nmpc_cfg_berth = nmpc_cfg_transit;
nmpc_cfg_berth.Q = Q_weights_berth;
nmpc_cfg_berth.R = R_weights_berth;
nmpc_cfg_berth.R_rate = R_rate_weights_berth;
nmpc_cfg_berth.max_obs = max(1, max_obs_slots_berth);
nmpc_cfg_berth.r_safety = r_safety_berth;
nmpc_cfg_berth.collision_model = 'oriented-rectangle';
nmpc_cfg_berth.hull_clearance_m = hull_cfg.nmpc_clearance_m * hull_clearance_scale_berth;
nmpc_cfg_berth.soft_obs_weight = soft_obstacle_cfg.penalty_weight;
nmpc_cfg_berth.soft_obs_default_max_m = 0.0;

fprintf('\n--- Building NMPC solver (Phase A transit, %d obstacle slots) ---\n', ...
    nmpc_cfg_transit.max_obs);
fprintf('  Transit collision model: point, r_safety=%.1f m\n', nmpc_cfg_transit.r_safety);
nmpc_transit = NMPC_Container_final(nmpc_cfg_transit);
nmpc_transit.buildSolver();

fprintf('\n--- Building NMPC solver (Phase B precision-berth, %d obstacle slots) ---\n', ...
    nmpc_cfg_berth.max_obs);
fprintf('  Berth collision model: oriented rectangle %.1f m x %.1f m (clearance %.1f m)\n', ...
    hull_cfg.length_m, hull_cfg.beam_m, nmpc_cfg_berth.hull_clearance_m);
nmpc_berth = NMPC_Container_final(nmpc_cfg_berth);
nmpc_berth.buildSolver();

fprintf('\n  Phase switching radius: R_s = T_hor * sqrt(u_max^2 + v_max^2) = %.1f m\n', ...
    phase_switch_cfg.R_s_m);
fprintf('    (T_hor=%.1f s, u_max=%.2f m/s, v_max=%.2f m/s)\n', ...
    phase_switch_cfg.T_hor_s, phase_switch_cfg.u_max_mps, phase_switch_cfg.v_max_mps);

%% ===== Initial state ====================================================
x0_heading = atan2(waypoints(2,2) - waypoints(1,2), ...
                   waypoints(2,1) - waypoints(1,1));
% State: [u, v, r, x, y, psi, n1, n2, n3]
x = [7; 0; 0; waypoints(1,1); waypoints(1,2); x0_heading; n1_cruise; n2_cruise; n3_cruise];

%% ===== Simulation setup =================================================
dt = nmpc_cfg_transit.dt;
t  = 0:dt:T_final;
wp_idx = 1;
final_capture_count = 0;
final_capture_steps_needed = max(1, ceil(final_capture_hold_s / dt));
safe_terminal_count = 0;
safe_terminal_steps_needed = max(1, ceil(safe_terminal_hold_s / dt));
terminal_mode_announced = false;

% Preallocate logging
traj     = zeros(9, length(t)+1);
ctrl     = zeros(5, length(t));
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
brake_margin_log  = nan(1, length(t));
phase_mode_log    = zeros(1, length(t));
soft_slack_max_log = nan(1, length(t));
soft_slack_sum_log = nan(1, length(t));
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

% Previous control for NMPC rate limiting (NEW!)
u_prev = [0; 0; n1_cruise; n2_cruise; n3_cruise];

% Tight corridor mode state tracking
loose_prev_was_tight = false;
terminal_precision_was_active = false;
force_wp_recap_prev = false;
force_wp_recap_latched = false;
phase_mode_is_berth = false;
phase_mode_prev = false;
nmpc_fail_streak = 0;

% Missed-approach detector state (script-scope, persists across loop iterations)
d_final_prev = inf;
d_final_increasing_count = 0;
missed_approach_recovering = false;
d_final_decreasing_count = 0;
u_prev_ship = x(1);

fprintf('\n  Waypoints: ');
for i = 1:size(waypoints, 1)
    fprintf('(%d, %d) ', waypoints(i,1), waypoints(i,2));
end

%  MAIN SIMULATION LOOP

% Main simulation loop
for i = 1:length(t)
    t_step = tic;

    % ---- 1) Waypoint guidance -------------------------------------------
    t_seg = tic;
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, waypoints, cruise_speed_mps, wp_idx, R_accept, terminal_precision_cfg);
    xte = computeXTE(x, waypoints, wp_idx);
    d_final_now = norm(x(4:5) - waypoints(end,:)');
    on_final_leg = wp_idx >= max(1, size(waypoints,1)-1);
    d_prefinal_gate = inf;
    if on_final_leg
        pre_idx = max(1, size(waypoints,1)-1);
        d_prefinal_gate = norm(x(4:5) - waypoints(pre_idx,:)');
    end

    % Explicit maneuver-phase switching:
    % Keep Phase A behavior for all non-final-waypoint legs.
    % Enter Phase B only when final waypoint is the active target and close.
    n_wps = size(waypoints, 1);
    on_final_waypoint = wp_idx >= n_wps;
    phase_switch_final_dist_m = min(phase_switch_cfg.R_s_m, terminal_precision_cfg.activation_dist_m);
    if phase_switch_cfg.enabled
        phase_mode_is_berth = on_final_waypoint && (d_final_now <= phase_switch_final_dist_m);
    else
        phase_mode_is_berth = terminal_precision_cfg.enabled && on_final_waypoint && ...
            (d_final_now < terminal_precision_cfg.activation_dist_m);
    end

    if phase_mode_is_berth && ~phase_mode_prev
        fprintf('  [phase-switch] Phase B (final-waypoint precision) ON at d_final=%.1f m (R_sw=%.1f m)\n', ...
            d_final_now, phase_switch_final_dist_m);
    elseif ~phase_mode_is_berth && phase_mode_prev
        fprintf('  [phase-switch] Phase A (transit) ON at d_final=%.1f m\n', d_final_now);
    end
    phase_mode_prev = phase_mode_is_berth;

    terminal_precision_mode_active = terminal_precision_cfg.enabled && on_final_waypoint && ...
        phase_mode_is_berth;

    if terminal_precision_mode_active && ~terminal_precision_was_active
        fprintf('  [terminal-precision ON] relaxing min forward speed for tight final recapture (d_final=%.1f m)\n', d_final_now);
    elseif ~terminal_precision_mode_active && terminal_precision_was_active
        fprintf('  [terminal-precision OFF] restoring nominal forward-speed floor\n');
    end
    terminal_precision_was_active = terminal_precision_mode_active;
    guide_time_log(i) = toc(t_seg);

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
    obs_map = struct('position', {}, 'radius', {});
    
    if enable_map_obstacles && ~isempty(map_sample_pts)
        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead_now = min(map_lookahead_max_m, max(map_lookahead_min_m, U_now * map_lookahead_time_s));
        half_width_now = min(map_half_width_max_m, max(map_half_width_min_m, 0.45 * lookahead_now));

        on_final_leg = wp_idx >= max(1, size(waypoints,1)-1);
        terminal_mode_active = enable_terminal_map_relax && on_final_leg && (d_final_now < terminal_relax_activation_dist_m);
        if terminal_mode_active && ~terminal_mode_announced
            fprintf('  [terminal-mode] final-leg map-sample relaxation active (d_final=%.1f m)\n', d_final_now);
            terminal_mode_announced = true;
        end

        % Near the terminal waypoint, reduce virtual-map corridor aggressiveness.
        if terminal_mode_active
            relax_ratio = max(0.45, d_final_now / max(terminal_relax_dist_m, 1));
            lookahead_now = max(120, relax_ratio * lookahead_now);
            half_width_now = max(80, relax_ratio * half_width_now);
        end

        if phase_mode_is_berth
            max_map_obs_step = max_map_obstacles_berth;
        else
            max_map_obs_step = max_map_obstacles_transit;
        end

        obs_map = selectMapObstaclesFromSamples( ...
            map_sample_pts, x(4:5), chi_d, max_map_obs_step, ...
            lookahead_now, half_width_now, map_sample_radius_m);

        % Keep map sample obstacles during terminal approach to preserve
        % corridor awareness in constrained final segments.

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
    end
    
    obs_dyn = struct('position', {}, 'radius', {});
    obs_dyn_latent = struct('position', {}, 'radius', {});
    dyn_threat_active = false;
    if enable_dynamic_obstacles && ~isempty(dynamic_obstacles)
        obs_dyn = dynamicToCircleObstacles(dynamic_obstacles);
        if isfield(dynamic_latent_awareness, 'enabled') && dynamic_latent_awareness.enabled
            obs_dyn_latent = buildLatentDynamicAwarenessObstacles(dynamic_obstacles, x(4:5), dynamic_latent_awareness);
        end
        obs_local = [obs_local, obs_dyn_latent, obs_dyn];
        obs_pack_drift_log(i) = computeDynamicPackagingDrift(dynamic_obstacles, obs_local);

        if dynamic_threat_cfg.enabled && ~isempty(obs_dyn)
            min_dyn_clear = inf;
            for kk = 1:length(obs_dyn)
                d_cent = norm(x(4:5) - obs_dyn(kk).position(1:2));
                d_clear = d_cent - obs_dyn(kk).radius;
                min_dyn_clear = min(min_dyn_clear, d_clear);
            end
            dyn_threat_active = min_dyn_clear < dynamic_threat_cfg.clearance_m;
        end
    end

    % Speed governor: cap U_d using local obstacle distance risk and
    % dynamic-obstacle TCPA/DCPA closure risk.
    if speed_governor.enabled
        speed_governor_step = speed_governor;
        if terminal_precision_mode_active
            speed_governor_step.min_speed_mps = min(speed_governor_step.min_speed_mps, terminal_precision_cfg.terminal_min_speed_mps);
        end
        if speed_governor.use_map_samples_for_dist_cap
            obs_for_speed = obs_local;
        else
            obs_for_speed = [static_obstacles, obs_dyn_latent, obs_dyn];
        end
        U_d = applySpeedGovernor(U_d, x, obs_for_speed, dynamic_obstacles, speed_governor_step);
    end

    if tight_corridor_mode_active
        U_d = min(U_d, tight_corridor_speed_cap_mps);
    end

    if dyn_threat_active
        U_d = min(U_d, dynamic_threat_cfg.speed_cap_mps);
    end

    if on_final_leg
        U_now_ship = hypot(x(1), x(2));
        nearest_clear = inf;
        for kk = 1:length(obs_local)
            d_cent = norm(x(4:5) - obs_local(kk).position(1:2));
            d_clear = d_cent - obs_local(kk).radius;
            nearest_clear = min(nearest_clear, d_clear);
        end
        
        % ----- FINAL APPROACH SPEED LIMITING (CRITICAL FIX) -----
        % Assume max deceleration ~0.02 m/s┬ for large vessel
        a_decel_max = 0.02;  % m/s┬ - conservative for container ship
        stopping_distance_estimate = (U_now_ship^2) / (2 * a_decel_max) + 50;  % +50m safety
        
        % Hard speed caps based on distance to final waypoint
        if d_final_now < 500
            U_d = min(U_d, 3.0);   % Never exceed 3 m/s within 500m
        end
        if d_final_now < 350
            U_d = min(U_d, 2.0);   % Never exceed 2 m/s within 350m  
        end
        if d_final_now < 200
            U_d = min(U_d, 1.2);   % Never exceed 1.2 m/s within 200m
        end
        if d_final_now < 100
            U_d = min(U_d, 0.8);   % Final creep speed
        end

        % Once the vessel has cleared the immediate obstacle cluster, let the
        % commanded speed climb back up instead of staying stuck at creep speed.
        if ~terminal_precision_mode_active && ~dyn_threat_active && isfinite(nearest_clear)
            if nearest_clear > 180 && U_now_ship < 0.85 * cruise_speed_mps
                recovery_factor = min(1.0, (nearest_clear - 180) / 220);
                U_recover = 0.55 * cruise_speed_mps + 0.35 * U_now_ship + 0.80 * recovery_factor;
                U_d = max(U_d, min(cruise_speed_mps, U_recover));
            end
        end
        
        % Proactive deceleration: if stopping distance > distance to target, SLOW DOWN NOW
        % DISABLED: Stopping distance braking conflicts with NMPC's planned deceleration.
        % The solver is already formulating a trajectory toward the target; external
        % braking overrides this and causes the boat to get stuck in low-speed oscillation.
        % if stopping_distance_estimate > d_final_now * 0.8
        %     U_d_brake = sqrt(2 * a_decel_max * d_final_now * 0.6);
        %     if U_d_brake < U_d
        %         U_d = U_d_brake;
        %     end
        % end
        
        if enable_final_leg_debug
            fprintf('  [FINAL-LEG DEBUG] d_final=%.1f m, U_ship=%.2f m/s, U_d=%.2f m/s, XTE=%.1f m\n', ...
                d_final_now, U_now_ship, U_d, xte);
        end
        
        if missed_approach_cfg.enabled && d_final_now < missed_approach_cfg.monitor_dist_m
            delta_d_final = d_final_now - d_final_prev;
            if isfinite(delta_d_final) && (delta_d_final > missed_approach_cfg.increase_tol_m)
                d_final_increasing_count = d_final_increasing_count + 1;
                d_final_decreasing_count = 0;
            elseif isfinite(delta_d_final) && (delta_d_final < -missed_approach_cfg.decrease_tol_m)
                d_final_decreasing_count = d_final_decreasing_count + 1;
                d_final_increasing_count = max(0, d_final_increasing_count - 1);
            end

            if ~missed_approach_recovering && d_final_increasing_count >= missed_approach_cfg.trigger_steps
                missed_approach_recovering = true;
                fprintf('  [MISSED-APPROACH RECOVERY ON] d_final increased for %d steps (d_final=%.1f m)\n', ...
                    d_final_increasing_count, d_final_now);
            elseif missed_approach_recovering && d_final_decreasing_count >= missed_approach_cfg.release_steps
                missed_approach_recovering = false;
                d_final_increasing_count = 0;
                d_final_decreasing_count = 0;
                fprintf('  [MISSED-APPROACH RECOVERY OFF] final-distance trend recovered (d_final=%.1f m)\n', d_final_now);
            end

            if d_final_increasing_count >= missed_approach_cfg.trigger_steps && ...
                    mod(d_final_increasing_count - missed_approach_cfg.trigger_steps, 10) == 0
                fprintf('  [MISSED APPROACH WARNING] d_final regression streak=%d (d_final=%.1f m)\n', ...
                    d_final_increasing_count, d_final_now);
            end
        else
            d_final_increasing_count = 0;
            d_final_decreasing_count = 0;
            missed_approach_recovering = false;
        end
        d_final_prev = d_final_now;
    
    end  % end if on_final_leg

    if on_final_leg && ~terminal_precision_mode_active && ~dyn_threat_active
        U_d = max(U_d, final_leg_cruise_floor_mps);
    end

    obs_time_log(i) = toc(t_seg);

    % ---- 2.5) TIGHT CORRIDOR MODE DETECTION & WEIGHT SWITCHING -----------
    if enable_tight_corridor_mode && ~isempty(obs_local)
        tight_corridor_mode_active = detectTightCorridor(x, obs_local, hull_cfg.beam_m, ...
            tight_corridor_threshold_m, tight_corridor_hysteresis_m, ...
            tight_corridor_density_radius_m, tight_corridor_min_obs_count, ...
            tight_corridor_mode_active);
    else
        tight_corridor_mode_active = false;
    end

    mode_switched = (tight_corridor_mode_active ~= loose_prev_was_tight);
    if mode_switched
        if tight_corridor_mode_active
            fprintf('  [mode-switch] tight-corridor ON: applying conservative speed + corridor-aware ref shaping\n');
        else
            fprintf('  [mode-switch] tight-corridor OFF: restoring nominal guidance shaping\n');
        end
    end

    desired_u_min_forward = u_min_forward_nav;
    if terminal_precision_mode_active
        desired_u_min_forward = min(desired_u_min_forward, terminal_precision_u_min_forward_mps);
    end
    if phase_mode_is_berth
        desired_u_min_forward = max(phase_berth_u_min_floor_mps, desired_u_min_forward);
    end

    if phase_mode_is_berth
        nmpc_active = nmpc_berth;
    else
        nmpc_active = nmpc_transit;
    end

    loose_prev_was_tight = tight_corridor_mode_active;

    % ---- 3) Build reference trajectory ----------------------------------
    t_seg = tic;
    avoid_ref_step = avoid_ref_cfg;
    if dyn_threat_active
        avoid_ref_step.base_margin_m = dynamic_threat_cfg.avoid_margin_scale * avoid_ref_step.base_margin_m;
    end
    if tight_corridor_mode_active
        avoid_ref_step.base_margin_m = tight_corridor_avoid_scale * avoid_ref_step.base_margin_m;
        avoid_ref_step.speed_gain_s = tight_corridor_avoid_scale * avoid_ref_step.speed_gain_s;
        avoid_ref_step.deflect_sigma = 0.90 * avoid_ref_step.deflect_sigma;
        avoid_ref_step.r_ref_max = max(avoid_ref_step.r_ref_max, tight_corridor_r_ref_max);
    end
    if enable_terminal_map_relax && wp_idx >= max(1, size(waypoints,1)-1) && d_final_now < terminal_relax_activation_dist_m
        avoid_scale = max(terminal_min_avoid_scale, d_final_now / max(terminal_relax_dist_m, 1));
        avoid_ref_step.base_margin_m = avoid_ref_cfg.base_margin_m * avoid_scale;
        avoid_ref_step.speed_gain_s = avoid_ref_cfg.speed_gain_s * avoid_scale;
    end

    % General XTE-recapture shaping to prevent prolonged off-segment drift.
    if xte_recovery_cfg.enabled
        xte_abs = abs(xte);
        if xte_abs > xte_recovery_cfg.trigger_m
            beta = (xte_abs - xte_recovery_cfg.trigger_m) / ...
                max(xte_recovery_cfg.full_m - xte_recovery_cfg.trigger_m, 1e-6);
            beta = max(0, min(1, beta));

            avoid_scale_xte = max(xte_recovery_cfg.min_avoid_scale, ...
                1 - beta * (1 - xte_recovery_cfg.min_avoid_scale));
            speed_gain_scale_xte = max(xte_recovery_cfg.min_speed_gain_scale, ...
                1 - beta * (1 - xte_recovery_cfg.min_speed_gain_scale));

            avoid_ref_step.base_margin_m = avoid_ref_step.base_margin_m * avoid_scale_xte;
            avoid_ref_step.speed_gain_s = avoid_ref_step.speed_gain_s * speed_gain_scale_xte;
            avoid_ref_step.r_ref_max = max(avoid_ref_step.r_ref_max, xte_recovery_cfg.recapture_r_ref_max);

            U_cap_xte = cruise_speed_mps - beta * (cruise_speed_mps - xte_recovery_cfg.speed_cap_full_mps);
            U_d = min(U_d, U_cap_xte);
        end
    end

    % Emergency non-terminal recapture mode:
    % if the vessel drifts far from the active segment and nearest obstacle
    % clearance is already comfortable, steer directly toward the active
    % waypoint and reduce reference-deflection inflation to avoid large loops.
    force_wp_recap = false;
    if getOr(xte_recovery_cfg, 'force_wp_mode_enabled', false) && wp_idx < size(waypoints, 1)
        nearest_clear = inf;
        for kk = 1:length(obs_local)
            d_cent = norm(x(4:5) - obs_local(kk).position(1:2));
            d_clear = d_cent - obs_local(kk).radius;
            nearest_clear = min(nearest_clear, d_clear);
        end

        dyn_blocked = false;
        if getOr(xte_recovery_cfg, 'force_wp_disable_with_dynamic', true)
            min_dyn_clear = inf;
            any_dyn_active = false;
            for kk = 1:length(dynamic_obstacles)
                if ~isfield(dynamic_obstacles(kk), 'enabled') || ~dynamic_obstacles(kk).enabled
                    continue;
                end
                if ~isfield(dynamic_obstacles(kk), 'active') || ~dynamic_obstacles(kk).active
                    continue;
                end
                any_dyn_active = true;
                d_dyn = norm(x(4:5) - dynamic_obstacles(kk).position(1:2)) - dynamic_obstacles(kk).radius;
                min_dyn_clear = min(min_dyn_clear, d_dyn);
            end
            dyn_clear_req = getOr(xte_recovery_cfg, 'force_wp_dynamic_clearance_m', 260);
            dyn_blocked = any_dyn_active && (min_dyn_clear < dyn_clear_req);
        end

        if ~force_wp_recap_latched && ...
                abs(xte) > getOr(xte_recovery_cfg, 'force_wp_xte_m', 120) && ...
                nearest_clear > getOr(xte_recovery_cfg, 'force_wp_clearance_m', 170) && ...
                ~dyn_blocked
            force_wp_recap_latched = true;
        end

        if force_wp_recap_latched
            xte_release = getOr(xte_recovery_cfg, 'force_wp_release_m', 22);
            if (abs(xte) < xte_release) || dyn_blocked
                force_wp_recap_latched = false;
            end
        end

        if force_wp_recap_latched
            chi_base = chi_d;
            p_wp = waypoints(min(wp_idx + 1, size(waypoints, 1)), :)';
            chi_wp = atan2(p_wp(2) - x(5), p_wp(1) - x(4));

            % Prefer rejoining the active segment first, then blend toward
            % the waypoint direction to preserve smooth progress.
            chi_target = chi_wp;
            if getOr(xte_recovery_cfg, 'force_wp_segment_rejoin', true)
                p_from = waypoints(max(1, wp_idx), :)';
                p_to = waypoints(min(wp_idx + 1, size(waypoints, 1)), :)';
                seg = p_to - p_from;
                seg_len = norm(seg);
                if seg_len > 1e-6
                    pos_xy = x(4:5);
                    s_proj = dot(pos_xy - p_from, seg) / seg_len;
                    s_proj = max(0, min(seg_len, s_proj));
                    rejoin_lookahead_m = getOr(xte_recovery_cfg, 'force_wp_rejoin_lookahead_m', 15);
                    rejoin_xte_gain = getOr(xte_recovery_cfg, 'force_wp_rejoin_xte_gain', 0.20);
                    s_target = min(seg_len, s_proj + rejoin_lookahead_m + rejoin_xte_gain * abs(xte));
                    p_rejoin = p_from + (s_target / seg_len) * seg;
                    chi_rejoin = atan2(p_rejoin(2) - pos_xy(2), p_rejoin(1) - pos_xy(1));

                    wp_blend_min = getOr(xte_recovery_cfg, 'force_wp_wp_blend_min', 0.10);
                    wp_blend_max = getOr(xte_recovery_cfg, 'force_wp_wp_blend_max', 0.30);
                    xte_norm = min(1.0, abs(xte) / max(getOr(xte_recovery_cfg, 'force_wp_xte_m', 55), 1e-6));
                    wp_blend = wp_blend_max - (wp_blend_max - wp_blend_min) * xte_norm;

                    chi_target = atan2((1 - wp_blend) * sin(chi_rejoin) + wp_blend * sin(chi_wp), ...
                                       (1 - wp_blend) * cos(chi_rejoin) + wp_blend * cos(chi_wp));
                end
            end

            target_blend = getOr(xte_recovery_cfg, 'force_wp_target_blend', 0.80);
            chi_d = atan2((1 - target_blend) * sin(chi_base) + target_blend * sin(chi_target), ...
                          (1 - target_blend) * cos(chi_base) + target_blend * cos(chi_target));
            U_d = min(U_d, getOr(xte_recovery_cfg, 'force_wp_speed_cap_mps', 2.1));
            avoid_scale_force = getOr(xte_recovery_cfg, 'force_wp_avoid_scale', 0.55);
            avoid_ref_step.base_margin_m = avoid_ref_step.base_margin_m * avoid_scale_force;
            avoid_ref_step.speed_gain_s = avoid_ref_step.speed_gain_s * avoid_scale_force;
            avoid_ref_step.r_ref_max = max(avoid_ref_step.r_ref_max, getOr(xte_recovery_cfg, 'force_wp_r_ref_max', 0.30));
            force_wp_recap = true;
        end
    end

    if force_wp_recap && ~force_wp_recap_prev
        fprintf('  [xte-recapture] emergency waypoint recapture ON (xte=%.1f m)\n', xte);
    elseif ~force_wp_recap && force_wp_recap_prev
        fprintf('  [xte-recapture] emergency waypoint recapture OFF\n');
    end

    % If final-distance keeps worsening near berth, force direct-goal recapture
    % regardless of active segment details until trend recovers.
    if missed_approach_recovering
        wp_idx = max(wp_idx, max(1, size(waypoints, 1) - 1));
        p_goal = waypoints(end, :)';
        chi_d = atan2(p_goal(2) - x(5), p_goal(1) - x(4));
        U_d = min(U_d, missed_approach_cfg.speed_cap_mps);
        U_d = max(U_d, missed_approach_cfg.speed_floor_mps);
        avoid_ref_step.base_margin_m = avoid_ref_step.base_margin_m * missed_approach_cfg.avoid_scale;
        avoid_ref_step.speed_gain_s = avoid_ref_step.speed_gain_s * missed_approach_cfg.avoid_scale;
        avoid_ref_step.r_ref_max = max(avoid_ref_step.r_ref_max, missed_approach_cfg.recapture_r_ref_max);
        force_wp_recap = true;
    end

    force_wp_recap_prev = force_wp_recap;

    obs_for_ref = static_obstacles;
    if getOr(avoid_ref_step, 'include_map_samples', false)
        obs_for_ref = [obs_for_ref, obs_map];
    end
    if getOr(avoid_ref_step, 'include_dynamic_obstacles', false)
        obs_for_ref = [obs_for_ref, obs_dyn_latent, obs_dyn];
    end

    x_ref = buildObstacleAwareRef8(x, chi_d, U_d, nmpc_active.N, dt, ...
                                   n1_cruise, n2_cruise, n3_cruise, obs_for_ref, avoid_ref_step);
    ref_time_log(i) = toc(t_seg);

    % ---- 4) Solve NMPC (MODIFIED - now passes u_prev) -------------------
    t_seg = tic;
    solve_opts = struct();
    enable_soft_here = soft_obstacle_cfg.enabled && phase_mode_is_berth && ...
        ((d_final_now < soft_obstacle_cfg.activation_dist_m) || ...
         (nmpc_fail_streak >= soft_obstacle_cfg.fail_streak_trigger));
    solve_opts.enable_soft_obstacles = enable_soft_here;
    solve_opts.soft_obs_max_m = soft_obstacle_cfg.max_slack_m;
    if terminal_precision_mode_active
        solve_opts.n3_max = nmpc_active.n_bow_max;
    else
        solve_opts.n3_max = 0;
    end

    [u_opt, ~, info] = nmpc_active.solve(x, x_ref, obs_local, u_prev, desired_u_min_forward, solve_opts);
    solve_call_log(i) = toc(t_seg);
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
    if isfield(info, 'max_soft_slack_m')
        soft_slack_max_log(i) = info.max_soft_slack_m;
    end
    if isfield(info, 'sum_soft_slack_m')
        soft_slack_sum_log(i) = info.sum_soft_slack_m;
    end

    % ---- 5) PID fallback if NMPC fails ----------------------------------
    if ~info.success
        nmpc_fail_streak = nmpc_fail_streak + 1;
        psi_err = wrapToPi(chi_d - x(6));
        psi_err_int = psi_err_int + psi_err * dt;
        psi_err_int = max(-1, min(1, psi_err_int));
        psi_err_dot = (psi_err - psi_err_prev) / dt;
        psi_err_prev = psi_err;
        
        alpha = pid_Kp * psi_err + pid_Ki * psi_err_int + pid_Kd * psi_err_dot;
        alpha = max(-pi/4, min(pi/4, alpha));
        
        n1_cmd = n1_cruise * (U_d / max(cruise_speed_mps, 1e-3));
        n1_cmd = max(0, min(160, n1_cmd));
        
        u_opt = [alpha; alpha; n1_cmd; n1_cmd; 0];
        fallback(i) = true;
        
        if sum(fallback(1:i)) <= 3
            fprintf('  ⚠ NMPC fail at t=%.0f s, using PID fallback\n', t(i));
        end
    else
        nmpc_fail_streak = 0;
    end

    % ---- 6) Simulate plant (RK4) ----------------------------------------
    t_seg = tic;
    x = rk4Step9(x, u_opt, dt, desired_u_min_forward);
    integr_time_log(i) = toc(t_seg);
    du_surge = x(1) - u_prev_ship;
    brake_margin_log(i) = du_surge + max_brake_rate * dt;
    u_prev_ship = x(1);
    step_time_log(i) = toc(t_step);
    rt_ratio_log(i) = step_time_log(i) / max(dt, 1e-9);

    % ---- 7) Collision checks (map + obstacle sources) ------------------
    [hit_static, ~, ~] = detectHullCircleHit(x, static_obstacles, hull_cfg, 0.0);
    [hit_map_samples, ~, ~] = detectHullCircleHit(x, obs_map, hull_cfg, 0.0);
    [hit_dyn_latent, ~, ~] = detectHullCircleHit(x, obs_dyn_latent, hull_cfg, 0.0);
    [hit_dyn_real, ~, ~] = detectHullCircleHit(x, obs_dyn, hull_cfg, 0.0);
    hit_obs = hit_static || hit_map_samples || hit_dyn_latent || hit_dyn_real;
    [hit_map, ~, ~] = detectHullMapHit(x, hull_cfg, map);
    if hit_obs || hit_map
        collision_log(i) = true;
        fprintf(['  [COLLISION] t=%.1f s hit_obs=%d hit_map=%d ', ...
                 '(static=%d map_samples=%d dyn_latent=%d dyn_real=%d)\n'], ...
            t(i), hit_obs, hit_map, hit_static, hit_map_samples, hit_dyn_latent, hit_dyn_real);
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
    phase_mode_log(i) = phase_mode_is_berth;
    
    % Update previous control for next iteration (NEW!)
    u_prev = u_opt;

    % ---- 9) Progress print ----------------------------------------------
    if i == 1 || mod(i, 20) == 0 || ~info.success || rt_ratio_log(i) > 1.0
        d_nearest_obs = inf;
        for j = 1:length(obs_local)
            d_nearest_obs = min(d_nearest_obs, norm(x(4:5) - obs_local(j).position));
        end
        phase_char = 'A';
        if phase_mode_is_berth
            phase_char = 'B';
        end
        slack_print = 0.0;
        if isfinite(soft_slack_max_log(i))
            slack_print = soft_slack_max_log(i);
        end
        fprintf(['  [t=%5.1f] phase=%s pos=(%7.1f,%6.1f) psi=%+6.1fdeg wp=%d xte=%+.1fm ', ...
                 'obs_d=%.0fm ok=%d comp=%.1fms solve=%.1fms RT=%.2f obs=%d slack=%.2fm fb=%d\n'], ...
            t(i), phase_char, x(4), x(5), rad2deg(x(6)), wp_idx, xte, d_nearest_obs, info.success, ...
            1e3*step_time_log(i), 1e3*solve_time_log(i), rt_ratio_log(i), ...
            round(n_obs_log(i)), slack_print, fallback(i));
    end

    % ---- 10) Check if done ----------------------------------------------
    d_final_now = norm(x(4:5) - waypoints(end,:)');
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

% Trim logs
if ~exist('steps', 'var') || isempty(steps) || steps < 1
    fprintf('⚠ No simulation steps completed. Skipping output generation.\n');
    if enable_terminal_log_recording
        diary off;
        if exist('terminal_log_cleanup', 'var') && ~isempty(terminal_log_cleanup)
            clear terminal_log_cleanup;
        end
    end
    return;
end

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
brake_margin_log = brake_margin_log(1:steps);
phase_mode_log = phase_mode_log(1:steps);
soft_slack_max_log = soft_slack_max_log(1:steps);
soft_slack_sum_log = soft_slack_sum_log(1:steps);

% Wrap all output generation in try-catch to ensure it runs even if errors occur
output_gen_error = false;
try

n_ok = sum(solve_ok);  n_tot = length(solve_ok);
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  SUMMARY\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  NMPC solves: %d/%d (%.1f%%)\n', n_ok, n_tot, 100*n_ok/max(n_tot,1));
fprintf('  PID fallback: %d times\n', sum(fallback));
fprintf('  Mean |XTE|: %.1f m, Max |XTE|: %.1f m\n', mean(abs(xte_log)), max(abs(xte_log)));
fprintf('  Final position: (%.1f, %.1f)\n', traj(4,end), traj(5,end));
fprintf('  Phase-B occupancy: %.1f%% of steps\n', 100 * mean(phase_mode_log > 0.5));
if any(isfinite(obs_pack_drift_log))
    fprintf('  Dynamic packaging drift [m]: max=%.3f\n', max(obs_pack_drift_log(isfinite(obs_pack_drift_log))));
end
fprintf('  Collisions detected: %d\n', sum(collision_log));
if any(isfinite(soft_slack_max_log))
    max_soft_slack_used = max(soft_slack_max_log(isfinite(soft_slack_max_log)));
    sum_soft_slack_used = sum(soft_slack_sum_log(isfinite(soft_slack_sum_log)));
    fprintf('  Soft obstacle slack [m]: max=%.3f, cumulative=%.3f\n', ...
        max_soft_slack_used, sum_soft_slack_used);
end
if any(isfinite(brake_margin_log))
    n_brake_viol = sum(brake_margin_log < -1e-6);
    fprintf('  Brake-rate margin [m/s]: min=%.4f, violations=%d\n', ...
        min(brake_margin_log(isfinite(brake_margin_log))), n_brake_viol);
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
fprintf('  Worst RT ratio: %.3f\n', max(rt_ratio_log(valid_step)));

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
legend('\alpha_1 (port stern)', '\alpha_2 (starboard stern)', 'Location', 'best');

subplot(3,2,3);
plot(t_ctrl, ctrl(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ctrl, ctrl(4,:), 'r--', 'LineWidth', 1.5);
plot(t_ctrl, ctrl(5,:), 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Commanded rpm');
title('Shaft Speed Commands'); grid on;
legend('n_{1,c}', 'n_{2,c}', 'n_{3,c}', 'Location', 'best');

subplot(3,2,4);
plot(t_sim, traj(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_sim, traj(8,:), 'r--', 'LineWidth', 1.5);
plot(t_sim, traj(9,:), 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Actual rpm');
title('Actual Shaft Speeds'); grid on;
legend('n_1', 'n_2', 'n_3', 'Location', 'best');

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
cfg_anim.flipShipImageVertical = false;  % Visualization-only: set true if icon appears bow/stern inverted

% Recording options (passed to animateSimResult)
cfg_anim.recordVideo = enable_animation_recording;
cfg_anim.recordFps   = record_fps;
record_dir = record_output_dir;
if cfg_anim.recordVideo && ~exist(record_dir, 'dir')
    mkdir(record_dir);
end
cfg_anim.videoFile = fullfile(record_dir, sprintf('nmpc_run_%s.mp4', run_timestamp));

% Add static obstacles to animation
for j = 1:length(static_obstacles)
    cfg_anim.circObs(j).position = static_obstacles(j).position;
    cfg_anim.circObs(j).radius   = static_obstacles(j).radius;
end
if ~isempty(dynamic_obstacles)
    cfg_anim.dynamicObsHistory = dyn_obs_hist(:, :, 1:steps+1);
    cfg_anim.dynamicObsRadius = dynamic_obs_radius_m;
end

% Add hull footprint config for visualization
cfg_anim.hullCfg = hull_cfg;

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
    warning('run_nmpc:AnimationNoReplay', 'Animation/recording failed; skipping replay to avoid restart-from-beginning behavior.');
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

if enable_terminal_log_recording
    diary off;
    if ~isempty(terminal_log_cleanup)
        clear terminal_log_cleanup;
    end
    if isfile(terminal_log_file)
        fprintf('  Terminal log saved: %s\n', terminal_log_file);
    else
        fprintf('  WARNING: expected terminal log file not found: %s\n', terminal_log_file);
    end
end

fprintf('\nDone. Check figures.\n');

catch ME
    % Capture any error during output generation but still close diary
    output_gen_error = true;
    fprintf('\n⚠ Error during output generation: %s (line %d)\n', ME.message, ME.stack(1).line);
    fprintf('⚠ Attempting to still save logs and partial outputs...\n\n');
end


fprintf('==============================================================\n');
fprintf('  Simulation and output generation complete\n');
fprintf('==============================================================\n');


%  LOCAL FUNCTIONS

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, cruise_speed_mps, wp_idx, R_accept, terminal_cfg)
% Simple waypoint steering for 9-state model
    if nargin < 6 || isempty(terminal_cfg)
        terminal_cfg = struct();
    end
    terminal_cfg.enabled = getOr(terminal_cfg, 'enabled', false);
    terminal_cfg.activation_dist_m = getOr(terminal_cfg, 'activation_dist_m', 120);
    terminal_cfg.standard_min_speed_mps = getOr(terminal_cfg, 'standard_min_speed_mps', 0.8);
    terminal_cfg.terminal_min_speed_mps = getOr(terminal_cfg, 'terminal_min_speed_mps', 0.05);
    terminal_cfg.terminal_max_speed_mps = getOr(terminal_cfg, 'terminal_max_speed_mps', 1.2);
    terminal_cfg.close_dist_m = getOr(terminal_cfg, 'close_dist_m', 24);
    terminal_cfg.close_speed_mps = getOr(terminal_cfg, 'close_speed_mps', 0.45);
    terminal_cfg.heading_hold_radius_m = getOr(terminal_cfg, 'heading_hold_radius_m', 12);
    terminal_cfg.velocity_heading_switch_mps = getOr(terminal_cfg, 'velocity_heading_switch_mps', 0.35);

    n_wps = size(wp, 1);
    pos   = [x(4); x(5)];
    d_final = norm(pos - wp(end,:)');
    wp_idx = min(max(1, wp_idx), n_wps);
    on_final_leg = wp_idx >= max(1, n_wps - 1);

    % Loop allows intermediate waypoints to be evaluated for skipping.
    % Once wp_idx reaches n_wps-1, we still check it once more for completion, then lock to final.
    loop_count = 0;
    max_loops = n_wps;  % Prevent infinite loops
    advanced_this_step = false;
    while wp_idx < n_wps && loop_count < max_loops && ~advanced_this_step
        loop_count = loop_count + 1;
        p_from = wp(wp_idx, :)';
        p_to   = wp(min(wp_idx + 1, n_wps), :)';
        seg    = p_to - p_from;
        seg_l2 = seg' * seg;
        seg_len = sqrt(seg_l2);

        if seg_l2 < 1e-9
            wp_idx = wp_idx + 1;
            continue;
        end

        proj = dot(pos - p_from, seg) / seg_l2;
        d_to_waypoint = norm(pos - p_to);

        % Keep skipping flexible in long routes, but lock progression near
        % terminal segments to prevent premature wp jumps.
        terminal_lock_last_n = 1;                  % Only final segment is stricter
        prefinal_accept_scale = 0.45;              % Fraction of R_accept for pre-final gate
        prefinal_accept_m = max(25, prefinal_accept_scale * R_accept);
        in_terminal_lock_window = wp_idx >= max(2, n_wps - terminal_lock_last_n);
        
        p_next = wp(min(wp_idx + 2, n_wps), :)';
        d_to_next_waypoint = norm(pos - p_next);
        xte_seg = abs(((pos(1)-p_from(1))*seg(2) - (pos(2)-p_from(2))*seg(1)) / max(seg_len, 1e-6));
        near_segment = xte_seg <= max(0.9*R_accept, 50);

        % Intermediate checkpoints are intentionally skippable when
        % obstacle/dynamic constraints push the vessel off the nominal line.
        skip_due_to_progress = (proj >= 0.99) && near_segment;
        % Robust progression for non-terminal legs: once well past the gate,
        % advance even if obstacle avoidance temporarily pushed us off-segment.
        skip_due_to_overshoot = (proj >= 1.05);
        % Missed-gate recovery for non-terminal waypoints: if we are close to
        % the gate neighborhood and largely past it in projection, advance to
        % avoid getting trapped on the previous leg after obstacle bypass.
        skip_due_to_missed_gate = (proj >= 0.92) && ...
            (d_to_waypoint <= max(2.5 * R_accept, 180));
        skip_due_to_better_next = (proj >= 0.90) && near_segment && (d_to_next_waypoint < d_to_waypoint);

        if in_terminal_lock_window
            % In terminal lock window, disallow aggressive "better next"
            % skipping so the vessel converges through each final gate.
            skip_due_to_better_next = false;
        end

        if wp_idx == n_wps - 1
            % Pre-final -> final transition must be true proximity; do not
            % allow progression based on being near the current waypoint.
            can_skip = (d_to_waypoint <= prefinal_accept_m) || ...
                       ((proj >= 1.0) && near_segment && (d_to_waypoint <= R_accept));
        elseif in_terminal_lock_window
            % For the remaining terminal-lock segments, require actual gate
            % reach (with mild projection support), no lookahead skip.
            can_skip = (d_to_waypoint <= R_accept) || ...
                       ((proj >= 1.0) && near_segment && (d_to_waypoint <= 1.1 * R_accept));
        else
            can_skip = (d_to_waypoint <= R_accept || skip_due_to_progress || ...
                        skip_due_to_better_next || skip_due_to_overshoot || ...
                        skip_due_to_missed_gate || ...
                        (proj >= 1.0 && near_segment));
        end
        
        if can_skip
            prev_wp_idx = wp_idx;
            wp_idx = wp_idx + 1;
            advanced_this_step = true;  % avoid wp2->wp4 jumps in one solver step
            fprintf('  [wp-advance] %d -> %d (proj=%.2f, d_wp=%.1f m, xte=%.1f m)\n', ...
                prev_wp_idx, wp_idx, proj, d_to_waypoint, xte_seg);
        else
            break;
        end
    end
    
    % Clamp to valid range
    wp_idx = min(wp_idx, n_wps);

    % Robust final-leg mode for dense waypoint sets:
    % once on final leg, target final waypoint directly to avoid
    % segment-projection drift around the pre-final gate.
    if on_final_leg && getOr(terminal_cfg, 'direct_target_mode', true)
        p_goal = wp(end, :)';
        dp_goal = p_goal - pos;
        d_goal = norm(dp_goal);
        chi_d = atan2(dp_goal(2), dp_goal(1));

        U_d = cruise_speed_mps;

        psi_err_abs = abs(wrapToPi(chi_d - x(6)));
        psi_err_enter = deg2rad(12);
        psi_err_full  = deg2rad(48);
        turn_alpha = (psi_err_abs - psi_err_enter) / max(psi_err_full - psi_err_enter, 1e-6);
        turn_alpha = max(0, min(1, turn_alpha));
        U_turn_cap = 4.2 + (cruise_speed_mps - 4.2) * (1 - turn_alpha);
        U_d = min(U_d, U_turn_cap);

        if d_goal < 350
            U_d = min(U_d, 0.8 + 0.012 * d_goal);
        end
        if d_goal < 100
            U_d = min(U_d, 0.6);
        end

        if terminal_cfg.enabled && d_goal < terminal_cfg.activation_dist_m
            U_d = min(U_d, terminal_cfg.terminal_max_speed_mps);
            if d_goal < terminal_cfg.close_dist_m
                U_d = min(U_d, terminal_cfg.close_speed_mps);
            end
            U_d = max(terminal_cfg.terminal_min_speed_mps, U_d);
        else
            U_d = max(terminal_cfg.standard_min_speed_mps, U_d);
        end
        return;
    end

    p1 = wp(wp_idx, :)';
    p2 = wp(min(wp_idx + 1, n_wps), :)';
    seg = p2 - p1;
    seg_len = norm(seg);
    xte_active = 0;
    xte_signed = 0;

    if seg_len < 1e-6
        target = p2;
    else
        s_proj = dot(pos - p1, seg) / seg_len;
        s_proj = max(0, min(seg_len, s_proj));

        % Cross-track relative to currently active segment.
        xte_signed = ((pos(1)-p1(1))*seg(2) - (pos(2)-p1(2))*seg(1)) / max(seg_len, 1e-6);
        xte_active = abs(xte_signed);

        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead_nominal = max(90, 7 * U_now);
        lookahead_terminal = 12 + 0.10 * min(d_final, 200);

        % Recapture logic: if too far from corridor centerline, prioritize rejoining
        % the segment before advancing deeply ahead.
        xte_rejoin_thr = max(45, 0.35 * R_accept);
        if xte_active > xte_rejoin_thr
            s_target = s_proj;
        else
            lookahead = max(18, min(lookahead_nominal, lookahead_terminal));
            xte_soft = max(0.7 * xte_rejoin_thr, 25);
            recapture_gain = max(0.35, min(1.0, 1.0 - xte_active / max(xte_soft, 1e-6)));
            lookahead = recapture_gain * lookahead;
            s_target = min(seg_len, s_proj + lookahead);
        end

        % On final leg, keep the virtual target ahead on the segment to
        % avoid lingering around the pre-terminal waypoint after bypass.
        if on_final_leg && xte_active < 90
            min_forward_push_m = max(35, 0.18 * seg_len);
            s_target = max(s_target, min(seg_len, max(0, s_proj + min_forward_push_m)));
        end

        % Safety clamp: never target behind the segment start.
        s_target = max(0, min(seg_len, s_target));

        target = p1 + (s_target / seg_len) * seg;
    end

    dp = target - pos;
    d_target = norm(dp);
    chi_d = atan2(dp(2), dp(1));

    % Final corridor centerline correction: bias heading toward the active
    % segment direction with a bounded cross-track correction term.
    if on_final_leg && seg_len > 1e-6 && xte_active > 10
        chi_seg = atan2(seg(2), seg(1));
        chi_corr = -0.012 * xte_signed;
        chi_corr = max(-deg2rad(24), min(deg2rad(24), chi_corr));
        chi_d = wrapToPi(chi_seg + chi_corr);
    end

    % Close to terminal target, dp can become tiny/noisy. Stabilize heading
    % to avoid lateral pass-through and improve tight recapture behavior.
    if on_final_leg && d_target < terminal_cfg.heading_hold_radius_m
        v_n = x(1) * cos(x(6)) - x(2) * sin(x(6));
        v_e = x(1) * sin(x(6)) + x(2) * cos(x(6));
        U_ground = hypot(v_n, v_e);

        p_prev = wp(max(1, n_wps-1), :)';
        p_goal = wp(end, :)';
        final_seg = p_goal - p_prev;
        if norm(final_seg) > 1e-6
            chi_final_leg = atan2(final_seg(2), final_seg(1));
        else
            chi_final_leg = x(6);
        end

        % Prefer the final-segment heading unless we are already well aligned.
        chi_d = chi_final_leg;
        if U_ground > terminal_cfg.velocity_heading_switch_mps && xte_active < 12
            blend = max(0.0, min(1.0, 1.0 - xte_active / 12));
            chi_vel = atan2(v_e, v_n);
            chi_d = atan2((1 - blend) * sin(chi_final_leg) + blend * sin(chi_vel), ...
                (1 - blend) * cos(chi_final_leg) + blend * cos(chi_vel));
        end
    end

    U_d = cruise_speed_mps;

    % Turn-aware speed shaping: slow down for large heading error to reduce sway
    % and help the vessel rotate before accelerating along the segment.
    psi_err_abs = abs(wrapToPi(chi_d - x(6)));
    psi_err_enter = deg2rad(12);
    psi_err_full  = deg2rad(48);
    turn_alpha = (psi_err_abs - psi_err_enter) / max(psi_err_full - psi_err_enter, 1e-6);
    turn_alpha = max(0, min(1, turn_alpha));
    U_turn_cap = 4.2 + (cruise_speed_mps - 4.2) * (1 - turn_alpha);
    U_d = min(U_d, U_turn_cap);

    % Turn-first gate on final leg: enforce heading alignment before translation.
    if on_final_leg
        psi_turn_gate = deg2rad(22);
        if psi_err_abs > psi_turn_gate
            turn_gate_alpha = (psi_err_abs - psi_turn_gate) / deg2rad(35);
            turn_gate_alpha = max(0, min(1, turn_gate_alpha));
            U_turn_first_cap = 1.6 - 0.9 * turn_gate_alpha;  % 1.6 -> 0.7 m/s as heading error grows
            U_d = min(U_d, U_turn_first_cap);
        end

        % If lateral body speed is still high, keep surge modest until recentered.
        if abs(x(2)) > 0.35
            U_d = min(U_d, 1.6);
        end
    end

    if d_final < 350
        U_d = min(U_d, 0.8 + 0.012 * d_final);
    end
    if d_final < 100
        U_d = min(U_d, 0.6);
    end

    % Final-leg re-centering: if the vessel is still off the tunnel centerline,
    % keep some speed but do not let it carry lateral motion unchecked.
    if on_final_leg && xte_active > 12
        U_d = min(U_d, 2.2);
    end

    if terminal_cfg.enabled && on_final_leg && d_final < terminal_cfg.activation_dist_m
        U_d = min(U_d, terminal_cfg.terminal_max_speed_mps);
        if d_final < terminal_cfg.close_dist_m
            U_d = min(U_d, terminal_cfg.close_speed_mps);
        end
        U_d = max(terminal_cfg.terminal_min_speed_mps, U_d);
    else
        U_d = max(terminal_cfg.standard_min_speed_mps, U_d);
    end
end

function U_cmd = applySpeedGovernor(U_base, x, obs_local, dynamic_obstacles, cfg)
% Blend distance-based and dynamic collision-risk speed caps.
    if nargin < 5 || isempty(cfg)
        U_cmd = U_base;
        return;
    end

    U_min = max(0.1, getOr(cfg, 'min_speed_mps', 0.2));
    U_cruise = getOr(cfg, 'cruise_speed_mps', max(U_base, U_min));
    U_cap = U_cruise;

    % 1) Distance cap against all currently packaged circular obstacles.
    if nargin >= 3 && ~isempty(obs_local)
        ship_pos = x(4:5);
        d_clear_min = inf;
        extra_clear = getOr(cfg, 'clearance_buffer_m', 0);
        for kk = 1:length(obs_local)
            d_cent = norm(ship_pos - obs_local(kk).position(1:2));
            d_clear = d_cent - (obs_local(kk).radius + extra_clear);
            d_clear_min = min(d_clear_min, d_clear);
        end

        d_trig = getOr(cfg, 'dist_trigger_m', 200);
        d_stop = min(d_trig - 1e-3, getOr(cfg, 'dist_stop_m', 40));
        if isfinite(d_clear_min) && d_trig > d_stop
            alpha = (d_clear_min - d_stop) / (d_trig - d_stop);
            alpha = max(0, min(1, alpha));
            U_cap_dist = U_min + alpha * (U_cruise - U_min);
            U_cap = min(U_cap, U_cap_dist);
        end
    end

    % 2) TCPA/DCPA cap against dynamic obstacles (if available).
    if nargin >= 4 && ~isempty(dynamic_obstacles)
        p_ship = x(4:5);
        psi = x(6);
        u_b = x(1);
        v_b = x(2);
        v_ship = [u_b * cos(psi) - v_b * sin(psi);
                  u_b * sin(psi) + v_b * cos(psi)];

        tcpa_h = getOr(cfg, 'tcpa_horizon_s', 40);
        dcpa_trig = getOr(cfg, 'dcpa_trigger_m', 80);
        risk_gain = getOr(cfg, 'tcpa_risk_gain', 1.0);
        risk_peak = 0;

        for kk = 1:length(dynamic_obstacles)
            if ~isfield(dynamic_obstacles(kk), 'enabled') || ~dynamic_obstacles(kk).enabled
                continue;
            end
            if ~isfield(dynamic_obstacles(kk), 'active') || ~dynamic_obstacles(kk).active
                continue;
            end

            p_obs = dynamic_obstacles(kk).position(1:2);
            spd = dynamic_obstacles(kk).speed;
            hdg = dynamic_obstacles(kk).heading;
            v_obs = spd * [cos(hdg); sin(hdg)];

            p_rel = p_obs - p_ship;
            v_rel = v_obs - v_ship;
            v_rel_sq = max(1e-6, dot(v_rel, v_rel));
            tcpa = -dot(p_rel, v_rel) / v_rel_sq;
            if tcpa <= 0 || tcpa > tcpa_h
                continue;
            end

            dcpa_vec = p_rel + tcpa * v_rel;
            dcpa = norm(dcpa_vec) - dynamic_obstacles(kk).radius;
            if dcpa >= dcpa_trig
                continue;
            end

            w_t = 1 - tcpa / max(tcpa_h, 1e-3);
            w_d = 1 - dcpa / max(dcpa_trig, 1e-3);
            risk_k = max(0, min(1, w_t * w_d));
            risk_peak = max(risk_peak, risk_k);
        end

        risk_peak = max(0, min(1, risk_gain * risk_peak));
        if risk_peak > 0
            U_cap_tcpa = U_cruise - risk_peak * (U_cruise - U_min);
            U_cap = min(U_cap, U_cap_tcpa);
        end
    end

    U_cmd = min(U_base, U_cap);
    U_cmd = max(U_min, U_cmd);
end

function is_tight = detectTightCorridor(x_state, obs_list, hull_beam_m, clearance_enter_m, clearance_exit_m, crowd_radius_m, crowd_count_enter, mode_was_active)
% Detect tight-corridor conditions from either:
% 1) Low available clearance (map walls or close obstacles), or
% 2) High local obstacle crowding that forms a constrained channel.
    if nargin < 8
        mode_was_active = false;
    end
    if isempty(obs_list)
        is_tight = false;
        return;
    end

    ship_pos = x_state(4:5);
    min_clearance = inf;
    near_count = 0;

    for k = 1:length(obs_list)
        obs_pos = obs_list(k).position(1:2);
        obs_rad = obs_list(k).radius;
        d_center = norm(ship_pos - obs_pos);
        clearance_k = max(0.1, d_center - obs_rad - hull_beam_m/2);
        min_clearance = min(min_clearance, clearance_k);
        if d_center <= crowd_radius_m
            near_count = near_count + 1;
        end
    end

    if mode_was_active
        crowd_count_thresh = max(1, crowd_count_enter - 1);
        clearance_thresh = clearance_exit_m;
    else
        crowd_count_thresh = max(1, crowd_count_enter);
        clearance_thresh = clearance_enter_m;
    end

    is_tight_by_clearance = min_clearance < clearance_thresh;
    is_tight_by_crowding = near_count >= crowd_count_thresh;
    is_tight = is_tight_by_clearance || is_tight_by_crowding;
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

function x_ref = buildSimpleRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref, n3_ref, turn_cfg)
% Build 9-state reference trajectory
    if nargin < 9 || isempty(turn_cfg)
        turn_cfg = struct();
    end
    r_gain = getOr(turn_cfg, 'r_gain', 0.35);
    r_ref_max = getOr(turn_cfg, 'r_ref_max', 0.10);
    ramp_r_scale = getOr(turn_cfg, 'ramp_r_scale', 0.15);

    x_ref = zeros(9, N+1);
    x_ref(:, 1) = x0;
    
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));
    r_d = r_gain * psi_err;
    r_d = max(-r_ref_max, min(r_ref_max, r_d));
    track_chi = chi_d;
    
    % Smooth heading ramp to avoid spawning sideways.
    % Large heading changes get a longer turn-first window.
    n_ramp = min(N, max(5, round(5 + 8 * min(1, abs(psi_err) / deg2rad(60)))));
    
    for k = 2:(N+1)
        x_ref(2, k) = 0;
        x_ref(7, k) = n1_ref;
        x_ref(8, k) = n2_ref;
        x_ref(9, k) = n3_ref;
        
        % NEW: Smooth ramp for heading instead of step change
        if k <= n_ramp + 1
            % Linear ramp from current heading to desired heading
            ramp_factor = (k - 1) / (n_ramp + 1);
            x_ref(6, k) = x0(6) + ramp_factor * psi_err;
            % Reduce rotation rate during ramp
            x_ref(3, k) = ramp_r_scale * r_d * sin((k-1) / (n_ramp + 1) * pi);

            % Turn-first progress: keep translational progress modest during early heading capture.
            U_prog = U_d * max(0.18, ramp_factor^1.5);
            if abs(psi_err) > deg2rad(25)
                U_prog = min(U_prog, 1.2);
            end
        else
            % After ramp, use full heading and rotation rate
            x_ref(6, k) = chi_d;
            x_ref(3, k) = r_d;
            U_prog = U_d;
        end

        x_ref(1, k) = U_prog;
        
        % Keep translation on the commanded track bearing while the
        % heading state ramps smoothly toward it.
        dx = U_prog * dt * cos(track_chi);
        dy = U_prog * dt * sin(track_chi);
        x_ref(4, k) = x_ref(4, k-1) + dx;
        x_ref(5, k) = x_ref(5, k-1) + dy;
    end
end     

function x_ref = buildObstacleAwareRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref, n3_ref, obstacles, avoid_cfg)
% Reference trajectory with obstacle deflection
    if nargin < 10 || isempty(avoid_cfg)
        avoid_cfg = struct('base_margin_m', 80, 'speed_gain_s', 0.0, ...
            'obs_radius_gain', 0.5, 'deflect_sigma', 0.22, 'r_ref_max', 0.10);
    end
    safety_margin = avoid_cfg.base_margin_m + avoid_cfg.speed_gain_s * max(0, U_d);
    turn_cfg = struct('r_gain', 0.35, 'r_ref_max', 0.10, 'ramp_r_scale', 0.15);
    x_ref = buildSimpleRef8(x0, chi_d, U_d, N, dt, n1_ref, n2_ref, n3_ref, turn_cfg);

    if nargin < 9 || isempty(obstacles)
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
        if abs(lateral) >= (safety_margin + obs_r)
            continue;
        end

        if lateral >= 0
            side_sign = -1;
        else
            side_sign = +1;
        end

        deflect = side_sign * (safety_margin + avoid_cfg.obs_radius_gain * obs_r - lateral * side_sign);

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
            % NEW: Preserve heading ramp during first few steps to avoid spawning sideways
            n_ramp = min(5, N);
            for k = 2:(N+1)
                if k <= n_ramp + 1
                    % Keep the smooth ramp from buildSimpleRef8
                    % Don't overwrite heading during initial ramp
                    continue;
                else
                    x_ref(3, k) = r_d2;
                    x_ref(6, k) = chi_deflected;
                end
            end
        end
    end
end

function x_next = rk4Step9(x, u_ctrl, dt_s, u_min_step)
% RK4 integration for 9-state container model
    if nargin < 4
        u_min_step = 0.5;
    end

    % Enforce minimum forward speed constraint in integration
    x(1) = max(x(1), u_min_step);
    
    [k1, ~] = container(x, u_ctrl);
    
    x2 = x + k1*dt_s/2;
    x2(1) = max(x2(1), u_min_step);
    [k2, ~] = container(x2, u_ctrl);
    
    x3 = x + k2*dt_s/2;
    x3(1) = max(x3(1), u_min_step);
    [k3, ~] = container(x3, u_ctrl);
    
    x4 = x + k3*dt_s;
    x4(1) = max(x4(1), u_min_step);
    [k4, ~] = container(x4, u_ctrl);
    
    x_next = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
    x_next(1) = max(x_next(1), u_min_step);
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
