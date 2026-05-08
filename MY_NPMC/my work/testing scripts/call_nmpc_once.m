% call_nmpc_once.m
% Safe one-time NMPC call to trigger parameter packing and debug print
try
    load('diagnose_map_obstacles_data.mat', 'hp_local');
catch
    hp_local = [];
end

% Minimal NMPC config (match run_nmpc defaults)
nmpc_cfg = struct();
nmpc_cfg.N = 50; nmpc_cfg.dt = 1.0;
nmpc_cfg.Q = diag([0.0, 0.04, 0.08, 0, 0, 0, 0.001, 0.001, 0.001]);
nmpc_cfg.R = diag([0.10, 0.10, 0.006, 0.006, 0.010]);
npc_cfg_present = true; %#ok<NASGU>

nmpc_cfg.R_rate = diag([0.12, 0.12, 0.004, 0.004, 0.010]);
nmpc_cfg.max_obs = 6; nmpc_cfg.max_halfplanes = max(0, round(8));
nmpc_cfg.r_safety = 40; nmpc_cfg.collision_model = 'oriented-rectangle';

% hull config (scaled as per run_nmpc defaults)
hull_nominal_length_m = 175; hull_nominal_beam_m = 25.4; hull_scale = 0.5; r_safety = 40;
hull_cfg.length_m = max(4, hull_scale * hull_nominal_length_m);
hull_cfg.beam_m = max(2, hull_scale * hull_nominal_beam_m);
hull_cfg.half_length_m = 0.5 * hull_cfg.length_m;
hull_cfg.half_beam_m = 0.5 * hull_cfg.beam_m;
hull_cfg.circ_radius_m = hypot(hull_cfg.half_length_m, hull_cfg.half_beam_m);
hull_cfg.nmpc_clearance_m = max(2.0, r_safety - hull_cfg.circ_radius_m);

% Build NMPC and solver
nmpc = NMPC_Container_final(nmpc_cfg);
nmpc.buildSolver();

% initial state and path_ref
waypoints = [-3800, -1500; -3400, -1300];
x0_heading = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
x0 = [7;0;0; waypoints(1,1); waypoints(1,2); x0_heading; 100; 100; 0];
wp_start_xy = waypoints(1, :)'; wp_end_xy = waypoints(2, :)';
path_ref = struct('wp_start', wp_start_xy, 'wp_end', wp_end_xy, 'goal_heading_rad', 0, 'goal_heading_enable', false);

% Build obstacles (empty) and halfplanes from diagnostic
obs_local = struct('position', {}, 'radius', {});
map_halfplanes = hp_local;

% solve_opts similar to run_nmpc
solve_opts = struct();
solve_opts.map_halfplanes = map_halfplanes;
solve_opts.map_barrier_weight = 2.5e5;
solve_opts.map_soft_margin_m = max(18.0, hull_cfg.nmpc_clearance_m + 25.0);

% Print the map parameters that will be sent to the solver
try
    if ~isempty(map_halfplanes)
        fprintf('\n[CALL] map_halfplanes (n=%d)\n', length(map_halfplanes));
        for k = 1:length(map_halfplanes)
            n = map_halfplanes(k).normal(:)'; b = map_halfplanes(k).offset;
            fprintf('  hp %d: n=(%.3f, %.3f) b=%.2f\n', k, n(1), n(2), b);
        end
    else
        fprintf('\n[CALL] map_halfplanes: none\n');
    end
    fprintf('[CALL] map_barrier_weight = %.3g\n', solve_opts.map_barrier_weight);
    fprintf('[CALL] map_soft_margin_m = %.3g\n\n', solve_opts.map_soft_margin_m);
catch printErr
    fprintf('Failed to print call parameters: %s\n', printErr.message);
end

% Call solver once
try
    [u_opt, X_pred, info] = nmpc.solve(x0, path_ref, obs_local, [], 0.3, solve_opts);
    fprintf('call_nmpc_once: solver returned info.success=%d, cost=%.3g\n', double(getOr(info, 'success', 0)), getOr(info, 'cost', NaN));
catch err
    fprintf('call_nmpc_once: solver call failed: %s\n', err.message);
end

function v = getOr(s, name, def)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = def;
    end
end
