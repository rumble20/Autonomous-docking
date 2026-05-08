% test_heading_logic_detailed.m
% Detailed heading logic analysis: simulate multiple waypoint segments,
% log heading references and actual values, detect discontinuities

clear; clc; close all;

fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('  DETAILED HEADING LOGIC ANALYSIS\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% Setup
waypoints = [-3800, -1500; -3400, -1300; -3200, -1350; -3000, -1400; -2600, -1800; -2400, -2100; -2000, -2050];
n_waypoints = size(waypoints, 1);

% Simple NMPC config
nmpc_cfg = struct();
nmpc_cfg.N = 25;
nmpc_cfg.dt = 1.0;
nmpc_cfg.Q = diag([0.0, 0.04, 0.08, 0, 0, 0, 0.001, 0.001, 0.001]);
nmpc_cfg.R = diag([0.10, 0.10, 0.006, 0.006, 0.010]);
nmpc_cfg.R_rate = diag([0.12, 0.12, 0.004, 0.004, 0.010]);
nmpc_cfg.max_obs = 6;
nmpc_cfg.max_halfplanes = 8;

% Initial state at first waypoint
x0_heading_rad = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));
x = [7; 0; 0; waypoints(1,1); waypoints(1,2); x0_heading_rad; 100; 100; 0];

fprintf('Initial state:\n');
fprintf('  Position: (%.1f, %.1f)\n', x(4), x(5));
fprintf('  Heading: %.3f rad = %.2f deg\n\n', x(6), rad2deg(x(6)));

% Build solver
fprintf('Building NMPC solver (N=%d)...\n', nmpc_cfg.N);
nmpc = NMPC_Container_final(nmpc_cfg);
nmpc.buildSolver();
fprintf('Solver ready.\n\n');

%% Logging arrays
max_steps = 300;  % Up to 5 min at dt=1s
t_log = [];
x_pos_log = [];
x_heading_log = [];
wp_idx_log = [];
seg_heading_log = [];
chi_ctrl_log = [];
goal_heading_enable_log = [];
goal_heading_rad_log = [];
heading_err_log = [];
dist_to_wp_log = [];
dist_to_seg_end_log = [];

dt = nmpc_cfg.dt;
u_prev = [0; 0; 100; 100; 0];
step = 0;

%% Simulation loop through waypoints
for wp_idx = 1:(n_waypoints - 1)
    
    % Compute segment
    wp_start = waypoints(wp_idx, :)';
    wp_end = waypoints(wp_idx+1, :)';
    seg_vec = wp_end - wp_start;
    seg_length = norm(seg_vec);
    seg_hat = seg_vec / seg_length;
    
    % Segment heading
    chi_seg_rad = atan2(seg_vec(2), seg_vec(1));
    chi_seg_deg = rad2deg(chi_seg_rad);
    
    fprintf(['─────────────────────────────────────────────────────────\n', ...
             'Segment %d: WP%d → WP%d\n'], wp_idx, wp_idx, wp_idx+1);
    fprintf('  Start: (%.1f, %.1f)\n', wp_start(1), wp_start(2));
    fprintf('  End: (%.1f, %.1f)\n', wp_end(1), wp_end(2));
    fprintf('  Length: %.1f m\n', seg_length);
    fprintf('  Heading: %.3f rad = %.2f deg\n\n', chi_seg_rad, chi_seg_deg);
    
    % Simulate this segment
    steps_in_segment = 0;
    while norm(x(4:5) - wp_end) > 50 && step < max_steps
        step = step + 1;
        steps_in_segment = steps_in_segment + 1;
        
        % Compute path tracking metrics
        err_vec = x(4:5) - wp_start;
        xte = err_vec(1) * (-seg_hat(2)) + err_vec(2) * seg_hat(1);  % Cross-track error
        along = err_vec(1) * seg_hat(1) + err_vec(2) * seg_hat(2);   % Along-track
        dist_to_start = norm(err_vec);
        dist_to_end = norm(x(4:5) - wp_end);
        
        % Heading control reference
        chi_ctrl_rad = chi_seg_rad;
        
        % Determine if in sharp turn zone (look at next segment)
        goal_heading_enable_step = false;
        goal_heading_rad_step = chi_seg_rad;
        
        if wp_idx < (n_waypoints - 1)
            % Check if next segment exists and forms a sharp turn
            seg_next = waypoints(wp_idx+2, :)' - wp_end;
            chi_next_rad = atan2(seg_next(2), seg_next(1));
            turn_angle_rad = chi_next_rad - chi_seg_rad;
            turn_angle_rad = wrapToPi(turn_angle_rad);
            turn_angle_deg = abs(rad2deg(turn_angle_rad));
            
            % Simplified sharp turn detection: if turn > 28°
            if turn_angle_deg >= 28
                dist_to_end_m = dist_to_end;
                if dist_to_end_m <= 160  % sharp turn enable distance
                    goal_heading_enable_step = true;
                    goal_heading_rad_step = chi_seg_rad;
                end
            end
        end
        
        % Build path reference
        path_ref = struct();
        path_ref.wp_start = wp_start;
        path_ref.wp_end = wp_end;
        path_ref.goal_heading_enable = goal_heading_enable_step;
        path_ref.goal_heading_rad = goal_heading_rad_step;
        
        % Solve NMPC
        obs_local = struct('position', {}, 'radius', {});
        solve_opts = struct();
        solve_opts.map_halfplanes = [];
        solve_opts.map_barrier_weight = 2.5e5;
        solve_opts.map_soft_margin_m = 27;
        
        [u_opt, ~, info] = nmpc.solve(x, path_ref, obs_local, u_prev, 0.3, solve_opts);
        
        if ~info.success
            fprintf('  [step %d] Solver failed\n', step);
            break;
        end
        
        % Integrate
        x = rk4Step9(x, u_opt, dt);
        u_prev = u_opt;
        
        % Heading error
        heading_err = wrapToPi(chi_ctrl_rad - x(6));
        
        % Log
        t_log = [t_log; step * dt];
        x_pos_log = [x_pos_log; x(4:5)'];
        x_heading_log = [x_heading_log; x(6)];
        wp_idx_log = [wp_idx_log; wp_idx];
        seg_heading_log = [seg_heading_log; chi_seg_rad];
        chi_ctrl_log = [chi_ctrl_log; chi_ctrl_rad];
        goal_heading_enable_log = [goal_heading_enable_log; goal_heading_enable_step];
        goal_heading_rad_log = [goal_heading_rad_log; goal_heading_rad_step];
        heading_err_log = [heading_err_log; heading_err];
        dist_to_wp_log = [dist_to_wp_log; dist_to_end];
        dist_to_seg_end_log = [dist_to_seg_end_log; along];
        
        % Print every 30 steps
        if mod(steps_in_segment, 30) == 1 || steps_in_segment == 1
            fprintf('  [t=%4.0fs, step=%3d] pos=(%.0f,%.0f) heading=%+7.2f° ' , ...
                t_log(end), steps_in_segment, x(4), x(5), rad2deg(x(6)));
            fprintf('seg=%+7.2f° err=%+6.2f° dist_to_end=%.0f m\n', ...
                rad2deg(chi_seg_rad), rad2deg(heading_err), dist_to_end);
        end
    end
    
    fprintf('  Completed: %d steps, final distance to WP end: %.1f m\n\n', steps_in_segment, dist_to_end);
end

%% Analysis
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('HEADING LOGIC ANALYSIS RESULTS\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

fprintf('Simulation Statistics:\n');
fprintf('  Total steps: %d\n', length(t_log));
fprintf('  Time simulated: %.0f s (%.1f min)\n', t_log(end), t_log(end)/60);
fprintf('  Segments traversed: %d\n', length(unique(wp_idx_log)));
fprintf('  Final position: (%.1f, %.1f)\n', x_pos_log(end, 1), x_pos_log(end, 2));
fprintf('  Final heading: %.2f deg\n\n', rad2deg(x_heading_log(end)));

%% Check for heading discontinuities
fprintf('Heading Discontinuity Analysis:\n');
fprintf('─────────────────────────────────\n\n');

dh_heading = diff(x_heading_log);
dh_heading_wrapped = wrapToPi(dh_heading);
dh_heading_deg = rad2deg(dh_heading_wrapped);

% Find large jumps (>5°)
large_jump_idx = find(abs(dh_heading_deg) > 5);
if ~isempty(large_jump_idx)
    fprintf('Found %d heading jumps >5°:\n', length(large_jump_idx));
    for idx = large_jump_idx(1:min(5, length(large_jump_idx)))  % Show first 5
        fprintf('  Step %d: Δheading = %+7.2f° (%.2f → %.2f°)\n', ...
            idx, dh_heading_deg(idx), rad2deg(x_heading_log(idx)), rad2deg(x_heading_log(idx+1)));
    end
else
    fprintf('✓ No heading discontinuities >5°\n');
end
fprintf('\n');

%% Heading error statistics
fprintf('Heading Error Statistics (chi_ctrl - x.heading):\n');
fprintf('─────────────────────────────────────────────────\n\n');

heading_err_deg = rad2deg(heading_err_log);

fprintf('Overall:\n');
fprintf('  Min: %+7.2f°\n', min(heading_err_deg));
fprintf('  Max: %+7.2f°\n', max(heading_err_deg));
fprintf('  Mean: %+7.2f°\n', mean(heading_err_deg));
fprintf('  StdDev: %.2f°\n', std(heading_err_deg));
fprintf('  RMS: %.2f°\n\n', sqrt(mean(heading_err_deg.^2)));

% Per-segment analysis
fprintf('Per-Segment Heading Errors:\n');
for wp = 1:(n_waypoints-1)
    mask = wp_idx_log == wp;
    if any(mask)
        err_seg = heading_err_deg(mask);
        fprintf('  Segment %d: mean=%+7.2f° max=%+7.2f° std=%.2f°\n', ...
            wp, mean(err_seg), max(abs(err_seg)), std(err_seg));
    end
end
fprintf('\n');

%% Heading mode transitions
fprintf('Goal Heading Enable Transitions:\n');
fprintf('─────────────────────────────────\n\n');

trans_idx = find(diff(goal_heading_enable_log) ~= 0);
if ~isempty(trans_idx)
    fprintf('Found %d transitions in goal_heading_enable:\n', length(trans_idx));
    for i = 1:min(5, length(trans_idx))
        idx = trans_idx(i);
        fprintf('  Step %d: %.0fs | %d→%d | heading_err was %+.2f° then %+.2f°\n', ...
            idx, t_log(idx), goal_heading_enable_log(idx), goal_heading_enable_log(idx+1), ...
            heading_err_deg(idx), heading_err_deg(idx+1));
    end
else
    fprintf('✓ No transitions in goal_heading_enable\n');
end
fprintf('\n');

%% Goal heading reference changes
fprintf('Goal Heading Reference Changes:\n');
fprintf('────────────────────────────────\n\n');

dgoal_rad = diff(goal_heading_rad_log);
dgoal_wrapped = wrapToPi(dgoal_rad);
dgoal_deg = rad2deg(dgoal_wrapped);

large_goal_change_idx = find(abs(dgoal_deg) > 2);
if ~isempty(large_goal_change_idx)
    fprintf('Found %d goal_heading_rad changes >2°:\n', length(large_goal_change_idx));
    for idx = large_goal_change_idx(1:min(3, length(large_goal_change_idx)))
        fprintf('  Step %d: Δgoal = %+7.2f° (%.2f → %.2f°)\n', ...
            idx, dgoal_deg(idx), rad2deg(goal_heading_rad_log(idx)), rad2deg(goal_heading_rad_log(idx+1)));
    end
else
    fprintf('✓ Goal heading reference stable (no changes >2°)\n');
end
fprintf('\n');

%% Convergence analysis
fprintf('Heading Convergence per Segment:\n');
fprintf('─────────────────────────────────\n\n');

for wp = 1:(n_waypoints-1)
    mask = wp_idx_log == wp;
    if sum(mask) > 10
        err_seg = heading_err_deg(mask);
        steps_seg = 1:sum(mask);
        
        % Fit exponential decay (approximate)
        err_abs = abs(err_seg);
        err_first = err_abs(1);
        err_last = err_abs(end);
        
        fprintf('  Segment %d: error %.2f° → %.2f° (%.0f%% reduction)\n', ...
            wp, err_first, err_last, 100*(err_first-err_last)/err_first);
    end
end
fprintf('\n');

%% Plot heading trajectory
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('Generating heading analysis plot...\n\n');

fig = figure('Position', [100 100 1400 800]);

subplot(3,2,1);
plot(t_log, rad2deg(x_heading_log), 'b-', 'LineWidth', 1.5); hold on;
plot(t_log, rad2deg(chi_ctrl_log), 'r--', 'LineWidth', 1.5);
plot(t_log, rad2deg(goal_heading_rad_log), 'g:', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Heading (deg)');
title('Heading: Actual vs Control vs Goal');
legend('Actual (x.heading)', 'Control (chi_ctrl)', 'Goal (goal_heading_rad)');
grid on;

subplot(3,2,2);
plot(t_log, heading_err_deg, 'b-', 'LineWidth', 1.5); hold on;
yline(0, 'k--', 'LineWidth', 1);
yline(5, 'r--', 'LineWidth', 1);
yline(-5, 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Heading Error (deg)');
title('Heading Error: chi_ctrl - x.heading');
grid on;

subplot(3,2,3);
plot(t_log, x_pos_log(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_log, x_pos_log(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position (m)');
title('X-Y Position vs Time');
legend('X', 'Y');
grid on;

subplot(3,2,4);
plot(x_pos_log(:,1), x_pos_log(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Trajectory');
legend('Trajectory', 'Waypoints');
grid on; axis equal;

subplot(3,2,5);
scatter(t_log, goal_heading_enable_log, 20, goal_heading_enable_log, 'filled'); hold on;
xlabel('Time (s)'); ylabel('goal_heading_enable');
title('Goal Heading Enable Status');
colormap(gca, [0.7 0.7 0.7; 0 0.5 0]);
ylim([-0.2, 1.2]);
grid on;

subplot(3,2,6);
plot(t_log(2:end), diff(x_heading_log), 'b-', 'LineWidth', 1.5); hold on;
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('ΔHeading / dt (rad/s)');
title('Heading Rate of Change');
grid on;

savefig([pwd, '\heading_analysis.fig']);
fprintf('✓ Saved heading_analysis.fig\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% Helper functions
function x_next = rk4Step9(x, u_ctrl, dt_s)
    [k1, ~] = container(x, u_ctrl);
    x2 = x + k1*dt_s/2;
    [k2, ~] = container(x2, u_ctrl);
    x3 = x + k2*dt_s/2;
    [k3, ~] = container(x3, u_ctrl);
    x4 = x + k3*dt_s;
    [k4, ~] = container(x4, u_ctrl);
    x_next = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
end

function ang = wrapToPi(angle)
    ang = mod(angle + pi, 2*pi) - pi;
end
