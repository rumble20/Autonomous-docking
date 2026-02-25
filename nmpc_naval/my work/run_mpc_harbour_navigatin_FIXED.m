%% run_mpc_harbour_navigatin_FIXED.m
% NMPC-based harbor navigation - CORRECTED VERSION
%
% KEY FIXES:
% 1. Use NMPC_Container_Fixed with persistent optimizer
% 2. Dynamic reference trajectory (not constant goal)
% 3. Reduced active obstacles (from 166 to 30 max)
% 4. Warm starting enabled for convergence
% 5. Better tolerances and feasibility margins
%
% Author: Riccardo Legnini
% Date: 2026-02-25

clear; close all; clc;

fprintf('=== NMPC HARBOR NAVIGATION (FIXED) ===\n\n');

%% STEP 1: CONFIGURE NMPC
config = struct();
config.N = 7;           % REDUCED horizon: 7 steps × 1s = 7 seconds look-ahead
config.dt = 1.0;        % 1 second sample time

% State weights: position more important in tight harbor
config.Q = diag([1, 1, 1, 20, 20, 1, 0.05, 0.05, 0.1, 0.05]);

% Control weights: prefer forward motion over aggressive steering
config.R = diag([0.5, 0.1]);

% Create NMPC controller (FIXED VERSION)
nmpc = NMPC_Container_Fixed(config);

%% STEP 2: DEFINE HARBOR WITH OBSTACLES
harbor = HarborObstacles();

if exist('helsinki_harbour.mat', 'file')
    load('helsinki_harbour.mat');  
    harbor.loadPolygonMap(map);
    fprintf('✓ Loaded polygon map with obstacles\n');
else
    fprintf('❌ Map file not found\n');
end

% Add static obstacles
harbor.addStaticObstacle([300, 150], 20, 'Pier 1');
harbor.addStaticObstacle([300, -50], 20, 'Pier 2');
harbor.addStaticObstacle([450, 80], 25, 'Moored Vessel');

% Add dynamic obstacles
harbor.addDynamicObstacle([200, 100], 15, [1.0, -0.5], 'Patrol Boat');

%% STEP 3: SIMULATION SETUP
T_final = 200;  % Total simulation time
dt = config.dt;
t = 0:dt:T_final;

% Initial state [u v r x y psi p phi delta n]
x = [7; 0; 0; 0; 0; 0; 0; 0; 0; 70];

% Goal location
x_goal = [0; 0; 0; 500; 50; deg2rad(90); 0; 0; 0; 0];  % Zero velocity at goal

% IMPORTANT: Waypoint-based reference (not constant)
% Generate smooth reference trajectory toward goal
num_steps = length(t);
reference_trajectory = zeros(10, length(t));
for i = 1:length(t)
    % Linear interpolation toward goal (slow down near end)
    progress = min(1, t(i) / T_final * 1.2);
    
    reference_trajectory(4:5, i) = x(4:5) + progress * (x_goal(4:5) - x(4:5));
    reference_trajectory(6, i) = x_goal(6);
    reference_trajectory(10, i) = 70;  % Maintain optimal thrust
    
    % Velocity references (for path following)
    if progress < 0.8
        reference_trajectory(1, i) = 7;  % Maintain surge velocity
    else
        reference_trajectory(1, i) = 7*(1-progress)/0.2;  % Slow down
    end
end

% Data logging
trajectory = x;
control_log = [];
solve_times = [];
success_log = [];

%% STEP 4: MAIN SIMULATION LOOP
fprintf('Starting simulation (T=%.1f s)...\n', T_final);

for i = 1:min(length(t), num_steps)
    
    % Reference trajectory for next N+1 steps
    idx_start = min(i, num_steps);
    idx_end = min(i + nmpc.N, num_steps);
    x_ref = zeros(10, nmpc.N + 1);
    
    for k = 0:nmpc.N
        idx_ref = min(i + k, num_steps);
        x_ref(:, k+1) = reference_trajectory(:, idx_ref);
    end
    
    % Get current obstacles
    obstacles = harbor.getAllObstacles();
    
    % SOLVE NMPC
    [u_opt, X_pred, info] = nmpc.solve(x, x_ref, obstacles);
    
    % Display progress every 20 steps
    if mod(i, 20) == 0
        fprintf('[t=%.1f] Pos=(%.1f, %.1f), Success=%d, Time=%.3f s, Fails=%d\n', ...
            t(i), x(4), x(5), info.success, info.solve_time, nmpc.solve_fail_count);
    end
    
    % Apply control to container.m dynamics
    [xdot, ~] = container(x, u_opt);
    x = x + xdot * dt;  % Euler integration
    
    % Update dynamic obstacles
    harbor.updateDynamicObstacles(dt);
    
    % Log data
    trajectory = [trajectory, x];
    control_log = [control_log, u_opt];
    solve_times = [solve_times, info.solve_time];
    success_log = [success_log, info.success];
    
    % Check if goal reached
    if norm(x(4:5) - x_goal(4:5)) < 15
        fprintf('\n✓✓✓ GOAL REACHED at t=%.1f s (%.0f m from dock)!\n', ...
            t(i), norm(x(4:5) - x_goal(4:5)));
        break;
    end
    
    % Safety check: abort if too far off course
    if norm(x(4:5)) > 800
        fprintf('\n❌ Out of bounds at t=%.1f s\n', t(i));
        break;
    end
end

%% STEP 5: ANALYSIS AND PLOTTING
t_actual = t(1:size(trajectory, 2));

fprintf('\n=== RESULTS ===\n');
fprintf('Total simulation time: %.1f s\n', t_actual(end));
fprintf('Final position: (%.1f, %.1f) m\n', trajectory(4, end), trajectory(5, end));
fprintf('Distance to goal: %.1f m\n', norm(trajectory(4:5, end) - x_goal(4:5)));
fprintf('Average solve time: %.3f s\n', mean(solve_times));
fprintf('Max solve time: %.3f s\n', max(solve_times));
fprintf('Success rate: %.1f%% (%d/%d)\n', ...
    100*sum(success_log)/length(success_log), sum(success_log), length(success_log));
fprintf('Consecutive failures: %d\n', nmpc.solve_fail_count);

%% PLOTTING
figure('Position', [100, 100, 1200, 800]);

% Trajectory plot
subplot(2, 2, [1, 3]);
plot(trajectory(5, :), trajectory(4, :), 'b-', 'LineWidth', 2);
hold on; grid on; axis equal;
plot(x_goal(5), x_goal(4), 'r*', 'MarkerSize', 20, 'DisplayName', 'Goal');
plot(x(5), x(4), 'go', 'MarkerSize', 10, 'DisplayName', 'Final Position');
harbor.plotObstacles();
xlabel('East [m]'); ylabel('North [m]');
title('Harbor Navigation Trajectory');
legend('Path', 'Goal', 'Final Pos', 'Location', 'best');

% Control inputs
subplot(2, 2, 2);
t_control = t_actual(1:length(control_log));
stairs(t_control, rad2deg(control_log(1, :)), 'LineWidth', 1.5);
grid on; ylabel('Rudder Angle [deg]');
title('Commanded Rudder Angle');
ylim([-15, 15]);

subplot(2, 2, 4);
stairs(t_control, control_log(2, :), 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Shaft Speed [RPM]');
title('Commanded Propeller Speed');
ylim([0, 200]);

saveas(gcf, 'nmpc_harbor_navigation_FIXED_results.png');

fprintf('\n✓ Plotting complete!\n');
fprintf('  Check CONSOLE for diagnostic messages\n');

%% DIAGNOSTIC INFO
fprintf('\n=== DIAGNOSTIC INFORMATION ===\n');
fprintf('Problem characteristics:\n');
fprintf('  • Optimizer: Built ONCE (persistent structure)\n');
fprintf('  • Warm starting: ENABLED\n');
fprintf('  • Prediction horizon: %d steps = %.1f seconds\n', nmpc.N, nmpc.N*nmpc.dt);
fprintf('  • Max active obstacles: %d (filtered from map)\n', nmpc.max_obs);
fprintf('  • Safety margin: %.1f m\n', nmpc.r_safety);
fprintf('  • Slack penalty: %.0f\n', nmpc.penalty_slack);
fprintf('  • Reference type: DYNAMIC waypoint-based\n');
fprintf('\nIf solver still fails:\n');
fprintf('  1. Reduce horizon N to 5 (faster solving)\n');
fprintf('  2. Increase r_safety to 12+ (relax constraints)\n');
fprintf('  3. Reduce penalty_slack to 100 (accept some slack)\n');
fprintf('  4. Check obstacle constraints feasibility\n');
