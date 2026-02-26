%% run_mpc_harbour_navigatin.m
% NMPC harbor navigation — MINIMAL WORKING TEST
%
% This script tests the NMPC with:
%   TEST A — Zero obstacles, straight-ahead goal   (easiest possible)
%   TEST B — Zero obstacles, goal requiring a turn  (moderate)
%   TEST C — One obstacle in the path               (obstacle avoidance)
%
% Author: Riccardo Legnini
% Date: 2026-02-25

clear; close all; clc;
fprintf('=== NMPC HARBOR NAVIGATION — MINIMAL WORKING TEST ===\n\n');

%% ----- STEP 0: Verify container() works at initial state ----------------
fprintf('--- Step 0: Sanity check on container.m ---\n');
x_test = [7; 0; 0; 0; 0; 0; 0; 0; 0; 70];   % u=7 m/s, n=70 RPM
u_test = [0; 70];                              % zero rudder, maintain RPM
[xdot_test, U_test] = container(x_test, u_test);

fprintf('  State:    u=%.1f m/s, n=%.0f RPM, U=%.2f m/s\n', x_test(1), x_test(10), U_test);
fprintf('  xdot(1):  u_dot   = %+.6f m/s^2  (small → near equilibrium)\n', xdot_test(1));
fprintf('  xdot(4):  x_dot   = %+.4f m/s    (should be ~%.1f)\n', xdot_test(4), x_test(1));
fprintf('  xdot(5):  y_dot   = %+.4f m/s    (should be ~0)\n', xdot_test(5));
fprintf('  xdot(6):  psi_dot = %+.6f rad/s  (should be ~0)\n', xdot_test(6));
fprintf('  xdot(10): n_dot   = %+.4f RPM/s  (should be ~0)\n\n', xdot_test(10));

%% ----- STEP 1: Configure NMPC -------------------------------------------
fprintf('--- Step 1: Configure NMPC ---\n');
config = struct();
config.N  = 10;           % 10 prediction steps
config.dt = 1.0;          % 1 second sample time → 10 s horizon

% State weights:  [u   v    r    x     y    psi   p    phi  delta n   ]
config.Q = diag([  1   0.1  0.1  0.01  0.01  5    0.01 0.01 0.1  0.01]);
%  • psi weight = 5 : heading tracking is important
%  • x,y weight = 0.01 : gentle position pull (avoids huge gradients)
%  • u weight = 1 : maintain desired speed

% Control weights:  [delta_c   n_c  ]
config.R = diag([     1       0.001 ]);
%  • Penalise rudder more than thrust

config.use_obstacles = true;           % Enable constraints in NMPC
config.max_obs = 25;                   % Total constraints slots (map + external)
config.use_map_obstacles = true;       % Integrate map polygons in NMPC
config.max_map_obs = 20;               % Closest polygons used each solve
nmpc = NMPC_Container(config);

%% ----- STEP 1B: Load Helsinki harbor map -------------------------------
fprintf('\n--- Step 1B: Load harbor map (helsinki_harbour.mat) ---\n');
map = [];
if exist('helsinki_harbour.mat', 'file')
    S = load('helsinki_harbour.mat');
    if isfield(S, 'map')
        map = S.map;
        fprintf('  Loaded map with %d obstacle polygons and %d boundary polygons\n', ...
            length(map.polygons), length(map.mapPoly));
        nmpc.setMap(map);
    else
        fprintf('  Warning: file loaded but variable ''map'' not found\n');
    end
else
    fprintf('  Warning: helsinki_harbour.mat not found. Running without map.\n');
end

%% ----- STEP 2: Verify CasADi dynamics matches container() ---------------
fprintf('\n--- Step 2: CasADi dynamics consistency check ---\n');
import casadi.*
x_sym = MX.sym('x', 10, 1);
u_sym = MX.sym('u', 2, 1);
xdot_sym = nmpc.dynamics_casadi(x_sym, u_sym);
f_casadi = casadi.Function('f', {x_sym, u_sym}, {xdot_sym});

xdot_cas = full(f_casadi(x_test, u_test));
err = abs(xdot_test - xdot_cas);

fprintf('  %-10s  %12s  %12s  %12s\n', 'State', 'container()', 'CasADi', '|error|');
labels = {'u_dot','v_dot','r_dot','x_dot','y_dot','psi_dot','p_dot','phi_dot','del_dot','n_dot'};
for i = 1:10
    fprintf('  %-10s  %+12.6f  %+12.6f  %12.2e\n', labels{i}, xdot_test(i), xdot_cas(i), err(i));
end
max_err = max(err);
fprintf('  Max absolute error: %.2e\n', max_err);
if max_err < 1e-6
    fprintf('  >> PASS: CasADi dynamics matches container() perfectly.\n\n');
elseif max_err < 1e-2
    fprintf('  >> WARN: Small differences (safeguards). Should be OK.\n\n');
else
    fprintf('  >> FAIL: Large mismatch! Check dynamics_casadi().\n\n');
end

%% ========================================================================
%  TEST A — Go straight ahead, no obstacles
%  Goal at (500, 0), ship already heading east (psi=0).
%  Expected: solver converges, ship accelerates forward.
%% ========================================================================
fprintf('======================================================\n');
fprintf('  TEST A: Straight ahead, ZERO extra obstacles (map integrated)\n');
fprintf('======================================================\n');

T_final = 120;           % 2 minutes
dt = config.dt;
t  = 0:dt:T_final;

% Initial state
x = [7; 0; 0; -5800; -2800; 0; 0; 0; 0; 70];

% Goal: straight ahead at 500 m
x_goal = [7; 0; 0; -5300; -2800; 0; 0; 0; 0; 70];

% Logging
traj = x;
ctrl = [];
solve_ok = [];
collision_A = false;

fprintf('Running %d simulation steps ...\n', length(t));
for i = 1:length(t)
    
    % ---- reference trajectory: "carrot" that moves at ship speed ----
    x_ref = zeros(10, nmpc.N+1);
    bearing = atan2(x_goal(5)-x(5), x_goal(4)-x(4));
    dist_to_goal = max(norm(x_goal(4:5) - x(4:5)), 1);
    for k = 0:nmpc.N
        travel = x_goal(1) * (k+1) * dt;           % how far ship can go
        frac   = min(1.0, travel / dist_to_goal);   % fraction toward goal
        x_ref(4, k+1)  = x(4) + frac*(x_goal(4)-x(4));  % x
        x_ref(5, k+1)  = x(5) + frac*(x_goal(5)-x(5));  % y
        x_ref(6, k+1)  = bearing;                         % heading
        x_ref(1, k+1)  = x_goal(1);                       % surge speed
        x_ref(10, k+1) = x_goal(10);                      % RPM
    end
    
    % ---- solve NMPC (map obstacles auto-integrated, no extra circles) ----
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);
    
    % ---- progress ----
    if i == 1 || mod(i,20) == 0
        fprintf('  [t=%5.1f]  pos=(%7.1f, %6.1f)  psi=%+5.1f deg  ok=%d  dt=%.3fs\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), info.success, info.solve_time);
    end
    
    % ---- simulate with container() ----
    [xdot_sim, ~] = container(x, u_opt);
    x = x + xdot_sim * dt;

    % ---- map collision check (inpolygon) ----
    if ~isempty(map)
        in_collision = false;
        for j = 1:length(map.polygons)
            if inpolygon(x(4), x(5), map.polygons(j).X, map.polygons(j).Y)
                in_collision = true;
                break;
            end
        end
        if in_collision
            fprintf('  >> COLLISION with map polygon at t=%.1f s, pos=(%.1f, %.1f)\n', ...
                t(i), x(4), x(5));
            collision_A = true;
            break;
        end
    end
    
    % ---- log ----
    traj = [traj, x];
    ctrl = [ctrl, u_opt];
    solve_ok = [solve_ok, info.success];
    
    % ---- goal check ----
    if norm(x(4:5) - x_goal(4:5)) < 20
        fprintf('  >> GOAL REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

n_ok = sum(solve_ok);  n_tot = length(solve_ok);
fprintf('  Solver success: %d / %d (%.0f%%)\n', n_ok, n_tot, 100*n_ok/n_tot);
fprintf('  Final pos: (%.1f, %.1f),  dist to goal: %.1f m\n\n', ...
    x(4), x(5), norm(x(4:5) - x_goal(4:5)));
if collision_A
    fprintf('  Map collision: YES\n\n');
else
    fprintf('  Map collision: NO\n\n');
end

if n_ok / n_tot > 0.5
    fprintf('  >> TEST A PASSED\n\n');
else
    fprintf('  >> TEST A FAILED — stopping here.\n');
    plot(traj(5,:), traj(4,:), 'b-', 'LineWidth', 2); hold on; grid on; axis equal;
    plot(traj(5,:), traj(4,:), 'b-', 'LineWidth', 2);
    plot(0, 500, 'r*', 'MarkerSize', 15);
    xlabel('y [m]'); ylabel('x [m]'); title('A: Straight ahead');
    return;
end

%% ========================================================================
%  TEST B — Turn toward a side goal (requires steering)
%% ========================================================================
fprintf('======================================================\n');
fprintf('  TEST B: Turn to (+400, +200) relative to start, ZERO extra obstacles (map integrated)\n');
fprintf('======================================================\n');

% Reset
x = [7; 0; 0; -5800; -2800; 0; 0; 0; 0; 70];
x_goal_B = [7; 0; 0; -5800+400; -2800+200; atan2(200, 400); 0; 0; 0; 70];

nmpc.prev_X_sol = [];  nmpc.prev_U_sol = [];  % reset warm-start

traj_B = x;  ctrl_B = [];  solve_ok_B = [];
collision_B = false;

for i = 1:length(t)
    
    bearing = atan2(x_goal_B(5)-x(5), x_goal_B(4)-x(4));
    dist_to_goal = max(norm(x_goal_B(4:5) - x(4:5)), 1);
    x_ref = zeros(10, nmpc.N+1);
    for k = 0:nmpc.N
        travel = x_goal_B(1) * (k+1) * dt;
        frac   = min(1.0, travel / dist_to_goal);
        x_ref(4, k+1)  = x(4) + frac*(x_goal_B(4)-x(4));
        x_ref(5, k+1)  = x(5) + frac*(x_goal_B(5)-x(5));
        x_ref(6, k+1)  = bearing;
        x_ref(1, k+1)  = x_goal_B(1);
        x_ref(10, k+1) = x_goal_B(10);
    end
    
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);
    
    if i == 1 || mod(i,20) == 0
        fprintf('  [t=%5.1f]  pos=(%7.1f, %6.1f)  psi=%+5.1f deg  ok=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), info.success);
    end
    
    [xdot_sim, ~] = container(x, u_opt);
    x = x + xdot_sim * dt;

    % ---- map collision check (inpolygon) ----
    if ~isempty(map)
        in_collision = false;
        for j = 1:length(map.polygons)
            if inpolygon(x(4), x(5), map.polygons(j).X, map.polygons(j).Y)
                in_collision = true;
                break;
            end
        end
        if in_collision
            fprintf('  >> COLLISION with map polygon at t=%.1f s, pos=(%.1f, %.1f)\n', ...
                t(i), x(4), x(5));
            collision_B = true;
            break;
        end
    end
    
    traj_B = [traj_B, x];  ctrl_B = [ctrl_B, u_opt];  solve_ok_B = [solve_ok_B, info.success];
    
    if norm(x(4:5) - x_goal_B(4:5)) < 30
        fprintf('  >> GOAL REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

n_ok = sum(solve_ok_B);  n_tot = length(solve_ok_B);
fprintf('  Solver success: %d / %d (%.0f%%)\n', n_ok, n_tot, 100*n_ok/n_tot);
fprintf('  Final pos: (%.1f, %.1f)\n\n', x(4), x(5));
if collision_B
    fprintf('  Map collision: YES\n\n');
else
    fprintf('  Map collision: NO\n\n');
end

%% ========================================================================
%  TEST C — One obstacle blocking the path
%% ========================================================================
fprintf('======================================================\n');
fprintf('  TEST C: Straight ahead with ONE obstacle at (+250, 0) relative to start\n');
fprintf('======================================================\n');

x = [7; 0; 0; -5800; -2800; 0; 0; 0; 0; 70];
x_goal_C = [7; 0; 0; -5800+500; -2800; 0; 0; 0; 0; 70];

% Single obstacle
obs_C(1).position = [250-5800; -2800];  % relative to ship's initial position
obs_C(1).radius   = 30;

nmpc.prev_X_sol = [];  nmpc.prev_U_sol = [];
nmpc.use_obstacles = true;

traj_C = x;  ctrl_C = [];  solve_ok_C = [];
collision_C = false;

for i = 1:length(t)
    bearing = atan2(x_goal_C(5)-x(5), x_goal_C(4)-x(4));
    dist_to_goal = max(norm(x_goal_C(4:5) - x(4:5)), 1);
    x_ref = zeros(10, nmpc.N+1);
    for k = 0:nmpc.N
        travel = x_goal_C(1) * (k+1) * dt;
        frac   = min(1.0, travel / dist_to_goal);
        x_ref(4, k+1)  = x(4) + frac*(x_goal_C(4)-x(4));
        x_ref(5, k+1)  = x(5) + frac*(x_goal_C(5)-x(5));
        x_ref(6, k+1)  = bearing;
        x_ref(1, k+1)  = x_goal_C(1);
        x_ref(10, k+1) = x_goal_C(10);
    end
    
    [u_opt, ~, info] = nmpc.solve(x, x_ref, obs_C);
    
    if i == 1 || mod(i,20) == 0
        fprintf('  [t=%5.1f]  pos=(%7.1f, %6.1f)  psi=%+5.1f deg  ok=%d\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), info.success);
    end
    
    [xdot_sim, ~] = container(x, u_opt);
    x = x + xdot_sim * dt;

    % ---- map collision check (inpolygon) ----
    if ~isempty(map)
        in_collision = false;
        for j = 1:length(map.polygons)
            if inpolygon(x(4), x(5), map.polygons(j).X, map.polygons(j).Y)
                in_collision = true;
                break;
            end
        end
        if in_collision
            fprintf('  >> COLLISION with map polygon at t=%.1f s, pos=(%.1f, %.1f)\n', ...
                t(i), x(4), x(5));
            collision_C = true;
            break;
        end
    end
    
    traj_C = [traj_C, x];  ctrl_C = [ctrl_C, u_opt];  solve_ok_C = [solve_ok_C, info.success];
    
    if norm(x(4:5) - x_goal_C(4:5)) < 30
        fprintf('  >> GOAL REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

n_ok = sum(solve_ok_C);  n_tot = length(solve_ok_C);
fprintf('  Solver success: %d / %d (%.0f%%)\n', n_ok, n_tot, 100*n_ok/n_tot);
fprintf('  Final pos: (%.1f, %.1f)\n\n', x(4), x(5));
if collision_C
    fprintf('  Map collision: YES\n\n');
else
    fprintf('  Map collision: NO\n\n');
end

%% ----- STEP 5: PLOTTING ------------------------------------------------
fprintf('--- Plotting results ---\n');
figure('Position', [50 50 1400 800], 'Name', 'NMPC Tests');

% --- Test A trajectory ---
subplot(2,3,1);
plot(traj(5,:), traj(4,:), 'b-', 'LineWidth', 2); hold on; grid on; axis equal;
if ~isempty(map)
    for kk = 1:length(map.polygons)
        patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
            'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, 'EdgeColor', [0.6 0 0]);
    end
    for kk = 1:length(map.mapPoly)
        patch(map.mapPoly(kk).Y, map.mapPoly(kk).X, 'c', ...
            'FaceColor', [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
    end
end
plot(traj(5,:), traj(4,:), 'b-', 'LineWidth', 2);
plot(x_goal(5), x_goal(4), 'r*', 'MarkerSize', 15);
xlabel('y [m]'); ylabel('x [m]'); title('A: Straight ahead');

% --- Test A controls ---
subplot(2,3,4);
yyaxis left;  stairs(rad2deg(ctrl(1,:)), 'b-'); ylabel('Rudder [deg]');
yyaxis right; stairs(ctrl(2,:), 'r-');           ylabel('RPM');
xlabel('Step'); title('A: Controls'); grid on;

% --- Test B trajectory ---
subplot(2,3,2);
plot(traj_B(5,:), traj_B(4,:), 'b-', 'LineWidth', 2); hold on; grid on; axis equal;
if ~isempty(map)
    for kk = 1:length(map.polygons)
        patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
            'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, 'EdgeColor', [0.6 0 0]);
    end
    for kk = 1:length(map.mapPoly)
        patch(map.mapPoly(kk).Y, map.mapPoly(kk).X, 'c', ...
            'FaceColor', [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
    end
end
plot(traj_B(5,:), traj_B(4,:), 'b-', 'LineWidth', 2);
plot(x_goal_B(5), x_goal_B(4), 'r*', 'MarkerSize', 15);
xlabel('y [m]'); ylabel('x [m]'); title('B: Turn');

% --- Test B controls ---
subplot(2,3,5);
yyaxis left;  stairs(rad2deg(ctrl_B(1,:)), 'b-'); ylabel('Rudder [deg]');
yyaxis right; stairs(ctrl_B(2,:), 'r-');           ylabel('RPM');
xlabel('Step'); title('B: Controls'); grid on;

% --- Test C trajectory ---
subplot(2,3,3);
plot(traj_C(5,:), traj_C(4,:), 'b-', 'LineWidth', 2); hold on; grid on; axis equal;
if ~isempty(map)
    for kk = 1:length(map.polygons)
        patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
            'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, 'EdgeColor', [0.6 0 0]);
    end
    for kk = 1:length(map.mapPoly)
        patch(map.mapPoly(kk).Y, map.mapPoly(kk).X, 'c', ...
            'FaceColor', [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5]);
    end
end
plot(traj_C(5,:), traj_C(4,:), 'b-', 'LineWidth', 2);
plot(x_goal_C(5), x_goal_C(4), 'r*', 'MarkerSize', 15);
% draw obstacle
theta = linspace(0, 2*pi, 50);
obs_x = obs_C(1).position(2) + obs_C(1).radius*cos(theta);
obs_y = obs_C(1).position(1) + obs_C(1).radius*sin(theta);
fill(obs_x, obs_y, 'r', 'FaceAlpha', 0.3);
xlabel('y [m]'); ylabel('x [m]'); title('C: Obstacle avoidance');

% --- Test C controls ---
subplot(2,3,6);
yyaxis left;  stairs(rad2deg(ctrl_C(1,:)), 'b-'); ylabel('Rudder [deg]');
yyaxis right; stairs(ctrl_C(2,:), 'r-');           ylabel('RPM');
xlabel('Step'); title('C: Controls'); grid on;

sgtitle('NMPC Container Ship — Working Tests', 'FontSize', 14);

fprintf('\nDone! Check figures.\n');
fprintf('Solver stats — OK: %d,  Fail: %d\n', nmpc.solve_ok, nmpc.solve_fail);
