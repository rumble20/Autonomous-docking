%% RRT^X for Non-Holonomic Naval Vehicle (Handle Class Version)
% Dynamic-obstacle ready data structures + edge-block flags (fast / low memory)
%
% NOTE:
% - This script includes a local class (RRTXData) at the end.
% - If your MATLAB version does not support local classes in scripts,
%   move the classdef into a separate file named: RRTXData.m

clear; clc; close all;

%% LOAD MAP
load('helsinki_harbour_UPDATED.mat');

num_polygons = length(map.polygons);
OBSTACLE_BBOXES = zeros(num_polygons, 4);

for j = 1:num_polygons
    OBSTACLE_BBOXES(j, :) = [min(map.polygons(j).X), max(map.polygons(j).X), ...
                             min(map.polygons(j).Y), max(map.polygons(j).Y)];
end

MAP_BOUNDS = [-6000, 0, -4000, 4000];
%% DYNAMIC OBSTACLES (local variables, passed via Copy-on-Write)
dynamic_polygons = {};
dynamic_bboxes = zeros(0, 4);

%% GLOBAL PARAMETERS
V_START = [-5000, -3000, deg2rad(45)];
V_GOAL  = [-2000, -2000, deg2rad(70)];

K_MAX = 8000;
MAX_NEIGHBORS = 50;
DELTA = 40;
GAMMA = 150;
THETA_MAX = deg2rad(15);
EPSILON = 1e-3;
GOAL_TOLERANCE = 15;
MAX_ITERATIONS = 10000;
COLLISION_SAMPLES = 10;
INCONSISTENCY_ITERATIONS = 300;

%% CREATE DATA CONTAINER (Handle Class - Passed by Reference)
data = RRTXData(K_MAX, MAX_NEIGHBORS);

%% INITIALIZATION
data.num_nodes = 1;
data.nodes(1, :) = V_GOAL;
data.parents(1) = 0;
data.G(1) = 0;
data.LMC(1) = 0;

v_bot_state = V_START;
v_bot_idx = 0;

fprintf('RRT^X Initialized:\n');
fprintf('  Goal: [%.1f, %.1f, %.1f°]\n', V_GOAL(1), V_GOAL(2), rad2deg(V_GOAL(3)));
fprintf('  Start (Robot): [%.1f, %.1f, %.1f°]\n', V_START(1), V_START(2), rad2deg(V_START(3)));
fprintf('  K_max: %d, Delta: %.1f, Theta_max: %.1f°\n', K_MAX, DELTA, rad2deg(THETA_MAX));

%% MAIN RRT^X LOOP
tic;
path_found = false;
best_path_cost = inf;

for iter = 1:MAX_ITERATIONS

    r = shrinkingBallRadius(data.num_nodes, GAMMA, 3, DELTA);

    v_rand = randomNode(MAP_BOUNDS, v_bot_state, 0.1);

    [idx_nearest, ~] = nearest(data, v_rand);

    v_new = saturate(v_rand, data.nodes(idx_nearest,:), DELTA, THETA_MAX);

    was_added = extend(data, v_new, r, map, OBSTACLE_BBOXES, dynamic_polygons, dynamic_bboxes, THETA_MAX, COLLISION_SAMPLES);

    if was_added
        new_idx = data.num_nodes;

        rewireNeighbors(data, new_idx, r, EPSILON, THETA_MAX);

        reduceInconsistency(data, v_bot_idx, EPSILON, r, THETA_MAX, INCONSISTENCY_ITERATIONS);

        dist_to_robot = sqrt((v_new(1) - v_bot_state(1))^2 + (v_new(2) - v_bot_state(2))^2);
        if dist_to_robot < GOAL_TOLERANCE
            if isCollisionFree(v_new, v_bot_state, map, OBSTACLE_BBOXES, dynamic_polygons, dynamic_bboxes, COLLISION_SAMPLES)
                if (data.LMC(new_idx) + dist_to_robot) < best_path_cost
                    v_bot_idx = new_idx;
                    best_path_cost = data.LMC(new_idx) + dist_to_robot;
                    if ~path_found
                        fprintf('  [Iter %d] Path FOUND! Cost: %.2f\n', iter, best_path_cost);
                        path_found = true;
                    else
                        fprintf('  [Iter %d] Path IMPROVED! Cost: %.2f\n', iter, best_path_cost);
                    end
                end
            end
        end
    end
   
    if mod(iter, 2000) == 0
        fprintf('  Iteration %d/%d, Nodes: %d, Queue size: %d\n', ...
                iter, MAX_ITERATIONS, data.num_nodes, sum(data.in_queue));
    end

    if data.num_nodes >= K_MAX - 100
        fprintf('  Warning: Approaching node limit, stopping.\n');
        break;
    end
end

elapsed_time = toc;

%% RESULTS
fprintf('\n========== RRT^X RESULTS ==========\n');
fprintf('Execution time: %.4f seconds\n', elapsed_time);
fprintf('Iterations completed: %d\n', iter);
fprintf('Total nodes in tree: %d\n', data.num_nodes);
fprintf('Nodes remaining in queue Q: %d\n', sum(data.in_queue));

if path_found
    fprintf('PATH FOUND!\n');
    fprintf('Total path cost: %.2f\n', best_path_cost);
    path_indices = extractPath(v_bot_idx, data.parents);
    fprintf('Path length (nodes): %d\n', length(path_indices));
else
    dists = sqrt((data.nodes(1:data.num_nodes,1) - v_bot_state(1)).^2 + ...
                 (data.nodes(1:data.num_nodes,2) - v_bot_state(2)).^2);
    [min_dist, closest_idx] = min(dists);
    fprintf('PATH NOT FOUND.\n');
    fprintf('Minimum distance from robot: %.2f (node %d)\n', min_dist, closest_idx);
    path_indices = extractPath(closest_idx, data.parents);
end

%%  OUTPUT PATH MATRIX (robot -> goal)
if path_found
    start_idx_out = v_bot_idx;
else
    start_idx_out = closest_idx; % il migliore che hai trovato
end

path_idx = extractPath(start_idx_out, data.parents);   % [start ... goal]
path_nodes = data.nodes(path_idx, :);                  % Nx3 [North East theta]

% put the position of the boat as first node:
path_nodes_robot_to_goal = [v_bot_state; path_nodes];


% Salva su MAT
save('rrtx_tree_path_robot_to_goal.mat', 'path_nodes_robot_to_goal', 'path_idx', 'path_found');
fprintf('First point (robot): N=%.1f E=%.1f\n', path_nodes_robot_to_goal(1,1), path_nodes_robot_to_goal(1,2));
fprintf('Last point  (goal) : N=%.1f E=%.1f\n', path_nodes_robot_to_goal(end,1), path_nodes_robot_to_goal(end,2));

% Se vuoi anche CSV (leggibile ovunque)
%writematrix(path_nodes_robot_to_goal, 'rrtx_tree_path_robot_to_goal.csv');
%% VISUALIZATION (OPTIMIZED WITH COLOR BINS)  ---- NED CONSISTENT ----
% Plot axes: X = East (y), Y = North (x)

% FIX BOUNDS FORMAT (IMPORTANT):
% MAP_BOUNDS = [North_min North_max East_min East_max];
% Example (adjust to your case):
% MAP_BOUNDS = [-4000, 3000, -6000, 1000];

figure('Position', [100, 100, 900, 700]);
hold on;

legend_handles = [];
legend_labels  = {};

% --- Obstacles (static) ---
% map.polygons(j).X = North, map.polygons(j).Y = East  -> plot(East, North)
h_obs = fill(map.polygons(1).Y, map.polygons(1).X, [0.7 0.7 0.7], 'EdgeColor', 'k');
for j = 2:length(map.polygons)
    fill(map.polygons(j).Y, map.polygons(j).X, [0.7 0.7 0.7], 'EdgeColor', 'k');
end
legend_handles = [legend_handles, h_obs];
legend_labels  = [legend_labels, 'Obstacles'];

% ===== TREE EDGES (FAST VERSION WITH COLOR BINS) =====
valid_costs = data.LMC(1:data.num_nodes);
if any(~isinf(valid_costs))
    valid_costs(isinf(valid_costs)) = max(valid_costs(~isinf(valid_costs))) * 1.1;
else
    valid_costs(:) = 1;
end
max_cost = max(valid_costs);
if max_cost == 0 || isnan(max_cost), max_cost = 1; end

NUM_BINS = 10;

edge_bins_X = cell(NUM_BINS, 1); % East
edge_bins_Y = cell(NUM_BINS, 1); % North
for b = 1:NUM_BINS
    edge_bins_X{b} = [];
    edge_bins_Y{b} = [];
end

for i = 2:data.num_nodes
    p_idx = data.parents(i);
    if p_idx > 0
        cost_norm = min(valid_costs(i) / max_cost, 1);
        bin_idx   = max(1, min(NUM_BINS, ceil(cost_norm * NUM_BINS)));

        % Plot: X=East=data.nodes(:,2), Y=North=data.nodes(:,1)
        edge_bins_X{bin_idx} = [edge_bins_X{bin_idx}, data.nodes(p_idx,2), data.nodes(i,2), NaN];
        edge_bins_Y{bin_idx} = [edge_bins_Y{bin_idx}, data.nodes(p_idx,1), data.nodes(i,1), NaN];
    end
end

h_tree = [];
for b = 1:NUM_BINS
    if ~isempty(edge_bins_X{b})
        cost_norm  = (b - 0.5) / NUM_BINS;
        edge_color = [cost_norm, 0, 1 - cost_norm];
        h = plot(edge_bins_X{b}, edge_bins_Y{b}, '-', 'Color', edge_color, 'LineWidth', 0.5);
        if isempty(h_tree), h_tree = h; end
    end
end
if ~isempty(h_tree)
    legend_handles = [legend_handles, h_tree];
    legend_labels  = [legend_labels, 'Tree'];
end
% =====================================================

% --- Nodes ---
h_nodes = plot(data.nodes(1:data.num_nodes, 2), data.nodes(1:data.num_nodes, 1), 'b.', 'MarkerSize', 3);
legend_handles = [legend_handles, h_nodes];
legend_labels  = [legend_labels, 'Nodes'];

% --- Best path + direction arrows ---
if path_found && ~isempty(path_indices) && length(path_indices) > 1
    path_coords = data.nodes(path_indices, 1:2); % [North East]
    h_path = plot(path_coords(:,2), path_coords(:,1), 'g-', 'LineWidth', 3); % (East,North)
    legend_handles = [legend_handles, h_path];
    legend_labels  = [legend_labels, 'Best Path'];

    for i = 1:size(path_coords, 1)-1
        mid_point = (path_coords(i, :) + path_coords(i+1, :)) / 2;      % [N E]
        direction = (path_coords(i+1, :) - path_coords(i, :));          % [dN dE]
        direction = direction / norm(direction) * 5;

        quiver(mid_point(2), mid_point(1), direction(2), direction(1), 0, ...
               'Color', [0 0.5 0], 'LineWidth', 1.5, 'MaxHeadSize', 2);
    end
end

% --- Start ---
h_start = plot(V_START(2), V_START(1), 'go', 'MarkerSize', 12, 'LineWidth', 3, 'MarkerFaceColor', 'g');
legend_handles = [legend_handles, h_start];
legend_labels  = [legend_labels, 'Start'];

% --- Goal region circle ---
theta_circle = linspace(0, 2*pi, 50);
goal_circle_e = V_GOAL(2) + GOAL_TOLERANCE * cos(theta_circle); % East
goal_circle_n = V_GOAL(1) + GOAL_TOLERANCE * sin(theta_circle); % North
h_region = plot(goal_circle_e, goal_circle_n, 'r--', 'LineWidth', 2);
legend_handles = [legend_handles, h_region];
legend_labels  = [legend_labels, 'Goal Region'];

% --- Required heading arrow (psi is heading in NED: 0 = North) ---
arrow_length = 15;
h_heading = quiver(V_GOAL(2), V_GOAL(1), ...
                   arrow_length * sin(V_GOAL(3)), ... % East component
                   arrow_length * cos(V_GOAL(3)), ... % North component
                   0, 'Color', [0.8 0 0], 'LineWidth', 2.5, 'MaxHeadSize', 1.5);
legend_handles = [legend_handles, h_heading];
legend_labels  = [legend_labels, sprintf('Required Heading (%.0f°)', rad2deg(V_GOAL(3)))];

% --- Dynamic obstacles (if any) ---
if ~isempty(dynamic_polygons)
    h_dyn = fill(dynamic_polygons{1}.Y, dynamic_polygons{1}.X, [1, 0.4, 0.4], ...
                 'EdgeColor', [0.8, 0, 0], 'LineWidth', 2);
    for j = 2:length(dynamic_polygons)
        fill(dynamic_polygons{j}.Y, dynamic_polygons{j}.X, [1, 0.4, 0.4], ...
             'EdgeColor', [0.8, 0, 0], 'LineWidth', 2);
    end
    legend_handles = [legend_handles, h_dyn];
    legend_labels  = [legend_labels, 'Dynamic Obstacle'];
end

% --- Goal marker ---
if path_found
    h_goal = plot(V_GOAL(2), V_GOAL(1), 'ro', 'MarkerSize', 12, 'LineWidth', 3, 'MarkerFaceColor', 'g');
    goal_status = 'REACHED';
else
    h_goal = plot(V_GOAL(2), V_GOAL(1), 'ro', 'MarkerSize', 12, 'LineWidth', 3, 'MarkerFaceColor', 'r');
    goal_status = 'NOT REACHED';
end
legend_handles = [legend_handles, h_goal];
legend_labels  = [legend_labels, 'Goal'];

title(sprintf('RRT^X Kinematic (\\theta_{max}=%.0f°) - %d nodes - Goal %s', ...
              rad2deg(THETA_MAX), data.num_nodes, goal_status));

xlabel('East / y');
ylabel('North / x');
axis equal; grid on;

% Axis limits (NOTE: MAP_BOUNDS = [Nmin Nmax Emin Emax])
xlim([MAP_BOUNDS(3), MAP_BOUNDS(4)]); % East range
ylim([MAP_BOUNDS(1), MAP_BOUNDS(2)]); % North range

legend(legend_handles, legend_labels, 'Location', 'best');
hold off;

%% ==================== CORE RRT^X FUNCTIONS ====================

function r = shrinkingBallRadius(n, gamma, dim, delta)
    if n < 2
        r = gamma;
    else
        r_theoretical = gamma * (log(n) / n)^(1/dim);
        r_minimum = delta * 3;
        r = max(r_theoretical, r_minimum);
    end
end

function v = randomNode(map_bounds, v_target, goal_bias)
    if rand < goal_bias
        v = v_target + [randn*5, randn*5, randn*0.3];
    else
        v = [map_bounds(1) + rand * (map_bounds(2) - map_bounds(1)), ...
             map_bounds(3) + rand * (map_bounds(4) - map_bounds(3)), ...
             -pi + rand * 2 * pi];
    end
    v(3) = atan2(sin(v(3)), cos(v(3)));
end

function [idx_nearest, dist_nearest] = nearest(data, v)
    if data.num_nodes == 0
        idx_nearest = 0;
        dist_nearest = inf;
        return;
    end
    dx = data.nodes(1:data.num_nodes, 1) - v(1);
    dy = data.nodes(1:data.num_nodes, 2) - v(2);
    distances_sq = dx.^2 + dy.^2;
    
    % ---> IL FIX: Ignora sia gli orfani attuali, sia i nodi "zombie" (LMC = Inf)
    active = ~data.is_orphaned(1:data.num_nodes) & isfinite(data.LMC(1:data.num_nodes));
    distances_sq(~active) = inf;
    
    [dist_sq_min, idx_nearest] = min(distances_sq);
    dist_nearest = sqrt(dist_sq_min);
    
    % Sicurezza: se tutti i nodi sono morti, restituisci 0
    if isinf(dist_nearest)
        idx_nearest = 0;
    end
end

function V_near = near(data, v, r)
    if data.num_nodes == 0
        V_near = [];
        return;
    end
    
    dx = data.nodes(1:data.num_nodes, 1) - v(1);
    dy = data.nodes(1:data.num_nodes, 2) - v(2);
    distances_sq = dx.^2 + dy.^2;
    
    % ---> IL FIX: Creiamo la maschera dei nodi sani
    active = ~data.is_orphaned(1:data.num_nodes) & isfinite(data.LMC(1:data.num_nodes));
    
    % Assicuriamoci che active sia un vettore colonna per fare l'AND logico con le distanze
    active = active(:); 
    
    % ---> Restituiamo SOLO i nodi dentro il raggio E che sono "vivi"
    V_near = find((distances_sq <= r^2) & active);
end

function v_new = saturate(v_rand, v_nearest, delta, theta_max)
    angle_near_to_rand = atan2(v_rand(2) - v_nearest(2), v_rand(1) - v_nearest(1));
    path_angle = angle_near_to_rand + pi;
    delta_angle = atan2(sin(path_angle - v_nearest(3)), cos(path_angle - v_nearest(3)));

    if delta_angle > theta_max
        path_angle = v_nearest(3) + theta_max;
    elseif delta_angle < -theta_max
        path_angle = v_nearest(3) - theta_max;
    end

    angle_out = path_angle + pi;
    dist_exp = sqrt((v_rand(1) - v_nearest(1))^2 + (v_rand(2) - v_nearest(2))^2);
    step = min(delta, dist_exp);

    new_x = v_nearest(1) + step * cos(angle_out);
    new_y = v_nearest(2) + step * sin(angle_out);
    heading_actual = atan2(sin(path_angle), cos(path_angle));

    v_new = [new_x, new_y, heading_actual];
end

function is_valid = checkKinematicConstraint(node_from, node_to, theta_max)
    path_angle = atan2(node_to(2) - node_from(2), node_to(1) - node_from(1));
    delta_start = abs(atan2(sin(node_from(3) - path_angle), cos(node_from(3) - path_angle)));
    delta_end = abs(atan2(sin(path_angle - node_to(3)), cos(path_angle - node_to(3))));
    tol = 1e-4;
    is_valid = (delta_start <= theta_max + tol) && (delta_end <= theta_max + tol);
end

function d = computeTrajectoryDistance(p1, p2)
    d = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
end

%% ==================== COLLISION CHECKING ====================

function is_free = isCollisionFree(q1, q2, map, bboxes, dynamic_polygons, dynamic_bboxes, num_samples)
    is_free = true;
    seg_bbox = [min(q1(1), q2(1)), max(q1(1), q2(1)), ...
                min(q1(2), q2(2)), max(q1(2), q2(2))];

    samples_generated = false;

    % Static obstacles
    for j = 1:length(map.polygons)
        if seg_bbox(1) > bboxes(j,2) || seg_bbox(2) < bboxes(j,1) || ...
           seg_bbox(3) > bboxes(j,4) || seg_bbox(4) < bboxes(j,3)
            continue;
        end
        if ~samples_generated
            t = linspace(0, 1, num_samples);
            pts_x = q1(1) + t * (q2(1) - q1(1));
            pts_y = q1(2) + t * (q2(2) - q1(2));
            samples_generated = true;
        end
        if any(inpolygon(pts_x, pts_y, map.polygons(j).X, map.polygons(j).Y))
            is_free = false;
            return;
        end
    end

    % Dynamic obstacles
    num_dynamic = size(dynamic_bboxes, 1);
    for j = 1:num_dynamic
        if seg_bbox(1) > dynamic_bboxes(j,2) || seg_bbox(2) < dynamic_bboxes(j,1) || ...
           seg_bbox(3) > dynamic_bboxes(j,4) || seg_bbox(4) < dynamic_bboxes(j,3)
            continue;
        end
        if ~samples_generated
            t = linspace(0, 1, num_samples);
            pts_x = q1(1) + t * (q2(1) - q1(1));
            pts_y = q1(2) + t * (q2(2) - q1(2));
            samples_generated = true;
        end
        if any(inpolygon(pts_x, pts_y, dynamic_polygons{j}.X, dynamic_polygons{j}.Y))
            is_free = false;
            return;
        end
    end
end

%% ==================== EXTEND FUNCTION (FAST) ====================

function was_added = extend(data, v, r, map, bboxes, dynamic_polygons, dynamic_bboxes, theta_max, collision_samples)
    was_added = false;

    V_near = near(data, v, r);
    if isempty(V_near)
        return;
    end

    [parent_idx, lmc_v] = findParent(data, v, V_near, r, map, bboxes, dynamic_polygons, dynamic_bboxes, collision_samples, theta_max);
    if parent_idx == 0
        return;
    end

    data.num_nodes = data.num_nodes + 1;
    new_idx = data.num_nodes;

    data.nodes(new_idx, :) = v;
    data.parents(new_idx) = parent_idx;
    data.G(new_idx) = inf;
    data.LMC(new_idx) = lmc_v;

    cnt = data.C_minus_T_count(parent_idx) + 1;
    if cnt <= size(data.C_minus_T, 2)
        data.C_minus_T(parent_idx, cnt) = new_idx;
        data.C_minus_T_count(parent_idx) = cnt;
    end

    maxN = size(data.N_plus_0, 2);  % cache

    for i = 1:length(V_near)
        u_idx = V_near(i);
        u = data.nodes(u_idx, :);

        % Directed edge (v -> u): stored in N_plus_0(v) and N_minus_r(u)
        if checkKinematicConstraint(v, u, theta_max)
            if isCollisionFree(v, u, map, bboxes, dynamic_polygons, dynamic_bboxes, collision_samples)
                cnt_v = data.N_plus_0_count(new_idx) + 1;
                if cnt_v <= maxN
                    data.N_plus_0(new_idx, cnt_v) = u_idx;     % no uint32()
                    % flag already false by default -> NO WRITE
                    data.N_plus_0_count(new_idx) = cnt_v;
                end

                cnt_u = data.N_minus_r_count(u_idx) + 1;
                if cnt_u <= maxN
                    data.N_minus_r(u_idx, cnt_u) = new_idx;    % no uint32()
                    % flag already false by default -> NO WRITE
                    data.N_minus_r_count(u_idx) = cnt_u;
                end
            end
        end

        % Directed edge (u -> v): stored in N_plus_r(u) and N_minus_0(v)
        if checkKinematicConstraint(u, v, theta_max)
            if isCollisionFree(u, v, map, bboxes, dynamic_polygons, dynamic_bboxes, collision_samples)
                cnt_u = data.N_plus_r_count(u_idx) + 1;
                if cnt_u <= maxN
                    data.N_plus_r(u_idx, cnt_u) = new_idx;     % no uint32()
                    % flag already false by default -> NO WRITE
                    data.N_plus_r_count(u_idx) = cnt_u;
                end

                cnt_v = data.N_minus_0_count(new_idx) + 1;
                if cnt_v <= maxN
                    data.N_minus_0(new_idx, cnt_v) = u_idx;    % no uint32()
                    % flag already false by default -> NO WRITE
                    data.N_minus_0_count(new_idx) = cnt_v;
                end
            end
        end
    end

    % Initialize running outgoing neighbors for new node (copy original list)
    n0 = data.N_plus_0_count(new_idx);
    if n0 > 0
        data.N_plus_r(new_idx, 1:n0) = data.N_plus_0(new_idx, 1:n0);
        data.is_edge_blocked_plus_r(new_idx, 1:n0) = data.is_edge_blocked_plus_0(new_idx, 1:n0);
        data.N_plus_r_count(new_idx) = n0;
    end

    was_added = true;
end

%% ==================== FIND PARENT ====================

function [best_parent, best_lmc] = findParent(data, v, U, r, map, bboxes, dynamic_polygons, dynamic_bboxes, collision_samples, theta_max)
    best_parent = 0;
    best_lmc = inf;

    for i = 1:length(U)
        u_idx = U(i);
        if data.is_orphaned(u_idx)
            continue;
        end
        u = data.nodes(u_idx, :);
        d_vu = computeTrajectoryDistance(v(1:2), u(1:2));

        if d_vu <= r && ...
           best_lmc > d_vu + data.LMC(u_idx) && ...
           checkKinematicConstraint(v, u, theta_max) && ...
           isCollisionFree(v, u, map, bboxes, dynamic_polygons, dynamic_bboxes, collision_samples)

            best_parent = u_idx;
            best_lmc = d_vu + data.LMC(u_idx);
        end
    end
end

%% ==================== REWIRE NEIGHBORS ====================  %%aggiusta il fatto che non faccia il check degli ostacoli

function rewireNeighbors(data, v_idx, r, epsilon, theta_max)
    if data.G(v_idx) - data.LMC(v_idx) <= epsilon
        return;
    end

    cullNeighbors(data, v_idx, r);
    parent_v = data.parents(v_idx);
    v = data.nodes(v_idx, :);

    % Incoming ORIGINAL neighbors (u -> v) stored in N_minus_0(v)
    for i = 1:data.N_minus_0_count(v_idx)
        u_idx = data.N_minus_0(v_idx, i);

        if u_idx == 0 || u_idx == parent_v || data.is_orphaned(u_idx)
            continue;
        end
        if data.is_edge_blocked_minus_0(v_idx, i)
            continue;
        end

        u = data.nodes(u_idx, :);
        d_uv = computeTrajectoryDistance(u(1:2), v(1:2));

        if data.LMC(u_idx) > d_uv + data.LMC(v_idx)
            if checkKinematicConstraint(u, v, theta_max)
                data.LMC(u_idx) = d_uv + data.LMC(v_idx);
                makeParentOf(data, v_idx, u_idx);

                if data.G(u_idx) - data.LMC(u_idx) > epsilon
                    verifyQueue(data, u_idx);
                end
            end
        end
    end

    % Incoming RUNNING neighbors (u -> v) stored in N_minus_r(v)
    for i = 1:data.N_minus_r_count(v_idx)
        u_idx = data.N_minus_r(v_idx, i);

        if u_idx == 0 || u_idx == parent_v || data.is_orphaned(u_idx)
            continue;
        end
        if data.is_edge_blocked_minus_r(v_idx, i)
            continue;
        end

        u = data.nodes(u_idx, :);
        d_uv = computeTrajectoryDistance(u(1:2), v(1:2));

        if data.LMC(u_idx) > d_uv + data.LMC(v_idx)
            if checkKinematicConstraint(u, v, theta_max)
                data.LMC(u_idx) = d_uv + data.LMC(v_idx);
                makeParentOf(data, v_idx, u_idx);

                if data.G(u_idx) - data.LMC(u_idx) > epsilon
                    verifyQueue(data, u_idx);
                end
            end
        end
    end
end

%% ==================== CULL NEIGHBORS ====================

function cullNeighbors(data, v_idx, r)
    parent_v = data.parents(v_idx);
    count = data.N_plus_r_count(v_idx);
    v = data.nodes(v_idx, :);

    i = count;
    while i >= 1
        u_idx = data.N_plus_r(v_idx, i);

        if u_idx == 0
            i = i - 1;
            continue;
        end

        u = data.nodes(u_idx, :);
        d_vu = computeTrajectoryDistance(v(1:2), u(1:2));

        if r < d_vu && parent_v ~= u_idx
            % swap-and-pop on N_plus_r + flags
            data.N_plus_r(v_idx, i) = data.N_plus_r(v_idx, count);
            data.is_edge_blocked_plus_r(v_idx, i) = data.is_edge_blocked_plus_r(v_idx, count);

            data.N_plus_r(v_idx, count) = 0;
            data.is_edge_blocked_plus_r(v_idx, count) = false;

            data.N_plus_r_count(v_idx) = count - 1;
            count = count - 1;

            % remove v from u's incoming running list (N_minus_r(u))
            removeFromNeighborList(data, v_idx, u_idx, 'N_minus_r');
        end

        i = i - 1;
    end
end

%% ==================== REDUCE INCONSISTENCY ====================
function reduceInconsistency(data, v_bot_idx, epsilon, r, theta_max, max_iterations)
    iter_count = 0;
    while iter_count < max_iterations
        if ~any(data.in_queue)
            break;
        end
        if v_bot_idx > 0
            [~, top_key1, top_key2] = top(data);
            v_bot_key1 = min(data.G(v_bot_idx), data.LMC(v_bot_idx));
            v_bot_key2 = data.G(v_bot_idx);
            is_key_less = (top_key1 < v_bot_key1) || ...
                          (top_key1 == v_bot_key1 && top_key2 < v_bot_key2);
            lmc_neq_g = abs(data.LMC(v_bot_idx) - data.G(v_bot_idx)) > epsilon;
            g_is_inf = isinf(data.G(v_bot_idx));
            v_bot_in_queue = data.in_queue(v_bot_idx);
            if ~(is_key_less || lmc_neq_g || g_is_inf || v_bot_in_queue)
                break;
            end
        end
        
        v_idx = pop(data);
        if v_idx == 0
            break;
        end
        
        % ---> IL FIX: Anti-NaN per la rigenerazione di RRT^X
        diffGL = data.G(v_idx) - data.LMC(v_idx);
        if (diffGL > epsilon) || isinf(data.G(v_idx))
            updateLMC(data, v_idx, r, theta_max);
            rewireNeighbors(data, v_idx, r, epsilon, theta_max);
        end
        
        data.G(v_idx) = data.LMC(v_idx);
        iter_count = iter_count + 1;
    end
end

%% ==================== UPDATE LMC ====================

function updateLMC(data, v_idx, r, theta_max)
    cullNeighbors(data, v_idx, r);

    v = data.nodes(v_idx, :);
    best_parent = 0;
    best_lmc = data.LMC(v_idx);

    % ORIGINAL outgoing neighbors
    count_0 = data.N_plus_0_count(v_idx);
    for i = 1:count_0
        u_idx = data.N_plus_0(v_idx, i);

        if u_idx == 0 || data.is_orphaned(u_idx) || data.parents(u_idx) == v_idx
            continue;
        end
        if data.is_edge_blocked_plus_0(v_idx, i)
            continue;
        end

        u = data.nodes(u_idx, :);
        d_vu = sqrt((v(1) - u(1))^2 + (v(2) - u(2))^2);

        if best_lmc > d_vu + data.LMC(u_idx)
            if checkKinematicConstraint(v, u, theta_max)
                best_parent = u_idx;
                best_lmc = d_vu + data.LMC(u_idx);
            end
        end
    end

    % RUNNING outgoing neighbors
    count_r = data.N_plus_r_count(v_idx);
    for i = 1:count_r
        u_idx = data.N_plus_r(v_idx, i);

        if u_idx == 0 || data.is_orphaned(u_idx) || data.parents(u_idx) == v_idx
            continue;
        end
        if data.is_edge_blocked_plus_r(v_idx, i)
            continue;
        end

        u = data.nodes(u_idx, :);
        d_vu = sqrt((v(1) - u(1))^2 + (v(2) - u(2))^2);

        if best_lmc > d_vu + data.LMC(u_idx)
            if checkKinematicConstraint(v, u, theta_max)
                best_parent = u_idx;
                best_lmc = d_vu + data.LMC(u_idx);
            end
        end
    end

    if best_parent > 0 && best_parent ~= data.parents(v_idx)
        data.LMC(v_idx) = best_lmc;
        makeParentOf(data, best_parent, v_idx);
    end
end

%% ==================== PRIORITY QUEUE OPERATIONS ====================

function verifyQueue(data, v_idx)
    data.keys_1(v_idx) = min(data.G(v_idx), data.LMC(v_idx));
    data.keys_2(v_idx) = data.G(v_idx);

    if ~data.in_queue(v_idx)
        data.in_queue(v_idx) = true;
        data.queue_count = data.queue_count + 1;
        data.queue_list(data.queue_count) = v_idx; % no uint32()
    end
end

function [v_idx, key1, key2] = top(data)
    if data.queue_count == 0
        v_idx = 0;
        key1 = inf;
        key2 = inf;
        return;
    end

    q = data.queue_list(1:data.queue_count);
    q_keys1 = data.keys_1(q);
    q_keys2 = data.keys_2(q);

    [min_k1, ~] = min(q_keys1);
    candidates = (q_keys1 == min_k1);
    num_candidates = sum(candidates);

    if num_candidates > 1
        q_keys2_masked = q_keys2;
        q_keys2_masked(~candidates) = inf;
        [~, local_idx] = min(q_keys2_masked);
    else
        [~, local_idx] = min(q_keys1);
    end

    v_idx = q(local_idx);
    key1 = data.keys_1(v_idx);
    key2 = data.keys_2(v_idx);
end

function v_idx = pop(data)
    if data.queue_count == 0
        v_idx = 0;
        return;
    end

    q = data.queue_list(1:data.queue_count);
    q_keys1 = data.keys_1(q);
    q_keys2 = data.keys_2(q);

    [min_k1, ~] = min(q_keys1);
    candidates = (q_keys1 == min_k1);
    num_candidates = sum(candidates);

    if num_candidates > 1
        q_keys2_masked = q_keys2;
        q_keys2_masked(~candidates) = inf;
        [~, local_idx] = min(q_keys2_masked);
    else
        [~, local_idx] = min(q_keys1);
    end

    v_idx = q(local_idx);

    data.in_queue(v_idx) = false;
    data.queue_list(local_idx) = data.queue_list(data.queue_count);
    data.queue_list(data.queue_count) = 0;
    data.queue_count = data.queue_count - 1;

    data.keys_1(v_idx) = inf;
    data.keys_2(v_idx) = inf;
end

%% ==================== TREE OPERATIONS ====================

function makeParentOf(data, new_parent_idx, child_idx)
    old_parent = data.parents(child_idx);

    if old_parent > 0
        count = data.C_minus_T_count(old_parent);
        for i = 1:count
            if data.C_minus_T(old_parent, i) == child_idx
                data.C_minus_T(old_parent, i) = data.C_minus_T(old_parent, count);
                data.C_minus_T(old_parent, count) = 0;
                data.C_minus_T_count(old_parent) = count - 1;
                break;
            end
        end
    end

    data.parents(child_idx) = new_parent_idx;

    if new_parent_idx > 0
        cnt = data.C_minus_T_count(new_parent_idx) + 1;
        if cnt <= size(data.C_minus_T, 2)
            data.C_minus_T(new_parent_idx, cnt) = child_idx;
            data.C_minus_T_count(new_parent_idx) = cnt;
        end
    end
end

function removeFromNeighborList(data, node_to_remove, list_owner, list_name)
    switch list_name
        case 'N_minus_r'
            count = data.N_minus_r_count(list_owner);
            for i = 1:count
                if data.N_minus_r(list_owner, i) == node_to_remove
                    data.N_minus_r(list_owner, i) = data.N_minus_r(list_owner, count);
                    data.is_edge_blocked_minus_r(list_owner, i) = data.is_edge_blocked_minus_r(list_owner, count);

                    data.N_minus_r(list_owner, count) = 0;
                    data.is_edge_blocked_minus_r(list_owner, count) = false;

                    data.N_minus_r_count(list_owner) = count - 1;
                    break;
                end
            end

        case 'N_plus_r'
            count = data.N_plus_r_count(list_owner);
            for i = 1:count
                if data.N_plus_r(list_owner, i) == node_to_remove
                    data.N_plus_r(list_owner, i) = data.N_plus_r(list_owner, count);
                    data.is_edge_blocked_plus_r(list_owner, i) = data.is_edge_blocked_plus_r(list_owner, count);

                    data.N_plus_r(list_owner, count) = 0;
                    data.is_edge_blocked_plus_r(list_owner, count) = false;

                    data.N_plus_r_count(list_owner) = count - 1;
                    break;
                end
            end
    end
end

%% ==================== EDGE FLAG HELPER (NEW) ====================

function setEdgeBlocked(data, v_idx, u_idx, is_blocked)
%SETEDGEBLOCKED Set blocked flag for directed edge (v -> u) wherever stored.
% Updates:
%   v outgoing lists: N_plus_0(v,:), N_plus_r(v,:)
%   u incoming lists: N_minus_0(u,:), N_minus_r(u,:)

    is_blocked = logical(is_blocked);

    % v -> u in N_plus_0(v)
    c = data.N_plus_0_count(v_idx);
    for i = 1:c
        if data.N_plus_0(v_idx, i) == u_idx
            data.is_edge_blocked_plus_0(v_idx, i) = is_blocked;
            break;
        end
    end

    % v -> u in N_plus_r(v)
    c = data.N_plus_r_count(v_idx);
    for i = 1:c
        if data.N_plus_r(v_idx, i) == u_idx
            data.is_edge_blocked_plus_r(v_idx, i) = is_blocked;
            break;
        end
    end

    % v -> u in N_minus_0(u)  (incoming to u)
    c = data.N_minus_0_count(u_idx);
    for i = 1:c
        if data.N_minus_0(u_idx, i) == v_idx
            data.is_edge_blocked_minus_0(u_idx, i) = is_blocked;
            break;
        end
    end

    % v -> u in N_minus_r(u)  (incoming to u)
    c = data.N_minus_r_count(u_idx);
    for i = 1:c
        if data.N_minus_r(u_idx, i) == v_idx
            data.is_edge_blocked_minus_r(u_idx, i) = is_blocked;
            break;
        end
    end
end

%% ==================== DYNAMIC OBSTACLES: WAVE PROPAGATION ====================

function verifyOrphan(data, v_idx)
    if data.in_queue(v_idx)
        data.in_queue(v_idx) = false;

        pos = find(data.queue_list(1:data.queue_count) == v_idx, 1);
        if ~isempty(pos)
            data.queue_list(pos) = data.queue_list(data.queue_count);
            data.queue_count = data.queue_count - 1;
        end

        data.keys_1(v_idx) = inf;
        data.keys_2(v_idx) = inf;
    end

    data.is_orphaned(v_idx) = true;
end

function propagateDescendants(data, initial_orphans)
    if isempty(initial_orphans)
        return;
    end

    bfs_queue = zeros(data.num_nodes, 1, 'uint32');
    bfs_head = 1;
    bfs_tail = 0;

    for i = 1:length(initial_orphans)
        v = initial_orphans(i);
        bfs_tail = bfs_tail + 1;
        bfs_queue(bfs_tail) = v;

        if data.in_queue(v)
            data.in_queue(v) = false;
            pos = find(data.queue_list(1:data.queue_count) == v, 1);
            if ~isempty(pos)
                data.queue_list(pos) = data.queue_list(data.queue_count);
                data.queue_count = data.queue_count - 1;
            end
            data.keys_1(v) = inf;
            data.keys_2(v) = inf;
        end
        data.is_orphaned(v) = true;
    end

    % Phase 1: collect descendants
    while bfs_head <= bfs_tail
        v = bfs_queue(bfs_head);
        bfs_head = bfs_head + 1;

        for i = 1:data.C_minus_T_count(v)
            child = data.C_minus_T(v, i);
            if child > 0 && ~data.is_orphaned(child)
                bfs_tail = bfs_tail + 1;
                bfs_queue(bfs_tail) = child;

                if data.in_queue(child)
                    data.in_queue(child) = false;
                    pos = find(data.queue_list(1:data.queue_count) == child, 1);
                    if ~isempty(pos)
                        data.queue_list(pos) = data.queue_list(data.queue_count);
                        data.queue_count = data.queue_count - 1;
                    end
                    data.keys_1(child) = inf;
                    data.keys_2(child) = inf;
                end
                data.is_orphaned(child) = true;
            end
        end
    end

    V_c_T = bfs_queue(1:bfs_tail);

    % Phase 2: poke healthy neighbors
    for k = 1:bfs_tail
        v = V_c_T(k);

        neighbors_to_poke = [data.N_plus_0(v, 1:data.N_plus_0_count(v)), ...
                             data.N_plus_r(v, 1:data.N_plus_r_count(v)), ...
                             data.parents(v)];

        for i = 1:length(neighbors_to_poke)
            u = neighbors_to_poke(i);
            if u > 0 && ~data.is_orphaned(u)
                data.G(u) = inf;
                verifyQueue(data, u);
            end
        end
    end

    % Phase 3: reset & detach
    for k = 1:bfs_tail
        v = V_c_T(k);

        data.is_orphaned(v) = false;
        data.G(v) = inf;
        data.LMC(v) = inf;

        p = data.parents(v);
        if p > 0
            count = data.C_minus_T_count(p);
            for i = 1:count
                if data.C_minus_T(p, i) == v
                    data.C_minus_T(p, i) = data.C_minus_T(p, count);
                    data.C_minus_T(p, count) = 0;
                    data.C_minus_T_count(p) = count - 1;
                    break;
                end
            end
            data.parents(v) = 0;
        end
    end
end
%% ==================== DYNAMIC OBSTACLE INJECTION ====================
function initial_orphans = addNewObstacles(data, new_poly, new_bbox, num_samples)
    initial_orphans = [];
    
    for v = 1:data.num_nodes
        v_pos = data.nodes(v, 1:2);
        
        % ---> IL FIX: Raccogliamo TUTTI gli archi uscenti (Originali + Running)
        u_list = [data.N_plus_0(v, 1:data.N_plus_0_count(v)), ...
                  data.N_plus_r(v, 1:data.N_plus_r_count(v))];
                  
        u_list = unique(u_list); % Togliamo i duplicati per non fare controlli doppi
        
        for i = 1:length(u_list)
            u = u_list(i);
            if u == 0
                continue; 
            end
            
            u_pos = data.nodes(u, 1:2);
            
            % Broad-Phase: BBox
            seg_bbox = [min(v_pos(1), u_pos(1)), max(v_pos(1), u_pos(1)), ...
                        min(v_pos(2), u_pos(2)), max(v_pos(2), u_pos(2))];
                        
            if seg_bbox(1) > new_bbox(2) || seg_bbox(2) < new_bbox(1) || ...
               seg_bbox(3) > new_bbox(4) || seg_bbox(4) < new_bbox(3)
                continue;
            end
            
            % Narrow-Phase: Interpolazione
            t = linspace(0, 1, num_samples);
            pts_x = v_pos(1) + t * (u_pos(1) - v_pos(1));
            pts_y = v_pos(2) + t * (u_pos(2) - v_pos(2));
            
            if any(inpolygon(pts_x, pts_y, new_poly.X, new_poly.Y))
                % L'arco v -> u è tranciato!
                setEdgeBlocked(data, v, u, true);
                
                % Se u era il padre di v, v ha appena perso la via verso il Goal
                if data.parents(v) == u
                    initial_orphans(end+1) = v;
                end
            end
        end
    end
    
    initial_orphans = unique(initial_orphans);
end


%% ==================== ALGORITHM 10: removeObstacle ====================
function removeOldObstacle(data, old_poly, old_bbox, r, theta_max, epsilon, map, static_bboxes, dyn_polys, dyn_bboxes, num_samples)
    for v = 1:data.num_nodes
        % ---> FIX: Prendiamo tutti e 3 gli elementi [x, y, theta], non solo 1:2
        v_node = data.nodes(v, :); 
        
        u_list = [data.N_plus_0(v, 1:data.N_plus_0_count(v)), ...
                  data.N_plus_r(v, 1:data.N_plus_r_count(v))];
        u_list = unique(u_list);
        
        for i = 1:length(u_list)
            u = u_list(i);
            if u == 0, continue; end
            
            % ---> FIX: Prendiamo tutti e 3 gli elementi [x, y, theta]
            u_node = data.nodes(u, :); 
            
            seg_bbox = [min(v_node(1), u_node(1)), max(v_node(1), u_node(1)), ...
                        min(v_node(2), u_node(2)), max(v_node(2), u_node(2))];
                        
            % BBox Check contro la VECCHIA posizione
            if seg_bbox(1) > old_bbox(2) || seg_bbox(2) < old_bbox(1) || ...
               seg_bbox(3) > old_bbox(4) || seg_bbox(4) < old_bbox(3)
                continue;
            end
            
            t = linspace(0, 1, num_samples);
            pts_x = v_node(1) + t * (u_node(1) - v_node(1));
            pts_y = v_node(2) + t * (u_node(2) - v_node(2));
            
            % Se l'arco ERA dentro il vecchio ostacolo...
            if any(inpolygon(pts_x, pts_y, old_poly.X, old_poly.Y))
                % ...controlliamo se ADESSO è libero (isCollisionFree guarda la nuova posizione)
                if isCollisionFree(v_node, u_node, map, static_bboxes, dyn_polys, dyn_bboxes, num_samples)
                    
                    % 1. Sblocchiamo l'arco!
                    setEdgeBlocked(data, v, u, false);
                    
                    % 2. Algoritmo 10: Se l'arco sbloccato migliora il costo, aggiorna e metti in coda
                    d_vu = computeTrajectoryDistance(v_node, u_node);
                    if data.LMC(u) + d_vu < data.LMC(v)
                        if checkKinematicConstraint(v_node, u_node, theta_max)
                            data.LMC(v) = data.LMC(u) + d_vu;
                            makeParentOf(data, u, v);
                            
                            if data.G(v) - data.LMC(v) > epsilon
                                verifyQueue(data, v);
                            end
                        end
                    end
                end
            end
        end
    end
end
%% ==================== PATH EXTRACTION ====================

function path_indices = extractPath(start_idx, parents)
    path_indices = start_idx;
    current = start_idx;

    max_iter = 10000;
    iter = 0;

    while current > 0 && parents(current) > 0 && iter < max_iter
        current = parents(current);
        path_indices = [path_indices; current];
        iter = iter + 1;
    end
end


