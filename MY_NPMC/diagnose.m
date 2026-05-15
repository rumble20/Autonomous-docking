% This script prints diagnostics at t_interest = 146. Adjust t_interest as needed.
t_interest = 146;

k = find(t_sim >= t_interest, 1, 'first');
if isempty(k)
    error('t_sim does not reach %.1f s.', t_interest);
end

xk = traj(:, k);

lambda_tight = lambda_tight_log(k);
lambda_stop = lambda_stop_log(k);
lambda_turn = lambda_turn_log(k);
lambda_berth = lambda_berth_log(k);
lambda_yield = lambda_yield_log(k);
lambda_return = lambda_return_log(k);
lambda_total = lambda_total_log(k);

u_cap = u_cap_log(k);
u_floor = sched_cfg.soft_speed_floor_ratio * u_cap;
soft_speed_cap_weight = lerpLocal(sched_cfg.soft_speed_weight_far, sched_cfg.soft_speed_weight_near, lambda_total);
soft_speed_floor_weight = lerpLocal(sched_cfg.soft_speed_floor_weight_far, sched_cfg.soft_speed_floor_weight_near, lambda_total);
path_along_weight = lerpLocal(path_cost_cfg.W_along, sched_cfg.w_along_min, lambda_yield);

u_min_sched = lerpLocal(sched_cfg.u_min_forward_far, sched_cfg.u_min_forward_near, lambda_yield);

% Active segment geometry
n_wps = size(waypoints, 1);
last_seg_idx = max(1, n_wps - 1);
seg_start_idx = min(max(1, wp_idx_log(k)), last_seg_idx);
seg_end_idx = min(seg_start_idx + 1, n_wps);
wp_start = waypoints(seg_start_idx, 1:2)';
wp_end = waypoints(seg_end_idx, 1:2)';

seg_vec = wp_end - wp_start;
seg_len = max(1e-9, norm(seg_vec));
t_hat = seg_vec / seg_len;
n_hat = [-t_hat(2); t_hat(1)];

chi_seg = atan2(seg_vec(2), seg_vec(1));
chi_next = chi_seg;
turn_deg = 0.0;
if seg_end_idx < n_wps
    wp_next = waypoints(seg_end_idx + 1, 1:2)';
    chi_next = atan2(wp_next(2) - wp_end(2), wp_next(1) - wp_end(1));
    turn_deg = abs(rad2deg(wrapToPiLocal(chi_next - chi_seg)));
end

err_vec = xk(4:5) - wp_start;
xte_now = err_vec(1) * n_hat(1) + err_vec(2) * n_hat(2);
along_now = err_vec(1) * t_hat(1) + err_vec(2) * t_hat(2);
dist_to_end = norm(xk(4:5) - wp_end);

geom_caution = max([lambda_tight, lambda_turn, lambda_berth]);
lambda_path_return = max(geom_caution, lambda_return);
path_tube_half_width = lerpLocal(sched_cfg.tube_far_m, sched_cfg.tube_near_m, lambda_path_return);
path_xte_weight = lerpLocal(sched_cfg.xte_weight_far, sched_cfg.xte_weight_near, lambda_path_return);
path_heading_weight = max([ ...
    1.5, ...
    0.35 * lambda_turn  * sched_cfg.heading_weight_turn, ...
    0.10 * lambda_berth * sched_cfg.heading_weight_berth, ...
    lambda_return       * sched_cfg.heading_weight_return]);

path_heading_weight_eff = path_heading_weight;
terminal_heading_weight_eff = NaN;
goal_heading_enable_eff = NaN;
goal_heading_rad_eff = NaN;
map_barrier_weight_eff = NaN;
u_min_forward_eff = NaN;
if exist('path_heading_weight_log', 'var') && numel(path_heading_weight_log) >= k
    path_heading_weight_eff = path_heading_weight_log(k);
end
if exist('terminal_heading_weight_log', 'var') && numel(terminal_heading_weight_log) >= k
    terminal_heading_weight_eff = terminal_heading_weight_log(k);
end
if exist('goal_heading_enable_log', 'var') && numel(goal_heading_enable_log) >= k
    goal_heading_enable_eff = goal_heading_enable_log(k);
end
if exist('goal_heading_rad_log', 'var') && numel(goal_heading_rad_log) >= k
    goal_heading_rad_eff = goal_heading_rad_log(k);
end
if exist('map_barrier_weight_log', 'var') && numel(map_barrier_weight_log) >= k
    map_barrier_weight_eff = map_barrier_weight_log(k);
end
if exist('u_min_forward_log', 'var') && numel(u_min_forward_log) >= k
    u_min_forward_eff = u_min_forward_log(k);
end

psi_ref = psi_ref_log(k);
heading_err_deg = rad2deg(heading_err_log(k));

fprintf('t=%.1f s  pos=(%.1f, %.1f)  psi=%.1f deg\n', ...
    t_sim(k), xk(4), xk(5), rad2deg(xk(6)));
fprintf('u=%.2f v=%.2f r=%.3f  u_cap=%.2f  u_floor=%.2f  u_min=%.2f\n', ...
    xk(1), xk(2), xk(3), u_cap, u_floor, u_min_sched);
if isfinite(u_min_forward_eff)
    fprintf('u_min_forward_eff=%.2f\n', u_min_forward_eff);
end
fprintf('psi_ref=%.1f deg  heading_err=%.1f deg\n', rad2deg(psi_ref), heading_err_deg);
if isfinite(goal_heading_rad_eff)
    fprintf('goal_heading_enable=%.0f  goal_heading_rad=%.1f deg\n', ...
        goal_heading_enable_eff, rad2deg(goal_heading_rad_eff));
end
fprintf('seg=%d->%d  chi_seg=%.1f deg  chi_next=%.1f deg  turn=%.1f deg\n', ...
    seg_start_idx, seg_end_idx, rad2deg(chi_seg), rad2deg(chi_next), turn_deg);
fprintf('along=%.1f  xte=%.1f  dist_to_end=%.1f\n', along_now, xte_now, dist_to_end);

fprintf('lambda_tight=%.2f  lambda_stop=%.2f  lambda_turn=%.2f  lambda_berth=%.2f\n', ...
    lambda_tight, lambda_stop, lambda_turn, lambda_berth);
fprintf('lambda_yield=%.2f  lambda_return=%.2f  lambda_total=%.2f\n', ...
    lambda_yield, lambda_return, lambda_total);
fprintf('d_edge=%.1f  d_stop=%.1f  xte_log=%.1f\n', ...
    d_edge_log(k), d_stop_log(k), xte_log(k));

fprintf('weights: speed_cap=%.2f  speed_floor=%.2f  along=%.2f\n', ...
    soft_speed_cap_weight, soft_speed_floor_weight, path_along_weight);
fprintf('path: tube=%.1f  xte_w=%.1f  heading_w=%.1f\n', ...
    path_tube_half_width, path_xte_weight, path_heading_weight_eff);
if isfinite(terminal_heading_weight_eff)
    fprintf('terminal_heading_weight=%.1f\n', terminal_heading_weight_eff);
end

map_barrier_weight = sched_cfg.map_barrier_w_base + ...
    max(lambda_tight, lambda_return) * (sched_cfg.map_barrier_w_near - sched_cfg.map_barrier_w_base);
if isfinite(map_barrier_weight_eff)
    fprintf('map_barrier_weight=%.2f (logged)\n', map_barrier_weight_eff);
else
    fprintf('map_barrier_weight=%.2f\n', map_barrier_weight);
end

sharp_turn_active = (turn_deg >= route_follow_cfg.sharp_turn_deg) && ...
    (dist_to_end <= route_follow_cfg.sharp_turn_heading_enable_dist_m);
if sharp_turn_active
    fprintf('sharp_turn_active=1  sharp_turn_heading_weight=%.1f\n', ...
        route_follow_cfg.sharp_turn_heading_weight);
else
    fprintf('sharp_turn_active=0\n');
end

% Windowed heading divergence (last 20 s)
dt = mean(diff(t_sim));
if ~isfinite(dt) || dt <= 0
    dt = 1.0;
end
k_start = max(1, k - round(20 / dt));
heading_err_deg_window = rad2deg(heading_err_log(k_start:k));
[max_err, max_idx_rel] = max(abs(heading_err_deg_window));
k_peak = k_start + max_idx_rel - 1;
fprintf('peak_heading_err=%.1f deg at t=%.1f s\n', max_err, t_sim(k_peak));
fprintf('  lambdas at peak: tight=%.2f stop=%.2f turn=%.2f berth=%.2f\n', ...
    lambda_tight_log(k_peak), lambda_stop_log(k_peak), lambda_turn_log(k_peak), lambda_berth_log(k_peak));

function y = lerpLocal(a, b, t)
    y = (1 - t) * a + t * b;
end

function ang = wrapToPiLocal(x)
    ang = mod(x + pi, 2*pi) - pi;
end