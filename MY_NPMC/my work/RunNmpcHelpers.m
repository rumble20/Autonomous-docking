classdef RunNmpcHelpers
    methods(Static)
        function wp_idx = updateWaypointIndexManaged(x, wp, wp_idx, R_accept, switch_cfg)
            % Robust waypoint progression.
            % Only decides segment handoff.
            % No heading generation. No speed shaping.

            if nargin < 5 || isempty(switch_cfg)
                switch_cfg = struct();
            end

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
                p_to = wp(wp_idx + 1, 1:2)';
                seg = p_to - p_from;
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

                if sharp_turn
                    % Relaxed thresholds for sharp turns to avoid last-moment handoff
                    advance_now = ...
                        (d_to_waypoint <= max(35, 0.50 * R_accept)) && ...
                        (proj >= 0.7) && ...
                        (xte_seg <= max(35, 0.60 * R_accept));
                elseif approaching_final
                    advance_now = ...
                        (d_to_waypoint <= max(25, 0.35 * R_accept)) && ...
                        (proj >= 0.7) && ...
                        (xte_seg <= max(25, 0.55 * R_accept));
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
                    if ~RunNmpcHelpers.getOr(switch_cfg, 'allow_multi_skip', true)
                        return;
                    end
                    continue;
                end

                return;
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

            cfg.enabled = logical(RunNmpcHelpers.getOr(cfg, 'enabled', false));
            final_xy = waypoints(end,1:2)';
            cfg.target_xy = RunNmpcHelpers.getOr(cfg, 'target_xy', final_xy);
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

            cfg.heading_deg = RunNmpcHelpers.getOr(cfg, 'heading_deg', heading_deg_default);
            cfg.prepare_last_n_segments = max(1, round(RunNmpcHelpers.getOr(cfg, 'prepare_last_n_segments', 2)));
            cfg.activate_dist_m = RunNmpcHelpers.getOr(cfg, 'activate_dist_m', 260);
            cfg.preview_heading_weight = RunNmpcHelpers.getOr(cfg, 'preview_heading_weight', 18.0);
            cfg.preview_goal_weight_gain = RunNmpcHelpers.getOr(cfg, 'preview_goal_weight_gain', 1.35);
            cfg.capture_radius_m = RunNmpcHelpers.getOr(cfg, 'capture_radius_m', 12);
            cfg.capture_speed_mps = RunNmpcHelpers.getOr(cfg, 'capture_speed_mps', 0.35);
            % If true, berth preview/mode can only activate on the final route segment.
            cfg.final_leg_only = logical(RunNmpcHelpers.getOr(cfg, 'final_leg_only', true));

            cfg.use_corridor = logical(RunNmpcHelpers.getOr(cfg, 'use_corridor', false));
            cfg.corridor_origin_xy = RunNmpcHelpers.getOr(cfg, 'corridor_origin_xy', final_xy);
            cfg.corridor_origin_xy = cfg.corridor_origin_xy(:);
            if numel(cfg.corridor_origin_xy) < 2
                cfg.corridor_origin_xy = final_xy;
            else
                cfg.corridor_origin_xy = cfg.corridor_origin_xy(1:2);
            end
            cfg.corridor_heading_deg = RunNmpcHelpers.getOr(cfg, 'corridor_heading_deg', cfg.heading_deg);
            cfg.corridor_half_width_m = RunNmpcHelpers.getOr(cfg, 'corridor_half_width_m', 12);
            cfg.corridor_along_min_m = RunNmpcHelpers.getOr(cfg, 'corridor_along_min_m', -200);
            cfg.corridor_along_max_m = RunNmpcHelpers.getOr(cfg, 'corridor_along_max_m', 20);
            % Corridor approach distance used when creating origin from berth heading
            cfg.corridor_approach_dist_m = RunNmpcHelpers.getOr(cfg, 'corridor_approach_dist_m', 300);
            % Relaxation / fallback parameters when berth heading differs strongly
            cfg.corridor_relax_max_scale = RunNmpcHelpers.getOr(cfg, 'corridor_relax_max_scale', 3.0);
            cfg.corridor_relax_along_extra_m = RunNmpcHelpers.getOr(cfg, 'corridor_relax_along_extra_m', 150);
            % If heading difference between approach vector and berth heading exceeds this (deg)
            % then derive the corridor origin from berth heading instead of approach vector.
            cfg.corridor_force_origin_by_heading_threshold_deg = RunNmpcHelpers.getOr(cfg, 'corridor_force_origin_by_heading_threshold_deg', 90);

            % If the berth heading differs strongly from the approach vector, derive
            % the corridor origin from the berth heading so the corridor frame is
            % consistent with the final parking attitude (helps large-rotation berths).
            try
                if size(waypoints,1) >= 2
                    v = waypoints(end,1:2)' - waypoints(end-1,1:2)';
                    approach_heading_deg = rad2deg(atan2(v(2), v(1)));
                else
                    approach_heading_deg = rad2deg(0);
                end
                corr_heading_rad = deg2rad(cfg.corridor_heading_deg);
                ddeg = abs(rad2deg(abs(wrapToPi(corr_heading_rad - deg2rad(approach_heading_deg)))));
                if ddeg >= cfg.corridor_force_origin_by_heading_threshold_deg
                    cfg.corridor_origin_xy = cfg.target_xy(:) + [cos(corr_heading_rad); sin(corr_heading_rad)] * cfg.corridor_approach_dist_m;
                end
            catch
                % keep provided corridor_origin_xy on error
            end

            cfg.pose_eps_xy_m = RunNmpcHelpers.getOr(cfg, 'pose_eps_xy_m', [4; 4]);
            cfg.pose_eps_xy_m = cfg.pose_eps_xy_m(:);
            if numel(cfg.pose_eps_xy_m) == 1
                cfg.pose_eps_xy_m = [cfg.pose_eps_xy_m; cfg.pose_eps_xy_m];
            else
                cfg.pose_eps_xy_m = cfg.pose_eps_xy_m(1:2);
            end

            cfg.pose_eps_psi_deg = RunNmpcHelpers.getOr(cfg, 'pose_eps_psi_deg', 3.0);
            cfg.vel_max_u_mps = RunNmpcHelpers.getOr(cfg, 'vel_max_u_mps', 0.30);
            cfg.vel_max_v_mps = RunNmpcHelpers.getOr(cfg, 'vel_max_v_mps', 0.20);
            cfg.vel_max_r_radps = RunNmpcHelpers.getOr(cfg, 'vel_max_r_radps', deg2rad(2.5));
            cfg.pose_slack_max = RunNmpcHelpers.getOr(cfg, 'pose_slack_max', 0.5);
            cfg.u_min_final_mps = RunNmpcHelpers.getOr(cfg, 'u_min_final_mps', -0.8);
            cfg.n3_max = RunNmpcHelpers.getOr(cfg, 'n3_max', 110);
            cfg.max_azimuth_split_rad = RunNmpcHelpers.getOr(cfg, 'max_azimuth_split_rad', deg2rad(10));
            cfg.max_stern_cmd_split_rpm = RunNmpcHelpers.getOr(cfg, 'max_stern_cmd_split_rpm', 8);

            % Capture heading gating: require final heading within threshold for mission capture
            cfg.capture_heading_deg = RunNmpcHelpers.getOr(cfg, 'capture_heading_deg', 6.0);
        end

        function xte = computeXTE(x, wp, wp_idx)
            % Cross-track error
            n_wps = size(wp, 1);
            idx_from = max(1, wp_idx);
            idx_to = min(idx_from + 1, n_wps);

            p1 = wp(idx_from, 1:2)';
            p2 = wp(idx_to, 1:2)';
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
                d_i = RunNmpcHelpers.pointToSegmentDistance(p, edge_set(ii).p1, edge_set(ii).p2);
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
                    if RunNmpcHelpers.pointToSegmentDistance(goal_xy(:), edge_set(ii).p1, edge_set(ii).p2) < goal_exclusion_m
                        continue;
                    end
                end

                keep(ii) = true;
                d_seg = RunNmpcHelpers.pointToSegmentDistance(pos, edge_set(ii).p1, edge_set(ii).p2);
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

                if RunNmpcHelpers.isPositionOutsideBounds(dynamic_obstacles(k).position, bounds)
                    switch lower(strtrim(boundary_policy))
                        case 'clip'
                            dynamic_obstacles(k).position(1) = min(max(dynamic_obstacles(k).position(1), bounds.xmin), bounds.xmax);
                            dynamic_obstacles(k).position(2) = min(max(dynamic_obstacles(k).position(2), bounds.ymin), bounds.ymax);
                        case 'wrap'
                            dynamic_obstacles(k).position(1) = RunNmpcHelpers.wrapLinear(dynamic_obstacles(k).position(1), bounds.xmin, bounds.xmax);
                            dynamic_obstacles(k).position(2) = RunNmpcHelpers.wrapLinear(dynamic_obstacles(k).position(2), bounds.ymin, bounds.ymax);
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
                is_active = isfield(dynamic_obstacles(k), 'active') && dynamic_obstacles(k).active;
                if ~(is_enabled && is_active)
                    continue;
                end

                if isfield(dynamic_obstacles(k), 'moving') && ~dynamic_obstacles(k).moving
                    continue;
                end

                out_idx = out_idx + 1;

                pos_k = dynamic_obstacles(k).position(1:2);
                obs_dyn(out_idx).position = pos_k(:);
                obs_dyn(out_idx).radius = dynamic_obstacles(k).radius + radius_guard_m;

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
            if ~RunNmpcHelpers.getOr(cfg, 'enabled', false)
                return;
            end

            horizon_s = max(0, RunNmpcHelpers.getOr(cfg, 'horizon_s', 0));
            n_samples = max(0, round(RunNmpcHelpers.getOr(cfg, 'n_samples', 0)));
            if horizon_s <= 0 || n_samples <= 0
                return;
            end

            radius_scale = max(1.0, RunNmpcHelpers.getOr(cfg, 'radius_scale', 1.0));
            awareness_distance_m = max(0, RunNmpcHelpers.getOr(cfg, 'awareness_distance_m', inf));
            only_when_not_moving = RunNmpcHelpers.getOr(cfg, 'only_when_not_moving', true);

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

            hull_poly = RunNmpcHelpers.buildHullPolygon(x_state(4:5), x_state(6), hull_cfg);

            if isfield(map, 'polygons') && ~isempty(map.polygons)
                [hit_poly, idx_poly] = RunNmpcHelpers.hullHitsPolygonSet(hull_poly, map.polygons);
                if hit_poly
                    hit = true;
                    zone_type = 'polygons';
                    zone_idx = idx_poly;
                    return;
                end
            end

            if isfield(map, 'mapPoly') && ~isempty(map.mapPoly)
                [hit_poly, idx_poly] = RunNmpcHelpers.hullHitsPolygonSet(hull_poly, map.mapPoly);
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
                rings = RunNmpcHelpers.splitPolygonRings(px, py);
                for rr = 1:length(rings)
                    ring = rings{rr};
                    if size(ring,1) < 3
                        continue;
                    end
                    if RunNmpcHelpers.polygonsIntersectOrContain(hull_poly, ring)
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

            local = [ hx, hy;
                      hx, -hy;
                     -hx, -hy;
                     -hx, hy];

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
                    if RunNmpcHelpers.segmentsIntersect2D(a1, a2, b1, b2)
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
            o1 = RunNmpcHelpers.orient2d(p1, p2, q1);
            o2 = RunNmpcHelpers.orient2d(p1, p2, q2);
            o3 = RunNmpcHelpers.orient2d(q1, q2, p1);
            o4 = RunNmpcHelpers.orient2d(q1, q2, p2);

            if ((o1 > eps_v && o2 < -eps_v) || (o1 < -eps_v && o2 > eps_v)) && ...
               ((o3 > eps_v && o4 < -eps_v) || (o3 < -eps_v && o4 > eps_v))
                tf = true;
                return;
            end

            tf = (abs(o1) <= eps_v && RunNmpcHelpers.onSegment2D(p1, q1, p2, eps_v)) || ...
                 (abs(o2) <= eps_v && RunNmpcHelpers.onSegment2D(p1, q2, p2, eps_v)) || ...
                 (abs(o3) <= eps_v && RunNmpcHelpers.onSegment2D(q1, p1, q2, eps_v)) || ...
                 (abs(o4) <= eps_v && RunNmpcHelpers.onSegment2D(q1, p2, q2, eps_v));
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
            obs_dyn = RunNmpcHelpers.dynamicToCircleObstacles(dynamic_obstacles);
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
                a = RunNmpcHelpers.propagateDynamicObstacles(a, dt, bounds, policy);
                b = RunNmpcHelpers.propagateDynamicObstacles(b, dt, bounds, policy);
            end
            da = RunNmpcHelpers.dynamicToCircleObstacles(a);
            db = RunNmpcHelpers.dynamicToCircleObstacles(b);
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
            if x1 <= x0
                y = double(x >= x1);
            else
                y = RunNmpcHelpers.sat01((x - x0) / (x1 - x0));
            end
        end

        function y = revRamp01(x, x_lo, x_hi)
            if x_hi <= x_lo
                y = double(x <= x_lo);
            else
                y = RunNmpcHelpers.sat01((x_hi - x) / (x_hi - x_lo));
            end
        end

        function y = lerp(a, b, lam)
            y = (1 - lam) * a + lam * b;
        end

        function ang_deg = wrapTo180Deg(ang_deg)
            ang_deg = mod(ang_deg + 180, 360) - 180;
        end

        function d = distanceToBerthCorridorEnd(pos_xy, berth_cfg)
            if ~isfield(berth_cfg, 'corridor_origin_xy') || isempty(berth_cfg.corridor_origin_xy)
                d = inf;
                return;
            end
            p = pos_xy(:) - berth_cfg.corridor_origin_xy(:);
            psi = deg2rad(berth_cfg.corridor_heading_deg);
            t_hat = [cos(psi); sin(psi)];
            d = berth_cfg.corridor_along_max_m - dot(p, t_hat);
        end

        function d = distanceToBerthCorridorEntry(pos_xy, berth_cfg)
            if ~isfield(berth_cfg, 'corridor_origin_xy') || isempty(berth_cfg.corridor_origin_xy)
                d = inf;
                return;
            end
            p = pos_xy(:) - berth_cfg.corridor_origin_xy(:);
            psi = deg2rad(berth_cfg.corridor_heading_deg);
            t_hat = [cos(psi); sin(psi)];
            d = dot(p, t_hat) - berth_cfg.corridor_along_min_m;
        end

        function turn_deg = getNextTurnAngleDeg(waypoints, seg_start_idx)
            n_wps = size(waypoints,1);
            if seg_start_idx < 1 || seg_start_idx+2 > n_wps
                turn_deg = 0;
                return;
            end
            v1 = waypoints(seg_start_idx+1,:)' - waypoints(seg_start_idx,:)';
            v2 = waypoints(seg_start_idx+2,:)' - waypoints(seg_start_idx+1,:)';
            if norm(v1)<1e-9 || norm(v2)<1e-9
                turn_deg = 0;
                return;
            end
            turn_deg = abs(RunNmpcHelpers.wrapTo180Deg(rad2deg(atan2(v2(2),v2(1)) - atan2(v1(2),v1(1)))));
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
                obs_out(k).radius = obs_in(k).radius;

                if isfield(obs_in(k), 'speed') && ~isempty(obs_in(k).speed)
                    obs_out(k).speed = obs_in(k).speed;
                end
                if isfield(obs_in(k), 'heading') && ~isempty(obs_in(k).heading)
                    obs_out(k).heading = obs_in(k).heading;
                end
            end
        end

        function plotMapBackground(map, useLightTheme)
            if isempty(map), return; end
            hold on;
            if nargin < 2 || isempty(useLightTheme)
                useLightTheme = false;
            end
            if useLightTheme
                fillColor = [0.96 0.72 0.72];
                edgeColor = [0.70 0.12 0.12];
                faceAlpha = 0.14;
            else
                fillColor = [0.90 0.20 0.20];
                edgeColor = [1.00 0.25 0.25];
                faceAlpha = 0.10;
            end
            if isfield(map, 'polygons')
                for kk = 1:length(map.polygons)
                    patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
                        'FaceColor', fillColor, 'FaceAlpha', faceAlpha, ...
                        'EdgeColor', edgeColor, 'LineWidth', 1.5);
                end
            end
        end
    end
end
