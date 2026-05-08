% diagnose_map_obstacles.m
% Safe diagnostic to verify map sample -> circle obstacles and half-planes
% Does not crash: wraps operations in try/catch and saves outputs.

try
    out = struct();
    % Try load map file used by run_nmpc
    map = [];
    if exist('helsinki_harbour_UPDATED.mat', 'file')
        S = load('helsinki_harbour_UPDATED.mat');
        if isfield(S, 'map'), map = S.map; end
    end
    if isempty(map) && exist('helsinki_harbour.mat', 'file')
        S = load('helsinki_harbour.mat');
        if isfield(S, 'map'), map = S.map; end
    end

    if isempty(map)
        fprintf('No harbor map file found in current folder. Exiting safely.\n');
        return;
    end

    % Parameters (match run_nmpc defaults)
    map_edge_spacing_m = 60;
    map_include_interior_samples = false;
    map_interior_spacing_m = 120;
    map_sample_radius_m = 20;
    max_map_obstacles = 8;
    max_map_halfplanes = 6;
    map_halfplane_min_edge_m = 15;
    lookahead_m = 600;
    half_width_m = 300;
    radius_m = 15;

    % Example ship position: use first waypoint segment from run_nmpc
    waypoints = [-3800, -1500; -3400, -1300; -3200, -1350; -3000, -1400; -2600, -1800; -2400, -2100; -2000, -2050];
    pos_xy = waypoints(1, :)';
    seg = waypoints(2, :)' - waypoints(1, :)';
    chi_d = atan2(seg(2), seg(1));

    % Build samples and edge set (re-implemented from run_nmpc)
    pts = buildMapSamplePoints_local(map, map_edge_spacing_m, map_include_interior_samples, map_interior_spacing_m);
    out.n_map_samples = size(pts,1);
    edge_set = buildMapHalfPlaneEdgeSet_local(map, map_halfplane_min_edge_m);
    out.n_map_edges = length(edge_set);

    obs_map = selectMapObstaclesFromSamples_local(pts, pos_xy, chi_d, max_map_obstacles, lookahead_m, half_width_m, radius_m);
    hp_local = selectMapHalfPlanesFromEdges_local(edge_set, pos_xy, chi_d, max_map_halfplanes, lookahead_m, half_width_m, [], 0, 1.0);

    fprintf('Map polygons: %d | samples: %d | edges: %d\n', length(map.polygons), out.n_map_samples, out.n_map_edges);
    fprintf('Selected circle obstacles: %d | Selected half-planes: %d\n', length(obs_map), length(hp_local));

    for k = 1:length(obs_map)
        p = obs_map(k).position(:)'; r = obs_map(k).radius;
        fprintf('  obs %d: pos=(%.1f, %.1f) r=%.1f\n', k, p(1), p(2), r);
    end
    for k = 1:length(hp_local)
        n = hp_local(k).normal(:)'; b = hp_local(k).offset;
        fprintf('  hp %d: normal=(%.3f, %.3f) offset=%.1f\n', k, n(1), n(2), b);
    end

    % Plot and save figure (safe)
    try
        h = figure('Visible','off'); hold on; axis equal; grid on;
        % plot polygons
        for kk = 1:length(map.polygons)
            px = map.polygons(kk).X(:); py = map.polygons(kk).Y(:);
            finite_idx = isfinite(px) & isfinite(py);
            patch(py(finite_idx), px(finite_idx), [0.8 0.8 0.8], 'EdgeColor','k');
        end
        % plot samples
        if ~isempty(pts)
            plot(pts(:,2), pts(:,1), 'b.', 'MarkerSize', 6);
        end
        % plot selected circle obs
        for k = 1:length(obs_map)
            p = obs_map(k).position; r = obs_map(k).radius;
            th = linspace(0,2*pi,64);
            xs = p(2) + r * cos(th); ys = p(1) + r * sin(th);
            plot(xs, ys, 'r-');
            plot(p(2), p(1), 'rx');
        end
        % plot half-planes as line segments
        for k = 1:length(hp_local)
            n = hp_local(k).normal; b = hp_local(k).offset;
            % line: n' * [x; y] = b. Solve for points along map extent
            % produce a long line segment for visualization
            t = linspace(-2000, 2000, 2);
            % pick a vector perpendicular to normal
            perp = [-n(2); n(1)];
            mid = n * b;
            pts_line = mid + perp * t;
            plot(pts_line(2,:), pts_line(1,:), 'g-','LineWidth',1);
        end
        % ship pos
        plot(pos_xy(2), pos_xy(1), 'ko', 'MarkerFaceColor','y');
        title(sprintf('Map obstacles diagnostic: obs=%d hp=%d', length(obs_map), length(hp_local)));
        saveas(h, 'diagnose_map_obstacles.png');
        close(h);
        fprintf('Saved diagnostic figure diagnose_map_obstacles.png\n');
        % Always save diagnostic data for downstream inspection
        save('diagnose_map_obstacles_data.mat', 'obs_map', 'hp_local', 'pts', 'edge_set', 'map');
        fprintf('Saved diagnostic data to diagnose_map_obstacles_data.mat\n');
    catch plotErr
        fprintf('Plotting failed: %s\n', plotErr.message);
        save('diagnose_map_obstacles_data.mat', 'obs_map', 'hp_local', 'pts', 'edge_set', 'map');
        fprintf('Saved diagnostic data to diagnose_map_obstacles_data.mat\n');
    end

catch err_main
    fprintf('Diagnostic script failed safely: %s\n', err_main.message);
end

%% Local re-implementations of functions from run_nmpc.m
function pts = buildMapSamplePoints_local(map, spacing_m, include_interior, interior_spacing_m)
    pts = zeros(0,2);
    if nargin < 2 || isempty(spacing_m), spacing_m = 100; end
    if isempty(map) || ~isfield(map, 'polygons') || isempty(map.polygons), return; end
    for kk = 1:length(map.polygons)
        px = map.polygons(kk).X(:); py = map.polygons(kk).Y(:);
        finite = isfinite(px) & isfinite(py);
        px = px(finite); py = py(finite);
        if numel(px) < 3, continue; end
        if px(1) ~= px(end) || py(1) ~= py(end)
            px(end+1) = px(1); py(end+1) = py(1);
        end
        for ii = 1:(numel(px)-1)
            p1 = [px(ii), py(ii)]; p2 = [px(ii+1), py(ii+1)]; seg = p2 - p1; seg_len = norm(seg);
            if seg_len < 1e-6, continue; end
            n_samp = max(1, ceil(seg_len / spacing_m));
            a = (0:n_samp)' / n_samp;
            seg_pts = p1 + a .* seg;
            pts = [pts; seg_pts]; %#ok<AGROW>
        end
        if include_interior
            xmin = min(px); xmax = max(px); ymin = min(py); ymax = max(py);
            xv = xmin:interior_spacing_m:xmax; yv = ymin:interior_spacing_m:ymax;
            if ~isempty(xv) && ~isempty(yv)
                [XX, YY] = meshgrid(xv, yv);
                in = inpolygon(XX(:), YY(:), px, py);
                if any(in), pts = [pts; [XX(in), YY(in)]]; end
            end
        end
    end
end

function edge_set = buildMapHalfPlaneEdgeSet_local(map, min_edge_len_m)
    edge_set = struct('p1', {}, 'p2', {}, 'mid', {}, 't', {}, 'n_left', {}, 'len', {});
    if nargin < 2 || isempty(min_edge_len_m), min_edge_len_m = 10; end
    if isempty(map) || ~isfield(map, 'polygons') || isempty(map.polygons), return; end
    out_idx = 0;
    for kk = 1:length(map.polygons)
        px = map.polygons(kk).X(:); py = map.polygons(kk).Y(:);
        finite = isfinite(px) & isfinite(py);
        px = px(finite); py = py(finite);
        if numel(px) < 3, continue; end
        if px(1) ~= px(end) || py(1) ~= py(end)
            px(end+1) = px(1); py(end+1) = py(1);
        end
        for ii = 1:(numel(px)-1)
            p1 = [px(ii); py(ii)]; p2 = [px(ii+1); py(ii+1)]; seg = p2 - p1; seg_len = norm(seg);
            if seg_len < min_edge_len_m, continue; end
            t_hat = seg / seg_len; n_left = [-t_hat(2); t_hat(1)];
            out_idx = out_idx + 1;
            edge_set(out_idx).p1 = p1; edge_set(out_idx).p2 = p2; edge_set(out_idx).mid = 0.5 * (p1 + p2);
            edge_set(out_idx).t = t_hat; edge_set(out_idx).n_left = n_left; edge_set(out_idx).len = seg_len;
        end
    end
end

function obs_local = selectMapObstaclesFromSamples_local(sample_pts, pos_xy, chi_d, max_keep, lookahead_m, half_width_m, radius_m)
    obs_local = struct('position', {}, 'radius', {});
    if isempty(sample_pts) || max_keep <= 0, return; end
    if nargin < 5 || isempty(lookahead_m), lookahead_m = 400; end
    if nargin < 6 || isempty(half_width_m), half_width_m = 150; end
    if nargin < 7 || isempty(radius_m), radius_m = 15; end
    fwd = [cos(chi_d); sin(chi_d)]; side = [-sin(chi_d); cos(chi_d)];
    rel = sample_pts - pos_xy(:)'; along = rel * fwd; lat = rel * side;
    keep = along >= -40 & along <= lookahead_m & abs(lat) <= half_width_m;
    if ~any(keep), return; end
    cand = sample_pts(keep, :);
    dc = vecnorm((cand - pos_xy(:)'), 2, 2);
    [~, ord] = sort(dc, 'ascend'); cand_ord = cand(ord, :);
    min_sep = max(2*radius_m, 25);
    cand = zeros(0,2);
    for ii = 1:size(cand_ord,1)
        p = cand_ord(ii, :);
        if isempty(cand), cand = p; else dmin = min(vecnorm(cand - p, 2, 2)); if dmin >= min_sep, cand = [cand; p]; end; end
        if size(cand,1) >= max_keep, break; end
    end
    for k = 1:size(cand,1)
        obs_local(k).position = cand(k, :)'; obs_local(k).radius = radius_m;
    end
end

function hp_local = selectMapHalfPlanesFromEdges_local(edge_set, pos_xy, chi_d, max_keep, lookahead_m, half_width_m, goal_xy, goal_exclusion_m, relax_ratio)
    hp_local = struct('normal', {}, 'offset', {});
    if isempty(edge_set) || max_keep <= 0, return; end
    if nargin < 5 || isempty(lookahead_m), lookahead_m = 400; end
    if nargin < 6 || isempty(half_width_m), half_width_m = 150; end
    if nargin < 7, goal_xy = []; end
    if nargin < 8 || isempty(goal_exclusion_m), goal_exclusion_m = 0; end
    if nargin < 9 || isempty(relax_ratio), relax_ratio = 1.0; end
    relax_ratio = max(0.35, min(1.0, relax_ratio)); pos = pos_xy(:); fwd = [cos(chi_d); sin(chi_d)]; side = [-sin(chi_d); cos(chi_d)];
    n_e = length(edge_set); keep = false(1, n_e); score = inf(1, n_e);
    for ii = 1:n_e
        mid_i = edge_set(ii).mid; rel_i = mid_i - pos; along_i = dot(rel_i, fwd); lat_i = dot(rel_i, side);
        if along_i < -50 || along_i > lookahead_m, continue; end
        if abs(lat_i) > half_width_m, continue; end
        if ~isempty(goal_xy)
            if pointToSegmentDistance_local(goal_xy(:), edge_set(ii).p1, edge_set(ii).p2) < goal_exclusion_m, continue; end
        end
        keep(ii) = true;
        d_seg = pointToSegmentDistance_local(pos, edge_set(ii).p1, edge_set(ii).p2);
        score(ii) = d_seg + 0.30 * abs(lat_i) + 0.06 * max(along_i, 0);
    end
    idx = find(keep); if isempty(idx), return; end
    [~, ord_local] = sort(score(idx), 'ascend'); idx = idx(ord_local);
    min_line_sep = max(12, 30 * relax_ratio); out_idx = 0; used_n = zeros(2, 0); used_b = zeros(0, 1);
    for kk = 1:length(idx)
        e = edge_set(idx(kk)); n_l = e.n_left;
        if dot(n_l, pos - e.p1) >= 0, n_use = n_l; else n_use = -n_l; end
        b_use = dot(n_use, e.p1);
        too_close = false;
        for jj = 1:size(used_n, 2)
            if dot(used_n(:,jj), n_use) > 0.97 && abs(used_b(jj) - b_use) < min_line_sep, too_close = true; break; end
        end
        if too_close, continue; end
        out_idx = out_idx + 1; hp_local(out_idx).normal = n_use; hp_local(out_idx).offset = b_use; used_n(:, end+1) = n_use; used_b(end+1, 1) = b_use;
        if out_idx >= max_keep, break; end
    end
end

function d = pointToSegmentDistance_local(p, a, b)
    ab = b - a; ab2 = dot(ab, ab);
    if ab2 < 1e-12, d = norm(p - a); return; end
    t = dot(p - a, ab) / ab2; t = max(0, min(1, t)); proj = a + t * ab; d = norm(p - proj);
end
