classdef NavUtils
    % NavUtils - Navigation utility functions (static class)
    %
    % Ported from autobargesim toolbox (colav/utils.m, guidance/utils.m)
    % with extensions for map validation.
    %
    % Usage:
    %   wrapped = NavUtils.wrap_angle_to_pmpi(angle);
    %   y = NavUtils.sat(x, x_min, x_max);
    %   [d, pt] = NavUtils.pointToSegment(px, py, x1, y1, x2, y2);
    %
    % Author: Riccardo Legnini (refactored from autobargesim)
    % Date: 2025
    
    methods (Static)
        
        function wrapped = wrap_angle_to_pmpi(angle)
            % Wrap angle to [-pi, pi]
            wrapped = NavUtils.wrap_min_max(angle, -pi, pi);
        end
        
        function wrapped = wrap_angle_diff_to_pmpi(a1, a2)
            % Wrap (a1 - a2) to [-pi, pi]
            diff = NavUtils.wrap_angle_to_pmpi(a1) - NavUtils.wrap_angle_to_pmpi(a2);
            wrapped = NavUtils.wrap_min_max(diff, -pi, pi);
        end
        
        function wrapped = wrap_min_max(x, x_min, x_max)
            % Wrap x into [x_min, x_max) using modular arithmetic
            wrapped = x_min + mod((x - x_min), (x_max - x_min));
        end
        
        function nv = normalize_vec(v)
            % Normalize a vector; return unchanged if near-zero
            n = norm(v);
            if n < 1e-10
                nv = v;
            else
                nv = v / n;
            end
        end
        
        function y = sat(x, x_min, x_max)
            % Clamp x to [x_min, x_max]
            y = min(x_max, max(x_min, x));
        end
        
        function n = getNumSamples(dt, T)
            % Number of discrete samples in horizon T with step dt
            n = round(T / dt);
        end
        
        function [d, nearest_pt] = pointToSegment(px, py, x1, y1, x2, y2)
            % Minimum distance from point (px,py) to line segment (x1,y1)-(x2,y2)
            % Returns distance d and the closest point on segment.
            dx = x2 - x1;
            dy = y2 - y1;
            len2 = dx^2 + dy^2;
            if len2 < 1e-12
                nearest_pt = [x1; y1];
                d = sqrt((px - x1)^2 + (py - y1)^2);
                return;
            end
            t = max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / len2));
            nx = x1 + t*dx;
            ny = y1 + t*dy;
            nearest_pt = [nx; ny];
            d = sqrt((px - nx)^2 + (py - ny)^2);
        end
        
        function navigable = isNavigable(point, map)
            % Check if point [x;y] is in navigable water (not inside any polygon)
            navigable = true;
            if isempty(map), return; end
            
            if isfield(map, 'polygons')
                for j = 1:length(map.polygons)
                    px = map.polygons(j).X(:);
                    py = map.polygons(j).Y(:);
                    valid = ~isnan(px) & ~isnan(py);
                    if inpolygon(point(1), point(2), px(valid), py(valid))
                        navigable = false;
                        return;
                    end
                end
            end
        end
        
        function valid = validatePath(waypoints, map, resolution)
            % Check if a waypoint path stays in navigable water
            % waypoints: Nx2 matrix [x1 y1; x2 y2; ...]
            % resolution: interpolation step [m] (default 10)
            if nargin < 3, resolution = 10; end
            valid = true;
            for i = 1:size(waypoints, 1)-1
                p1 = waypoints(i, :);
                p2 = waypoints(i+1, :);
                d = norm(p2 - p1);
                n_pts = max(2, ceil(d / resolution));
                for j = 0:n_pts
                    frac = j / n_pts;
                    pt = p1 + frac * (p2 - p1);
                    if ~NavUtils.isNavigable(pt(:), map)
                        valid = false;
                        return;
                    end
                end
            end
        end
        
        function inMap = isInsideMapBoundary(point, map)
            % Check if point is inside at least one mapPoly boundary
            inMap = false;
            if isempty(map) || ~isfield(map, 'mapPoly'), return; end
            for j = 1:length(map.mapPoly)
                px = map.mapPoly(j).X(:);
                py = map.mapPoly(j).Y(:);
                valid = ~isnan(px) & ~isnan(py);
                if inpolygon(point(1), point(2), px(valid), py(valid))
                    inMap = true;
                    return;
                end
            end
        end
        
    end
end
