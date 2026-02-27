classdef HarborObstacles < handle
    % HarborObstacles - Manages static/dynamic obstacles + polygon maps
    %
    % CHANGES FROM PREVIOUS VERSION:
    %   - Uses NavUtils for segment distance computations
    %   - Target ships stored with full kinematic state [u v r x y psi]
    %     so SB-MPC COLAV can use them directly
    %   - Prediction of dynamic obstacle future positions
    %   - Cleaner polygon processing (NaN-safe)
    %
    % Author: Riccardo Legnini (refactored)
    % Date: 2025
    
    properties
        static_obstacles      % Struct array: .position [2x1], .radius, .name
        dynamic_obstacles     % Struct array: .position [2x1], .radius, .velocity [2x1], .name
        target_ships          % Struct array: .state [6x1] = [u v r x y psi], .radius, .name
        map                   % Polygon map structure (.polygons, .mapPoly)
        current_time = 0
    end
    
    methods
        function obj = HarborObstacles()
            obj.static_obstacles  = [];
            obj.dynamic_obstacles = [];
            obj.target_ships      = [];
            obj.map               = [];
        end
        
        %% LOAD MAP -------------------------------------------------------
        function loadPolygonMap(obj, map_struct)
            obj.map = map_struct;
            n_poly = 0;  n_bnd = 0;
            if isfield(map_struct, 'polygons'), n_poly = length(map_struct.polygons); end
            if isfield(map_struct, 'mapPoly'),  n_bnd  = length(map_struct.mapPoly);  end
            fprintf('Loaded polygon map: %d obstacle polys, %d boundary polys\n', n_poly, n_bnd);
        end
        
        %% ADD OBSTACLES ---------------------------------------------------
        function addStaticObstacle(obj, position, radius, name)
            obs.position = position(:);
            obs.radius   = radius;
            obs.velocity = [0; 0];
            obs.type     = 'static';
            obs.name     = name;
            obj.static_obstacles = [obj.static_obstacles; obs];
        end
        
        function addDynamicObstacle(obj, position, radius, velocity, name)
            obs.position = position(:);
            obs.radius   = radius;
            obs.velocity = velocity(:);
            obs.type     = 'dynamic';
            obs.name     = name;
            obj.dynamic_obstacles = [obj.dynamic_obstacles; obs];
        end
        
        function addTargetShip(obj, state_6dof, radius, name)
            % Add a target ship with full kinematic state for SB-MPC
            %   state_6dof: [u v r x y psi] (column or row)
            ts.state  = state_6dof(:);
            ts.radius = radius;
            ts.name   = name;
            obj.target_ships = [obj.target_ships; ts];
        end
        
        %% GET OBSTACLE ARRAYS FOR NMPC -----------------------------------
        function all_obs = getAllCircularObstacles(obj)
            % Return static + dynamic circular obstacles for NMPC
            all_obs = [obj.static_obstacles; obj.dynamic_obstacles];
        end
        
        function x_ts = getTargetShipStates(obj)
            % Return Mx6 matrix of target ship states for SB-MPC
            M = length(obj.target_ships);
            x_ts = zeros(M, 6);
            for i = 1:M
                x_ts(i, :) = obj.target_ships(i).state(:)';
            end
        end
        
        function obs_with_map = getAllObstaclesIncludingMap(obj, vessel_pos, K, obs_radius)
            % Get circular obstacles + K closest map-polygon points
            if nargin < 3, K = 10; end
            if nargin < 4, obs_radius = 15; end
            
            obs_with_map = obj.getAllCircularObstacles();
            if isempty(obj.map), return; end
            
            % Nearest points on each polygon to vessel
            vx = vessel_pos(1);  vy = vessel_pos(2);
            allPolys = [];
            if isfield(obj.map, 'polygons'), allPolys = obj.map.polygons; end
            
            n_poly = length(allPolys);
            nearest_pts = zeros(2, n_poly);
            min_dists   = inf(n_poly, 1);
            
            for j = 1:n_poly
                px = allPolys(j).X(:);  py = allPolys(j).Y(:);
                valid = ~isnan(px) & ~isnan(py);
                px = px(valid);  py = py(valid);
                if length(px) < 2, continue; end
                
                best_d = inf;  best_pt = [px(1); py(1)];
                nv = length(px);
                for kk = 1:nv
                    k2 = mod(kk, nv) + 1;
                    [d, pt] = NavUtils.pointToSegment(vx, vy, px(kk), py(kk), px(k2), py(k2));
                    if d < best_d
                        best_d  = d;
                        best_pt = pt;
                    end
                end
                nearest_pts(:, j) = best_pt;
                min_dists(j) = best_d;
            end
            
            % Take K closest
            [~, idx] = sort(min_dists, 'ascend');
            n_take = min(K, length(idx));
            for i = 1:n_take
                j = idx(i);
                if min_dists(j) > 500, break; end  % skip far polygons
                obs.position = nearest_pts(:, j);
                obs.radius   = obs_radius;
                obs.velocity = [0; 0];
                obs.type     = 'map_segment';
                obs.name     = sprintf('Map_%d', i);
                obs_with_map = [obs_with_map; obs];
            end
        end
        
        %% UPDATE ----------------------------------------------------------
        function updateDynamicObstacles(obj, dt)
            for i = 1:length(obj.dynamic_obstacles)
                obj.dynamic_obstacles(i).position = ...
                    obj.dynamic_obstacles(i).position + ...
                    obj.dynamic_obstacles(i).velocity * dt;
            end
            % Also propagate target ships (constant heading + speed)
            for i = 1:length(obj.target_ships)
                s = obj.target_ships(i).state;
                psi = s(6);
                s(4) = s(4) + (cos(psi)*s(1) - sin(psi)*s(2)) * dt;
                s(5) = s(5) + (sin(psi)*s(1) + cos(psi)*s(2)) * dt;
                obj.target_ships(i).state = s;
            end
            obj.current_time = obj.current_time + dt;
        end
        
        function predictDynamicPositions(obj, t_ahead)
            % Return predicted positions at t_ahead seconds from now
            % (Does NOT modify internal state)
            for i = 1:length(obj.dynamic_obstacles)
                obj.dynamic_obstacles(i).predicted_pos = ...
                    obj.dynamic_obstacles(i).position + ...
                    obj.dynamic_obstacles(i).velocity * t_ahead;
            end
        end
        
        %% COLLISION CHECK -------------------------------------------------
        function [in_collision, poly_idx] = checkMapCollision(obj, pos)
            % Check if pos [x;y] is inside any obstacle polygon
            in_collision = false;
            poly_idx = 0;
            if isempty(obj.map) || ~isfield(obj.map, 'polygons'), return; end
            
            for j = 1:length(obj.map.polygons)
                px = obj.map.polygons(j).X(:);  py = obj.map.polygons(j).Y(:);
                valid = ~isnan(px) & ~isnan(py);
                if inpolygon(pos(1), pos(2), px(valid), py(valid))
                    in_collision = true;
                    poly_idx = j;
                    return;
                end
            end
        end
        
        function [in_collision, obs_idx] = checkCircularCollision(obj, pos)
            in_collision = false;
            obs_idx = 0;
            all = obj.getAllCircularObstacles();
            for i = 1:length(all)
                if norm(pos(:) - all(i).position(:)) < all(i).radius
                    in_collision = true;
                    obs_idx = i;
                    return;
                end
            end
        end
        
        %% PLOTTING --------------------------------------------------------
        function plotMap(obj)
            if isempty(obj.map), return; end
            hold on;
            if isfield(obj.map, 'polygons')
                for kk = 1:length(obj.map.polygons)
                    patch(obj.map.polygons(kk).Y, obj.map.polygons(kk).X, 'k', ...
                        'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, ...
                        'EdgeColor', 'r', 'LineWidth', 1.5);
                end
            end
            if isfield(obj.map, 'mapPoly')
                for kk = 1:length(obj.map.mapPoly)
                    patch(obj.map.mapPoly(kk).Y, obj.map.mapPoly(kk).X, 'c', ...
                        'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2);
                end
            end
        end
        
        function plotObstacles(obj)
            hold on;
            theta = linspace(0, 2*pi, 50);
            for i = 1:length(obj.static_obstacles)
                obs = obj.static_obstacles(i);
                cx = obs.position(2) + obs.radius*cos(theta);
                cy = obs.position(1) + obs.radius*sin(theta);
                fill(cx, cy, 'r', 'FaceAlpha', 0.3);
                text(obs.position(2), obs.position(1), obs.name, ...
                    'HorizontalAlignment', 'center', 'FontSize', 9);
            end
            for i = 1:length(obj.dynamic_obstacles)
                obs = obj.dynamic_obstacles(i);
                cx = obs.position(2) + obs.radius*cos(theta);
                cy = obs.position(1) + obs.radius*sin(theta);
                fill(cx, cy, 'b', 'FaceAlpha', 0.2);
                if norm(obs.velocity) > 0.01
                    quiver(obs.position(2), obs.position(1), ...
                        obs.velocity(2)*10, obs.velocity(1)*10, 'b', 'LineWidth', 1.5);
                end
                text(obs.position(2), obs.position(1)+obs.radius+5, obs.name, ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', 'b');
            end
            for i = 1:length(obj.target_ships)
                ts = obj.target_ships(i);
                cx = ts.state(5) + ts.radius*cos(theta);
                cy = ts.state(4) + ts.radius*sin(theta);
                fill(cx, cy, [1 0.5 0], 'FaceAlpha', 0.3);
                text(ts.state(5), ts.state(4)+ts.radius+5, ts.name, ...
                    'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.8 0.4 0]);
            end
        end
        
        function plotAll(obj, vessel_pos)
            obj.plotMap();
            obj.plotObstacles();
            if nargin > 1
                plot(vessel_pos(2), vessel_pos(1), 'gp', 'MarkerSize', 12, ...
                    'MarkerFaceColor', 'g');
            end
        end
    end
end
