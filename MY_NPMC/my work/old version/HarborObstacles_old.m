classdef HarborObstacles < handle
    % HarborObstacles - Manages static and dynamic obstacles + polygon maps
    
    properties
        static_obstacles    % Array of static obstacle structs (circles)
        dynamic_obstacles   % Array of dynamic obstacle structs (circles)
        map                 % Polygon map structure
        current_time = 0    % Simulation time
    end
    
    methods
        function obj = HarborObstacles()
            obj.static_obstacles = [];
            obj.dynamic_obstacles = [];
            obj.map = [];
        end
        
        %% LOAD POLYGON MAP
        function loadPolygonMap(obj, map_struct)
            % Load polygon map structure
            %
            % map_struct should have fields:
            %   - polygons: array of structs with X, Y fields (obstacles)
            %   - mapPoly: array of structs with X, Y fields (land/boundaries)
            
            obj.map = map_struct;
            fprintf('✓ Loaded polygon map:\n');
            fprintf('  %d obstacle polygons\n', length(map_struct.polygons));
            fprintf('  %d boundary polygons\n', length(map_struct.mapPoly));
        end
        
        %% ADD OBSTACLES (Circular)
        function addStaticObstacle(obj, position, radius, name)
            % Add static circular obstacle (pier, moored vessel, etc.)
            obs.position = position(:);  % Force column vector [x; y]
            obs.radius = radius;
            obs.velocity = [0; 0];
            obs.type = 'static';
            obs.name = name;
            obj.static_obstacles = [obj.static_obstacles; obs];
        end
        
        function addDynamicObstacle(obj, position, radius, velocity, name)
            % Add dynamic circular obstacle (moving vessel)
            obs.position = position(:);      % Force column vector
            obs.radius = radius;
            obs.velocity = velocity(:);      % Force column vector
            obs.type = 'dynamic';
            obs.name = name;
            obj.dynamic_obstacles = [obj.dynamic_obstacles; obs];
        end
        
        %% CONVERT POLYGON MAP TO LINE SEGMENTS
        function line_segments = getMapLineSegments(obj)
            % Extract all line segments from polygon map
            % Returns: Nx4 matrix [x1, y1, x2, y2] for each segment
            
            line_segments = [];
            
            if isempty(obj.map)
                return;
            end
            
            % Process obstacle polygons (map.polygons)
            for kk = 1:length(obj.map.polygons)
                poly = obj.map.polygons(kk);
                X = poly.X(:);
                Y = poly.Y(:);
                
                % Create line segments from polygon edges
                for i = 1:length(X)-1
                    seg = [Y(i), X(i), Y(i+1), X(i+1)];  % [x1, y1, x2, y2]
                    line_segments = [line_segments; seg];
                end
                
                % Close polygon (last -> first)
                seg = [Y(end), X(end), Y(1), X(1)];
                line_segments = [line_segments; seg];
            end
            
            % Process boundary polygons (map.mapPoly)
            for kk = 1:length(obj.map.mapPoly)
                poly = obj.map.mapPoly(kk);
                X = poly.X(:);
                Y = poly.Y(:);
                
                for i = 1:length(X)-1
                    seg = [Y(i), X(i), Y(i+1), X(i+1)];
                    line_segments = [line_segments; seg];
                end
                
                seg = [Y(end), X(end), Y(1), X(1)];
                line_segments = [line_segments; seg];
            end
        end
        
        %% GET ALL OBSTACLES (for MPC constraint generation)
        function all_obs = getAllObstacles(obj)
            % Return all circular obstacles (static + dynamic)
            all_obs = [obj.static_obstacles; obj.dynamic_obstacles];
        end
        
        function all_obstacles_with_map = getAllObstaclesIncludingMap(obj, vessel_pos)
            % Get all obstacles INCLUDING closest map segments as virtual obstacles
            %
            % vessel_pos: [x; y] current vessel position
            %
            % Returns: Array of obstacle structs with:
            %   - position: [x; y]
            %   - radius: obstacle radius
            %   - line_segment: [x1, y1; x2, y2] (for map obstacles)
            
            % Start with circular obstacles
            all_obstacles_with_map = obj.getAllObstacles();
            
            % Add virtual obstacles from map line segments
            if ~isempty(obj.map)
                line_segs = obj.getMapLineSegments();
                
                % Find K closest line segments to vessel
                K = 10;  % Number of closest segments to consider
                
                if ~isempty(line_segs)
                    % Compute distance from vessel to each segment
                    distances = zeros(size(line_segs, 1), 1);
                    for i = 1:size(line_segs, 1)
                        seg = line_segs(i, :);
                        p1 = [seg(1); seg(2)];
                        p2 = [seg(3); seg(4)];
                        
                        % Distance from point to line segment
                        distances(i) = obj.pointToSegmentDistance(vessel_pos, p1, p2);
                    end
                    
                    % Select K closest
                    [~, idx] = sort(distances, 'ascend');
                    K_actual = min(K, length(idx));
                    closest_segs = line_segs(idx(1:K_actual), :);
                    
                    % Convert to obstacle structs
                    for i = 1:size(closest_segs, 1)
                        seg = closest_segs(i, :);
                        obs.position = [(seg(1)+seg(3))/2; (seg(2)+seg(4))/2];  % Midpoint
                        obs.radius = 5;  % Small radius for line segments
                        obs.velocity = [0; 0];
                        obs.type = 'map_segment';
                        obs.name = sprintf('Map_%d', i);
                        obs.line_segment = [seg(1), seg(2); seg(3), seg(4)];  % Store actual segment
                        
                        all_obstacles_with_map = [all_obstacles_with_map; obs];
                    end
                end
            end
        end
        
        %% UPDATE DYNAMIC OBSTACLES
        function updateDynamicObstacles(obj, dt)
            % Update positions of moving obstacles
            for i = 1:length(obj.dynamic_obstacles)
                obj.dynamic_obstacles(i).position = ...
                    obj.dynamic_obstacles(i).position + ...
                    obj.dynamic_obstacles(i).velocity * dt;
            end
            obj.current_time = obj.current_time + dt;
        end
        
        %% PLOTTING
        function plotMap(obj)
            % Plot polygon map (obstacles and boundaries)
            
            if isempty(obj.map)
                return;
            end
            
            hold on;
            
            % Plot obstacle polygons (red with transparency)
            for kk = 1:length(obj.map.polygons)
                patch(obj.map.polygons(kk).Y, obj.map.polygons(kk).X, 'k', ...
                    'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, ...
                    'EdgeColor', 'r', 'LineWidth', 1.5);
            end
            
            % Plot boundary polygons (grey)
            for kk = 1:length(obj.map.mapPoly)
                patch(obj.map.mapPoly(kk).Y, obj.map.mapPoly(kk).X, 'c', ...
                    'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2);
            end
        end
        
        function plotObstacles(obj)
            % Visualize circular obstacles only (not map)
            hold on;
            
            % Plot static obstacles (red circles)
            for i = 1:length(obj.static_obstacles)
                obs = obj.static_obstacles(i);
                center = obs.position(:)';  % Row vector for viscircles
                viscircles(center, obs.radius, 'Color', 'r', 'LineWidth', 2);
                text(obs.position(1), obs.position(2), obs.name, ...
                    'HorizontalAlignment', 'center', 'FontSize', 10);
            end
            
            % Plot dynamic obstacles (blue dashed circles)
            for i = 1:length(obj.dynamic_obstacles)
                obs = obj.dynamic_obstacles(i);
                center = obs.position(:)';
                viscircles(center, obs.radius, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 2);
                
                % Show velocity vector
                if norm(obs.velocity) > 0.01
                    quiver(obs.position(1), obs.position(2), ...
                        obs.velocity(1)*10, obs.velocity(2)*10, 'b', ...
                        'LineWidth', 1.5, 'MaxHeadSize', 2);
                end
                
                text(obs.position(1), obs.position(2) + obs.radius + 5, obs.name, ...
                    'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'b');
            end
        end
        
        function plotAll(obj, vessel_pos)
            % Plot map + obstacles + show closest segments
            obj.plotMap();
            obj.plotObstacles();
            
            % Optional: visualize closest map segments
            if nargin > 1 && ~isempty(obj.map)
                line_segs = obj.getMapLineSegments();
                if ~isempty(line_segs)
                    distances = zeros(size(line_segs, 1), 1);
                    for i = 1:size(line_segs, 1)
                        seg = line_segs(i, :);
                        p1 = [seg(1); seg(2)];
                        p2 = [seg(3); seg(4)];
                        distances(i) = obj.pointToSegmentDistance(vessel_pos, p1, p2);
                    end
                    
                    [~, idx] = sort(distances, 'ascend');
                    % Highlight 5 closest segments
                    for i = 1:min(5, length(idx))
                        seg = line_segs(idx(i), :);
                        plot([seg(1), seg(3)], [seg(2), seg(4)], ...
                            'g-', 'LineWidth', 3);
                    end
                end
            end
        end
        
        %% HELPER FUNCTIONS
        function dist = pointToSegmentDistance(~, point, p1, p2)
            % Compute minimum distance from point to line segment [p1, p2]
            %
            % point: [x; y]
            % p1, p2: [x; y] endpoints of segment
            
            v = p2 - p1;  % Segment vector
            w = point - p1;  % Point vector from p1
            
            c1 = dot(w, v);
            if c1 <= 0  % Before p1
                dist = norm(point - p1);
                return;
            end
            
            c2 = dot(v, v);
            if c1 >= c2  % After p2
                dist = norm(point - p2);
                return;
            end
            
            % Projection on segment
            t = c1 / c2;
            projection = p1 + t * v;
            dist = norm(point - projection);
        end
    end
end