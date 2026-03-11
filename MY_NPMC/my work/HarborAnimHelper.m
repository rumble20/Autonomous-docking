classdef HarborAnimHelper < handle
    % HarborAnimHelper  Minimal map-drawing helper for animateSimResult
    %
    % animateSimResult calls harbor.plotMap() if available.
    % This tiny class wraps a map struct so the animation works
    % without the full HarborObstacles class.
    %
    % Usage:
    %   h = HarborAnimHelper(map_struct);
    %   animateSimResult(traj, wp, t, h, cfg);

    properties
        map
    end

    methods
        function obj = HarborAnimHelper(map_struct)
            obj.map = map_struct;
        end

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
    end
end
