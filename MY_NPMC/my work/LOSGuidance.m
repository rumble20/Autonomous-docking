classdef LOSGuidance < handle
    % LOSGuidance - Line-of-Sight guidance with waypoint management
    %
    % Ported from autobargesim guidance/LOSguidance.m and guidance/guidance.m.
    % Provides heading + speed references from waypoints using LOS guidance
    % with cross-track error correction, rate-limited heading updates,
    % and automatic waypoint segment switching.
    %
    % Usage:
    %   los = LOSGuidance('K_p', 1/300, 'R_a', 50);
    %   wp_idx = los.findActiveSegment(wp_pos, state, wp_idx);
    %   [chi_d, U_d] = los.computeRef(wp_pos, wp_speed, state, wp_idx, ...
    %                                  chi_d_prev, iter);
    %
    % Author: Riccardo Legnini (ported from autobargesim)
    % Date: 2025
    
    properties
        K_p = 1/300            % LOS proportional gain (1/lookahead_distance)
        R_a = 50               % Radius of acceptance [m]
        pass_angle_threshold = 90   % Pass-angle for wp switching [deg]
        chi_rate_max = pi/60   % Max heading rate [rad/step]
        update_interval = 3    % Update heading every N steps
    end
    
    methods
        function obj = LOSGuidance(varargin)
            p = inputParser;
            addParameter(p, 'K_p', obj.K_p);
            addParameter(p, 'R_a', obj.R_a);
            addParameter(p, 'pass_angle_threshold', obj.pass_angle_threshold);
            addParameter(p, 'chi_rate_max', obj.chi_rate_max);
            addParameter(p, 'update_interval', obj.update_interval);
            parse(p, varargin{:});
            obj.K_p = p.Results.K_p;
            obj.R_a = p.Results.R_a;
            obj.pass_angle_threshold = p.Results.pass_angle_threshold;
            obj.chi_rate_max = p.Results.chi_rate_max;
            obj.update_interval = p.Results.update_interval;
        end
        
        function wp_idx = findActiveSegment(obj, wp_pos, x, wp_idx)
            % Find active waypoint segment based on vessel position
            %
            %   wp_pos: Nx2 waypoints [x y; ...]
            %   x:      state vector (at least 6 elements) [u v r x y psi ...]
            %   wp_idx: current waypoint index
            %
            % Returns updated wp_idx
            
            n_wps = size(wp_pos, 1);
            
            for i = wp_idx:(n_wps - 1)
                d_0wp = wp_pos(i+1, :)' - [x(4); x(5)];  % vec from vessel to next wp
                L_seg = wp_pos(i+1, :)' - wp_pos(i, :)';  % segment vector
                
                if obj.checkSegmentSwitch(L_seg, d_0wp)
                    wp_idx = wp_idx + 1;
                else
                    break;
                end
            end
            
            % Clamp to valid range
            wp_idx = min(wp_idx, n_wps - 1);
        end
        
        function [chi_d, U_d] = computeRef(obj, wp_pos, wp_speed, x, wp_idx, chi_d_prev, iter)
            % Compute LOS heading and speed reference
            %
            %   wp_pos:      Nx2 waypoints [x y; ...]
            %   wp_speed:    Nx1 desired speed at each waypoint [m/s]
            %   x:           state [u v r x y psi ...] (row or col)
            %   wp_idx:      current segment index
            %   chi_d_prev:  previous heading reference [rad]
            %   iter:        iteration counter
            %
            % Returns:
            %   chi_d: LOS heading reference [rad]
            %   U_d:   speed reference [m/s]
            
            n_wps = size(wp_pos, 1);
            
            % Segment direction
            if wp_idx >= n_wps
                L_seg = wp_pos(n_wps, :)' - wp_pos(n_wps-1, :)';
                wp_from = wp_pos(n_wps-1, :)';
            else
                L_seg = wp_pos(wp_idx+1, :)' - wp_pos(wp_idx, :)';
                wp_from = wp_pos(wp_idx, :)';
            end
            
            % Path tangent angle
            alpha = atan2(L_seg(2), L_seg(1));
            
            % Cross-track error (signed distance from path line)
            e = -(x(4) - wp_from(1)) * sin(alpha) + ...
                 (x(5) - wp_from(2)) * cos(alpha);
            
            % LOS heading correction (heading mode: subtract sideslip)
            U_total = sqrt(x(1)^2 + x(2)^2) + 1e-6;
            chi_r = atan2(-(obj.K_p * e), 1) - asin(NavUtils.sat(x(2)/U_total, -1, 1));
            
            chi_d_raw = NavUtils.wrap_angle_to_pmpi(alpha + chi_r);
            
            % Rate limiter
            chi_d_rate = NavUtils.wrap_angle_diff_to_pmpi(chi_d_raw, chi_d_prev);
            chi_d_rate = NavUtils.sat(chi_d_rate, -obj.chi_rate_max, obj.chi_rate_max);
            
            % Update at specified interval (smooth behavior)
            if mod(iter, obj.update_interval) == 0 || iter <= 1
                chi_d = NavUtils.wrap_angle_to_pmpi(chi_d_prev + chi_d_rate);
            else
                chi_d = chi_d_prev;
            end
            
            % Speed reference from waypoint speed schedule
            U_d = wp_speed(min(wp_idx, length(wp_speed)));
        end
        
        function [xte, along_track] = computeXTE(~, wp_pos, x, wp_idx)
            % Compute cross-track and along-track errors
            %   xte:         signed cross-track error [m]
            %   along_track: signed along-track distance to next wp [m]
            
            n_wps = size(wp_pos, 1);
            if wp_idx >= n_wps
                L_seg = wp_pos(n_wps, :)' - wp_pos(n_wps-1, :)';
                wp_from = wp_pos(n_wps-1, :)';
                wp_to   = wp_pos(n_wps, :)';
            else
                L_seg = wp_pos(wp_idx+1, :)' - wp_pos(wp_idx, :)';
                wp_from = wp_pos(wp_idx, :)';
                wp_to   = wp_pos(wp_idx+1, :)';
            end
            
            alpha = atan2(L_seg(2), L_seg(1));
            dx = x(4) - wp_from(1);
            dy = x(5) - wp_from(2);
            
            xte         = -dx*sin(alpha) + dy*cos(alpha);
            along_track = norm(wp_to - [x(4); x(5)]);
        end
    end
    
    methods (Access = private)
        function passed = checkSegmentSwitch(obj, wp_segment, d_0wp)
            % Check if vessel has passed the next waypoint
            %   wp_segment: vector from wp(i) to wp(i+1)
            %   d_0wp:      vector from vessel to wp(i+1)
            
            wp_seg_n = NavUtils.normalize_vec(wp_segment);
            d_norm   = norm(d_0wp);
            d_0wp_n  = NavUtils.normalize_vec(d_0wp);
            
            % Passed if: angle between segment and to-wp exceeds threshold
            %             OR distance within acceptance radius
            angle_check = dot(wp_seg_n, d_0wp_n) < cosd(obj.pass_angle_threshold);
            dist_check  = d_norm <= obj.R_a;
            
            passed = angle_check || dist_check;
        end
    end
end
