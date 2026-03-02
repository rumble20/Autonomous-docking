classdef PerformanceMetrics
    % PerformanceMetrics - Cross-track error, heading error, and path metrics
    %
    % Based on autobargesim controlClass.XTECalc.
    % Provides static methods for computing navigation performance metrics.
    %
    % Usage:
    %   [xte, psi_err] = PerformanceMetrics.computeXTE(state, psi_d, wp_pos, wp_idx);
    %   stats = PerformanceMetrics.computeStats(log);
    %
    % Author: Riccardo Legnini (ported from autobargesim)
    % Date: 2025
    
    methods (Static)
        
        function [xte, psi_err] = computeXTE(state, psi_d, wp_pos, wp_idx)
            % Compute cross-track error and heading error
            %
            %   state:  10x1 or Nx1 state vector (needs indices 4,5,6)
            %   psi_d:  desired heading [rad]
            %   wp_pos: Mx2 waypoints [x y; ...]
            %   wp_idx: current waypoint segment index
            %
            % Returns:
            %   xte:     absolute cross-track error [m]
            %   psi_err: heading error [rad]
            
            xp  = state(4);
            yp  = state(5);
            psi = state(6);
            
            n_wps = size(wp_pos, 1);
            if wp_idx >= n_wps
                wp_vec = wp_pos(n_wps, :) - wp_pos(n_wps-1, :);
                wp_from = wp_pos(n_wps-1, :);
            else
                wp_vec  = wp_pos(wp_idx+1, :) - wp_pos(wp_idx, :);
                wp_from = wp_pos(wp_idx, :);
            end
            
            % Cross-track error via cross product
            pos_vec    = [xp - wp_from(1), yp - wp_from(2)];
            cross_prod = wp_vec(1)*pos_vec(2) - wp_vec(2)*pos_vec(1);
            xte = abs(cross_prod) / max(norm(wp_vec), 1e-6);
            
            % Heading error (wrapped to [0, pi])
            psi_err = abs(NavUtils.wrap_angle_diff_to_pmpi(psi, psi_d));
        end
        
        function [nom_time, nom_dist, act_time, act_dist] = computePerformance(...
                wp_pos, traj_x, traj_y, u_d, dt, n_steps)
            % Compute overall path performance metrics
            %
            %   wp_pos:  Mx2 waypoints
            %   traj_x:  trajectory x positions
            %   traj_y:  trajectory y positions
            %   u_d:     desired speed [m/s]
            %   dt:      time step [s]
            %   n_steps: number of completed steps
            
            % Actual
            act_time = n_steps * dt;
            act_dist = 0;
            for i = 2:length(traj_x)
                act_dist = act_dist + norm([traj_x(i)-traj_x(i-1), traj_y(i)-traj_y(i-1)]);
            end
            
            % Nominal (straight-line)
            nom_dist = 0;
            for i = 2:size(wp_pos, 1)
                nom_dist = nom_dist + norm(wp_pos(i,:) - wp_pos(i-1,:));
            end
            nom_time = nom_dist / max(u_d, 0.1);
        end
        
        function stats = computeStats(xte_log, psi_err_log, solve_ok_log, time_log)
            % Compute summary statistics from logged data
            %
            %   xte_log:      Nx1 cross-track errors [m]
            %   psi_err_log:  Nx1 heading errors [rad]
            %   solve_ok_log: Nx1 solver success flags
            %   time_log:     Nx1 solve times [s]
            
            stats.xte_mean    = mean(xte_log);
            stats.xte_max     = max(xte_log);
            stats.xte_rms     = sqrt(mean(xte_log.^2));
            stats.psi_err_mean = mean(rad2deg(psi_err_log));
            stats.psi_err_max  = max(rad2deg(psi_err_log));
            stats.solver_rate  = 100 * sum(solve_ok_log) / max(length(solve_ok_log), 1);
            stats.solve_time_mean = mean(time_log);
            stats.solve_time_max  = max(time_log);
        end
        
        function printStats(stats)
            % Print summary statistics
            fprintf('\n=== Performance Metrics ===\n');
            fprintf('  XTE  :  mean = %6.2f m,  max = %6.2f m,  RMS = %6.2f m\n', ...
                stats.xte_mean, stats.xte_max, stats.xte_rms);
            fprintf('  Heading error:  mean = %5.2f deg,  max = %5.2f deg\n', ...
                stats.psi_err_mean, stats.psi_err_max);
            fprintf('  Solver:  %.1f%% success rate,  mean dt = %.3f s,  max dt = %.3f s\n', ...
                stats.solver_rate, stats.solve_time_mean, stats.solve_time_max);
            fprintf('===========================\n\n');
        end
        
    end
end
