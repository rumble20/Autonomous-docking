classdef SBMPC_COLAV < handle
    % SBMPC_COLAV - Scenario-Based MPC for COLREGs-compliant collision avoidance
    %
    % Ported from autobargesim colav/sbmpc.m and colav/colav.m.
    % Evaluates discrete course/speed modifications (scenarios) and selects
    % the one minimizing combined collision risk + maneuvering cost + COLREGs cost.
    %
    % Architecture:
    %   LOS guidance --> SB-MPC COLAV --> NMPC (inner loop)
    %   chi_d, U_d --> chi_c, U_c    --> u_opt
    %
    % Usage:
    %   colav = SBMPC_COLAV(60, 2, 'D_CLOSE_', 300);
    %   [chi_c, U_c, chi_m, U_m] = colav.run(x, chi_d, U_d, chi_m_last, U_m_last, x_ts);
    %
    % Author: Riccardo Legnini (ported from autobargesim)
    % Date: 2025
    
    properties
        T                 % Prediction horizon [s]
        dt                % Prediction sample time [s]
        T_chi     = 3     % Course time constant [s]
        T_U       = 3     % Speed time constant [s]
        
        % Tuning parameters (defaults for harbour scale)
        D_CLOSE   = 300   % Close distance threshold [m]
        D_SAFE    = 150   % Safety distance [m]
        D_INIT    = 600   % Initial detection distance [m]
        KAPPA     = 7.0   % COLREGs violation penalty
        
        % Collision cost
        K_COLL    = 0.5   % Collision cost weight
        P         = 1.0   % Time exponent
        Q         = 4.0   % Distance exponent
        
        % Maneuvering cost
        K_P       = 2.5   % Speed reduction penalty
        K_CHI_SB  = 0.5   % Starboard course penalty
        K_CHI_P   = 8.5   % Port course penalty
        K_DP      = 2.0   % Speed change penalty
        K_DCHI_SB = 0     % Starboard course change penalty
        K_DCHI_P  = 0     % Port course change penalty
        
        % COLREGs angles
        PHI_AH    = deg2rad(68.5)    % Ahead sector half-angle
        PHI_OT    = deg2rad(68.5)    % Overtaking sector half-angle
        PHI_HO    = deg2rad(22.5)    % Head-on sector half-angle
        PHI_CR    = deg2rad(68.5)    % Crossing sector half-angle
        
        % Scenario candidates
        Chi_ca = deg2rad([-90 -75 -60 -45 -30 -15 0 15 30 45 60 75 90])
        U_ca   = [0 0.5 1.0]
    end
    
    methods
        function obj = SBMPC_COLAV(T, dt, varargin)
            obj.T  = T;
            obj.dt = dt;
            
            % Parse optional name-value pairs
            p = inputParser;
            addParameter(p, 'T_chi',     obj.T_chi);
            addParameter(p, 'T_U',       obj.T_U);
            addParameter(p, 'D_CLOSE',   obj.D_CLOSE);
            addParameter(p, 'D_SAFE',    obj.D_SAFE);
            addParameter(p, 'D_INIT',    obj.D_INIT);
            addParameter(p, 'KAPPA',     obj.KAPPA);
            addParameter(p, 'K_COLL',    obj.K_COLL);
            addParameter(p, 'P',         obj.P);
            addParameter(p, 'Q',         obj.Q);
            addParameter(p, 'K_P',       obj.K_P);
            addParameter(p, 'K_CHI_SB',  obj.K_CHI_SB);
            addParameter(p, 'K_CHI_P',   obj.K_CHI_P);
            addParameter(p, 'K_DP',      obj.K_DP);
            addParameter(p, 'Chi_ca',    obj.Chi_ca);
            addParameter(p, 'U_ca',      obj.U_ca);
            parse(p, varargin{:});
            
            fnames = fieldnames(p.Results);
            for i = 1:length(fnames)
                obj.(fnames{i}) = p.Results.(fnames{i});
            end
        end
        
        function [chi_c, U_c, chi_m, U_m] = run(obj, x, chi_d, U_d, chi_m_last, U_m_last, x_ts)
            % Run SB-MPC collision avoidance evaluation
            %
            %   x:           own ship state [u v r x y psi ...] (row or col)
            %   chi_d:       desired course from guidance [rad]
            %   U_d:         desired speed from guidance [m/s]
            %   chi_m_last:  previous course modification [rad]
            %   U_m_last:    previous speed modification factor
            %   x_ts:        target ships, each row [u v r x y psi]
            %                (Mx6 matrix, M = number of targets)
            %
            % Returns:
            %   chi_c:  modified course command [rad]
            %   U_c:    modified speed command [m/s]
            %   chi_m:  selected course modification [rad]
            %   U_m:    selected speed modification factor
            
            % Own ship kinematic state: [x, y, chi, U]
            os_U = sqrt(x(1)^2 + x(2)^2);
            os_chi = atan2(sin(x(6))*x(1) + cos(x(6))*x(2), ...
                           cos(x(6))*x(1) - sin(x(6))*x(2));  % course over ground
            % Simplified: use heading as course (small sideslip assumption)
            os_chi = x(6);
            os_state = [x(4), x(5), os_chi, os_U];
            
            n_samples = NavUtils.getNumSamples(obj.dt, obj.T);
            n_samples = max(n_samples, 2);
            
            cost_k_min = inf;
            chi_m = 0;
            U_m   = 1.0;
            
            num_ts = size(x_ts, 1);
            
            for i = 1:length(obj.Chi_ca)
                for j = 1:length(obj.U_ca)
                    cost_i_max = -1;
                    
                    % Own ship trajectory under this scenario
                    chi_cmd = chi_d + obj.Chi_ca(i);
                    U_cmd   = U_d * obj.U_ca(j);
                    os_traj = obj.calcVesselTraj(os_state, [chi_cmd, U_cmd], n_samples);
                    
                    % Evaluate against each target ship
                    for idx = 1:num_ts
                        ts_U   = sqrt(x_ts(idx,1)^2 + x_ts(idx,2)^2);
                        ts_chi = x_ts(idx, 6);  % heading as course
                        ts_state = [x_ts(idx,4), x_ts(idx,5), ts_chi, ts_U];
                        
                        % Target ship: constant course & speed
                        ts_traj = obj.calcVesselTraj(ts_state, [ts_chi, ts_U], n_samples);
                        
                        % Collision cost
                        collision_cost = obj.calcCostCollision(os_traj, ts_traj, n_samples);
                        
                        % COLREGs cost
                        colregs_cost = obj.calcCostCOLREGs(os_traj, ts_traj, n_samples);
                        
                        cost_i = max(colregs_cost + collision_cost);
                        if cost_i > cost_i_max
                            cost_i_max = cost_i;
                        end
                    end
                    
                    if num_ts == 0
                        cost_i_max = 0;  % No targets: zero obstacle cost
                    end
                    
                    % Maneuvering cost
                    F = obj.calcCostManeuvering(chi_d, U_d, obj.Chi_ca(i), obj.U_ca(j), ...
                                                chi_m_last, U_m_last);
                    
                    tot_cost = cost_i_max + F;
                    
                    if tot_cost < cost_k_min
                        cost_k_min = tot_cost;
                        chi_m = obj.Chi_ca(i);
                        U_m   = obj.U_ca(j);
                    end
                end
            end
            
            chi_c = NavUtils.wrap_angle_to_pmpi(chi_d + chi_m);
            U_c   = U_d * U_m;
        end
    end
    
    %% ====================================================================
    %  PRIVATE METHODS
    %% ====================================================================
    methods (Access = private)
        
        function traj = calcVesselTraj(obj, x0, u, n_samples)
            % Propagate kinematic vessel trajectory using RK4
            %   x0: [x, y, chi, U]  (1x4)
            %   u:  [chi_cmd, U_cmd] (1x2)
            %   Returns: n_samples x 4 trajectory
            
            traj = zeros(n_samples, 4);
            traj(1, :) = x0;
            for k = 2:n_samples
                traj(k, :) = obj.stepRK4(traj(k-1, :), u);
            end
        end
        
        function x_new = stepRK4(obj, x, u)
            % RK4 integration of kinematic model
            k1 = obj.kinModel(x, u);
            k2 = obj.kinModel(x + k1*obj.dt/2, u);
            k3 = obj.kinModel(x + k2*obj.dt/2, u);
            k4 = obj.kinModel(x + k3*obj.dt, u);
            x_new = x + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        
        function xdot = kinModel(obj, x, u)
            % 4-state kinematic model: [x, y, chi, U]
            %   u = [chi_cmd, U_cmd]
            xdot = zeros(1, 4);
            xdot(1) = x(4) * cos(x(3));     % x_dot
            xdot(2) = x(4) * sin(x(3));     % y_dot
            xdot(3) = NavUtils.wrap_angle_diff_to_pmpi(u(1), x(3)) / obj.T_chi;  % chi_dot
            xdot(4) = (u(2) - x(4)) / obj.T_U;                                   % U_dot
        end
        
        function cost_N = calcCostCollision(obj, os_traj, ts_traj, N)
            % Distance-based collision cost (time-weighted)
            cost_N = zeros(1, N);
            t = 0;
            for k = 1:N
                t = t + obj.dt;
                d = [ts_traj(k,1) - os_traj(k,1), ts_traj(k,2) - os_traj(k,2)];
                d_norm = norm(d);
                
                if d_norm < obj.D_SAFE && d_norm > 1e-3
                    R = (1 / abs(t)^obj.P) * (obj.D_SAFE / d_norm)^obj.Q;
                    
                    v_o = [ts_traj(k,4)*cos(ts_traj(k,3)), ts_traj(k,4)*sin(ts_traj(k,3))];
                    v_s = [os_traj(k,4)*cos(os_traj(k,3)), os_traj(k,4)*sin(os_traj(k,3))];
                    C = obj.K_COLL * norm(v_s - v_o)^2;
                    
                    cost_N(k) = C * R;
                end
            end
        end
        
        function cost_N = calcCostCOLREGs(obj, os_traj, ts_traj, N)
            % COLREGs rule compliance cost
            cost_N = zeros(1, N);
            
            for k = 1:N
                % Extract info
                d = [ts_traj(k,1) - os_traj(k,1), ts_traj(k,2) - os_traj(k,2)];
                d_norm = norm(d);
                
                v_o = [ts_traj(k,4)*cos(ts_traj(k,3)), ts_traj(k,4)*sin(ts_traj(k,3))];
                v_s = [os_traj(k,4)*cos(os_traj(k,3)), os_traj(k,4)*sin(os_traj(k,3))];
                
                % Close enough to apply rules?
                CL = d_norm <= obj.D_CLOSE;
                if ~CL, continue; end
                
                % Overtaking: target faster, same general direction
                v_s_n = norm(v_s); v_o_n = norm(v_o);
                if v_s_n < 1e-4 || v_o_n < 1e-4, continue; end
                
                OT = (dot(v_s, v_o) > cos(obj.PHI_OT)*v_s_n*v_o_n) && (v_s_n < v_o_n);
                
                % Starboard side check
                os_psi = os_traj(k, 3);
                phi_rel = NavUtils.wrap_angle_diff_to_pmpi(atan2(d(2), d(1)), os_psi);
                SB = phi_rel <= 0;  % Target is on starboard
                
                % Head-on
                los_dir = d / max(d_norm, 1e-6);
                HO = (dot(v_s, v_o) < -cos(obj.PHI_HO)*v_s_n*v_o_n) && ...
                     (dot(v_s, los_dir) > cos(obj.PHI_AH)*v_s_n);
                
                % Crossing
                CR = dot(v_s, v_o) < cos(obj.PHI_CR)*v_s_n*v_o_n;
                
                % COLREGs violation flag
                mu = (CL && SB && HO) || (CL && SB && CR && ~OT);
                
                cost_N(k) = obj.KAPPA * mu;
            end
        end
        
        function F = calcCostManeuvering(obj, chi_d, U_d, chi_m, U_m, chi_m_last, U_m_last)
            % Maneuvering cost: penalizes deviations from desired course/speed
            
            % Course deviation penalty (port vs starboard asymmetric)
            if chi_m < 0
                K_CHI = obj.K_CHI_SB;  % Starboard turn (preferred by COLREGs)
            elseif chi_m > 0
                K_CHI = obj.K_CHI_P;   % Port turn (penalized more)
            else
                K_CHI = 0;
            end
            
            % Course change rate penalty
            d_chi = chi_m - chi_m_last;
            if d_chi > 0
                K_DCHI = obj.K_DCHI_P;
            elseif d_chi < 0
                K_DCHI = obj.K_DCHI_SB;
            else
                K_DCHI = 0;
            end
            
            F = obj.K_P * (1 - U_m) + K_CHI * chi_m^2 + ...
                obj.K_DP * abs(U_m_last - U_m) + K_DCHI * d_chi^2;
        end
        
    end
end
