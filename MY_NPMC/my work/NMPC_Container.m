classdef NMPC_Container < handle
    % NMPC_Container - Nonlinear MPC for container ship using CasADi nlpsol
    %
    % KEY CHANGES FROM PREVIOUS VERSION (Opti-based):
    %   1) Uses CasADi nlpsol (build-once, reuse) instead of casadi.Opti
    %   2) Fixed obstacle slots — no rebuild when obstacle count changes
    %   3) Supports current/environment effects (V_c, beta_c)
    %   4) Supports 'full' or 'simplified' dynamics model
    %   5) Clean parameter passing via NLP parameter vector
    %   6) Warm start by shifting previous solution
    %
    % CasADi dynamics matches container.m (line-by-line verified).
    % Unused obstacle slots are placed at (1e8, 1e8) with radius 0.
    %
    % Architecture:
    %   LOS Guidance -> SB-MPC COLAV -> NMPC (this) -> container() plant
    %
    % Author: Riccardo Legnini
    % Date: 2025 (refactored from Opti version, inspired by autobargesim)
    
    properties
        % Horizon
        N               % Prediction horizon steps
        dt              % Sample time [s]
        
        % Cost weights
        Q               % State error weight  (10x10)
        R               % Control effort weight (2x2)
        Q_N             % Terminal cost weight (10x10)
        R_rate          % Control rate-of-change weight (2x2)
        
        % Dimensions
        nx = 12         % States:   [u v r x y psi p phi delta n]
        nu = 4            % Controls: [delta_c  n_c]
        
        % Limits (from container.m)
        delta_max_rad = deg2rad(35)
        n_max_rpm     = 160
        n_min_rpm     = 10
        
        % Obstacle settings
        use_obstacles        = false
        max_obs              = 15       % Fixed obstacle slots (never changes)
        r_safety             = 30       % Safety margin [m] (was 10 — too small)
        penalty_slack        = 5000     % Slack penalty in cost (was 500 — too weak vs tracking)
        
        % Map integration
        map                  = []
        use_map_obstacles    = false
        max_map_obs          = 10       % Map polygons used per solve
        map_obs_radius       = 15       % Avoidance radius [m]
        
        % Environment (current effects)
        V_c   = 0           % Current speed [m/s]
        beta_c = 0          % Current direction [rad] (NED)
        
        % Model type: 'full' or 'simplified'
        model_type = 'full'
        
        % --- nlpsol persistent solver ---
        solver               % CasADi nlpsol object
        solver_built = false
        
        % NLP structure
        lbx_vec              % Lower bounds on decision variables
        ubx_vec              % Upper bounds on decision variables
        lbg_vec              % Lower bounds on constraints
        ubg_vec              % Upper bounds on constraints
        np_total             % Total number of parameters
        
        % Warm start
        prev_sol = []        % Previous NLP solution (full vector)
        
        % Statistics
        solve_ok   = 0
        solve_fail = 0
    end
    
    %% ====================================================================
    methods
        
        %% CONSTRUCTOR ====================================================
        function obj = NMPC_Container(config)
            obj.N  = config.N;
            obj.dt = config.dt;
            obj.Q  = config.Q;
            obj.R  = config.R;
            obj.Q_N = config.Q * 2;
            % R_rate: penalize control rate-of-change (4x4 for [n_b, n_s, a_b, a_s])
            obj.R_rate = diag([0.001, 0.001, 10.0, 10.0]);
            
            if isfield(config, 'use_obstacles'),    obj.use_obstacles    = config.use_obstacles;    end
            if isfield(config, 'max_obs'),          obj.max_obs          = config.max_obs;          end
            if isfield(config, 'use_map_obstacles'), obj.use_map_obstacles = config.use_map_obstacles; end
            if isfield(config, 'max_map_obs'),      obj.max_map_obs      = config.max_map_obs;      end
            if isfield(config, 'V_c'),              obj.V_c              = config.V_c;              end
            if isfield(config, 'beta_c'),           obj.beta_c           = config.beta_c;           end
            if isfield(config, 'model_type'),       obj.model_type       = config.model_type;       end
            if isfield(config, 'R_rate'),           obj.R_rate           = config.R_rate;           end
            if isfield(config, 'r_safety'),         obj.r_safety         = config.r_safety;         end
            if isfield(config, 'penalty_slack'),    obj.penalty_slack    = config.penalty_slack;    end
            
            fprintf('NMPC_Container initialized (N=%d, dt=%.2f, model=%s, max_obs=%d)\n', ...
                obj.N, obj.dt, obj.model_type, obj.max_obs);
        end
        
        %% MAP INTEGRATION ================================================
        function setMap(obj, map_struct)
            obj.map = map_struct;
            obj.use_map_obstacles = true;
            fprintf('NMPC map integration enabled (%d polygons)\n', length(map_struct.polygons));
        end
        
        function setEnvironment(obj, V_c, beta_c)
            obj.V_c = V_c;
            obj.beta_c = beta_c;
        end
        
        %% MAP POLYGON -> CIRCULAR OBSTACLES ==============================
        function obs_from_map = mapPolygonsToObstacles(obj, vessel_xy)
            obs_from_map = [];
            if isempty(obj.map) || ~isfield(obj.map, 'polygons'), return; end
            
            proximity_threshold = 300;
            vx = vessel_xy(1);
            vy = vessel_xy(2);
            
            n_poly = length(obj.map.polygons);
            nearest_pts = zeros(2, n_poly);
            min_dists   = inf(n_poly, 1);
            
            for j = 1:n_poly
                px = obj.map.polygons(j).X(:);
                py = obj.map.polygons(j).Y(:);
                valid = ~isnan(px) & ~isnan(py);
                px = px(valid);  py = py(valid);
                if length(px) < 2, continue; end
                
                best_d = inf;
                best_pt = [px(1); py(1)];
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
            
            close_idx = find(min_dists < proximity_threshold);
            if isempty(close_idx), return; end
            
            [~, so] = sort(min_dists(close_idx), 'ascend');
            close_idx = close_idx(so);
            n_take = min(obj.max_map_obs, length(close_idx));
            close_idx = close_idx(1:n_take);
            
            obs_from_map = repmat(struct('position', [0;0], 'radius', 0, ...
                'velocity', [0;0], 'type', '', 'name', ''), n_take, 1);
            for k = 1:n_take
                obs_from_map(k).position = nearest_pts(:, close_idx(k));
                obs_from_map(k).radius   = obj.map_obs_radius;
                obs_from_map(k).velocity = [0; 0];
                obs_from_map(k).type     = 'map_polygon';
                obs_from_map(k).name     = sprintf('MapObs_%d', k);
            end
        end
        
        %% BUILD SOLVER (nlpsol — called ONCE) ============================
        function buildSolver(obj)
            import casadi.*
            
            % Clear any previous solution for fresh solve
            obj.prev_sol = [];
            
            N_h = obj.N;
            nx  = obj.nx;
            nu  = obj.nu;
            n_obs = obj.max_obs;
            
            X     = SX.sym('X', nx, N_h+1);
            U     = SX.sym('U', nu, N_h);
            slack = SX.sym('slack', n_obs, N_h+1);
            
            P_x0      = SX.sym('P_x0', nx, 1);
            P_xref    = SX.sym('P_xref', nx, N_h+1);
            P_uref    = SX.sym('P_uref', nu, N_h);
            P_obs_pos = SX.sym('P_obs_pos', 2, n_obs);
            P_obs_rad = SX.sym('P_obs_rad', n_obs, 1);
            P_env     = SX.sym('P_env', 2, 1);   % [V_c; beta_c]
            
            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_obs_pos(:), P_obs_rad, P_env);
            obj.np_total = size(P_all, 1);
            
            %% ---- Cost function ----
            % NMPC tracking architecture:
            %   - Track psi_ref (wrapped), r_ref and surge speed u_ref
            %   - Add mild x/y path-position pull (from x_ref rollout)
            %   - Keep control effort + control rate penalties
            
            Q_u   = obj.Q(1,1);   % surge speed gain
            Q_r   = obj.Q(3,3);   % yaw-rate gain
            Q_x   = obj.Q(4,4);   % x-position gain
            Q_y   = obj.Q(5,5);   % y-position gain
            Q_psi = obj.Q(6,6);   % heading gain
            
            J = 0;
            for k = 1:N_h
                % Wrapped heading error: atan2(sin(dpsi), cos(dpsi))
                dpsi = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));
                J = J + Q_psi * psi_err^2;

                % Speed + position errors
                u_err = X(1,k) - P_xref(1,k);
                x_err = X(4,k) - P_xref(4,k);
                y_err = X(5,k) - P_xref(5,k);
                J = J + Q_u * u_err^2 + Q_x * x_err^2 + Q_y * y_err^2;
                
                % Yaw rate error
                r_err = X(3,k) - P_xref(3,k);
                J = J + Q_r * r_err^2;
                
                % Control effort
                du = U(:,k) - P_uref(:,k);
                J  = J + du' * obj.R * du;
            end
            % Control rate-of-change
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J  = J + dU' * obj.R_rate * dU;
            end
            % Terminal cost (heading + yaw rate only)
            dpsi_N = X(6,N_h+1) - P_xref(6,N_h+1);
            psi_err_N = atan2(sin(dpsi_N), cos(dpsi_N));
            J = J + 2*Q_psi * psi_err_N^2 + 2*Q_r * (X(3,N_h+1) - P_xref(3,N_h+1))^2;
            J = J + 2*Q_u * (X(1,N_h+1) - P_xref(1,N_h+1))^2;
            J = J + 2*Q_x * (X(4,N_h+1) - P_xref(4,N_h+1))^2 + 2*Q_y * (X(5,N_h+1) - P_xref(5,N_h+1))^2;
            % Slack penalty
            J = J + obj.penalty_slack * sumsqr(slack);
            
            %% ---- Constraints ----
            g = [];
            
            % Initial condition: X(:,1) == x0
            g = vertcat(g, X(:,1) - P_x0);
            
            % Dynamics: X(:,k+1) == X(:,k) + f(X(:,k), U(:,k)) * dt
            for k = 1:N_h
                X_k = X(:,k);
                U_k = U(:,k);
                
                k1 = obj.dynamics_full_casadi(X_k, U_k, P_env);
                k2 = obj.dynamics_full_casadi(X_k + k1*obj.dt/2, U_k, P_env);
                k3 = obj.dynamics_full_casadi(X_k + k2*obj.dt/2, U_k, P_env);
                k4 = obj.dynamics_full_casadi(X_k + k3*obj.dt, U_k, P_env);
                                
                xdot_rk4 = (k1 + 2*k2 + 2*k3 + k4) / 6;
                g = vertcat(g, X(:,k+1) - (X_k + xdot_rk4 * obj.dt));
            end
            
            % Obstacle avoidance: dist_sq >= (r_obs + r_safety - slack)^2
            for k = 1:(N_h+1)
                for j = 1:n_obs
                    dx = X(4,k) - P_obs_pos(1,j);
                    dy = X(5,k) - P_obs_pos(2,j);
                    dist_sq = dx^2 + dy^2;
                    min_d = P_obs_rad(j) + obj.r_safety - slack(j,k);
                    g = vertcat(g, dist_sq - min_d^2);
                end
            end
            
            %% ---- Flatten decision variables ----
            OPT = vertcat(X(:), U(:), slack(:));
            n_vars = size(OPT, 1);
            
            %% ---- Bounds on decision variables ----
            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);
            
            % State bounds: X (indices 1..nx*(N_h+1))
            for k = 1:(N_h+1)
                base = (k-1)*nx;
                lbx(base + 1) = 0.1;       % u >= 0.1 m/s (surge)
                lbx(base + 2) = -1;        % v unconstrained (sway)
                lbx(base + 3) = -2*pi;     % r unconstrained (yaw rate)
                % x, y, psi unbounded by default
                lbx(base + 7) = -inf;      % p unbounded (roll rate)
                lbx(base + 8) = -pi/4;     % phi >= -45 deg (roll)
                ubx(base + 8) = pi/4;      % phi <= 45 deg
                lbx(base + 9) = 1;         % n_bow >= 1 RPM
                lbx(base + 10) = 1;        % n_stern >= 1 RPM
                % alpha (angles) unbounded [-2pi, 2pi]
            end
            
            % Control bounds: U (indices nx*(N_h+1)+1 .. nx*(N_h+1)+nu*N_h)
            u_offset = nx*(N_h+1);
            for k = 1:N_h
                base = u_offset + (k-1)*nu;
                lbx(base + 1) = -120;  % n_c_bow:  [-120, 120] RPM
                ubx(base + 1) =  120;
                lbx(base + 2) = -120;  % n_c_stern: [-120, 120] RPM
                ubx(base + 2) =  120;
                lbx(base + 3) = -pi;   % alpha_c_bow: [-pi, pi] rad
                ubx(base + 3) =  pi;
                lbx(base + 4) = -pi;   % alpha_c_stern: [-pi, pi] rad
                ubx(base + 4) =  pi;
            end
            
            % Slack bounds: >= 0
            s_offset = u_offset + nu*N_h;
            lbx(s_offset+1 : end) = 0;
            
            %% ---- Bounds on constraints ----
            % Equality: initial condition (nx) + dynamics (nx*N_h)
            n_eq   = nx + nx*N_h;
            % Inequalities: dist_sq >= min_d^2, i.e., dist_sq - min_d^2 >= 0
            n_ineq = n_obs * (N_h+1);
            
            lbg = [zeros(n_eq, 1); zeros(n_ineq, 1)];
            ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];
            
            %% ---- Create nlpsol ----
            nlp_prob = struct('f', J, 'x', OPT, 'g', g, 'p', P_all);
            
            opts = struct;
            opts.ipopt.print_level          = 0;
            opts.print_time                 = 0;
            opts.ipopt.max_iter             = 500;
            opts.ipopt.tol                  = 1e-3;
            opts.ipopt.acceptable_tol       = 1e-2;
            opts.ipopt.acceptable_iter      = 5;
            opts.ipopt.warm_start_init_point = 'yes';
            opts.ipopt.mu_strategy          = 'adaptive';
            opts.ipopt.nlp_scaling_method   = 'gradient-based';
            opts.ipopt.sb                   = 'yes';
            
            obj.solver  = nlpsol('nmpc_solver', 'ipopt', nlp_prob, opts);
            obj.lbx_vec = lbx;
            obj.ubx_vec = ubx;
            obj.lbg_vec = lbg;
            obj.ubg_vec = ubg;
            
            obj.solver_built = true;
            fprintf('nlpsol built (N=%d, model=%s, obs_slots=%d, vars=%d, constraints=%d)\n', ...
                N_h, obj.model_type, n_obs, n_vars, n_eq + n_ineq);
        end
        
        %% DYNAMICS — FULL (12-state nonlinear container.m) ===============
        %  EXACT PORT: Matches container.m with numerical safeguards
        function xdot = dynamics_full_casadi(obj, x, u_in, env)
            import casadi.*
            
            L = 175.0;
            
            % --- Speed and normalized velocities (EXACT match to container.m) ---
            U = sqrt(x(1)^2 + x(2)^2 + 1e-12);   % epsilon BEFORE sqrt
            U = if_else(U <= 0.1, 0.1, U);      % safeguard minimum
            
            u   = x(1) / U;
            v   = x(2) / U;
            p   = x(7) * L / U;
            r   = x(3) * L / U;
            phi = x(8);
            psi = x(6);
            
            % --- Actuator States---
            n_bow  = x(9);
            n_stern = x(10);
            alpha_bow = x(11);
            alpha_stern = x(12);
            
            % --- Commanded Inputs (from u_in) ---
            n_c_bow = u_in(1);
            n_c_stern = u_in(2);
            alpha_c_bow = u_in(3);
            alpha_c_stern = u_in(4);
            
            % --- Ship parameters (EXACT from container.m) ---
            m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
            Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
            ly = 0.0313;     Iz = 0.000456;
            Jx = 0.0000034;  Jz = 0.000419;
            B  = 25.40;      dF = 8.00;           g = 9.81;
            nabla = 21222;   D  = 6.533;          GM = 0.3/L;
            rho = 1025;      t_ded = 0.175;
            
            % --- Weight parameter ---
            W = rho*g*nabla / (rho*L^2*U^2/2 + 1e-9);  % epsilon prevents division by zero
            
            % --- Hydrodynamic coefficients (EXACT from container.m) ---
            Xuu = -0.0004226; Xvr = -0.00311;  Xrr = 0.00020; Xphiphi = -0.00020; Xvv = -0.00386;
            Kv = 0.0003026;   Kr = -0.000063;  Kp = -0.0000075; Kphi = -0.000021;
            Kvvv = 0.002843;  Krrr = -0.0000462; Kvvr = -0.000588; Kvrr = 0.0010565;
            Kvvphi = -0.0012012; Kvphiphi = -0.0000793; Krrphi = -0.000243; Krphiphi = 0.00003569;
            Yv = -0.0116;     Yr = 0.00242;    Yp = 0;  Yphi = -0.000063;
            Yvvv = -0.109;    Yrrr = 0.00177;  Yvvr = 0.0214; Yvrr = -0.0405;
            Yvvphi = 0.04605; Yvphiphi = 0.00304; Yrrphi = 0.009325; Yrphiphi = -0.001368;
            Nv = -0.0038545;  Nr = -0.00222;   Np = 0.000213; Nphi = -0.0001424;
            Nvvv = 0.001492;  Nrrr = -0.00229; Nvvr = -0.0424; Nvrr = 0.00156;
            Nvvphi = -0.019058; Nvphiphi = -0.0053766; Nrrphi = -0.0038592; Nrphiphi = 0.0024195;
            zR = 0.033;
            
            % --- Mass matrix (EXACT from container.m) ---
            m11 = m + mx;
            m22 = m + my;
            m32 = -my * ly;
            m42 = my * alphay;
            m33 = Ix + Jx;
            m44 = Iz + Jz;
            detM = m22*m33*m44 - m32^2*m44 - m42^2*m33 + 1e-9;  % epsilon for numerical stability
            
            % --- AZIPOD THRUSTER MODEL (EXACT from container.m) ---
            lx_bow  = 80.0;
            lx_stern = -80.0;
            KT0 = 0.527;
            eps = 0.001;
            
            n_bow_nd  = (n_bow/60.0) * L/U;
            n_stern_nd = (n_stern/60.0) * L/U;
            
            T_bow  = 2*rho*D^4 / (U^2*L^2*rho + 1e-11) * KT0 * n_bow_nd * sqrt(n_bow_nd^2 + eps);
            T_stern = 2*rho*D^4 / (U^2*L^2*rho + 1e-11) * KT0 * n_stern_nd * sqrt(n_stern_nd^2 + eps);
            
            Thrust_X = T_bow * cos(alpha_bow) + T_stern * cos(alpha_stern);
            Thrust_Y = T_bow * sin(alpha_bow) + T_stern * sin(alpha_stern);
            Thrust_N = (lx_bow/L) * T_bow * sin(alpha_bow) + (lx_stern/L) * T_stern * sin(alpha_stern);
            
            % --- FORCES & MOMENTS (EXACT from container.m, split for clarity) ---
            Xuu_term   = Xuu * u * u;
            Xvr_term   = Xvr * v * r;
            Xvv_term   = Xvv * v * v;
            Xrr_term   = Xrr * r * r;
            Xphiphi_term = Xphiphi * phi * phi;
            my_vr_term = (m + my) * v * r;
            
            X_f = Xuu_term + (1 - t_ded)*Thrust_X + Xvr_term + Xvv_term + Xrr_term + Xphiphi_term + my_vr_term;
            
            Yv_term = Yv * v;
            Yr_term = Yr * r;
            Yp_term = Yp * p;
            Yphi_term = Yphi * phi;
            Yvvv_term = Yvvv * v^3;
            Yrrr_term = Yrrr * r^3;
            Yvvr_term = Yvvr * v^2 * r;
            Yvrr_term = Yvrr * v * r^2;
            Yvvphi_term = Yvvphi * v^2 * phi;
            Yvphiphi_term = Yvphiphi * v * phi^2;
            Yrrphi_term = Yrrphi * r^2 * phi;
            Yrphiphi_term = Yrphiphi * r * phi^2;
            mx_ur_term = (m + mx) * u * r;
            
            Y_f = Yv_term + Yr_term + Yp_term + Yphi_term + Yvvv_term + Yrrr_term + Yvvr_term + Yvrr_term ...
                + Yvvphi_term + Yvphiphi_term + Yrrphi_term + Yrphiphi_term + Thrust_Y - mx_ur_term;
            
            Kv_term = Kv * v;
            Kr_term = Kr * r;
            Kp_term = Kp * p;
            Kphi_term = Kphi * phi;
            Kvvv_term = Kvvv * v^3;
            Krrr_term = Krrr * r^3;
            Kvvr_term = Kvvr * v^2 * r;
            Kvrr_term = Kvrr * v * r^2;
            Kvvphi_term = Kvvphi * v^2 * phi;
            Kvphiphi_term = Kvphiphi * v * phi^2;
            Krrphi_term = Krrphi * r^2 * phi;
            Krphiphi_term = Krphiphi * r * phi^2;
            stability_term = -W * GM * phi;
            thrust_roll_term = zR * Thrust_Y;
            inertia_coupling_term = mx * lx * u * r;
            
            K_f = Kv_term + Kr_term + Kp_term + Kphi_term + Kvvv_term + Krrr_term + Kvvr_term + Kvrr_term ...
                + Kvvphi_term + Kvphiphi_term + Krrphi_term + Krphiphi_term + stability_term + thrust_roll_term + inertia_coupling_term;
            
            Nv_term = Nv * v;
            Nr_term = Nr * r;
            Np_term = Np * p;
            Nphi_term = Nphi * phi;
            Nvvv_term = Nvvv * v^3;
            Nrrr_term = Nrrr * r^3;
            Nvvr_term = Nvvr * v^2 * r;
            Nvrr_term = Nvrr * v * r^2;
            Nvvphi_term = Nvvphi * v^2 * phi;
            Nvphiphi_term = Nvphiphi * v * phi^2;
            Nrrphi_term = Nrrphi * r^2 * phi;
            Nrphiphi_term = Nrphiphi * r * phi^2;
            
            N_f = Nv_term + Nr_term + Np_term + Nphi_term + Nvvv_term + Nrrr_term + Nvvr_term + Nvrr_term ...
                + Nvvphi_term + Nvphiphi_term + Nrrphi_term + Nrphiphi_term + Thrust_N;
            
            % --- STATE DERIVATIVES (EXACT from container.m) ---
            u_dot = X_f * (U^2/L) / m11;
            v_dot = -((-m33*m44*Y_f + m32*m44*K_f + m42*m33*N_f) / detM) * (U^2/L);
            r_dot = ((-m42*m33*Y_f + m32*m42*K_f + N_f*m22*m33 - N_f*m32^2) / detM) * (U^2/L^2);
            
            cos_psi = cos(psi);
            sin_psi = sin(psi);
            cos_phi = cos(phi);
            
            x_dot   = (cos_psi * u - sin_psi * cos_phi * v) * U;
            y_dot   = (sin_psi * u + cos_psi * cos_phi * v) * U;
            psi_dot = cos_phi * r * (U/L);
            
            p_dot = ((-m32*m44*Y_f + K_f*m22*m44 - K_f*m42^2 + m32*m42*N_f) / detM) * (U^2/L^2);
            phi_dot = p * (U/L);
            
            % --- ACTUATOR DYNAMICS (first-order response) ---
            n_dot_bow = (n_c_bow - n_bow) / 2.0;
            n_dot_stern = (n_c_stern - n_stern) / 2.0;
            alpha_dot_bow = (alpha_c_bow - alpha_bow) / 2.0;
            alpha_dot_stern = (alpha_c_stern - alpha_stern) / 2.0;
            
            % --- Assemble all state derivatives ---
            xdot = vertcat(u_dot, v_dot, r_dot, ...
                          x_dot, y_dot, psi_dot, ...
                          p_dot, phi_dot, ...
                          n_dot_bow, n_dot_stern, ...
                          alpha_dot_bow, alpha_dot_stern);
        end
        
        %% SOLVE ==========================================================
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            t_start = tic;
            
            % --- Build solver on first call ---
            if ~obj.solver_built
                obj.buildSolver();
            end
            
            % --- Assemble obstacle list (external + map) ---
            if nargin < 4 || isempty(obstacles)
                ext_obs = [];
            else
                ext_obs = obstacles;
            end
            
            map_obs = [];
            if obj.use_map_obstacles
                map_obs = obj.mapPolygonsToObstacles(x0(4:5));
            end
            
            if isempty(ext_obs) && isempty(map_obs)
                all_obs = [];
            elseif isempty(ext_obs)
                all_obs = map_obs;
            elseif isempty(map_obs)
                all_obs = ext_obs;
            else
                all_obs = [ext_obs(:); map_obs(:)];
            end
            
            n_real = 0;
            if obj.use_obstacles && ~isempty(all_obs)
                n_real = min(length(all_obs), obj.max_obs);
            end
            
            % --- Fill obstacle parameter arrays (fixed slots) ---
            obs_pos = 1e8 * ones(2, obj.max_obs);   % dummy far-away
            obs_rad = zeros(obj.max_obs, 1);         % zero radius
            
            for j = 1:n_real
                pp = all_obs(j).position(:);
                obs_pos(:,j) = pp(1:2);
                obs_rad(j)   = all_obs(j).radius;
            end
            
            % --- Assemble parameter vector ---
            % Reference control: maintain stern RPM, zero angles
            u_ref = repmat([0; max(x0(10), 10); 0; 0], 1, obj.N);
            p_val = [x0(:); x_ref(:); u_ref(:); obs_pos(:); obs_rad(:); obj.V_c; obj.beta_c];
            
            % --- Initial guess (warm start) ---
            N_h = obj.N;
            nx  = obj.nx;
            nu  = obj.nu;
            n_obs = obj.max_obs;
            
            if ~isempty(obj.prev_sol)
                % Shift previous solution by one step
                prev = obj.prev_sol;
                
                % Extract X from prev
                X_prev = reshape(prev(1 : nx*(N_h+1)), nx, N_h+1);
                X_init = [X_prev(:, 2:end), X_prev(:, end)];
                X_init(:,1) = x0;  % enforce current state
                
                % Extract U from prev
                u_start = nx*(N_h+1) + 1;
                U_prev  = reshape(prev(u_start : u_start + nu*N_h - 1), nu, N_h);
                U_init  = [U_prev(:, 2:end), U_prev(:, end)];
                
                % Slack: start from zeros
                s_init = zeros(n_obs, N_h+1);
                
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            else
                % Cold start: propagate with container()
                X_init = zeros(nx, N_h+1);
                X_init(:,1) = x0;
                % 4 Controls: [Bow RPM; Stern RPM; Bow Angle; Stern Angle]
                U_init = repmat([0; max(x0(10), 10); 0; 0], 1, N_h);
                for k = 1:N_h
                    try
                        xk = X_init(:,k);
                        xk(1)  = max(xk(1), 0.1);
                        xk(10) = max(xk(10), 1);
                        [xdot_k, ~] = container(xk, U_init(:,k));
                        X_init(:,k+1) = xk + xdot_k * obj.dt;
                    catch
                        X_init(:,k+1) = X_init(:,k);
                    end
                end
                s_init = zeros(n_obs, N_h+1);
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            end
            
            % --- Solve NLP ---
            try
                sol = obj.solver('x0', x0_guess, ...
                    'lbx', obj.lbx_vec, 'ubx', obj.ubx_vec, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val);
                
                sol_x = full(sol.x);
                
                % Extract solution
                X_sol = reshape(sol_x(1 : nx*(N_h+1)), nx, N_h+1);
                u_s   = nx*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s : u_s + nu*N_h - 1), nu, N_h);
                
                u_opt  = U_sol(:, 1);
                X_pred = X_sol;
                
                % Store for warm start
                obj.prev_sol = sol_x;
                
                info.success = true;
                info.cost    = full(sol.f);
                obj.solve_ok = obj.solve_ok + 1;
                
            catch ME
                % Fallback: zero angles, maintain Stern RPM
                u_opt  = [0; max(x0(10), 10); 0; 0];
                X_pred = repmat(x0, 1, N_h+1);
                
                info.success = false;
                info.error   = ME.message;
                obj.solve_fail = obj.solve_fail + 1;
                
                if obj.solve_fail <= 3
                    fprintf('  [NMPC FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
                
                % Clear warm start
                obj.prev_sol = [];
            end
            
            info.solve_time = toc(t_start);
            info.n_obs_real = n_real;
        end
        
        %% RESET ==========================================================
        function resetWarmStart(obj)
            obj.prev_sol = [];
        end
        
    end % methods
end
