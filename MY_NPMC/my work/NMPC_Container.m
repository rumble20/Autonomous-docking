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
        nx = 10         % States:   [u v r x y psi p phi delta n]
        nu = 2          % Controls: [delta_c  n_c]
        
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
            obj.R_rate = diag([0.5, 0.0001]);
            
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
                if strcmp(obj.model_type, 'simplified')
                    xdot_k = obj.dynamics_simplified_casadi(X(:,k), U(:,k), P_env);
                else
                    xdot_k = obj.dynamics_full_casadi(X(:,k), U(:,k), P_env);
                end
                g = vertcat(g, X(:,k+1) - (X(:,k) + xdot_k * obj.dt));
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
                lbx(base + 1)  = 0.1;      % u >= 0.1 m/s
                lbx(base + 10) = 1;         % n >= 1 RPM
            end
            
            % Control bounds: U (indices nx*(N_h+1)+1 .. nx*(N_h+1)+nu*N_h)
            u_offset = nx*(N_h+1);
            for k = 1:N_h
                base = u_offset + (k-1)*nu;
                lbx(base + 1) = -obj.delta_max_rad;
                ubx(base + 1) =  obj.delta_max_rad;
                lbx(base + 2) =  obj.n_min_rpm;
                ubx(base + 2) =  obj.n_max_rpm;
            end
            
            % Slack bounds: >= 0
            s_offset = u_offset + nu*N_h;
            lbx(s_offset+1 : end) = 0;
            
            %% ---- Bounds on constraints ----
            n_eq   = nx + nx*N_h;              % initial + dynamics
            n_ineq = n_obs * (N_h+1);         % obstacles
            
            lbg = zeros(n_eq + n_ineq, 1);
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
        
        %% DYNAMICS — FULL (10-state nonlinear container.m) ===============
        function xdot = dynamics_full_casadi(obj, x, u_in, env)
            import casadi.*
            
            L  = 175;
            V_c_   = env(1);
            beta_c_ = env(2);
            psi = x(6);
            phi = x(8);
            
            % --- Relative velocities (current effects) ---
            u_r = x(1) - V_c_ * cos(beta_c_ - psi);
            v_r = x(2) - V_c_ * sin(beta_c_ - psi);
            Uv  = sqrt(u_r^2 + v_r^2);
            Uv  = if_else(Uv < 0.1, 0.1, Uv);
            
            % --- Nondimensional states (use relative velocity) ---
            delta_c = u_in(1);
            n_c     = u_in(2) / 60 * L / Uv;
            
            u   = u_r / Uv;
            v   = v_r / Uv;
            p   = x(7) * L / Uv;
            r   = x(3) * L / Uv;
            delta = x(9);
            n   = x(10) / 60 * L / Uv;
            
            % --- Parameters (container.m) ---
            m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
            Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
            ly = 0.0313;     Iz = 0.000456;
            Jx = 0.0000034;  Jz = 0.000419;
            
            B     = 25.40;   dF = 8.00;    g_acc = 9.81;
            dA    = 9.00;    d  = 8.50;    nabla = 21222;
            KM    = 10.39;   KB = 4.6154;  AR    = 33.0376;
            Delta_hull = 1.8219;  D  = 6.533;  GM = 0.3/L;
            rho   = 1025;    t_ded = 0.175;
            n_max_revs = 160/60;
            
            W = rho*g_acc*nabla / (rho*L^2*Uv^2/2);
            
            Xuu      = -0.0004226;  Xvr    = -0.00311;   Xrr      =  0.00020;
            Xphiphi  = -0.00020;    Xvv    = -0.00386;
            
            Kv       =  0.0003026;  Kr     = -0.000063;  Kp       = -0.0000075;
            Kphi     = -0.000021;   Kvvv   =  0.002843;  Krrr     = -0.0000462;
            Kvvr     = -0.000588;   Kvrr   =  0.0010565; Kvvphi   = -0.0012012;
            Kvphiphi = -0.0000793;  Krrphi = -0.000243;  Krphiphi =  0.00003569;
            
            Yv       = -0.0116;     Yr     =  0.00242;   Yp       =  0;
            Yphi     = -0.000063;   Yvvv   = -0.109;     Yrrr     =  0.00177;
            Yvvr     =  0.0214;     Yvrr   = -0.0405;    Yvvphi   =  0.04605;
            Yvphiphi =  0.00304;    Yrrphi =  0.009325;  Yrphiphi = -0.001368;
            
            Nv       = -0.0038545;  Nr     = -0.00222;   Np       =  0.000213;
            Nphi     = -0.0001424;  Nvvv   =  0.001492;  Nrrr     = -0.00229;
            Nvvr     = -0.0424;     Nvrr   =  0.00156;   Nvvphi   = -0.019058;
            Nvphiphi = -0.0053766;  Nrrphi = -0.0038592; Nrphiphi =  0.0024195;
            
            kk   = 0.631;  epsilon = 0.921;  xR   = -0.5;
            wp   = 0.184;  tau     = 1.09;   xp   = -0.526;
            cpv  = 0.0;    cpr     = 0.0;    ga   = 0.088;
            cRr  = -0.156; cRrrr   = -0.275; cRrrv = 1.96;
            cRX  = 0.71;   aH      = 0.237;  zR   = 0.033;
            xH   = -0.48;
            
            % --- Mass matrix ---
            m11 = m + mx;
            m22 = m + my;
            m32 = -my * ly;
            m42 = my * alphay;
            m33 = Ix + Jx;
            m44 = Iz + Jz;
            
            % --- Rudder saturation & dynamics ---
            delta_max_r = 35*pi/180;
            delta_max  = 10*pi/180;
            
            delta_c_sat = if_else(delta_c >  delta_max_r,  delta_max_r, ...
                          if_else(delta_c < -delta_max_r, -delta_max_r, delta_c));
            delta_dot_raw = delta_c_sat - delta;
            delta_dot = if_else(delta_dot_raw >  delta_max,  delta_max, ...
                        if_else(delta_dot_raw < -delta_max, -delta_max, delta_dot_raw));
            
            % --- Shaft saturation & dynamics ---
            n_c_revs = n_c * Uv / L;
            n_revs   = n   * Uv / L;
            n_revs_safe = if_else(n_revs < 0.01, 0.01, n_revs);
            n_c_clamped = if_else(n_c_revs >  n_max_revs,  n_max_revs, ...
                          if_else(n_c_revs < 0, 0, n_c_revs));
            Tm = if_else(n_revs_safe > 0.3, 5.65 / n_revs_safe, 18.83);
            n_dot = (1/Tm) * (n_c_clamped - n_revs_safe) * 60;
            
            % --- Propeller / rudder ---
            vR = ga*v + cRr*r + cRrrr*r^3 + cRrrv*r^2*v;
            uP = u * ( (1 - wp) + tau*((v + xp*r)^2 + cpv*v + cpr*r) );
            
            J_prop = uP * Uv / (n_revs_safe * D);
            KT     = 0.527 - 0.455 * J_prop;
            
            sqrt_arg = 1 + 8*kk*KT / (pi*J_prop^2 + 1e-6);
            sqrt_arg = if_else(sqrt_arg < 1e-6, 1e-6, sqrt_arg);
            uR = uP * epsilon * sqrt(sqrt_arg);
            
            uR_safe = if_else(uR*uR < 1e-8, 1e-4, uR);
            alphaR = delta + atan2(vR, uR_safe);
            FN = -((6.13*Delta_hull)/(Delta_hull + 2.25)) * (AR/L^2) * ...
                  (uR^2 + vR^2) * sin(alphaR);
            T_thrust = 2*rho*D^4 / (Uv^2*L^2*rho) * KT * n_revs_safe * abs(n_revs_safe);
            
            % --- Forces & moments ---
            X_f = Xuu*u^2 + (1-t_ded)*T_thrust + Xvr*v*r + Xvv*v^2 + ...
                  Xrr*r^2 + Xphiphi*phi^2 + cRX*FN*sin(delta) + (m+my)*v*r;
            
            Y_f = Yv*v + Yr*r + Yp*p + Yphi*phi + Yvvv*v^3 + Yrrr*r^3 + ...
                  Yvvr*v^2*r + Yvrr*v*r^2 + Yvvphi*v^2*phi + Yvphiphi*v*phi^2 + ...
                  Yrrphi*r^2*phi + Yrphiphi*r*phi^2 + ...
                  (1+aH)*FN*cos(delta) - (m+mx)*u*r;
            
            K_f = Kv*v + Kr*r + Kp*p + Kphi*phi + Kvvv*v^3 + Krrr*r^3 + ...
                  Kvvr*v^2*r + Kvrr*v*r^2 + Kvvphi*v^2*phi + Kvphiphi*v*phi^2 + ...
                  Krrphi*r^2*phi + Krphiphi*r*phi^2 - ...
                  (1+aH)*zR*FN*cos(delta) + mx*lx*u*r - W*GM*phi;
            
            N_f = Nv*v + Nr*r + Np*p + Nphi*phi + Nvvv*v^3 + Nrrr*r^3 + ...
                  Nvvr*v^2*r + Nvrr*v*r^2 + Nvvphi*v^2*phi + Nvphiphi*v*phi^2 + ...
                  Nrrphi*r^2*phi + Nrphiphi*r*phi^2 + ...
                  (xR + aH*xH)*FN*cos(delta);
            
            % --- State derivatives ---
            detM = m22*m33*m44 - m32^2*m44 - m42^2*m33;
            
            xdot = [
                X_f*(Uv^2/L) / m11                                                          % u_dot
                -((-m33*m44*Y_f + m32*m44*K_f + m42*m33*N_f)/detM) * (Uv^2/L)               % v_dot
                 ((-m42*m33*Y_f + m32*m42*K_f + N_f*m22*m33 - N_f*m32^2)/detM) * (Uv^2/L^2) % r_dot
                cos(psi)*x(1) - sin(psi)*cos(phi)*x(2)                                      % x_dot  (absolute vel)
                sin(psi)*x(1) + cos(psi)*cos(phi)*x(2)                                      % y_dot  (absolute vel)
                cos(phi)*x(3)                                                                % psi_dot
                ((-m32*m44*Y_f + K_f*m22*m44 - K_f*m42^2 + m32*m42*N_f)/detM) * (Uv^2/L^2) % p_dot
                x(7)                                                                         % phi_dot
                delta_dot                                                                    % delta_dot
                n_dot                                                                        % n_dot [RPM/s]
            ];
        end
        
        %% DYNAMICS — SIMPLIFIED (linear damping from Lcontainer.m) =======
        function xdot = dynamics_simplified_casadi(obj, x, u_in, env)
            import casadi.*
            
            L  = 175;
            V_c_   = env(1);
            beta_c_ = env(2);
            psi = x(6);
            phi = x(8);
            
            % --- Relative velocities ---
            u_r = x(1) - V_c_ * cos(beta_c_ - psi);
            v_r = x(2) - V_c_ * sin(beta_c_ - psi);
            Uv  = sqrt(u_r^2 + v_r^2);
            Uv  = if_else(Uv < 0.1, 0.1, Uv);
            
            % --- Rudder dynamics (same as full) ---
            delta_c = u_in(1);
            delta = x(9);
            delta_max_r = 35*pi/180;
            Ddelta_max  = 10*pi/180;
            
            delta_c_sat = if_else(delta_c >  delta_max_r,  delta_max_r, ...
                          if_else(delta_c < -delta_max_r, -delta_max_r, delta_c));
            delta_dot_raw = delta_c_sat - delta;
            delta_dot = if_else(delta_dot_raw >  Ddelta_max,  Ddelta_max, ...
                        if_else(delta_dot_raw < -Ddelta_max, -Ddelta_max, delta_dot_raw));
            
            % --- Shaft dynamics (same as full) ---
            n_revs     = x(10) / 60;
            n_c_revs   = u_in(2) / 60;
            n_max_revs = 160 / 60;
            n_c_clamped = if_else(n_c_revs > n_max_revs, n_max_revs, ...
                          if_else(n_c_revs < 0, 0, n_c_revs));
            n_revs_safe = if_else(n_revs < 0.01, 0.01, n_revs);
            Tm = if_else(n_revs_safe > 0.3, 5.65 / n_revs_safe, 18.83);
            n_dot = (1/Tm) * (n_c_clamped - n_revs_safe) * 60;
            
            % --- Linearized sway-yaw-roll (Lcontainer.m M, N, G, b) ---
            % NOTE: Linear model is accurate near U0 = 7 m/s.
            T_mat    = [1 0 0; 0 1/L 0; 0 0 1/L];
            Tinv_mat = [1 0 0; 0 L 0; 0 0 L];
            M_mat = [0.01497, 0.0003525, -0.0002205; ...
                     0.0003525, 0.000875, 0; ...
                    -0.0002205, 0, 0.0000210];
            N_mat = [0.012035, 0.00522, 0; ...
                     0.0038436, 0.00243, -0.000213; ...
                    -0.000314, 0.0000692, 0.0000075];
            G_mat = [0, 0, 0.0000704; ...
                     0, 0, 0.0001468; ...
                     0, 0, 0.0004966];
            b_vec = [-0.002578; 0.00126; 0.0000855];
            
            M_bar     = T_mat * M_mat * Tinv_mat;
            M_bar_inv = inv(M_bar);
            N_bar     = T_mat * N_mat * Tinv_mat;
            G_bar     = T_mat * G_mat * Tinv_mat;
            b_bar     = T_mat * b_vec;
            
            nu_vec = [x(2); x(3); x(7)];    % [v, r, p] dimensional
            eta    = [x(5); x(6); x(8)];    % [y, psi, phi]
            
            nudot = M_bar_inv * ((Uv^2/L)*b_bar*delta - ...
                    (Uv/L)*N_bar*nu_vec - (Uv/L)^2*G_bar*eta);
            
            % --- Surge: simplified resistance + thrust ---
            m11 = 0.00792 + 0.000238;
            Xuu = -0.0004226;
            t_ded = 0.175;
            D_prop = 6.533;
            rho = 1025;
            
            u_nd = u_r / Uv;
            wp_frac = 0.184;
            uP = u_nd * (1 - wp_frac);
            J_prop = uP * Uv / (n_revs_safe * D_prop + 1e-6);
            KT = 0.527 - 0.455 * J_prop;
            T_thrust = 2*rho*D_prop^4 / (Uv^2*L^2*rho) * KT * n_revs_safe * abs(n_revs_safe);
            
            u_dot = (Xuu*u_nd^2 + (1-t_ded)*T_thrust) * (Uv^2/L) / m11;
            
            % --- State derivatives ---
            xdot = [
                u_dot                                                  % u_dot
                nudot(1)                                               % v_dot (linear)
                nudot(2)                                               % r_dot (linear)
                cos(psi)*x(1) - sin(psi)*cos(phi)*x(2)                % x_dot (absolute vel)
                sin(psi)*x(1) + cos(psi)*cos(phi)*x(2)                % y_dot (absolute vel)
                cos(phi)*x(3)                                          % psi_dot
                nudot(3)                                               % p_dot (linear)
                x(7)                                                   % phi_dot
                delta_dot                                              % delta_dot
                n_dot                                                  % n_dot
            ];
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
            u_ref = repmat([0; max(x0(10), obj.n_min_rpm)], 1, obj.N);
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
                U_init = repmat([0; max(x0(10), obj.n_min_rpm)], 1, N_h);
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
                % Fallback: zero rudder, maintain RPM
                u_opt  = [0; max(x0(10), obj.n_min_rpm)];
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
