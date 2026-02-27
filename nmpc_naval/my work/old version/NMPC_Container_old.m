classdef NMPC_Container < handle
    % NMPC_Container - Nonlinear MPC for container ship using CasADi
    %
    % CasADi dynamics EXACTLY matches container.m (line-by-line verified).
    % Optimizer is built ONCE and reused with warm starting.
    %
    % Author: Riccardo Legnini
    % Date: 2026-02-25
    
    properties
        % Horizon
        N               % Prediction horizon steps
        dt              % Sample time [s]
        
        % Cost weights
        Q               % State error weight  (10x10)
        R               % Control effort weight (2x2)
        Q_N             % Terminal cost weight (10x10)
        
        % Dimensions
        nx = 10         % States:   [u v r x y psi p phi delta n]
        nu = 2          % Controls: [delta_c  n_c]
        
        % Limits (from container.m)
        delta_max_rad   % Max rudder angle [rad]
        n_max_rpm       % Max shaft speed  [RPM]
        n_min_rpm       % Min shaft speed  [RPM]  (keep >0 so container() works)
        
        % Obstacle settings
        use_obstacles = false
        max_obs = 15            % Max obstacle slots in optimizer
        r_safety = 10           % Safety margin [m]
        penalty_slack = 500     % Slack penalty
        
        % Map integration
        map = []                % Loaded harbor map struct (from .mat)
        use_map_obstacles = false
        max_map_obs = 10        % Closest polygons used each solve
        map_obs_radius = 15     % Avoidance radius for map nearest-point obstacles [m]
        
        % Cost
        R_rate                  % Control rate-of-change weight (2x2)
        
        % CasADi persistent optimizer (built ONCE)
        opti
        X                       % State decision vars (10 x N+1)
        U                       % Control decision vars (2 x N)
        slack                   % Slack vars (max_obs x N+1)  or empty
        X0_param                % Parameter: initial state
        X_ref_param             % Parameter: reference trajectory
        U_ref_param             % Parameter: reference control (deviation cost)
        Obs_pos_param           % Parameter: obstacle positions
        Obs_rad_param           % Parameter: obstacle radii
        optimizer_built = false
        num_obs_built = 0       % Obstacles baked into current optimizer
        
        % Warm start storage
        prev_X_sol = []
        prev_U_sol = []
        
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
            obj.Q_N = config.Q * 2;     % terminal weight
            
            obj.delta_max_rad = 10 * pi/180;   % 10 deg
            obj.n_max_rpm     = 160;            % 160 RPM
            obj.n_min_rpm     = 10;             % keep RPM > 0
            obj.R_rate = diag([0.5, 0.0001]);   % smooth rudder changes
            
            if isfield(config, 'use_obstacles')
                obj.use_obstacles = config.use_obstacles;
            end
            if isfield(config, 'max_obs')
                obj.max_obs = config.max_obs;
            end
            if isfield(config, 'use_map_obstacles')
                obj.use_map_obstacles = config.use_map_obstacles;
            end
            if isfield(config, 'max_map_obs')
                obj.max_map_obs = config.max_map_obs;
            end
            
            fprintf('NMPC_Container initialized (N=%d, dt=%.2f)\n', obj.N, obj.dt);
        end

        function setMap(obj, map_struct)
            % setMap Store harbor map for automatic NMPC map constraints.
            obj.map = map_struct;
            obj.use_map_obstacles = true;
            fprintf('NMPC map integration enabled (%d polygons available)\n', length(map_struct.polygons));
        end

        function obs_from_map = mapPolygonsToObstacles(obj, vessel_xy)
            % Convert closest map polygons into circular obstacles using
            % nearest-point-on-polygon approach (much tighter than enclosing
            % circles, which wildly overestimate for elongated shapes).

            obs_from_map = [];
            if isempty(obj.map) || ~isfield(obj.map, 'polygons') || isempty(obj.map.polygons)
                return;
            end

            proximity_threshold = 300;  % [m] ignore polygons further than this
            vx = vessel_xy(1);
            vy = vessel_xy(2);

            n_poly = length(obj.map.polygons);
            nearest_pts = zeros(2, n_poly);
            min_dists = inf(n_poly, 1);

            for j = 1:n_poly
                px = obj.map.polygons(j).X(:);
                py = obj.map.polygons(j).Y(:);
                valid = ~isnan(px) & ~isnan(py);
                px = px(valid);  py = py(valid);
                if length(px) < 2, continue; end

                best_d = inf;
                best_pt = [px(1); py(1)];
                n_verts = length(px);
                for kk = 1:n_verts
                    k2 = mod(kk, n_verts) + 1;
                    [d, pt] = obj.pointToSegment(vx, vy, ...
                        px(kk), py(kk), px(k2), py(k2));
                    if d < best_d
                        best_d = d;
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

            obs_from_map = repmat(struct('position', [0;0], 'radius', 0), n_take, 1);
            for k = 1:n_take
                j = close_idx(k);
                obs_from_map(k).position = nearest_pts(:, j);
                obs_from_map(k).radius = obj.map_obs_radius;
            end
        end

        function [d, nearest_pt] = pointToSegment(~, px, py, x1, y1, x2, y2)
            % Minimum distance from point (px,py) to segment (x1,y1)-(x2,y2)
            dx = x2 - x1;  dy = y2 - y1;
            len2 = dx^2 + dy^2;
            if len2 < 1e-12
                nearest_pt = [x1; y1];
                d = sqrt((px-x1)^2 + (py-y1)^2);
                return;
            end
            t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy) / len2));
            near_x = x1 + t*dx;  near_y = y1 + t*dy;
            nearest_pt = [near_x; near_y];
            d = sqrt((px-near_x)^2 + (py-near_y)^2);
        end
        
        %% CASADI DYNAMICS ================================================
        % Line-by-line translation of container.m with CasADi safeguards.
        % ALL variable names match container.m to ensure correctness.
        %
        % KEY DIFFERENCES vs previous broken version:
        %   1) Kinematics (x_dot,y_dot) use NON-DIM u,v  (not dimensional)
        %   2) psi_dot, phi_dot use NON-DIM r,p           (not dimensional)
        %   3) Thrust T uses n in REV/S                   (not non-dim)
        %   4) Shaft Tm uses n in REV/S                   (not RPM)
        %   5) Rudder rate limiting included
        function xdot = dynamics_casadi(obj, x, u_in)
            import casadi.*
            
            % ---- Normalization variables (container.m lines 42-43) ------
            L  = 175;
            Uv = sqrt(x(1)^2 + x(2)^2);       % speed  (m/s)
            Uv = if_else(Uv < 0.1, 0.1, Uv);   % safeguard >0
            
            % ---- Nondimensional states & inputs (container.m lines 52-59)
            delta_c = u_in(1);                          % rad  (not non-dim)
            n_c     = u_in(2) / 60 * L / Uv;           % non-dim shaft cmd
            
            u   = x(1) / Uv;            % surge  (non-dim)
            v   = x(2) / Uv;            % sway   (non-dim)
            p   = x(7) * L / Uv;        % roll rate (non-dim)
            r   = x(3) * L / Uv;        % yaw rate  (non-dim)
            phi = x(8);                  % roll angle [rad]
            psi = x(6);                  % yaw angle  [rad]
            delta = x(9);               % rudder angle [rad]
            n   = x(10) / 60 * L / Uv;  % shaft speed (non-dim)
            
            % ---- Parameters (container.m lines 62-100) ------------------
            m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
            Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
            ly = 0.0313;     Iz = 0.000456;
            Jx = 0.0000034;  Jz = 0.000419;
            
            B     = 25.40;   dF = 8.00;    g     = 9.81;
            dA    = 9.00;    d  = 8.50;    nabla = 21222;
            KM    = 10.39;   KB = 4.6154;  AR    = 33.0376;
            Delta_hull = 1.8219;  D  = 6.533;  GM = 0.3/L;
            rho   = 1025;    t_ded = 0.175;              % thrust deduction
            n_max_revs = 160/60;                          % RPM to rev/s
            
            W = rho*g*nabla / (rho*L^2*Uv^2/2);
            
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
            
            % ---- Mass matrix (container.m lines 102-107) ----------------
            m11 = m + mx;
            m22 = m + my;
            m32 = -my * ly;
            m42 = my * alphay;
            m33 = Ix + Jx;
            m44 = Iz + Jz;
            
            % ---- Rudder saturation & dynamics (container.m lines 109-116)
            delta_max_r = 10*pi/180;
            Ddelta_max  = 5*pi/180;   % 5 deg/s rate limit
            
            delta_c_sat = if_else(delta_c >  delta_max_r,  delta_max_r, ...
                          if_else(delta_c < -delta_max_r, -delta_max_r, delta_c));
            
            delta_dot_raw = delta_c_sat - delta;
            delta_dot = if_else(delta_dot_raw >  Ddelta_max,  Ddelta_max, ...
                        if_else(delta_dot_raw < -Ddelta_max, -Ddelta_max, delta_dot_raw));
            
            % ---- Shaft saturation & dynamics (container.m lines 118-130)
            % Container.m re-dimensionalises n_c and n to rev/s before
            % computing Tm and n_dot.  We do the same here:
            %   n_c_revs = n_c * U / L  =  u_in(2)/60   [rev/s]
            %   n_revs   = n   * U / L  =  x(10)/60     [rev/s]
            n_c_revs = n_c * Uv / L;          % = u_in(2)/60  [rev/s]
            n_revs   = n   * Uv / L;          % = x(10)/60    [rev/s]
            
            % safeguard for division by n_revs
            n_revs_safe = if_else(n_revs < 0.01, 0.01, n_revs);
            
            % clamp commanded rev/s
            n_c_clamped = if_else(n_c_revs >  n_max_revs,  n_max_revs, ...
                          if_else(n_c_revs < 0, 0, n_c_revs));
            
            Tm = if_else(n_revs_safe > 0.3, 5.65 / n_revs_safe, 18.83);
            n_dot = (1/Tm) * (n_c_clamped - n_revs_safe) * 60;   % RPM/s
            
            % ---- Propeller / rudder (container.m lines 133-143) ---------
            % vR and uP use NON-DIMENSIONAL u,v,r  (matching container.m)
            % J and T use n_revs (rev/s) and Uv (m/s)  (matching container.m)
            vR = ga*v + cRr*r + cRrrr*r^3 + cRrrv*r^2*v;
            uP = u * ( (1 - wp) + tau*((v + xp*r)^2 + cpv*v + cpr*r) );
            
            J_prop = uP * Uv / (n_revs_safe * D);      % advance ratio
            KT     = 0.527 - 0.455 * J_prop;
            
            sqrt_arg = 1 + 8*kk*KT / (pi*J_prop^2 + 1e-6);
            sqrt_arg = if_else(sqrt_arg < 1e-6, 1e-6, sqrt_arg);
            uR = uP * epsilon * sqrt(sqrt_arg);
            
            uR_safe = if_else(abs(uR) < 1e-4, 1e-4, uR);
            alphaR = delta + atan(vR / uR_safe);
            FN = -((6.13*Delta_hull)/(Delta_hull + 2.25)) * (AR/L^2) * ...
                  (uR^2 + vR^2) * sin(alphaR);
            T_thrust = 2*rho*D^4 / (Uv^2*L^2*rho) * KT * n_revs_safe * abs(n_revs_safe);
            
            % ---- Forces & moments (container.m lines 146-161) -----------
            %   All use NON-DIMENSIONAL  u, v, r, p, phi, delta
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
            
            % ---- State derivatives (container.m lines 163-173) ----------
            %  ** Uses NON-DIM u,v,r,p for kinematics — matching container.m **
            detM = m22*m33*m44 - m32^2*m44 - m42^2*m33;
            
            xdot = [
                X_f*(Uv^2/L) / m11                                                            % u_dot
                -((-m33*m44*Y_f + m32*m44*K_f + m42*m33*N_f)/detM) * (Uv^2/L)                 % v_dot
                 ((-m42*m33*Y_f + m32*m42*K_f + N_f*m22*m33 - N_f*m32^2)/detM) * (Uv^2/L^2)  % r_dot
                (cos(psi)*u - sin(psi)*cos(phi)*v) * Uv                                        % x_dot
                (sin(psi)*u + cos(psi)*cos(phi)*v) * Uv                                        % y_dot
                cos(phi)*r * (Uv/L)                                                            % psi_dot
                ((-m32*m44*Y_f + K_f*m22*m44 - K_f*m42^2 + m32*m42*N_f)/detM) * (Uv^2/L^2)   % p_dot
                p * (Uv/L)                                                                     % phi_dot
                delta_dot                                                                      % delta_dot
                n_dot                                                                          % n_dot [RPM/s]
            ];
        end
        
        %% BUILD OPTIMIZER (called ONCE) ==================================
        function buildOptimizer(obj, num_obs)
            import casadi.*
            
            obj.opti = casadi.Opti();
            
            %% Decision variables
            obj.X = obj.opti.variable(obj.nx, obj.N + 1);
            obj.U = obj.opti.variable(obj.nu, obj.N);
            
            %% Parameters
            obj.X0_param    = obj.opti.parameter(obj.nx, 1);
            obj.X_ref_param = obj.opti.parameter(obj.nx, obj.N + 1);
            obj.U_ref_param = obj.opti.parameter(obj.nu, obj.N);
            
            %% Obstacle params & slack  (only if needed)
            if num_obs > 0
                obj.Obs_pos_param = obj.opti.parameter(2, num_obs);
                obj.Obs_rad_param = obj.opti.parameter(num_obs, 1);
                obj.slack = obj.opti.variable(num_obs, obj.N + 1);
                obj.opti.subject_to(0 <= obj.slack(:));
            else
                obj.slack = [];
            end
            
            %% Objective
            J = 0;
            for k = 1:obj.N
                e = obj.X(:,k) - obj.X_ref_param(:,k);
                J = J + e' * obj.Q * e;
                du = obj.U(:,k) - obj.U_ref_param(:,k);
                J = J + du' * obj.R * du;
            end
            % Control rate-of-change penalty (smooth rudder)
            for k = 1:obj.N-1
                dU = obj.U(:,k+1) - obj.U(:,k);
                J = J + dU' * obj.R_rate * dU;
            end
            % terminal
            eN = obj.X(:,obj.N+1) - obj.X_ref_param(:,obj.N+1);
            J = J + eN' * obj.Q_N * eN;
            % slack penalty
            if num_obs > 0
                J = J + obj.penalty_slack * sumsqr(obj.slack);
            end
            obj.opti.minimize(J);
            
            %% Dynamics constraints  (Euler integration, same as simulation)
            for k = 1:obj.N
                xdot_k = obj.dynamics_casadi(obj.X(:,k), obj.U(:,k));
                obj.opti.subject_to( obj.X(:,k+1) == obj.X(:,k) + xdot_k * obj.dt );
            end
            
            %% Initial condition
            obj.opti.subject_to( obj.X(:,1) == obj.X0_param );
            
            %% Control bounds
            for k = 1:obj.N
                obj.opti.subject_to( obj.U(1,k) >= -obj.delta_max_rad );
                obj.opti.subject_to( obj.U(1,k) <=  obj.delta_max_rad );
                obj.opti.subject_to( obj.U(2,k) >=  obj.n_min_rpm );
                obj.opti.subject_to( obj.U(2,k) <=  obj.n_max_rpm );
            end
            
            %% State bounds - keep surge velocity and RPM positive
            for k = 1:(obj.N+1)
                obj.opti.subject_to( obj.X(1,k) >= 0.1 );   % u > 0.1 m/s
                obj.opti.subject_to( obj.X(10,k) >= 1 );     % n > 1 RPM
            end
            
            %% Obstacle constraints  (squared-distance, no sqrt)
            if num_obs > 0
                for k = 1:(obj.N+1)
                    for j = 1:num_obs
                        dx = obj.X(4,k) - obj.Obs_pos_param(1,j);
                        dy = obj.X(5,k) - obj.Obs_pos_param(2,j);
                        dist_sq = dx^2 + dy^2;
                        min_d = obj.Obs_rad_param(j) + obj.r_safety - obj.slack(j,k);
                        obj.opti.subject_to( dist_sq >= min_d^2 );
                    end
                end
            end
            
            %% Solver options
            opts = struct;
            opts.ipopt.print_level = 0;
            opts.print_time        = 0;
            opts.ipopt.max_iter    = 500;
            opts.ipopt.tol         = 1e-3;
            opts.ipopt.acceptable_tol = 1e-2;
            opts.ipopt.acceptable_iter = 5;
            opts.ipopt.warm_start_init_point = 'yes';
            opts.ipopt.mu_strategy           = 'adaptive';
            opts.ipopt.nlp_scaling_method    = 'gradient-based';
            
            obj.opti.solver('ipopt', opts);
            
            obj.optimizer_built = true;
            obj.num_obs_built   = num_obs;
            fprintf('Optimizer built (N=%d, obstacles=%d)\n', obj.N, num_obs);
        end
        
        %% SOLVE ==========================================================
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            % solve  Run one NMPC step.
            %
            %   x0        : current state [10x1]
            %   x_ref     : reference trajectory [10 x N+1]
            %   obstacles : struct array with .position [2x1], .radius
            %               (can be empty [])
            
            t_start = tic;

            % ---- assemble full obstacle list (external + map-derived) ---
            if nargin < 4 || isempty(obstacles)
                ext_obs = [];
            else
                ext_obs = obstacles;
            end

            map_obs = [];
            if obj.use_map_obstacles
                map_obs = obj.mapPolygonsToObstacles(x0(4:5));
            end

            if isempty(ext_obs)
                all_obs = map_obs;
            elseif isempty(map_obs)
                all_obs = ext_obs;
            else
                all_obs = [ext_obs(:); map_obs(:)];
            end
            
            % ---- determine actual obstacle count -------------------------
            if ~obj.use_obstacles || isempty(all_obs)
                num_obs = 0;
            else
                num_obs = min(length(all_obs), obj.max_obs);
            end
            
            % ---- (re)build optimizer if needed ---------------------------
            if ~obj.optimizer_built || num_obs ~= obj.num_obs_built
                obj.buildOptimizer(num_obs);
                % clear warm start when rebuilding
                obj.prev_X_sol = [];
                obj.prev_U_sol = [];
            end
            
            % ---- set parameter values ------------------------------------
            obj.opti.set_value(obj.X0_param,    x0);
            obj.opti.set_value(obj.X_ref_param, x_ref);
            u_ref = repmat([0; max(x0(10), obj.n_min_rpm)], 1, obj.N);
            obj.opti.set_value(obj.U_ref_param, u_ref);
            
            if num_obs > 0
                pos_data = zeros(2, num_obs);
                rad_data = zeros(num_obs, 1);
                for j = 1:num_obs
                    pp = all_obs(j).position(:);
                    pos_data(:,j) = pp(1:2);
                    rad_data(j)   = all_obs(j).radius;
                end
                obj.opti.set_value(obj.Obs_pos_param, pos_data);
                obj.opti.set_value(obj.Obs_rad_param, rad_data);
            end
            
            % ---- initial guess -------------------------------------------
            if ~isempty(obj.prev_X_sol)
                % warm-start: shift previous solution by one step
                X_init = [obj.prev_X_sol(:,2:end), obj.prev_X_sol(:,end)];
                U_init = [obj.prev_U_sol(:,2:end), obj.prev_U_sol(:,end)];
            else
                % cold start: propagate with container()
                X_init = zeros(obj.nx, obj.N+1);
                X_init(:,1) = x0;
                U_init = repmat([0; max(x0(10), obj.n_min_rpm)], 1, obj.N);
                for k = 1:obj.N
                    try
                        xk = X_init(:,k);
                        xk(1)  = max(xk(1),  0.1);   % keep u > 0
                        xk(10) = max(xk(10), 1);      % keep n > 0
                        [xdot_k, ~] = container(xk, U_init(:,k));
                        X_init(:,k+1) = xk + xdot_k * obj.dt;
                    catch
                        X_init(:,k+1) = X_init(:,k);
                    end
                end
            end
            obj.opti.set_initial(obj.X, X_init);
            obj.opti.set_initial(obj.U, U_init);
            if num_obs > 0
                obj.opti.set_initial(obj.slack, zeros(num_obs, obj.N+1));
            end
            
            % ---- solve ---------------------------------------------------
            try
                sol = obj.opti.solve();
                
                U_sol  = sol.value(obj.U);
                X_sol  = sol.value(obj.X);
                u_opt  = U_sol(:,1);
                X_pred = X_sol;
                
                % store for warm start
                obj.prev_X_sol = X_sol;
                obj.prev_U_sol = U_sol;
                
                info.success = true;
                info.cost    = sol.value(obj.opti.f);
                obj.solve_ok = obj.solve_ok + 1;
                
            catch ME
                % fallback: zero-rudder, maintain RPM
                u_opt  = [0; max(x0(10), obj.n_min_rpm)];
                X_pred = repmat(x0, 1, obj.N+1);
                
                info.success = false;
                info.error   = ME.message;
                obj.solve_fail = obj.solve_fail + 1;
                
                % Print first few failures to help debug
                if obj.solve_fail <= 3
                    fprintf('  [NMPC FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
                
                % clear warm start so next call does cold-start
                obj.prev_X_sol = [];
                obj.prev_U_sol = [];
            end
            
            info.solve_time = toc(t_start);
        end
        
    end % methods
end
