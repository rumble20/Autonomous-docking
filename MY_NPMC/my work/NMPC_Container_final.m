classdef NMPC_Container_final < handle
    % NMPC_Container_final  NMPC for container ship with dual azipod thrusters
    %
    %   States:  x = [u v r x y psi n1 n2]'           (8)
    %   Controls: u = [alpha1 alpha2 n1_c n2_c]'      (4)

    properties
        N
        dt
        Q
        R
        R_rate
        nx = 8
        nu = 4

        % Ship parameters
        L = 175
        rho = 1025
        nabla = 21222
        D = 6.533
        t_ded = 0.175
        wp = 0.184
        wp_fwd = 0.0552
        x_azi1 = -78.75
        x_azi2 = 61.25

        % Actuator limits
        n_max = 160
        n_min = -80
        alpha_max = pi
        alpha_hard_max = 8*pi
        Dn_max = 10
        alpha_rate_max = 0.21  % Max azimuth rate [rad/s] = 12 deg/s (ABB spec)
        enforce_output_limits = true
        max_limit_event_log = 400
        limit_event_count = 0
        limit_events = struct('step', {}, 'channel', {}, 'requested', {}, 'bounded', {}, 'bound', {}, 'timestamp', {})

        % Obstacle settings
        max_obs = 5
        r_safety = 35

        % Solver internals
        solver
        solver_built = false
        lbx_vec
        ubx_vec
        lbg_vec
        ubg_vec
        np_total

        % Statistics
        solve_ok = 0
        solve_fail = 0

        % Warm-start
        enable_warm_start = true
        prev_sol
        prev_u

        % Lightweight caches for repeated solve setup
        cached_u_ref
        cached_u_ref_key = [NaN; NaN; NaN]
        cached_obs_pos
        cached_obs_rad
        cached_obs_signature = NaN
        cached_obs_count = -1

        enable_diagnostics = false
        enable_heading_hard_constraint = false
        heading_hard_max_err_rad = deg2rad(50)
        enable_corner_obstacle_constraints = true
        hitbox_length_m = 87.5
        hitbox_beam_m = 12.7
        hitbox_safety_margin_m = 2.0
    end

    methods

        function obj = NMPC_Container_final(cfg)
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);
            obj.Q = getOr(cfg, 'Q', diag([2.0, 0.1, 0.8, 3.0, 3.0, 6.0, 0.001, 0.001]));
            obj.R = getOr(cfg, 'R', diag([0.1, 0.1, 0.01, 0.01]));
            obj.R_rate = getOr(cfg, 'R_rate', diag([0.05, 0.05, 0.005, 0.005]));
            obj.max_obs = getOr(cfg, 'max_obs', 5);
            obj.r_safety = getOr(cfg, 'r_safety', 30);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));
            obj.enforce_output_limits = logical(getOr(cfg, 'enforce_output_limits', true));
            obj.enable_warm_start = logical(getOr(cfg, 'enable_warm_start', true));
            obj.max_limit_event_log = getOr(cfg, 'max_limit_event_log', obj.max_limit_event_log);
            obj.enable_heading_hard_constraint = logical(getOr(cfg, 'enable_heading_hard_constraint', false));
            heading_err_deg = getOr(cfg, 'heading_hard_max_err_deg', rad2deg(obj.heading_hard_max_err_rad));
            obj.heading_hard_max_err_rad = deg2rad(max(1.0, min(179.0, heading_err_deg)));
            obj.enable_corner_obstacle_constraints = logical(getOr(cfg, 'enable_corner_obstacle_constraints', true));
            obj.hitbox_length_m = getOr(cfg, 'hitbox_length_m', obj.hitbox_length_m);
            obj.hitbox_beam_m = getOr(cfg, 'hitbox_beam_m', obj.hitbox_beam_m);
            obj.hitbox_safety_margin_m = getOr(cfg, 'hitbox_safety_margin_m', obj.hitbox_safety_margin_m);
            obj.hitbox_length_m = max(1.0, obj.hitbox_length_m);
            obj.hitbox_beam_m = max(0.5, obj.hitbox_beam_m);
            obj.hitbox_safety_margin_m = max(0.0, obj.hitbox_safety_margin_m);

            fprintf('NMPC_Container_final: N=%d, dt=%.2f, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.max_obs);
            fprintf('  8-state model: [u v r x y psi n1 n2]\n');
            fprintf('  4 controls: [alpha1 alpha2 n1_c n2_c]\n');
            fprintf('  fixed azimuth range: [%.2f, %.2f] rad\n', ...
                -obj.alpha_hard_max, obj.alpha_hard_max);
            if obj.enable_warm_start
                fprintf('  warm-start: enabled\n');
            else
                fprintf('  warm-start: disabled\n');
            end
            if obj.enable_heading_hard_constraint
                fprintf('  heading hard corridor: |psi-psi_{guide}| <= %.1f deg\n', ...
                    rad2deg(obj.heading_hard_max_err_rad));
            end
            if obj.enable_corner_obstacle_constraints
                fprintf('  obstacle constraints: four-corner hitbox (L=%.1f, B=%.1f, margin=%.1f m)\n', ...
                    obj.hitbox_length_m, obj.hitbox_beam_m, obj.hitbox_safety_margin_m);
            else
                fprintf('  obstacle constraints: center-point distance\n');
            end
        end

        function buildSolver(obj)
            import casadi.*

            N_h = obj.N;
            nx = obj.nx;
            nu = obj.nu;
            n_obs = obj.max_obs;

            %  DECISION VARIABLES
            X = SX.sym('X', nx, N_h+1);
            U = SX.sym('U', nu, N_h);

            %  PARAMETERS
            P_x0       = SX.sym('P_x0', nx, 1);
            P_xref     = SX.sym('P_xref', nx, N_h+1);
            P_uref     = SX.sym('P_uref', nu, N_h);
            P_n_obs_real = SX.sym('P_n_obs_real', 1, 1);
            P_obs_pos  = SX.sym('P_obs_pos', 2, n_obs);
            P_obs_rad  = SX.sym('P_obs_rad', n_obs, 1);
            P_u_prev   = SX.sym('P_u_prev', nu, 1);  % NEW: Previous applied control
            P_psi_guide = SX.sym('P_psi_guide', N_h+1, 1);

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_n_obs_real, ...
                            P_obs_pos(:), P_obs_rad, P_u_prev, P_psi_guide);
            obj.np_total = size(P_all, 1);

            %  COST FUNCTION
            Q_u   = obj.Q(1,1);  Q_v   = obj.Q(2,2);  Q_r   = obj.Q(3,3);
            Q_x   = obj.Q(4,4);  Q_y   = obj.Q(5,5);  Q_psi = obj.Q(6,6);
            Q_n1  = obj.Q(7,7);  Q_n2  = obj.Q(8,8);

            J = 0;

            % Stage cost
            for k = 1:N_h
                dpsi = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));

                J = J + Q_u   * (X(1,k) - P_xref(1,k))^2;
                J = J + Q_v   * (X(2,k) - P_xref(2,k))^2;
                J = J + Q_r   * (X(3,k) - P_xref(3,k))^2;
                J = J + Q_x   * (X(4,k) - P_xref(4,k))^2;
                J = J + Q_y   * (X(5,k) - P_xref(5,k))^2;
                J = J + Q_psi * psi_err^2;
                J = J + Q_n1  * (X(7,k) - P_xref(7,k))^2;
                J = J + Q_n2  * (X(8,k) - P_xref(8,k))^2;

                du = U(:,k) - P_uref(:,k);
                J = J + du' * obj.R * du;
            end

            % Control rate cost
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J = J + dU' * obj.R_rate * dU;
            end

            % Terminal cost
            dpsi_N = X(6,N_h+1) - P_xref(6,N_h+1);
            psi_err_N = atan2(sin(dpsi_N), cos(dpsi_N));
            J = J + 2*Q_u   * (X(1,N_h+1) - P_xref(1,N_h+1))^2;
            J = J + 2*Q_x   * (X(4,N_h+1) - P_xref(4,N_h+1))^2;
            J = J + 2*Q_y   * (X(5,N_h+1) - P_xref(5,N_h+1))^2;
            J = J + 2*Q_psi * psi_err_N^2;

            %  CONSTRAINTS
            g = [];

            % --- Initial condition (nx equalities) ---
            g = vertcat(g, X(:,1) - P_x0);

            % --- Dynamics (nx * N_h equalities) ---
            for k = 1:N_h
                xdot_k = obj.dynamicsCasADi(X(:,k), U(:,k));
                x_next = X(:,k) + xdot_k * obj.dt;
                g = vertcat(g, X(:,k+1) - x_next);
            end

            % --- Obstacle avoidance (center or 4-corner hitbox inequalities) ---
            L_guard = obj.hitbox_length_m + 2 * obj.hitbox_safety_margin_m;
            B_guard = obj.hitbox_beam_m + 2 * obj.hitbox_safety_margin_m;
            halfL = 0.5 * L_guard;
            halfB = 0.5 * B_guard;

            if obj.enable_corner_obstacle_constraints
                corner_x = [halfL, halfL, -halfL, -halfL];
                corner_y = [halfB, -halfB, -halfB, halfB];
                for k = 1:(N_h+1)
                    psi_k = X(6,k);
                    cpsi = cos(psi_k);
                    spsi = sin(psi_k);
                    for j = 1:n_obs
                        for c = 1:4
                            cx = X(4,k) + cpsi * corner_x(c) - spsi * corner_y(c);
                            cy = X(5,k) + spsi * corner_x(c) + cpsi * corner_y(c);
                            dx = cx - P_obs_pos(1,j);
                            dy = cy - P_obs_pos(2,j);
                            dist = sqrt(dx^2 + dy^2 + 1e-3);
                            g = vertcat(g, dist - P_obs_rad(j) - obj.r_safety);
                        end
                    end
                end
            else
                for k = 1:(N_h+1)
                    for j = 1:n_obs
                        dx = X(4,k) - P_obs_pos(1,j);
                        dy = X(5,k) - P_obs_pos(2,j);
                        dist = sqrt(dx^2 + dy^2 + 1e-3);
                        g = vertcat(g, dist - P_obs_rad(j) - obj.r_safety);
                    end
                end
            end

            % --- Azimuth rate constraints ---
            % FIRST STEP: Constrain U(:,1) relative to previous control
            % This prevents the "spiral oscillation" bug!
            d_alpha1_first = U(1,1) - P_u_prev(1);
            g = vertcat(g, d_alpha1_first - obj.alpha_rate_max * obj.dt);
            g = vertcat(g, -d_alpha1_first - obj.alpha_rate_max * obj.dt);

            d_alpha2_first = U(2,1) - P_u_prev(2);
            g = vertcat(g, d_alpha2_first - obj.alpha_rate_max * obj.dt);
            g = vertcat(g, -d_alpha2_first - obj.alpha_rate_max * obj.dt);

            d_n1_first = U(3,1) - P_u_prev(3);
            g = vertcat(g, d_n1_first - obj.Dn_max * obj.dt);
            g = vertcat(g, -d_n1_first - obj.Dn_max * obj.dt);

            d_n2_first = U(4,1) - P_u_prev(4);
            g = vertcat(g, d_n2_first - obj.Dn_max * obj.dt);
            g = vertcat(g, -d_n2_first - obj.Dn_max * obj.dt);

            % SUBSEQUENT STEPS: Constrain consecutive controls
            for k = 1:(N_h-1)
                d_alpha1 = U(1,k+1) - U(1,k);
                g = vertcat(g, d_alpha1 - obj.alpha_rate_max * obj.dt);
                g = vertcat(g, -d_alpha1 - obj.alpha_rate_max * obj.dt);

                d_alpha2 = U(2,k+1) - U(2,k);
                g = vertcat(g, d_alpha2 - obj.alpha_rate_max * obj.dt);
                g = vertcat(g, -d_alpha2 - obj.alpha_rate_max * obj.dt);

                d_n1 = U(3,k+1) - U(3,k);
                g = vertcat(g, d_n1 - obj.Dn_max * obj.dt);
                g = vertcat(g, -d_n1 - obj.Dn_max * obj.dt);

                d_n2 = U(4,k+1) - U(4,k);
                g = vertcat(g, d_n2 - obj.Dn_max * obj.dt);
                g = vertcat(g, -d_n2 - obj.Dn_max * obj.dt);
            end

            % --- Guidance heading hard corridor ---
            n_heading_ineq = 0;
            if obj.enable_heading_hard_constraint
                cmax = cos(obj.heading_hard_max_err_rad);
                for k = 2:(N_h+1)
                    g = vertcat(g, cos(X(6,k) - P_psi_guide(k)) - cmax);
                    n_heading_ineq = n_heading_ineq + 1;
                end
            end

            %  VARIABLE BOUNDS
            OPT = vertcat(X(:), U(:));
            n_vars = size(OPT, 1);

            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);

            % State bounds
            for k = 1:(N_h+1)
                base = (k-1)*nx;
                lbx(base+1) = 0.1;        ubx(base+1) = 12;
                lbx(base+2) = -3;         ubx(base+2) = 3;
                lbx(base+3) = -0.15;      ubx(base+3) = 0.15;
                lbx(base+4) = -1e5;       ubx(base+4) = 1e5;
                lbx(base+5) = -1e5;       ubx(base+5) = 1e5;
                lbx(base+6) = -inf;       ubx(base+6) = inf;
                lbx(base+7) = obj.n_min;  ubx(base+7) = obj.n_max;
                lbx(base+8) = obj.n_min;  ubx(base+8) = obj.n_max;
            end

            % Control bounds
            u_off = nx * (N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*nu;
                lbx(base+1) = -obj.alpha_hard_max;  ubx(base+1) = obj.alpha_hard_max;
                lbx(base+2) = -obj.alpha_hard_max;  ubx(base+2) = obj.alpha_hard_max;
                lbx(base+3) = obj.n_min;       ubx(base+3) = obj.n_max;
                lbx(base+4) = obj.n_min;       ubx(base+4) = obj.n_max;
            end

            %  CONSTRAINT BOUNDS
            n_eq        = nx + nx*N_h;
            obs_factor = 1;
            if obj.enable_corner_obstacle_constraints
                obs_factor = 4;
            end
            n_obs_ineq  = obs_factor * n_obs * (N_h+1);
            n_rate_ineq = 8 + 8*(N_h-1);  % alpha + shaft-rate constraints
            n_constraints = n_eq + n_obs_ineq + n_rate_ineq + n_heading_ineq;

            lbg = zeros(n_constraints, 1);
            ubg = zeros(n_constraints, 1);

            % Equalities: g = 0
            lbg(1:n_eq) = 0;
            ubg(1:n_eq) = 0;

            % Obstacle: g >= 0
            obs_start = n_eq + 1;
            obs_end   = n_eq + n_obs_ineq;
            lbg(obs_start:obs_end) = 0;
            ubg(obs_start:obs_end) = inf;

            % Rate: g <= 0
            rate_start = obs_end + 1;
            rate_end   = obs_end + n_rate_ineq;
            lbg(rate_start:rate_end) = -inf;
            ubg(rate_start:rate_end) = 0;

            % Heading corridor: g >= 0
            if n_heading_ineq > 0
                head_start = rate_end + 1;
                head_end = n_constraints;
                lbg(head_start:head_end) = 0;
                ubg(head_start:head_end) = inf;
            end

            %  BUILD SOLVER
            nlp = struct('f', J, 'x', OPT, 'g', g, 'p', P_all);

            opts = struct;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.max_iter = 500;
            opts.ipopt.tol = 1e-3;
            opts.ipopt.acceptable_tol = 1e-2;
            opts.ipopt.acceptable_iter = 5;
            opts.ipopt.mu_strategy = 'adaptive';
            opts.ipopt.nlp_scaling_method = 'gradient-based';
            opts.ipopt.sb = 'yes';

            solver_tag = floor(1e7 * rem(now, 1));
            solver_name = sprintf('nmpc_unified_%d', solver_tag);
            obj.solver = nlpsol(solver_name, 'ipopt', nlp, opts);

            obj.lbx_vec = lbx;
            obj.ubx_vec = ubx;
            obj.lbg_vec = lbg;
            obj.ubg_vec = ubg;
            obj.solver_built = true;

            fprintf('  nlpsol built: %d vars, %d constraints\n', n_vars, n_constraints);
            fprintf('    - Equalities: %d (initial + dynamics)\n', n_eq);
            if obj.enable_corner_obstacle_constraints
                fprintf('    - Obstacle:   %d (slots × horizon × 4 corners)\n', n_obs_ineq);
            else
                fprintf('    - Obstacle:   %d (slots × horizon)\n', n_obs_ineq);
            end
            fprintf('    - Rate:       %d (first-step + consecutive)\n', n_rate_ineq);
            if n_heading_ineq > 0
                fprintf('    - Heading:    %d (hard corridor)\n', n_heading_ineq);
            end
        end


        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles, u_prev, psi_guide_ref)
            % solve  Solve NMPC problem
            %
            %   [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles, u_prev)
            %
            %   Inputs:
            %       x0        - Current state (8x1)
            %       x_ref     - Reference trajectory (8 x N+1)
            %       obstacles - Array of obstacles (optional)
            %       u_prev        - Previous applied control (4x1)
            %       psi_guide_ref - Guidance heading profile (N+1 x 1, rad)
            %
            %   Outputs:
            %       u_opt  - Optimal control for current step (4x1)
            %       X_pred - Predicted state trajectory (8 x N+1)
            %       info   - Solver information struct

            t_start = tic;

            if ~obj.solver_built
                obj.buildSolver();
            end

            N_h = obj.N;
            nx = obj.nx;
            nu = obj.nu;
            n_obs = obj.max_obs;

            % Handle previous control
            if nargin < 5 || isempty(u_prev)
                if ~isempty(obj.prev_u)
                    u_prev = obj.prev_u;
                else
                    u_prev = [0; 0; x0(7); x0(8)];
                end
            end

            if nargin < 6 || isempty(psi_guide_ref)
                psi_guide_ref = x_ref(6, :)';
            else
                psi_guide_ref = psi_guide_ref(:);
                if numel(psi_guide_ref) ~= (N_h+1)
                    psi_guide_ref = repmat(psi_guide_ref(1), N_h+1, 1);
                end
            end

            % Obstacle setup with cache reuse for unchanged sets
            [obs_pos, obs_rad, n_real] = obj.packObstacles(obstacles, n_obs);

            % Reference control (cached on current shaft-state and horizon)
            u_ref = obj.getCachedUref(x0(7), x0(8), nu, N_h);

            % Build parameter vector (now includes u_prev)
            p_val = [x0(:); x_ref(:); u_ref(:); n_real; obs_pos(:); obs_rad(:); u_prev(:); psi_guide_ref(:)];

            % Initial guess (warm start)
            if obj.enable_warm_start && obj.hasValidWarmStart(nx, nu, N_h)
                X_prev = reshape(obj.prev_sol(1:nx*(N_h+1)), nx, N_h+1);
                u_s = nx*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s:u_s+nu*N_h-1), nu, N_h);

                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;
                U_init = [U_prev(:,2:end), U_prev(:,end)];

                x0_guess = [X_init(:); U_init(:)];
            else
                X_init = repmat(x0, 1, N_h+1);
                U_init = u_ref;

                for k = 1:N_h
                    xk = X_init(:,k);
                    xk(1) = max(xk(1), 0.1);
                    try
                        [xdot_k, ~] = container(xk, U_init(:,k));
                        if any(isnan(xdot_k)) || any(isinf(xdot_k))
                            X_init(:,k+1) = xk;
                        else
                            X_init(:,k+1) = xk + xdot_k * obj.dt;
                        end
                    catch
                        X_init(:,k+1) = xk;
                    end
                end

                x0_guess = [X_init(:); U_init(:)];
            end

            % Fix initial state bounds
            lbx_local = obj.lbx_vec;
            ubx_local = obj.ubx_vec;
            for i = 1:nx
                lbx_local(i) = x0(i);
                ubx_local(i) = x0(i);
            end

            % Solve
            try
                sol = obj.solver('x0', x0_guess, ...
                    'lbx', lbx_local, 'ubx', ubx_local, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val);

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:nx*(N_h+1)), nx, N_h+1);
                u_s = nx*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s:u_s+nu*N_h-1), nu, N_h);

                u_req = U_sol(:,1);
                [u_opt, lim_events] = obj.enforceControlLimits(u_req, u_prev);
                X_pred = X_sol;

                obj.prev_sol = sol_x;
                obj.prev_u = u_opt;

                info.success = true;
                info.cost = full(sol.f);
                info.n_obs_real = n_real;
                info.limit_events = lim_events;
                obj.solve_ok = obj.solve_ok + 1;

            catch ME
                u_opt = [0; 0; x0(7); x0(8)];
                X_pred = repmat(x0, 1, N_h+1);

                info.success = false;
                info.error = ME.message;
                info.n_obs_real = n_real;
                obj.solve_fail = obj.solve_fail + 1;

                if obj.solve_fail <= 5
                    fprintf('  [NMPC FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
            end

            info.solve_time = toc(t_start);
        end

        function xdot = dynamicsCasADi(obj, x, u_in)
            import casadi.*

            L = obj.L;  rho = obj.rho;  nabla = obj.nabla;
            D = obj.D;  t_ded = obj.t_ded;  wp = obj.wp;  wp_fwd = obj.wp_fwd;
            x_azi1 = obj.x_azi1;  x_azi2 = obj.x_azi2;

            m_ship = rho * nabla;
            Izz_ship = 0.1 * m_ship * L^2;

            m_nd = 0.00792;  mx = 0.000238;  my = 0.007049;
            Iz_nd = 0.000456;  Jz = 0.000419;

            Xuu = -0.0004226;  Xvr = -0.00311;  Xrr = 0.00020;  Xvv = -0.00386;
            Yv = -0.0116;  Yr = 0.00242;  Yvvv = -0.109;  Yrrr = 0.00177;
            Yvvr = 0.0214;  Yvrr = -0.0405;
            Nv = -0.0038545;  Nr = -0.00222;  Nvvv = 0.001492;  Nrrr = -0.00229;
            Nvvr = -0.0424;  Nvrr = 0.00156;

            m11 = m_nd + mx;  m22 = m_nd + my;  m66 = Iz_nd + Jz;

            u = x(1);  v = x(2);  r = x(3);  psi = x(6);  n1 = x(7);  n2 = x(8);
            alpha1 = u_in(1);  alpha2 = u_in(2);  n1_c = u_in(3);  n2_c = u_in(4);

            U = sqrt(u^2 + v^2);
            U = if_else(U < 0.1, 0.1, U);

            u_nd = u / U;  v_nd = v / U;  r_nd = r * L / U;

            X_hyd = Xuu*u_nd^2 + Xvr*v_nd*r_nd + Xvv*v_nd^2 + Xrr*r_nd^2;
            Y_hyd = Yv*v_nd + Yr*r_nd + Yvvv*v_nd^3 + Yrrr*r_nd^3 + Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2;
            N_hyd = Nv*v_nd + Nr*r_nd + Nvvv*v_nd^3 + Nrrr*r_nd^3 + Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2;

            X_hyd = X_hyd + (m_nd + my) * v_nd * r_nd;
            Y_hyd = Y_hyd - (m_nd + mx) * u_nd * r_nd;

            n1_rps = n1 / 60;
            v_local1 = v + r * x_azi1;
            u_inflow1 = u * cos(alpha1) + v_local1 * sin(alpha1);
            u_a1 = u_inflow1 * (1 - wp);

            n1_abs = if_else(n1_rps >= 0, n1_rps, -n1_rps);
            n1_safe = if_else(n1_abs < 0.01, 0.01, n1_abs);
            J1 = u_a1 / (n1_safe * D);
            J1 = if_else(J1 > 1.2, 1.2, if_else(J1 < -0.5, -0.5, J1));

            KT0 = 0.527;  KT1 = -0.455;
            KT1_val = KT0 + KT1 * J1;
            KT1_val = if_else(KT1_val < 0.05, 0.05, KT1_val);

            T1_gross = rho * D^4 * n1_rps * n1_abs * KT1_val;
            T1 = (1 - t_ded) * T1_gross;

            n2_rps = n2 / 60;
            v_local2 = v + r * x_azi2;
            u_inflow2 = u * cos(alpha2) + v_local2 * sin(alpha2);
            u_a2 = u_inflow2 * (1 - wp_fwd);

            n2_abs = if_else(n2_rps >= 0, n2_rps, -n2_rps);
            n2_safe = if_else(n2_abs < 0.01, 0.01, n2_abs);
            J2 = u_a2 / (n2_safe * D);
            J2 = if_else(J2 > 1.2, 1.2, if_else(J2 < -0.5, -0.5, J2));

            KT2_val = KT0 + KT1 * J2;
            KT2_val = if_else(KT2_val < 0.05, 0.05, KT2_val);

            T2_gross = rho * D^4 * n2_rps * n2_abs * KT2_val;
            T2 = (1 - t_ded) * T2_gross;

            Fx1 = T1 * cos(alpha1);  Fy1 = T1 * sin(alpha1);
            Fx2 = T2 * cos(alpha2);  Fy2 = T2 * sin(alpha2);

            X_thrust = Fx1 + Fx2;
            Y_thrust = Fy1 + Fy2;
            N_thrust = x_azi1 * Fy1 + x_azi2 * Fy2;

            X_hyd_dim = X_hyd * 0.5 * rho * L^2 * U^2;
            Y_hyd_dim = Y_hyd * 0.5 * rho * L^2 * U^2;
            N_hyd_dim = N_hyd * 0.5 * rho * L^3 * U^2;

            X_total = X_hyd_dim + X_thrust;
            Y_total = Y_hyd_dim + Y_thrust;
            N_total = N_hyd_dim + N_thrust;

            u_dot = X_total / (m11 * m_ship / m_nd) + v * r;
            v_dot = Y_total / (m22 * m_ship / m_nd) - u * r;
            r_dot = N_total / (m66 * Izz_ship / Iz_nd);

            x_dot = cos(psi) * u - sin(psi) * v;
            y_dot = sin(psi) * u + cos(psi) * v;
            psi_dot = r;

            n1_abs_rpm = if_else(n1 >= 0, n1, -n1);
            n2_abs_rpm = if_else(n2 >= 0, n2, -n2);

            Tm1 = if_else(n1_abs_rpm > 18, 5.65 / (n1_abs_rpm/60 + 1e-6), 18.83);
            Tm2 = if_else(n2_abs_rpm > 18, 5.65 / (n2_abs_rpm/60 + 1e-6), 18.83);
            Tm1 = if_else(Tm1 > 20, 20, if_else(Tm1 < 1, 1, Tm1));
            Tm2 = if_else(Tm2 > 20, 20, if_else(Tm2 < 1, 1, Tm2));

            n1_c_sat = if_else(n1_c > obj.n_max, obj.n_max, if_else(n1_c < obj.n_min, obj.n_min, n1_c));
            n2_c_sat = if_else(n2_c > obj.n_max, obj.n_max, if_else(n2_c < obj.n_min, obj.n_min, n2_c));

            n1_dot = (n1_c_sat - n1) / Tm1;
            n2_dot = (n2_c_sat - n2) / Tm2;

            n1_dot = if_else(n1_dot > obj.Dn_max, obj.Dn_max, if_else(n1_dot < -obj.Dn_max, -obj.Dn_max, n1_dot));
            n2_dot = if_else(n2_dot > obj.Dn_max, obj.Dn_max, if_else(n2_dot < -obj.Dn_max, -obj.Dn_max, n2_dot));

            xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot; n1_dot; n2_dot];
        end

        function [u_safe, events] = enforceControlLimits(obj, u_req, u_prev)
            u_safe = u_req(:);
            events = struct('step', {}, 'channel', {}, 'requested', {}, 'bounded', {}, 'bound', {}, 'timestamp', {});

            if nargin < 3 || isempty(u_prev)
                u_prev = u_safe;
            else
                u_prev = u_prev(:);
            end

            % Always preserve azipod continuity by picking equivalent angles nearest to previous command.
            for ch = 1:2
                u_safe(ch) = obj.unwrapNear(u_safe(ch), u_prev(ch));
            end

            if ~obj.enforce_output_limits
                return;
            end

            ts = now;

            % Hard angle bounds
            for ch = 1:2
                req = u_safe(ch);
                bounded = min(obj.alpha_hard_max, max(-obj.alpha_hard_max, req));
                if bounded ~= req
                    evt = obj.makeLimitEvent(ch, req, bounded, obj.alpha_hard_max, ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                end
            end

            % Angle rate bounds relative to previously applied control
            max_da = obj.alpha_rate_max * obj.dt;
            for ch = 1:2
                req = u_safe(ch);
                delta = req - u_prev(ch);
                if delta > max_da
                    bounded = u_prev(ch) + max_da;
                    evt = obj.makeLimitEvent(ch, req, bounded, max_da, ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                elseif delta < -max_da
                    bounded = u_prev(ch) - max_da;
                    evt = obj.makeLimitEvent(ch, req, bounded, max_da, ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                end
            end

            % Shaft command bounds
            for ch = 3:4
                req = u_safe(ch);
                bounded = min(obj.n_max, max(obj.n_min, req));
                if bounded ~= req
                    evt = obj.makeLimitEvent(ch, req, bounded, max(abs(obj.n_min), abs(obj.n_max)), ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                end
            end

            % Shaft command rate bounds
            max_dn = obj.Dn_max * obj.dt;
            for ch = 3:4
                req = u_safe(ch);
                delta = req - u_prev(ch);
                if delta > max_dn
                    bounded = u_prev(ch) + max_dn;
                    evt = obj.makeLimitEvent(ch, req, bounded, max_dn, ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                elseif delta < -max_dn
                    bounded = u_prev(ch) - max_dn;
                    evt = obj.makeLimitEvent(ch, req, bounded, max_dn, ts);
                    events(end+1) = evt; %#ok<AGROW>
                    obj.recordLimitEvent(evt);
                    u_safe(ch) = bounded;
                end
            end
        end

        function y = unwrapNear(obj, angle_in, ref_angle)
            period = 2*pi;
            k = round((ref_angle - angle_in) / period);
            y = angle_in + k * period;
        end

        function evt = makeLimitEvent(obj, ch, req, bounded, bound_mag, ts)
            channel_names = {'alpha1', 'alpha2', 'n1_c', 'n2_c'};
            evt = struct();
            evt.step = obj.solve_ok + obj.solve_fail + 1;
            evt.channel = channel_names{ch};
            evt.requested = req;
            evt.bounded = bounded;
            evt.bound = bound_mag;
            evt.timestamp = ts;
        end

        function recordLimitEvent(obj, evt)
            obj.limit_event_count = obj.limit_event_count + 1;
            if numel(obj.limit_events) >= obj.max_limit_event_log
                obj.limit_events = obj.limit_events(2:end);
            end
            obj.limit_events(end+1) = evt;
        end

        function tf = hasValidWarmStart(obj, nx, nu, N_h)
            tf = false;
            if isempty(obj.prev_sol)
                return;
            end

            expected_len = nx * (N_h + 1) + nu * N_h;
            if numel(obj.prev_sol) ~= expected_len
                obj.prev_sol = [];
                return;
            end

            if any(~isfinite(obj.prev_sol))
                obj.prev_sol = [];
                return;
            end

            tf = true;
        end

        function u_ref = getCachedUref(obj, n1_ref, n2_ref, nu, N_h)
            key = [n1_ref; n2_ref; N_h];
            if ~isempty(obj.cached_u_ref) && all(abs(obj.cached_u_ref_key - key) < 1e-12)
                u_ref = obj.cached_u_ref;
                return;
            end

            u_ref = zeros(nu, N_h);
            u_ref(3,:) = n1_ref;
            u_ref(4,:) = n2_ref;
            obj.cached_u_ref = u_ref;
            obj.cached_u_ref_key = key;
        end

        function [obs_pos, obs_rad, n_real] = packObstacles(obj, obstacles, n_obs)
            obs_pos = 1e8 * ones(2, n_obs);
            obs_rad = zeros(n_obs, 1);
            n_real = 0;

            if isempty(obstacles)
                sig = 0;
            else
                n_real = min(length(obstacles), n_obs);
                sig = n_real;
                for j = 1:n_real
                    px = obstacles(j).position(1);
                    py = obstacles(j).position(2);
                    rr = obstacles(j).radius;
                    obs_pos(:,j) = [px; py];
                    obs_rad(j) = rr;
                    sig = sig + 1e-3 * j + px + 3 * py + 7 * rr;
                end
            end

            can_reuse = (~isempty(obj.cached_obs_pos)) && ...
                        (~isempty(obj.cached_obs_rad)) && ...
                        (obj.cached_obs_count == n_real) && ...
                        isfinite(obj.cached_obs_signature) && ...
                        abs(obj.cached_obs_signature - sig) < 1e-9;

            if can_reuse
                obs_pos = obj.cached_obs_pos;
                obs_rad = obj.cached_obs_rad;
                return;
            end

            obj.cached_obs_pos = obs_pos;
            obj.cached_obs_rad = obs_rad;
            obj.cached_obs_signature = sig;
            obj.cached_obs_count = n_real;
        end

    end
end

function v = getOr(s, name, default)
    if isfield(s, name)
        v = s.(name);
    else
        v = default;
    end
end
