classdef NMPC_Container_RTI < handle
    % NMPC_Container_RTI  Real-Time-Iteration NMPC for 8-state azipod vessel
    %
    % Replaces NMPC_Container_Lite with:
    %   1) RK4 multiple-shooting (matches plant integrator exactly)
    %   2) CasADi SQP-RTI (sqpmethod + qrqp) for real-time feasibility
    %   3) Time-indexed obstacle positions — ready for AIS-tracked vessels
    %
    %   States:   x = [u v r x y psi n1 n2]'           (8)
    %   Controls: u = [alpha1 alpha2 n1_c n2_c]'        (4)
    %
    % Usage:
    %   cfg = struct('N', 30, 'dt', 1.0);
    %   nmpc = NMPC_Container_RTI(cfg);
    %   nmpc.buildSolver();
    %   [u_opt, X_pred, info] = nmpc.solve(x0, x_ref, obstacles);
    %
    % Dynamic obstacle interface (future-ready):
    %   obstacles(j).position(:,k) = [x_obs; y_obs] at horizon step k
    %   obstacles(j).radius         = scalar radius [m]
    %   obstacles(j).velocity       = [vx; vy]  (optional, auto-predicted)
    %
    % Author: Auto-generated rebuild from NMPC_Container_Lite
    % Date:   2026-03-10

    properties
        % Horizon
        N               % Prediction steps
        dt              % Sample time [s]
        M_rk4 = 4       % RK4 sub-steps per shooting interval

        % Cost weights
        Q               % State error weight  (8×8)
        R               % Control effort weight (4×4)
        R_rate          % Control rate-of-change weight (4×4)
        Q_terminal      % Terminal state weight (8×8)

        % Dimensions
        nx = 8
        nu = 4

        % Ship parameters (Son & Nomoto 1982 + azipods)
        L       = 175
        rho     = 1025
        nabla   = 21222
        D       = 6.533
        t_ded   = 0.175
        wp      = 0.184
        wp_fwd  = 0.0552
        x_azi1  = -78.75
        x_azi2  = 61.25

        % Actuator limits
        n_max     = 160
        n_min     = -80
        alpha_max = pi
        Dn_max    = 10

        % Obstacle settings
        max_obs       = 15
        r_safety      = 30
        penalty_slack = 1e6

        % Solver internals
        solver
        solver_built = false
        lbx_vec
        ubx_vec
        lbg_vec
        ubg_vec
        np_total

        % CasADi integrator function (for cold-start propagation)
        F_rk4

        % Statistics
        solve_ok   = 0
        solve_fail = 0

        % Warm-start
        prev_sol
        prev_u
        prev_lam_g
        prev_lam_x

        % RTI settings
        max_sqp_iter = 1    % 1 = pure RTI (single QP), >1 = multi-iteration SQP
        qp_max_iter  = 200

        % Debug
        enable_diagnostics = false
    end

    methods

        %% CONSTRUCTOR ====================================================
        function obj = NMPC_Container_RTI(cfg)
            if nargin < 1, cfg = struct(); end

            obj.N      = getOr(cfg, 'N', 30);
            obj.dt     = getOr(cfg, 'dt', 1.0);
            obj.M_rk4  = getOr(cfg, 'M_rk4', 4);

            obj.Q      = getOr(cfg, 'Q', diag([2.0, 0.1, 0.8, 0.03, 0.03, 15, 0.001, 0.001]));
            obj.R      = getOr(cfg, 'R', diag([0.1, 0.1, 0.01, 0.01]));
            obj.R_rate = getOr(cfg, 'R_rate', diag([0.05, 0.05, 0.005, 0.005]));

            % Terminal weight: heavier on position/heading to pull toward goal
            Q_term_default = 2 * obj.Q;
            Q_term_default(4,4) = 4 * obj.Q(4,4);
            Q_term_default(5,5) = 4 * obj.Q(5,5);
            obj.Q_terminal = getOr(cfg, 'Q_terminal', Q_term_default);

            obj.max_obs       = getOr(cfg, 'max_obs', 15);
            obj.r_safety      = getOr(cfg, 'r_safety', 30);
            obj.penalty_slack = getOr(cfg, 'penalty_slack', 1e6);

            obj.max_sqp_iter  = getOr(cfg, 'max_sqp_iter', 1);
            obj.qp_max_iter   = getOr(cfg, 'qp_max_iter', 200);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));

            fprintf('NMPC_Container_RTI: N=%d, dt=%.2f, M_rk4=%d, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.M_rk4, obj.max_obs);
            fprintf('  SQP-RTI: max_iter=%d, qp_max_iter=%d\n', ...
                obj.max_sqp_iter, obj.qp_max_iter);
            fprintf('  8-state model, RK4 multiple-shooting, time-indexed obstacles\n');
        end

        %% BUILD SOLVER ===================================================
        function buildSolver(obj)
            import casadi.*

            N_h   = obj.N;
            nX    = obj.nx;
            nU    = obj.nu;
            n_obs = obj.max_obs;

            % -- Build CasADi RK4 integrator function ---------------------
            x_sym = SX.sym('x_rk4', nX, 1);
            u_sym = SX.sym('u_rk4', nU, 1);
            x_next = obj.rk4Integrate(x_sym, u_sym);
            obj.F_rk4 = Function('F_rk4', {x_sym, u_sym}, {x_next});

            % -- Decision variables ----------------------------------------
            X     = SX.sym('X', nX, N_h+1);
            U     = SX.sym('U', nU, N_h);
            slack = SX.sym('slack', n_obs, N_h+1);

            % -- Parameters ------------------------------------------------
            P_x0     = SX.sym('P_x0', nX, 1);
            P_xref   = SX.sym('P_xref', nX, N_h+1);
            P_uref   = SX.sym('P_uref', nU, N_h);

            % TIME-INDEXED obstacle positions: obs_pos(2, n_obs, N_h+1)
            % Flattened column-major: [obs1_x_t0; obs1_y_t0; obs2_x_t0; ... ; obs1_x_t1; ...]
            P_obs_pos = SX.sym('P_obs_pos', 2*n_obs, N_h+1);
            P_obs_rad = SX.sym('P_obs_rad', n_obs, 1);

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_obs_pos(:), P_obs_rad);
            obj.np_total = size(P_all, 1);

            % -- Cost function ---------------------------------------------
            J = 0;

            for k = 1:N_h
                % Wrapped heading error
                dpsi = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));

                % State tracking error
                ex = X(:,k) - P_xref(:,k);
                ex(6) = psi_err;  % Replace raw heading diff with wrapped version
                J = J + ex' * obj.Q * ex;

                % Control effort
                du = U(:,k) - P_uref(:,k);
                J = J + du' * obj.R * du;
            end

            % Control rate-of-change penalty
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J = J + dU' * obj.R_rate * dU;
            end

            % Terminal cost
            dpsi_N = X(6,N_h+1) - P_xref(6,N_h+1);
            ex_N = X(:,N_h+1) - P_xref(:,N_h+1);
            ex_N(6) = atan2(sin(dpsi_N), cos(dpsi_N));
            J = J + ex_N' * obj.Q_terminal * ex_N;

            % Slack penalty (L1 + L2)
            J = J + obj.penalty_slack * sumsqr(slack) + 1000 * sumsqr(slack.^2);

            % -- Constraints -----------------------------------------------
            g = [];

            % Initial condition: X(:,1) == x0
            g = vertcat(g, X(:,1) - P_x0);

            % Dynamics: RK4 multiple-shooting gaps
            for k = 1:N_h
                x_k_end = obj.rk4Integrate(X(:,k), U(:,k));
                g = vertcat(g, X(:,k+1) - x_k_end);
            end

            % Obstacle avoidance — TIME-INDEXED
            for k = 1:(N_h+1)
                % Extract obstacle positions at horizon step k
                obs_k = P_obs_pos(:, k);  % [2*n_obs × 1]
                for j = 1:n_obs
                    ox = obs_k(2*(j-1)+1);
                    oy = obs_k(2*(j-1)+2);
                    dx = X(4,k) - ox;
                    dy = X(5,k) - oy;
                    dist_sq = dx^2 + dy^2;
                    min_d = P_obs_rad(j) + obj.r_safety - slack(j,k);
                    g = vertcat(g, dist_sq - min_d^2);
                end
            end

            % -- Flatten decision variables --------------------------------
            OPT = vertcat(X(:), U(:), slack(:));
            n_vars = size(OPT, 1);

            % -- Variable bounds -------------------------------------------
            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);

            % State bounds over horizon
            for k = 1:(N_h+1)
                base = (k-1)*nX;
                lbx(base+1) = 0.1;          ubx(base+1) = 20;         % u
                lbx(base+2) = -5;            ubx(base+2) = 5;          % v
                lbx(base+3) = -0.5;          ubx(base+3) = 0.5;        % r
                lbx(base+4) = -1e5;          ubx(base+4) = 1e5;        % x
                lbx(base+5) = -1e5;          ubx(base+5) = 1e5;        % y
                lbx(base+6) = -inf;          ubx(base+6) = inf;        % psi
                lbx(base+7) = obj.n_min;     ubx(base+7) = obj.n_max;  % n1
                lbx(base+8) = obj.n_min;     ubx(base+8) = obj.n_max;  % n2
            end

            % Control bounds
            u_off = nX*(N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*nU;
                lbx(base+1) = -obj.alpha_max;  ubx(base+1) = obj.alpha_max;
                lbx(base+2) = -obj.alpha_max;  ubx(base+2) = obj.alpha_max;
                lbx(base+3) = obj.n_min;       ubx(base+3) = obj.n_max;
                lbx(base+4) = obj.n_min;       ubx(base+4) = obj.n_max;
            end

            % Slack >= 0
            s_off = u_off + nU*N_h;
            lbx(s_off+1:end) = 0;

            % -- Constraint bounds -----------------------------------------
            n_eq   = nX + nX*N_h;            % IC + shooting gaps
            n_ineq = n_obs * (N_h+1);       % obstacle constraints
            lbg = zeros(n_eq + n_ineq, 1);
            ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];

            % -- Create SQP-RTI solver ------------------------------------
            nlp = struct('f', J, 'x', OPT, 'g', g, 'p', P_all);

            opts = struct;
            opts.print_time = 0;
            opts.print_header = 0;

            % SQP method settings
            opts.max_iter     = obj.max_sqp_iter;   % 1 = RTI
            opts.print_iteration = false;
            opts.print_status = false;
            opts.tol_pr  = 1e-4;    % primal tolerance  (feasibility)
            opts.tol_du  = 1e-4;    % dual tolerance    (optimality)

            % QP solver: qrqp (CasADi built-in, no external deps)
            opts.qpsol = 'qrqp';
            opts.qpsol_options = struct('print_iter', false, ...
                                        'print_header', false, ...
                                        'max_iter', obj.qp_max_iter);

            % Convexification for SQP stability
            opts.convexify_strategy = 'regularize';
            opts.convexify_margin   = 1e-5;

            obj.solver = nlpsol('nmpc_rti', 'sqpmethod', nlp, opts);
            obj.lbx_vec = lbx;
            obj.ubx_vec = ubx;
            obj.lbg_vec = lbg;
            obj.ubg_vec = ubg;
            obj.solver_built = true;

            fprintf('  SQP-RTI solver built: %d vars, %d eq + %d ineq constraints\n', ...
                n_vars, n_eq, n_ineq);
            fprintf('  RK4 multiple-shooting: %d sub-steps per interval (dt=%.2fs)\n', ...
                obj.M_rk4, obj.dt);
            fprintf('  Time-indexed obstacles: %d slots × %d horizon steps\n', ...
                n_obs, N_h+1);
        end

        %% SOLVE ===========================================================
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            t_start = tic;

            if ~obj.solver_built
                obj.buildSolver();
            end

            N_h   = obj.N;
            nX    = obj.nx;
            nU    = obj.nu;
            n_obs = obj.max_obs;

            % --- Fill TIME-INDEXED obstacle slots -------------------------
            % obs_pos_all: (2*n_obs) × (N_h+1) — each column is one horizon step
            obs_pos_all = 1e8 * ones(2*n_obs, N_h+1);
            obs_rad     = zeros(n_obs, 1);
            n_real = 0;

            if nargin >= 4 && ~isempty(obstacles)
                n_real = min(length(obstacles), n_obs);
                for j = 1:n_real
                    obs_rad(j) = obstacles(j).radius;

                    % Check if full per-step trajectory is provided
                    if isfield(obstacles(j), 'position') && size(obstacles(j).position, 2) >= (N_h+1)
                        % Dynamic obstacle: use provided per-step positions
                        for k = 1:(N_h+1)
                            obs_pos_all(2*(j-1)+1, k) = obstacles(j).position(1, k);
                            obs_pos_all(2*(j-1)+2, k) = obstacles(j).position(2, k);
                        end
                    elseif isfield(obstacles(j), 'velocity') && ~isempty(obstacles(j).velocity)
                        % Dynamic obstacle with constant velocity prediction
                        p0 = obstacles(j).position(1:2, 1);
                        vel = obstacles(j).velocity(1:2);
                        for k = 1:(N_h+1)
                            t_k = (k-1) * obj.dt;
                            obs_pos_all(2*(j-1)+1, k) = p0(1) + vel(1) * t_k;
                            obs_pos_all(2*(j-1)+2, k) = p0(2) + vel(2) * t_k;
                        end
                    else
                        % Static obstacle: replicate position across all steps
                        p0 = obstacles(j).position(1:2, 1);
                        for k = 1:(N_h+1)
                            obs_pos_all(2*(j-1)+1, k) = p0(1);
                            obs_pos_all(2*(j-1)+2, k) = p0(2);
                        end
                    end
                end
            end

            % --- Control reference ----------------------------------------
            u_ref = zeros(nU, N_h);
            u_ref(3,:) = x0(7);
            u_ref(4,:) = x0(8);

            % --- Assemble parameter vector --------------------------------
            p_val = [x0(:); x_ref(:); u_ref(:); obs_pos_all(:); obs_rad(:)];

            % --- Initial guess / warm-start --------------------------------
            if ~isempty(obj.prev_sol) && (obj.solve_ok > 1)
                % Warm-start: shift previous solution forward
                X_prev = reshape(obj.prev_sol(1:nX*(N_h+1)), nX, N_h+1);
                u_s = nX*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s:u_s+nU*N_h-1), nU, N_h);
                s_s = u_s + nU*N_h;
                s_prev = reshape(obj.prev_sol(s_s:end), n_obs, N_h+1);

                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;
                U_init = [U_prev(:,2:end), U_prev(:,end)];
                s_init = [s_prev(:,2:end), s_prev(:,end)];

                x0_guess = [X_init(:); U_init(:); s_init(:)];
            else
                % Cold-start: RK4 forward propagation
                X_init = zeros(nX, N_h+1);
                X_init(:,1) = x0;
                U_init = u_ref;

                for k = 1:N_h
                    xk = X_init(:,k);
                    xk(1) = max(xk(1), 0.1);
                    try
                        x_next = full(obj.F_rk4(xk, U_init(:,k)));
                        if any(isnan(x_next)) || any(isinf(x_next))
                            X_init(:,k+1) = xk;
                        else
                            X_init(:,k+1) = x_next;
                        end
                    catch
                        X_init(:,k+1) = xk;
                    end
                end

                s_init = zeros(n_obs, N_h+1);
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            end

            % --- Enforce initial condition --------------------------------
            lbx_local = obj.lbx_vec;
            ubx_local = obj.ubx_vec;
            for i = 1:nX
                lbx_local(i) = x0(i);
                ubx_local(i) = x0(i);
            end

            % --- Solve NLP (SQP-RTI) -------------------------------------
            try
                sol_args = struct( ...
                    'x0',  x0_guess, ...
                    'lbx', lbx_local, 'ubx', ubx_local, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p',   p_val);

                % Supply dual warm-start if available
                if ~isempty(obj.prev_lam_g)
                    sol_args.lam_g0 = obj.prev_lam_g;
                end
                if ~isempty(obj.prev_lam_x)
                    sol_args.lam_x0 = obj.prev_lam_x;
                end

                sol = obj.solver(sol_args);

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:nX*(N_h+1)), nX, N_h+1);
                u_s = nX*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s:u_s+nU*N_h-1), nU, N_h);

                u_opt  = U_sol(:,1);
                X_pred = X_sol;

                % Store for warm-start
                obj.prev_sol   = sol_x;
                obj.prev_u     = u_opt;
                obj.prev_lam_g = full(sol.lam_g);
                obj.prev_lam_x = full(sol.lam_x);

                info.success    = true;
                info.cost       = full(sol.f);
                info.n_obs_real = n_real;
                obj.solve_ok    = obj.solve_ok + 1;

            catch ME
                u_opt  = [0; 0; x0(7); x0(8)];
                X_pred = repmat(x0, 1, N_h+1);

                info.success    = false;
                info.error      = ME.message;
                info.n_obs_real = n_real;
                obj.solve_fail  = obj.solve_fail + 1;

                if obj.solve_fail <= 5
                    fprintf('  [RTI FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
            end

            info.solve_time = toc(t_start);

            if obj.enable_diagnostics
                fprintf('  RTI solve: %.1f ms | ok=%d fail=%d\n', ...
                    info.solve_time*1000, obj.solve_ok, obj.solve_fail);
            end
        end

        %% RK4 INTEGRATION (CasADi symbolic) ==============================
        function x_next = rk4Integrate(obj, x, u)
            % Fixed-step RK4 with M sub-steps over one dt interval
            % Matches plant-side rk4Step8 integration exactly
            h = obj.dt / obj.M_rk4;
            x_k = x;
            for m = 1:obj.M_rk4
                k1 = obj.dynamicsCasADi(x_k,           u);
                k2 = obj.dynamicsCasADi(x_k + h/2*k1,  u);
                k3 = obj.dynamicsCasADi(x_k + h/2*k2,  u);
                k4 = obj.dynamicsCasADi(x_k + h*k3,    u);
                x_k = x_k + h/6 * (k1 + 2*k2 + 2*k3 + k4);
            end
            x_next = x_k;
        end

        %% DYNAMICS (CasADi) - MATCHES container_azipod.m =================
        function xdot = dynamicsCasADi(obj, x, u_in)
            import casadi.*

            L_ship  = obj.L;
            rho_w   = obj.rho;
            nabla_v = obj.nabla;
            D_prop  = obj.D;
            t_d     = obj.t_ded;
            wp_aft  = obj.wp;
            wp_f    = obj.wp_fwd;
            xa1     = obj.x_azi1;
            xa2     = obj.x_azi2;

            m_ship   = rho_w * nabla_v;
            Izz_ship = 0.1 * m_ship * L_ship^2;

            % Son & Nomoto 1982 nondimensional coefficients
            m_nd  = 0.00792;   mx = 0.000238;  my = 0.007049;
            Iz_nd = 0.000456;  Jz = 0.000419;

            Xuu = -0.0004226;  Xvr = -0.00311;  Xrr = 0.00020;  Xvv = -0.00386;
            Yv  = -0.0116;     Yr  = 0.00242;
            Yvvv = -0.109;     Yrrr = 0.00177;  Yvvr = 0.0214;  Yvrr = -0.0405;
            Nv  = -0.0038545;  Nr  = -0.00222;
            Nvvv = 0.001492;   Nrrr = -0.00229; Nvvr = -0.0424; Nvrr = 0.00156;

            m11 = m_nd + mx;
            m22 = m_nd + my;
            m66 = Iz_nd + Jz;

            u_s   = x(1);  v = x(2);  r = x(3);
            psi   = x(6);  n1 = x(7); n2 = x(8);
            alpha1 = u_in(1); alpha2 = u_in(2);
            n1_c   = u_in(3); n2_c   = u_in(4);

            U_spd = sqrt(u_s^2 + v^2);
            U_spd = if_else(U_spd < 0.1, 0.1, U_spd);

            u_nd = u_s / U_spd;
            v_nd = v   / U_spd;
            r_nd = r * L_ship / U_spd;

            % Hydrodynamic forces (nondimensional)
            X_hyd = Xuu*u_nd^2 + Xvr*v_nd*r_nd + Xvv*v_nd^2 + Xrr*r_nd^2;
            Y_hyd = Yv*v_nd + Yr*r_nd + Yvvv*v_nd^3 + Yrrr*r_nd^3 ...
                  + Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2;
            N_hyd = Nv*v_nd + Nr*r_nd + Nvvv*v_nd^3 + Nrrr*r_nd^3 ...
                  + Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2;

            % Coriolis
            X_hyd = X_hyd + (m_nd + my) * v_nd * r_nd;
            Y_hyd = Y_hyd - (m_nd + mx) * u_nd * r_nd;

            % Propeller model — thruster 1 (aft)
            n1_rps = n1 / 60;
            v_local1 = v + r * xa1;
            u_inflow1 = u_s * cos(alpha1) + v_local1 * sin(alpha1);
            u_a1 = u_inflow1 * (1 - wp_aft);
            n1_abs = if_else(n1_rps >= 0, n1_rps, -n1_rps);
            n1_safe = if_else(n1_abs < 0.01, 0.01, n1_abs);
            J1 = u_a1 / (n1_safe * D_prop);
            J1 = if_else(J1 > 1.2, 1.2, if_else(J1 < -0.5, -0.5, J1));
            KT0 = 0.527;  KT1_c = -0.455;
            KT1_val = KT0 + KT1_c * J1;
            KT1_val = if_else(KT1_val < 0.05, 0.05, KT1_val);
            T1 = (1 - t_d) * rho_w * D_prop^4 * n1_rps * n1_abs * KT1_val;

            % Propeller model — thruster 2 (fwd)
            n2_rps = n2 / 60;
            v_local2 = v + r * xa2;
            u_inflow2 = u_s * cos(alpha2) + v_local2 * sin(alpha2);
            u_a2 = u_inflow2 * (1 - wp_f);
            n2_abs = if_else(n2_rps >= 0, n2_rps, -n2_rps);
            n2_safe = if_else(n2_abs < 0.01, 0.01, n2_abs);
            J2 = u_a2 / (n2_safe * D_prop);
            J2 = if_else(J2 > 1.2, 1.2, if_else(J2 < -0.5, -0.5, J2));
            KT2_val = KT0 + KT1_c * J2;
            KT2_val = if_else(KT2_val < 0.05, 0.05, KT2_val);
            T2 = (1 - t_d) * rho_w * D_prop^4 * n2_rps * n2_abs * KT2_val;

            % Thrust forces
            Fx1 = T1 * cos(alpha1);  Fy1 = T1 * sin(alpha1);
            Fx2 = T2 * cos(alpha2);  Fy2 = T2 * sin(alpha2);
            X_thrust = Fx1 + Fx2;
            Y_thrust = Fy1 + Fy2;
            N_thrust = xa1 * Fy1 + xa2 * Fy2;

            % Convert to dimensional
            X_hyd_dim = X_hyd * 0.5 * rho_w * L_ship^2 * U_spd^2;
            Y_hyd_dim = Y_hyd * 0.5 * rho_w * L_ship^2 * U_spd^2;
            N_hyd_dim = N_hyd * 0.5 * rho_w * L_ship^3 * U_spd^2;

            X_total = X_hyd_dim + X_thrust;
            Y_total = Y_hyd_dim + Y_thrust;
            N_total = N_hyd_dim + N_thrust;

            % State derivatives
            u_dot   = X_total / (m11 * m_ship / m_nd) + v * r;
            v_dot   = Y_total / (m22 * m_ship / m_nd) - u_s * r;
            r_dot   = N_total / (m66 * Izz_ship / Iz_nd);
            x_dot   = cos(psi) * u_s - sin(psi) * v;
            y_dot   = sin(psi) * u_s + cos(psi) * v;
            psi_dot = r;

            % Shaft dynamics (same as container.m)
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

    end % methods
end

%% ========================================================================
function v = getOr(s, name, default)
    if isfield(s, name)
        v = s.(name);
    else
        v = default;
    end
end
