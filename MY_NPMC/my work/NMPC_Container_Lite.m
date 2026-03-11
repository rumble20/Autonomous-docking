classdef NMPC_Container_Lite < handle
    % NMPC_Container_Lite  NMPC for container ship with dual azipod thrusters
    %
    % Based on Son & Nomoto (1982) hydrodynamics with roll removed.
    % Matches container_azipod.m dynamics exactly.
    %
    %   States:  x = [u v r x y psi n1 n2]'           (8)
    %   Controls: u = [alpha1 alpha2 n1_c n2_c]'      (4)
    %            (azimuth angles in rad, commanded shaft speeds in rpm)
    %
    % Features:
    %   - Son & Nomoto (1982) hydrodynamic coefficients
    %   - Propeller model with KT-J curve, wake fraction, thrust deduction
    %   - Shaft dynamics with time constant
    %   - Twin azipod thrust vectoring (aft + forward)
    %   - Obstacle avoidance via hard sqrt-distance constraints (no slack)
    %   - Control rate-of-change penalty for smooth actuation
    %
    % Usage:
    %   cfg = struct('N', 20, 'dt', 1.0);
    %   nmpc = NMPC_Container_Lite(cfg);
    %   nmpc.buildSolver();
    %   [u_opt, X_pred, info] = nmpc.solve(x0, x_ref, obstacles);
    %
    % Author: Adapted from NMPC_Container_Lite for azipod configuration
    % Date:   2026-03-10

    properties
        % Horizon
        N               % Prediction steps
        dt              % Sample time [s]

        % Cost weights
        Q               % State error weight  (8×8)
        R               % Control effort weight (4×4)
        R_rate          % Control rate-of-change weight (4×4)

        % Dimensions (fixed for 8-state model)
        nx = 8          % States: [u v r x y psi n1 n2]
        nu = 4          % Controls: [alpha1 alpha2 n1_c n2_c]

        % Ship parameters (from container_azipod.m)
        L = 175                 % Ship length [m]
        rho = 1025              % Water density [kg/m³]
        nabla = 21222           % Displacement volume [m³]
        D = 6.533               % Propeller diameter [m]
        t_ded = 0.175           % Thrust deduction factor
        wp = 0.184              % Wake fraction (aft)
        wp_fwd = 0.0552         % Wake fraction (forward) = 0.3 * wp

        % Azipod positions [m from midship]
        x_azi1 = -78.75         % Aft azipod (-0.45 * L)
        x_azi2 = 61.25          % Forward azipod (0.35 * L)

        % Actuator limits
        n_max = 160             % Max shaft speed [rpm]
        n_min = -80             % Min shaft speed [rpm]
        alpha_max = pi          % Max azimuth angle [rad]
        Dn_max = 10             % Max shaft acceleration [rpm/s]

        % Obstacle settings
        max_obs = 5             % Max obstacle slots in NLP (only real ones get constraints)
        r_safety = 30           % Safety margin [m]

        % --- nlpsol internals ---
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

        % Warm-start storage
        prev_sol
        prev_u

        % Debug
        enable_diagnostics = false
    end

    methods

        %% CONSTRUCTOR ====================================================
        function obj = NMPC_Container_Lite(cfg)
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);

            % Q weights: [u, v, r, x, y, psi, n1, n2]
            % Position weights (x,y) must dominate heading (psi) so the
            % optimizer actively steers around obstacles instead of
            % rigidly tracking the desired heading.
            obj.Q = getOr(cfg, 'Q', diag([2.0, 0.1, 0.5, 5.0, 5.0, 3.0, 0.001, 0.001]));

            % R weights: [alpha1, alpha2, n1_c, n2_c]
            obj.R = getOr(cfg, 'R', diag([0.1, 0.1, 0.01, 0.01]));

            % R_rate weights for smooth control
            obj.R_rate = getOr(cfg, 'R_rate', diag([0.05, 0.05, 0.005, 0.005]));

            obj.max_obs = getOr(cfg, 'max_obs', 5);
            obj.r_safety = getOr(cfg, 'r_safety', 30);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));

            fprintf('NMPC_Container_Lite: N=%d, dt=%.2f, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.max_obs);
            fprintf('  8-state model: [u v r x y psi n1 n2]\n');
            fprintf('  4 controls: [alpha1 alpha2 n1_c n2_c]\n');
        end

        %% BUILD SOLVER ===================================================
        function buildSolver(obj)
            import casadi.*

            N_h = obj.N;
            nx = obj.nx;
            nu = obj.nu;
            n_obs = obj.max_obs;

            % Decision variables (no slack — hard obstacle constraints)
            X = SX.sym('X', nx, N_h+1);
            U = SX.sym('U', nu, N_h);

            % Parameters
            P_x0 = SX.sym('P_x0', nx, 1);
            P_xref = SX.sym('P_xref', nx, N_h+1);
            P_uref = SX.sym('P_uref', nu, N_h);
            P_n_obs_real = SX.sym('P_n_obs_real', 1, 1);  % number of active obstacles
            P_obs_pos = SX.sym('P_obs_pos', 2, n_obs);
            P_obs_rad = SX.sym('P_obs_rad', n_obs, 1);

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_n_obs_real, P_obs_pos(:), P_obs_rad);
            obj.np_total = size(P_all, 1);

            % ---- Cost function -------------------------------------------
            Q_u = obj.Q(1,1);
            Q_v = obj.Q(2,2);
            Q_r = obj.Q(3,3);
            Q_x = obj.Q(4,4);
            Q_y = obj.Q(5,5);
            Q_psi = obj.Q(6,6);
            Q_n1 = obj.Q(7,7);
            Q_n2 = obj.Q(8,8);

            J = 0;
            for k = 1:N_h
                % Wrapped heading error
                dpsi = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));
                J = J + Q_psi * psi_err^2;

                % State errors
                J = J + Q_u * (X(1,k) - P_xref(1,k))^2;
                J = J + Q_v * (X(2,k) - P_xref(2,k))^2;
                J = J + Q_r * (X(3,k) - P_xref(3,k))^2;
                J = J + Q_x * (X(4,k) - P_xref(4,k))^2;
                J = J + Q_y * (X(5,k) - P_xref(5,k))^2;
                J = J + Q_n1 * (X(7,k) - P_xref(7,k))^2;
                J = J + Q_n2 * (X(8,k) - P_xref(8,k))^2;

                % Control effort
                du = U(:,k) - P_uref(:,k);
                J = J + du' * obj.R * du;
            end

            % Control rate-of-change penalty
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J = J + dU' * obj.R_rate * dU;
            end

            % Terminal cost (2× stage weights)
            dpsi_N = X(6,N_h+1) - P_xref(6,N_h+1);
            psi_err_N = atan2(sin(dpsi_N), cos(dpsi_N));
            J = J + 2*Q_psi * psi_err_N^2;
            J = J + 2*Q_u * (X(1,N_h+1) - P_xref(1,N_h+1))^2;
            J = J + 2*Q_x * (X(4,N_h+1) - P_xref(4,N_h+1))^2;
            J = J + 2*Q_y * (X(5,N_h+1) - P_xref(5,N_h+1))^2;

            % ---- Constraints ---------------------------------------------
            g = [];

            % Initial condition
            g = vertcat(g, X(:,1) - P_x0);

            % Dynamics (Euler forward)
            for k = 1:N_h
                xdot_k = obj.dynamicsCasADi(X(:,k), U(:,k));
                g = vertcat(g, X(:,k+1) - (X(:,k) + xdot_k * obj.dt));
            end

            % Obstacle avoidance — hard sqrt-distance constraints
            % Following the Python reference implementation:
            %   sqrt(dx² + dy² + ε) - r_obs - r_safety >= 0
            % Only real obstacles get constraints (unused slots are at 1e8
            % so their constraints are trivially satisfied).
            % Using sqrt with a small epsilon (1e-3) prevents gradient
            % singularity at d=0, equivalent to bias trick in Python code.
            for k = 1:(N_h+1)
                for j = 1:n_obs
                    dx = X(4,k) - P_obs_pos(1,j);
                    dy = X(5,k) - P_obs_pos(2,j);
                    dist = sqrt(dx^2 + dy^2 + 1e-3);
                    g = vertcat(g, dist - P_obs_rad(j) - obj.r_safety);
                end
            end

            % ---- Flatten decision variables (no slack) -----------------
            OPT = vertcat(X(:), U(:));
            n_vars = size(OPT, 1);

            % ---- Variable bounds -----------------------------------------
            lbx = -inf(n_vars, 1);
            ubx = inf(n_vars, 1);

            % State bounds
            for k = 1:(N_h+1)
                base = (k-1)*nx;
                lbx(base+1) = 0.1;      ubx(base+1) = 20;       % u [m/s]
                lbx(base+2) = -5;       ubx(base+2) = 5;        % v [m/s]
                lbx(base+3) = -0.5;     ubx(base+3) = 0.5;      % r [rad/s]
                lbx(base+4) = -1e5;     ubx(base+4) = 1e5;      % x [m]
                lbx(base+5) = -1e5;     ubx(base+5) = 1e5;      % y [m]
                lbx(base+6) = -inf;     ubx(base+6) = inf;      % psi [rad]
                lbx(base+7) = obj.n_min; ubx(base+7) = obj.n_max; % n1 [rpm]
                lbx(base+8) = obj.n_min; ubx(base+8) = obj.n_max; % n2 [rpm]
            end

            % Control bounds
            u_off = nx*(N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*nu;
                lbx(base+1) = -obj.alpha_max;  ubx(base+1) = obj.alpha_max;  % alpha1
                lbx(base+2) = -obj.alpha_max;  ubx(base+2) = obj.alpha_max;  % alpha2
                lbx(base+3) = obj.n_min;       ubx(base+3) = obj.n_max;      % n1_c
                lbx(base+4) = obj.n_min;       ubx(base+4) = obj.n_max;      % n2_c
            end

            % ---- Constraint bounds ---------------------------------------
            n_eq = nx + nx*N_h;
            n_ineq = n_obs * (N_h+1);
            lbg = zeros(n_eq + n_ineq, 1);
            ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];

            % ---- Create IPOPT solver -------------------------------------
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

            obj.solver = nlpsol('nmpc_azipod', 'ipopt', nlp, opts);
            obj.lbx_vec = lbx;
            obj.ubx_vec = ubx;
            obj.lbg_vec = lbg;
            obj.ubg_vec = ubg;
            obj.solver_built = true;

            fprintf('  nlpsol built: %d vars, %d constraints\n', n_vars, n_eq+n_ineq);
        end

        %% SOLVE ===========================================================
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            t_start = tic;

            if ~obj.solver_built
                obj.buildSolver();
            end

            N_h = obj.N;
            nx = obj.nx;
            nu = obj.nu;
            n_obs = obj.max_obs;

            % --- Fill obstacle slots --------------------------------------
            obs_pos = 1e8 * ones(2, n_obs);
            obs_rad = zeros(n_obs, 1);
            n_real = 0;

            if nargin >= 4 && ~isempty(obstacles)
                n_real = min(length(obstacles), n_obs);
                for j = 1:n_real
                    obs_pos(:,j) = obstacles(j).position(1:2);
                    obs_rad(j) = obstacles(j).radius;
                end
            end

            % --- Control reference ----------------------------------------
            % Default: zero azimuth, maintain current shaft speeds
            u_ref = zeros(nu, N_h);
            u_ref(3,:) = x0(7);  % n1_c = current n1
            u_ref(4,:) = x0(8);  % n2_c = current n2

            % --- Assemble parameter vector --------------------------------
            % Includes n_real so the solver knows how many obstacles are active
            p_val = [x0(:); x_ref(:); u_ref(:); n_real; obs_pos(:); obs_rad(:)];

            % --- Initial guess --------------------------------------------
            % Always warm-start when a previous solution exists (no gating).
            % This matches the Python implementation which always provides
            % shifted previous solution as initial guess.
            if ~isempty(obj.prev_sol)
                % Warm-start: shift previous solution
                X_prev = reshape(obj.prev_sol(1:nx*(N_h+1)), nx, N_h+1);
                u_s = nx*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s:u_s+nu*N_h-1), nu, N_h);

                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;
                U_init = [U_prev(:,2:end), U_prev(:,end)];

                x0_guess = [X_init(:); U_init(:)];
            else
                % Cold-start
                X_init = repmat(x0, 1, N_h+1);
                U_init = u_ref;

                % Forward propagation for better initial guess
                for k = 1:N_h
                    xk = X_init(:,k);
                    xk(1) = max(xk(1), 0.1);  % Minimum speed
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

            % --- Enforce initial condition --------------------------------
            lbx_local = obj.lbx_vec;
            ubx_local = obj.ubx_vec;
            for i = 1:nx
                lbx_local(i) = x0(i);
                ubx_local(i) = x0(i);
            end

            % --- Solve NLP ------------------------------------------------
            try
                sol = obj.solver('x0', x0_guess, ...
                    'lbx', lbx_local, 'ubx', ubx_local, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val);

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:nx*(N_h+1)), nx, N_h+1);
                u_s = nx*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s:u_s+nu*N_h-1), nu, N_h);

                u_opt = U_sol(:,1);
                X_pred = X_sol;

                obj.prev_sol = sol_x;
                obj.prev_u = u_opt;

                info.success = true;
                info.cost = full(sol.f);
                info.n_obs_real = n_real;
                obj.solve_ok = obj.solve_ok + 1;

            catch ME
                u_opt = [0; 0; x0(7); x0(8)];  % Zero azimuth, maintain rpm
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

        %% DYNAMICS (CasADi) - MATCHES container_azipod.m ==================
        function xdot = dynamicsCasADi(obj, x, u_in)
            % 8-state dynamics matching container_azipod.m exactly
            % States: x = [u v r x_pos y_pos psi n1 n2]
            % Controls: u = [alpha1 alpha2 n1_c n2_c]
            import casadi.*

            % Ship parameters
            L = obj.L;
            rho = obj.rho;
            nabla = obj.nabla;
            D = obj.D;
            t_ded = obj.t_ded;
            wp = obj.wp;
            wp_fwd = obj.wp_fwd;
            x_azi1 = obj.x_azi1;
            x_azi2 = obj.x_azi2;

            % Mass and inertia
            m_ship = rho * nabla;                    % ~21.75 million kg
            Izz_ship = 0.1 * m_ship * L^2;

            % Nondimensional coefficients (Son & Nomoto 1982)
            m_nd = 0.00792;
            mx = 0.000238;
            my = 0.007049;
            Iz_nd = 0.000456;
            Jz = 0.000419;

            % Hydrodynamic derivatives (roll terms removed)
            Xuu = -0.0004226;
            Xvr = -0.00311;
            Xrr = 0.00020;
            Xvv = -0.00386;

            Yv = -0.0116;
            Yr = 0.00242;
            Yvvv = -0.109;
            Yrrr = 0.00177;
            Yvvr = 0.0214;
            Yvrr = -0.0405;

            Nv = -0.0038545;
            Nr = -0.00222;
            Nvvv = 0.001492;
            Nrrr = -0.00229;
            Nvvr = -0.0424;
            Nvrr = 0.00156;

            % Mass matrix components
            m11 = m_nd + mx;
            m22 = m_nd + my;
            m66 = Iz_nd + Jz;

            % Extract states
            u = x(1);
            v = x(2);
            r = x(3);
            psi = x(6);
            n1 = x(7);
            n2 = x(8);

            % Extract controls
            alpha1 = u_in(1);
            alpha2 = u_in(2);
            n1_c = u_in(3);
            n2_c = u_in(4);

            % Speed with safeguard
            U = sqrt(u^2 + v^2);
            U = if_else(U < 0.1, 0.1, U);

            % Nondimensional velocities
            u_nd = u / U;
            v_nd = v / U;
            r_nd = r * L / U;

            % === HYDRODYNAMIC FORCES (nondimensional) ===
            X_hyd = Xuu*u_nd^2 + Xvr*v_nd*r_nd + Xvv*v_nd^2 + Xrr*r_nd^2;
            Y_hyd = Yv*v_nd + Yr*r_nd + Yvvv*v_nd^3 + Yrrr*r_nd^3 + Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2;
            N_hyd = Nv*v_nd + Nr*r_nd + Nvvv*v_nd^3 + Nrrr*r_nd^3 + Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2;

            % Coriolis terms
            X_hyd = X_hyd + (m_nd + my) * v_nd * r_nd;
            Y_hyd = Y_hyd - (m_nd + mx) * u_nd * r_nd;

            % === PROPELLER MODEL (KT-J curve) ===
            % Thruster 1 (aft)
            n1_rps = n1 / 60;
            v_local1 = v + r * x_azi1;
            u_inflow1 = u * cos(alpha1) + v_local1 * sin(alpha1);
            u_a1 = u_inflow1 * (1 - wp);

            % Advance ratio with safeguard
            n1_abs = if_else(n1_rps >= 0, n1_rps, -n1_rps);
            n1_safe = if_else(n1_abs < 0.01, 0.01, n1_abs);
            J1 = u_a1 / (n1_safe * D);
            J1 = if_else(J1 > 1.2, 1.2, if_else(J1 < -0.5, -0.5, J1));

            % KT coefficient
            KT0 = 0.527;
            KT1 = -0.455;
            KT1_val = KT0 + KT1 * J1;
            KT1_val = if_else(KT1_val < 0.05, 0.05, KT1_val);

            % Thrust
            T1_gross = rho * D^4 * n1_rps * n1_abs * KT1_val;
            T1 = (1 - t_ded) * T1_gross;

            % Thruster 2 (forward) - same pattern
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

             % === THRUST FORCES ===
            Fx1 = T1 * cos(alpha1);  Fy1 = T1 * sin(alpha1);
            Fx2 = T2 * cos(alpha2);  Fy2 = T2 * sin(alpha2);

            X_thrust = Fx1 + Fx2;
            Y_thrust = Fy1 + Fy2;
            N_thrust = x_azi1 * Fy1 + x_azi2 * Fy2;

            % === CONVERT TO DIMENSIONAL ===
            X_hyd_dim = X_hyd * 0.5 * rho * L^2 * U^2;
            Y_hyd_dim = Y_hyd * 0.5 * rho * L^2 * U^2;
            N_hyd_dim = N_hyd * 0.5 * rho * L^3 * U^2;

            X_total = X_hyd_dim + X_thrust;
            Y_total = Y_hyd_dim + Y_thrust;
            N_total = N_hyd_dim + N_thrust;

            % === STATE DERIVATIVES ===
            u_dot = X_total / (m11 * m_ship / m_nd) + v * r;
            v_dot = Y_total / (m22 * m_ship / m_nd) - u * r;
            r_dot = N_total / (m66 * Izz_ship / Iz_nd);

            x_dot = cos(psi) * u - sin(psi) * v;
            y_dot = sin(psi) * u + cos(psi) * v;
            psi_dot = r;

            % Shaft dynamics
            n1_abs_rpm = if_else(n1 >= 0, n1, -n1);
            n2_abs_rpm = if_else(n2 >= 0, n2, -n2);

            Tm1 = if_else(n1_abs_rpm > 18, 5.65 / (n1_abs_rpm/60 + 1e-6), 18.83);
            Tm2 = if_else(n2_abs_rpm > 18, 5.65 / (n2_abs_rpm/60 + 1e-6), 18.83);
            Tm1 = if_else(Tm1 > 20, 20, if_else(Tm1 < 1, 1, Tm1));
            Tm2 = if_else(Tm2 > 20, 20, if_else(Tm2 < 1, 1, Tm2));

            % Saturate commanded values
            n1_c_sat = if_else(n1_c > obj.n_max, obj.n_max, if_else(n1_c < obj.n_min, obj.n_min, n1_c));
            n2_c_sat = if_else(n2_c > obj.n_max, obj.n_max, if_else(n2_c < obj.n_min, obj.n_min, n2_c));

            n1_dot = (n1_c_sat - n1) / Tm1;
            n2_dot = (n2_c_sat - n2) / Tm2;

            % Rate limiting
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
