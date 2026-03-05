classdef NMPC_Container_Lite < handle
    % NMPC_Container_Lite  Simplified NMPC for container ship (CasADi nlpsol)
    %
    % Minimal viable NMPC for path-following + static obstacle avoidance.
    % Uses the full 10-state nonlinear container.m dynamics in CasADi.
    %
    %   States:  x = [u v r x y psi p phi delta n]'   (10)
    %   Controls: u = [delta_c  n_c]'                  (2)
    %
    % Features kept:
    %   - Full nonlinear dynamics (matches container.m line-by-line)
    %   - Obstacle avoidance via slack-penalised distance constraints
    %   - Control rate-of-change penalty (R_rate) for smooth rudder
    %   - Build-once / solve-many nlpsol architecture
    %
    % Removed vs full version (deferred to Phase 2+):
    %   - Environment / current effects
    %   - Map polygon integration
    %   - Warm start shifting
    %   - Simplified (linear) dynamics option
    %
    % Usage:
    %   cfg = struct('N', 20, 'dt', 1.0, ...
    %                'Q', diag([2 0 0.8 0.03 0.03 15 0 0 0 0]), ...
    %                'R', diag([0.005, 0.001]));
    %   nmpc = NMPC_Container_Lite(cfg);
    %   nmpc.buildSolver();
    %   [u_opt, X_pred, info] = nmpc.solve(x0, x_ref, obstacles);
    %
    % Author: Riccardo Legnini (simplified from NMPC_Container)
    % Date:   2025

    properties
        % Horizon
        N               % Prediction steps
        dt              % Sample time [s]

        % Cost weights
        Q               % State error weight  (10×10)
        R               % Control effort weight (2×2)
        R_rate          % Control rate-of-change weight (2×2)

        % Dimensions (fixed)
        nx = 10
        nu = 2

        % Actuator limits (from container.m)
        delta_max_rad = deg2rad(35)
        n_max_rpm     = 160
        n_min_rpm     = 10

        % Obstacle settings
        max_obs       = 15      % Fixed obstacle slots in NLP
        r_safety      = 30      % Safety margin [m]
        penalty_slack = 10000   % Slack penalty in cost

        % --- nlpsol internals ---
        solver
        solver_built = false
        lbx_vec
        ubx_vec
        lbg_vec
        ubg_vec
        np_total

        % Statistics
        solve_ok   = 0
        solve_fail = 0
        
        % Warm-start storage
        prev_sol       % Previous solution vector for warm-start
        prev_u         % Previous control for rate-of-change
    end

    methods

        %% CONSTRUCTOR ====================================================
        function obj = NMPC_Container_Lite(cfg)
            % Handle missing or empty config
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            % Required fields with defaults
            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);
            obj.Q  = getOr(cfg, 'Q',  diag([2.0, 0, 0.8, 0.03, 0.03, 15, 0, 0, 0, 0]));
            obj.R  = getOr(cfg, 'R',  diag([0.005, 0.001]));

            % Optional fields with defaults
            obj.R_rate = getOr(cfg, 'R_rate', diag([0.08, 0.001]));
            obj.max_obs       = getOr(cfg, 'max_obs',       15);
            obj.r_safety      = getOr(cfg, 'r_safety',      30);
            obj.penalty_slack = getOr(cfg, 'penalty_slack',  10000);

            fprintf('NMPC_Container_Lite: N=%d, dt=%.2f, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.max_obs);
        end

        %% BUILD SOLVER (called once) =====================================
        function buildSolver(obj)
            import casadi.*

            N_h   = obj.N;
            nx    = obj.nx;
            nu    = obj.nu;
            n_obs = obj.max_obs;

            % Decision variables
            X     = SX.sym('X',     nx, N_h+1);
            U     = SX.sym('U',     nu, N_h);
            slack = SX.sym('slack', n_obs, N_h+1);

            % Parameters
            P_x0      = SX.sym('P_x0',      nx, 1);
            P_xref    = SX.sym('P_xref',     nx, N_h+1);
            P_uref    = SX.sym('P_uref',     nu, N_h);
            P_obs_pos = SX.sym('P_obs_pos',  2,  n_obs);
            P_obs_rad = SX.sym('P_obs_rad',  n_obs, 1);

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_obs_pos(:), P_obs_rad);
            obj.np_total = size(P_all, 1);

            % ---- Cost function -------------------------------------------
            Q_u   = obj.Q(1,1);
            Q_r   = obj.Q(3,3);
            Q_x   = obj.Q(4,4);
            Q_y   = obj.Q(5,5);
            Q_psi = obj.Q(6,6);

            J = 0;
            for k = 1:N_h
                % Wrapped heading error
                dpsi    = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));
                J = J + Q_psi * psi_err^2;

                % Speed + position errors
                J = J + Q_u * (X(1,k) - P_xref(1,k))^2;
                J = J + Q_x * (X(4,k) - P_xref(4,k))^2;
                J = J + Q_y * (X(5,k) - P_xref(5,k))^2;

                % Yaw rate error
                J = J + Q_r * (X(3,k) - P_xref(3,k))^2;

                % Control effort
                du = U(:,k) - P_uref(:,k);
                J  = J + du' * obj.R * du;
            end

            % Control rate-of-change penalty
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J  = J + dU' * obj.R_rate * dU;
            end

            % Terminal cost (2× stage weights on heading, speed, position)
            dpsi_N    = X(6,N_h+1) - P_xref(6,N_h+1);
            psi_err_N = atan2(sin(dpsi_N), cos(dpsi_N));
            J = J + 2*Q_psi * psi_err_N^2;
            J = J + 2*Q_r   * (X(3,N_h+1) - P_xref(3,N_h+1))^2;
            J = J + 2*Q_u   * (X(1,N_h+1) - P_xref(1,N_h+1))^2;
            J = J + 2*Q_x   * (X(4,N_h+1) - P_xref(4,N_h+1))^2;
            J = J + 2*Q_y   * (X(5,N_h+1) - P_xref(5,N_h+1))^2;

            % Slack penalty
            J = J + obj.penalty_slack * sumsqr(slack);

            % ---- Constraints ---------------------------------------------
            g = [];

            % Initial condition
            g = vertcat(g, X(:,1) - P_x0);

            % Dynamics (Euler forward with full nonlinear model)
            for k = 1:N_h
                xdot_k = obj.containerCasADi(X(:,k), U(:,k));
                g = vertcat(g, X(:,k+1) - (X(:,k) + xdot_k * obj.dt));
            end

            % Obstacle avoidance: dist² >= (r_obs + r_safety - slack)²
            for k = 1:(N_h+1)
                for j = 1:n_obs
                    dx = X(4,k) - P_obs_pos(1,j);
                    dy = X(5,k) - P_obs_pos(2,j);
                    dist_sq = dx^2 + dy^2;
                    min_d   = P_obs_rad(j) + obj.r_safety - slack(j,k);
                    g = vertcat(g, dist_sq - min_d^2);
                end
            end

            % ---- Flatten decision variables ------------------------------
            OPT    = vertcat(X(:), U(:), slack(:));
            n_vars = size(OPT, 1);

            % ---- Variable bounds -----------------------------------------
            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);

            % State bounds
            for k = 1:(N_h+1)
                base = (k-1)*nx;
                lbx(base + 1)  = 0.1;   % u > 0
                lbx(base + 10) = 1;     % n > 0
            end

            % Control bounds
            u_off = nx*(N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*nu;
                lbx(base+1) = -obj.delta_max_rad;
                ubx(base+1) =  obj.delta_max_rad;
                lbx(base+2) =  obj.n_min_rpm;
                ubx(base+2) =  obj.n_max_rpm;
            end

            % Slack >= 0
            s_off = u_off + nu*N_h;
            lbx(s_off+1 : end) = 0;

            % ---- Constraint bounds ---------------------------------------
            n_eq   = nx + nx*N_h;
            n_ineq = n_obs * (N_h+1);
            lbg = zeros(n_eq + n_ineq, 1);
            ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];

            % ---- Create IPOPT solver -------------------------------------
            nlp = struct('f', J, 'x', OPT, 'g', g, 'p', P_all);

            opts = struct;
            opts.ipopt.print_level           = 0;
            opts.print_time                  = 0;
            opts.ipopt.max_iter              = 500;
            opts.ipopt.tol                   = 1e-3;
            opts.ipopt.acceptable_tol        = 1e-2;
            opts.ipopt.acceptable_iter       = 5;
            opts.ipopt.mu_strategy           = 'adaptive';
            opts.ipopt.nlp_scaling_method    = 'gradient-based';
            opts.ipopt.sb                    = 'yes';

            obj.solver    = nlpsol('nmpc_lite', 'ipopt', nlp, opts);
            obj.lbx_vec   = lbx;
            obj.ubx_vec   = ubx;
            obj.lbg_vec   = lbg;
            obj.ubg_vec   = ubg;
            obj.solver_built = true;

            fprintf('  nlpsol built: %d vars, %d constraints, %d obs slots\n', ...
                n_vars, n_eq+n_ineq, n_obs);
        end

        %% SOLVE ===========================================================
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            t_start = tic;

            if ~obj.solver_built
                obj.buildSolver();
            end

            N_h   = obj.N;
            nx    = obj.nx;
            nu    = obj.nu;
            n_obs = obj.max_obs;

            % --- Fill obstacle slots (unused = far away, r=0) -------------
            obs_pos = 1e8 * ones(2, n_obs);
            obs_rad = zeros(n_obs, 1);
            n_real  = 0;

            if nargin >= 4 && ~isempty(obstacles)
                n_real = min(length(obstacles), n_obs);
                for j = 1:n_real
                    obs_pos(:,j) = obstacles(j).position(1:2);
                    obs_rad(j)   = obstacles(j).radius;
                end
            end

            % --- Control reference (zero rudder, current RPM) -------------
            u_ref = repmat([0; max(x0(10), obj.n_min_rpm)], 1, N_h);

            % --- Assemble parameter vector --------------------------------
            p_val = [x0(:); x_ref(:); u_ref(:); obs_pos(:); obs_rad(:)];

            % --- Warm-start initial guess (shift previous solution) -------
            if ~isempty(obj.prev_sol)
                % Extract previous solution
                X_prev = reshape(obj.prev_sol(1:nx*(N_h+1)), nx, N_h+1);
                u_s    = nx*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s : u_s + nu*N_h - 1), nu, N_h);
                s_s    = u_s + nu*N_h;
                s_prev = obj.prev_sol(s_s:end);
                
                % Shift by one step (discard old first step, repeat last)
                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;  % Fix initial condition
                U_init = [U_prev(:,2:end), U_prev(:,end)];
                s_init = s_prev;
                
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            else
                % Cold-start on first iteration
                X_init = zeros(nx, N_h+1);
                X_init(:,1) = x0;
                U_init = repmat([0; max(x0(10), obj.n_min_rpm)], 1, N_h);

                for k = 1:N_h
                    xk = X_init(:,k);
                    xk(1)  = max(xk(1), 0.1);
                    xk(10) = max(xk(10), 1);
                    try
                        [xdot_k, ~] = container(xk, U_init(:,k));
                        X_init(:,k+1) = xk + xdot_k * obj.dt;
                    catch
                        X_init(:,k+1) = xk;
                    end
                end

                s_init   = zeros(n_obs, N_h+1);
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            end

            % --- Solve NLP ------------------------------------------------
            try
                sol = obj.solver('x0', x0_guess, ...
                    'lbx', obj.lbx_vec, 'ubx', obj.ubx_vec, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val);

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:nx*(N_h+1)), nx, N_h+1);
                u_s   = nx*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s : u_s + nu*N_h - 1), nu, N_h);

                u_opt  = U_sol(:,1);
                X_pred = X_sol;

                % Store solution for warm-start next iteration
                obj.prev_sol = sol_x;
                obj.prev_u   = u_opt;

                info.success    = true;
                info.cost       = full(sol.f);
                info.n_obs_real = n_real;
                obj.solve_ok    = obj.solve_ok + 1;

            catch ME
                u_opt  = [0; max(x0(10), obj.n_min_rpm)];
                X_pred = repmat(x0, 1, N_h+1);

                info.success    = false;
                info.error      = ME.message;
                info.n_obs_real = n_real;
                obj.solve_fail  = obj.solve_fail + 1;

                if obj.solve_fail <= 5
                    fprintf('  [NMPC FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
            end

            info.solve_time = toc(t_start);
        end

        %% FULL NONLINEAR DYNAMICS (CasADi) ================================
        % Line-by-line match with container.m (Son & Nomoto 1982)
        function xdot = containerCasADi(~, x, u_in)
            import casadi.*

            L = 175;

            % Speed (with safeguard)
            Uv = sqrt(x(1)^2 + x(2)^2);
            Uv = if_else(Uv < 0.1, 0.1, Uv);

            psi   = x(6);
            phi   = x(8);
            delta = x(9);

            % Nondimensional states
            delta_c = u_in(1);
            n_c     = u_in(2) / 60 * L / Uv;

            u = x(1) / Uv;
            v = x(2) / Uv;
            p = x(7) * L / Uv;
            r = x(3) * L / Uv;
            n = x(10) / 60 * L / Uv;

            % Ship parameters (container.m)
            m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
            Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
            ly = 0.0313;     Iz = 0.000456;
            Jx = 0.0000034;  Jz = 0.000419;

            B     = 25.40;   dF = 8.00;    g_acc  = 9.81;
            dA    = 9.00;    d  = 8.50;    nabla  = 21222;
            KM    = 10.39;   KB = 4.6154;  AR     = 33.0376;
            Delta_hull = 1.8219;  D_prop = 6.533;
            GM    = 0.3/L;
            rho   = 1025;    t_ded = 0.175;
            n_max_revs = 160/60;

            W = rho*g_acc*nabla / (rho*L^2*Uv^2/2);

            % Hydrodynamic derivatives
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

            kk     =  0.631;  epsilon =  0.921;  xR    = -0.5;
            wp_frac =  0.184;  tau    =  1.09;   xp    = -0.526;
            cpv    =  0.0;    cpr     =  0.0;    ga    =  0.088;
            cRr    = -0.156;  cRrrr   = -0.275;  cRrrv =  1.96;
            cRX    =  0.71;   aH      =  0.237;  zR    =  0.033;
            xH     = -0.48;

            % Mass matrix
            m11 = m + mx;
            m22 = m + my;
            m32 = -my*ly;
            m42 = my*alphay;
            m33 = Ix + Jx;
            m44 = Iz + Jz;

            % Rudder saturation & dynamics
            delta_max_r = 35*pi/180;
            Ddelta_max  = 10*pi/180;

            delta_c_sat = if_else(delta_c >  delta_max_r,  delta_max_r, ...
                          if_else(delta_c < -delta_max_r, -delta_max_r, delta_c));
            delta_dot_raw = delta_c_sat - delta;
            delta_dot = if_else(delta_dot_raw >  Ddelta_max,  Ddelta_max, ...
                        if_else(delta_dot_raw < -Ddelta_max, -Ddelta_max, delta_dot_raw));

            % Shaft saturation & dynamics
            n_c_revs    = n_c * Uv / L;
            n_revs      = n   * Uv / L;
            n_revs_safe = if_else(n_revs < 0.01, 0.01, n_revs);
            n_c_clamped = if_else(n_c_revs >  n_max_revs,  n_max_revs, ...
                          if_else(n_c_revs < 0, 0, n_c_revs));
            Tm = if_else(n_revs_safe > 0.3, 5.65 / n_revs_safe, 18.83);
            n_dot = (1/Tm) * (n_c_clamped - n_revs_safe) * 60;

            % Propeller / rudder forces
            vR = ga*v + cRr*r + cRrrr*r^3 + cRrrv*r^2*v;
            uP = u * ((1 - wp_frac) + tau*((v + xp*r)^2 + cpv*v + cpr*r));

            J_prop = uP * Uv / (n_revs_safe * D_prop);
            KT     = 0.527 - 0.455 * J_prop;

            sqrt_arg = 1 + 8*kk*KT / (pi*J_prop^2 + 1e-6);
            sqrt_arg = if_else(sqrt_arg < 1e-6, 1e-6, sqrt_arg);
            uR = uP * epsilon * sqrt(sqrt_arg);

            uR_safe = if_else(uR*uR < 1e-8, 1e-4, uR);
            alphaR  = delta + atan2(vR, uR_safe);
            FN = -((6.13*Delta_hull)/(Delta_hull+2.25)) * (AR/L^2) * ...
                  (uR^2 + vR^2) * sin(alphaR);
            T_thrust = 2*rho*D_prop^4 / (Uv^2*L^2*rho) * KT * n_revs_safe * abs(n_revs_safe);

            % Forces & moments
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

            % Solve for accelerations
            detM = m22*m33*m44 - m32^2*m44 - m42^2*m33;

            xdot = [
                X_f*(Uv^2/L) / m11
               -((-m33*m44*Y_f + m32*m44*K_f + m42*m33*N_f) / detM) * (Uv^2/L)
                ((-m42*m33*Y_f + m32*m42*K_f + N_f*m22*m33 - N_f*m32^2) / detM) * (Uv^2/L^2)
                cos(psi)*x(1) - sin(psi)*cos(phi)*x(2)
                sin(psi)*x(1) + cos(psi)*cos(phi)*x(2)
                cos(phi)*x(3)
                ((-m32*m44*Y_f + K_f*m22*m44 - K_f*m42^2 + m32*m42*N_f) / detM) * (Uv^2/L^2)
                x(7)
                delta_dot
                n_dot
            ];
        end

    end % methods
end

%% ========================================================================
%  Local helper: struct field with default
%% ========================================================================
function v = getOr(s, name, default)
    if isfield(s, name)
        v = s.(name);
    else
        v = default;
    end
end
