classdef NMPC_Container_Lite < handle
    % NMPC_Container_Lite  Simplified NMPC for container ship with azipod thrusters (CasADi nlpsol)
    %
    % Minimal viable NMPC for path-following + static obstacle avoidance.
    % Uses simplified 6-DOF nonlinear dynamics with dual azipod thrusters (no roll).
    %
    %   States:  x = [u v r x y psi]'                  (6)
    %   Controls: u = [alpha1 alpha2 F1 F2]'           (4)
    %            (azimuth angles in rad, thrust in kN)
    %
    % Features:
    %   - Simplified nonlinear 6-DOF dynamics (roll removed)
    %   - Twin azipod thrust vectoring thrusters
    %   - Obstacle avoidance via slack-penalised distance constraints
    %   - Control rate-of-change penalty (R_rate) for smooth azimuth/thrust
    %   - Build-once / solve-many nlpsol architecture
    %
    % Removed for simplification:
    %   - Roll and pitch dynamics (roll = 0)
    %   - Rudder and propeller model (replaced by azipods)
    %   - Environment / current effects
    %   - Map polygon integration
    %
    % Usage:
    %   cfg = struct('N', 20, 'dt', 1.0, ...
    %                'Q', diag([2 0 0.8 0.03 0.03 15]), ...
    %                'R', diag([0.1, 0.1, 10, 10]));
    %   nmpc = NMPC_Container_Lite(cfg);
    %   nmpc.buildSolver();
    %   [u_opt, X_pred, info] = nmpc.solve(x0, x_ref, obstacles);
    %
    % Author: Riccardo Legnini (adapted for azipod thrusters)
    % Date:   2026-03-10

    properties
        % Horizon
        N               % Prediction steps
        dt              % Sample time [s]

        % Cost weights
        Q               % State error weight  (6×6)
        R               % Control effort weight (4×4)
        R_rate          % Control rate-of-change weight (4×4)

        % Dimensions (fixed)
        nx = 6          % States: [u v r x y psi]
        nu = 4          % Controls: [alpha1 alpha2 F1 F2]

        % Azipod thruster limits (based on ABB azipod specifications)
        % Ref: ABB Azipod XO/Pro/Ultra series for medium-large ships
        % For a 41,000 ton container ship (LOA ~175m): typical 2x300kN azipods
        F_max_kN     = 300    % Max thrust per thruster [kN] (300 kN ea. ~ 2.9 MW @ 10 m/s)
        alpha_max    = pi     % Max azimuth angle [rad] (bidirectional)
        alpha_rate_max = deg2rad(15)  % Max azimuth rate [rad/s] (15°/s is typical for azipods)

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

        % Runtime debug toggles
        enable_diagnostics = false
    end

    methods

        %% CONSTRUCTOR ====================================================
        function obj = NMPC_Container_Lite(cfg)
            % Handle missing or empty config
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            % Required fields with defaults (6-DOF model)
            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);
            % Q weights: [u, v, r, x, y, psi]
            obj.Q  = getOr(cfg, 'Q',  diag([2.0, 0, 0.8, 0.03, 0.03, 15]));
            % R weights: [alpha1, alpha2, F1, F2] - azimuth angles have lower priority
            obj.R  = getOr(cfg, 'R',  diag([0.1, 0.1, 10, 10]));

            % Optional fields with defaults
            obj.R_rate = getOr(cfg, 'R_rate', diag([0.05, 0.05, 5, 5]));
            obj.max_obs       = getOr(cfg, 'max_obs',       15);
            obj.r_safety      = getOr(cfg, 'r_safety',      30);
            obj.penalty_slack = getOr(cfg, 'penalty_slack',  10000);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));

            fprintf('NMPC_Container_Lite: N=%d, dt=%.2f, obs_slots=%d (6-DOF azipod model)\n', ...
                obj.N, obj.dt, obj.max_obs);
        end

        %% BUILD SOLVER (called once) =====================================
        function buildSolver(obj)
            import casadi.*

            N_h   = obj.N;
            nx    = obj.nx; %#ok<*PROP>
            nu    = obj.nu; %#ok<*PROP>
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
                lbx(base + 1)  = 0.1;   % u > 0.1 (prevent singularities)
            end

            for k = 1:(N_h+1)
                base = (k-1)*nx;
                % State bounds (all 6 DOF)
                lbx(base + 1)  =  0.1;      % u >= 0.1 m/s (minimum non-zero speed)
                ubx(base + 1)  = 20;        % u <= 20 m/s (max realistic speed)
                lbx(base + 2)  = -5;        % v in [-5, 5] m/s (sway velocity)
                ubx(base + 2)  = 5;
                lbx(base + 3)  = -1;        % r in [-1, 1] rad/s (yaw rate)
                ubx(base + 3)  = 1;
                lbx(base + 4)  = -10000;    % x in [-10km, 10km] (East position)
                ubx(base + 4)  = 10000;
                lbx(base + 5)  = -10000;    % y in [-10km, 10km] (North position)
                ubx(base + 5)  = 10000;
                lbx(base + 6)  = -pi;       % psi in [-pi, pi] (heading)
                ubx(base + 6)  = pi;
            end

            % Control bounds
            u_off = nx*(N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*nu;
                % Azimuth angles in [-pi, pi]
                lbx(base+1) = -obj.alpha_max;
                ubx(base+1) =  obj.alpha_max;
                lbx(base+2) = -obj.alpha_max;
                ubx(base+2) =  obj.alpha_max;
                % Thrust magnitudes in [-F_max, F_max] kN (bidirectional)
                lbx(base+3) = -obj.F_max_kN;
                ubx(base+3) =  obj.F_max_kN;
                lbx(base+4) = -obj.F_max_kN;
                ubx(base+4) =  obj.F_max_kN;
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
            nx    = obj.nx; %#ok<*PROPLC>
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

            % --- Control reference (zero azimuth, zero thrust) ------------------
            u_ref = zeros(nu, N_h);  % [alpha1, alpha2, F1, F2]

            % --- Assemble parameter vector --------------------------------
            p_val = [x0(:); x_ref(:); u_ref(:); obs_pos(:); obs_rad(:)];

            % Optional pre-solve diagnostics (verbose)
            if obj.enable_diagnostics
                obj.diagnosticsPreSolve(x0, x_ref, u_ref, p_val);
            end

            % --- Warm-start initial guess (shift previous solution) -------
            if ~isempty(obj.prev_sol) && (obj.solve_ok > 3)
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
                U_init = zeros(nu, N_h);  % [alpha1, alpha2, F1, F2]

                % ✅ Simple forward propagation with constant controls
                for k = 1:N_h
                    xk = X_init(:,k);
                    
                    % ✅ Safeguards (NO psi clamping!)
                    xk(1) = max(xk(1), 0.1);  % Minimum speed
                    
                    try
                        [xdot_k, ~] = container(xk, U_init(:,k));
                        
                        % ✅ Check for NaN in dynamics
                        if any(isnan(xdot_k)) || any(isinf(xdot_k))
                            warning('[Cold-start] NaN in container() at k=%d, using constant state', k);
                            X_init(:,k+1) = xk;
                        else
                            X_init(:,k+1) = xk + xdot_k * obj.dt;
                        end
                    catch ME
                        warning(ME.identifier, '[Cold-start] container() failed: %s)', ME.message);
                        X_init(:,k+1) = xk;
                    end
                end

                s_init   = zeros(n_obs, N_h+1);
                x0_guess = [X_init(:); U_init(:); s_init(:)];
            end


            % --- Solve NLP ------------------------------------------------
            % ✅ ENFORCE initial condition by explicitly bounding first state to x0
            lbx_local = obj.lbx_vec;
            ubx_local = obj.ubx_vec;
            for i = 1:nx
                lbx_local(i) = x0(i);  % X(i,1) >= x0(i)
                ubx_local(i) = x0(i);  % X(i,1) <= x0(i)  → X(i,1) = x0(i)
            end
            
            try
                sol = obj.solver('x0', x0_guess, ...
                    'lbx', lbx_local, 'ubx', ubx_local, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val);

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:nx*(N_h+1)), nx, N_h+1);
                u_s   = nx*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s : u_s + nu*N_h - 1), nu, N_h);

                u_opt  = U_sol(:,1);
                X_pred = X_sol;

                % ✅ VALIDATION: Verify initial condition is satisfied
                ic_error = X_pred(:,1) - x0;
                if norm(ic_error) > 1e-6
                    warning('NMPC: Initial condition violated by %.2e (expect <1e-6)', norm(ic_error));
                    fprintf('  x0 requested  = [%.3f, %.3f, %.3f, %.1f, %.1f, %.4f]\n', x0(1), x0(2), x0(3), x0(4), x0(5), x0(6));
                    fprintf('  X_pred(:,1)   = [%.3f, %.3f, %.3f, %.1f, %.1f, %.4f]\n', X_pred(1,1), X_pred(2,1), X_pred(3,1), X_pred(4,1), X_pred(5,1), X_pred(6,1));
                    fprintf('  IC error      = [%.3e, %.3e, %.3e, %.3e, %.3e, %.3e]\n', ic_error(1), ic_error(2), ic_error(3), ic_error(4), ic_error(5), ic_error(6));
                end

                % Store solution for warm-start next iteration
                obj.prev_sol = sol_x;
                obj.prev_u   = u_opt;

                info.success    = true;
                info.cost       = full(sol.f);
                info.n_obs_real = n_real;
                obj.solve_ok    = obj.solve_ok + 1;

            catch ME
                u_opt  = zeros(nu, 1);  % Zero azimuth angles and zero thrust
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

        %% PRE-SOLVE DIAGNOSTICS ============================================
        function diagnosticsPreSolve(obj, x0, x_ref, u_ref, p_val)
            % Run comprehensive diagnostics before calling solver.
            % Can be called manually for debugging, or automatically in solve().
            
            N_h = obj.N;
            nx = obj.nx;
            
            fprintf('\n========== PRE-SOLVE DIAGNOSTICS ==========\n');
            
            % 1. Check initial state
            fprintf('  Initial State (x0):\n');
            fprintf('    u=%.3f m/s, v=%.3f m/s, r=%.4f rad/s\n', x0(1), x0(2), x0(3));
            fprintf('    x=%.1f m, y=%.1f m, psi=%.2f°\n', x0(4), x0(5), rad2deg(x0(6)));
            
            if any(isnan(x0)) || any(isinf(x0))
                error('  ❌ NaN/Inf detected in x0!');
            end
            if x0(1) < 0.01
                warning('  ⚠️  Surge velocity u=%.3f is very low (may cause numerical issues)', x0(1));
            end
            fprintf('  ✅ x0 valid\n\n');
            
            % 2. Check reference trajectory
            fprintf('  Reference Trajectory (x_ref):\n');
            fprintf('    Size: %d states × %d horizon steps\n', nx, N_h+1);
            fprintf('    First point: u=%.3f, psi=%.2f°\n', x_ref(1,1), rad2deg(x_ref(6,1)));
            fprintf('    Last point:  u=%.3f, psi=%.2f°\n', x_ref(1,N_h+1), rad2deg(x_ref(6,N_h+1)));
            
            if any(isnan(x_ref(:))) || any(isinf(x_ref(:)))
                error('  ❌ NaN/Inf detected in x_ref!');
            end
            fprintf('  ✅ x_ref valid\n\n');
            
            % 3. Check control reference
            fprintf('  Control Reference (u_ref):\n');
            fprintf('    Mean azimuth:  α₁=%.3f rad, α₂=%.3f rad\n', ...
                mean(u_ref(1,:)), mean(u_ref(2,:)));
            fprintf('    Mean thrust:   F₁=%.1f kN, F₂=%.1f kN\n', ...
                mean(u_ref(3,:)), mean(u_ref(4,:)));
            
            if any(isnan(u_ref(:))) || any(isinf(u_ref(:)))
                error('  ❌ NaN/Inf detected in u_ref!');
            end
            fprintf('  ✅ u_ref valid\n\n');
            
            % 4. Check parameter vector
            fprintf('  Parameter Vector (p_val):\n');
            fprintf('    Length: %d elements\n', length(p_val));
            fprintf('    Expected: %d (6 + 6×%d + 4×%d + 2×15 + 15)\n', ...
                6 + 6*(N_h+1) + 4*N_h + 2*15 + 15, N_h+1, N_h);
            
            if any(isnan(p_val)) || any(isinf(p_val))
                error('  ❌ NaN/Inf detected in parameter vector!');
            end
            fprintf('  ✅ p_val valid\n\n');
            
            % 5. Test dynamics at x0
            fprintf('  Testing Dynamics at x0:\n');
            try
                u_test = zeros(4, 1);  % Zero control input
                xdot_ref = container(x0, u_test);  % Use reference model (works with doubles)
                
                fprintf('    u_dot=%.6f, v_dot=%.6f, r_dot=%.6f\n', ...
                    xdot_ref(1), xdot_ref(2), xdot_ref(3));
                
                if any(isnan(xdot_ref)) || any(isinf(xdot_ref))
                    error('  ❌ NaN/Inf in dynamics!');
                end
                fprintf('  ✅ Dynamics valid\n\n');
                
            catch ME
                fprintf('  ❌ Dynamics evaluation failed: %s\n', ME.message);
                rethrow(ME);
            end
            
            % 6. Summary
            fprintf('========== ALL DIAGNOSTICS PASSED ✅ ==========\n\n');
        end

        %% FULL NONLINEAR DYNAMICS (CasADi) ================================
        % Line-by-line match with container.m (Son & Nomoto 1982)
        function xdot = containerCasADi(~, x, u_in)
            % Simplified 6-DOF model with dual azipod thrusters (MUST MATCH container.m exactly!)
            % States: x = [u, v, r, x_pos, y_pos, psi]
            % Controls: u = [alpha1, alpha2, F1, F2]
            import casadi.*

            L = 175;          % Ship length [m]
            m_ship = 41000 * 1000;   % Total mass [kg]
            
            % State extraction
            u = x(1);
            v = x(2);
            r = x(3);
            x_pos = x(4); %#ok<*NASGU>
            y_pos = x(5);
            psi = x(6);
            
            % Control extraction
            alpha1 = u_in(1);  % Azimuth angle thruster 1 [rad]
            alpha2 = u_in(2);  % Azimuth angle thruster 2 [rad]
            F1 = u_in(3);      % Thrust magnitude thruster 1 [kN]
            F2 = u_in(4);      % Thrust magnitude thruster 2 [kN]
            
            % Speed magnitude with safeguard
            U = sqrt(u^2 + v^2);
            U = if_else(U < 0.1, 0.1, U);
            
            % === AZIPOD FORCES (body-fixed) ===
            Fx_thrust = F1 * cos(alpha1) + F2 * cos(alpha2);  % kN
            Fy_thrust = F1 * sin(alpha1) + F2 * sin(alpha2);  % kN
            
            % Yaw moment from thrusters
            x_thruster_1 = -0.2;  % Normalized position (same as container.m)
            x_thruster_2 = -0.2;
            Mz_thrust = F1 * sin(alpha1) * x_thruster_1 + F2 * sin(alpha2) * x_thruster_2;
            
            % === HYDRODYNAMIC DAMPING (EXACT MATCH TO container.m) ===
            % Hydrodynamic forces [normalized]
            Xuu = -0.04;
            Yv = -0.5;
            Yr = 0.25;
            Nr = -0.15;
            Nv = -0.1;
            Xvv = -0.1;
            Yvv = -2.0;
            Yrrr = 0.05;
            Nvv = -0.05;
            Nrr = -0.1;
            
            % ========== CORRECTED FORCE CALCULATION ==========

            % 1. Hydrodynamic forces (NORMALIZED, dimensionless coefficients)
            Xhyd = Xuu * u * abs(u) + Xvv * v * abs(v);
            Yhyd = Yv * v + Yr * r + Yvv * v * abs(v) + Yrrr * r^2 * abs(r);
            Nhyd = Nv * v + Nr * r + Nvv * v * abs(v) + Nrr * r * abs(r);

            % 2. Denormalize ONLY hydrodynamic terms to physical units (kN)
            rho = 1025;  % kg/m³
            X_hydro_kN = Xhyd * (0.5 * rho * L^2 * U^2) / 1000;  % kN
            Y_hydro_kN = Yhyd * (0.5 * rho * L^2 * U^2) / 1000;  % kN
            N_hydro_kN = Nhyd * (0.5 * rho * L^3 * U^2) / 1000;  % kN·m

            % 3. Azipod thrust forces (ALREADY in kN, NO scaling needed!)
            Fx_thrust = F1 * cos(alpha1) + F2 * cos(alpha2);  % kN
            Fy_thrust = F1 * sin(alpha1) + F2 * sin(alpha2);  % kN
            Mz_thrust = F1 * sin(alpha1) * x_thruster_1 + F2 * sin(alpha2) * x_thruster_2;  % kN·m (dimensionless x L)

            % Convert moment to physical units
            Mz_thrust_kN = Mz_thrust * L;  % kN·m

            % 4. Total forces (both in kN now!)
            X_total = X_hydro_kN + Fx_thrust;   % kN
            Y_total = Y_hydro_kN + Fy_thrust;   % kN
            N_total = N_hydro_kN + Mz_thrust_kN; % kN·m

            % ========== STATE DERIVATIVES ==========

            u_dot = (X_total * 1000 / m_ship) + v * r;  % Convert kN → N, then m/s²
            v_dot = (Y_total * 1000 / m_ship) - u * r;
            r_dot = (N_total * 1000) / (0.1 * m_ship * L);  % rad/s²

            
            x_dot = cos(psi) * u - sin(psi) * v;
            y_dot = sin(psi) * u + cos(psi) * v;
            psi_dot = r;
            
            xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot];
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
