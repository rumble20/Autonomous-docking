classdef NMPC_Container_final < handle
    % NMPC_Container_final  NMPC for container ship with twin stern azipods + bow thruster
    %
%   States:  x = [u v r x y psi n1 n2 n3]'        (9)
%   Controls: u = [alpha1 alpha2 n1_c n2_c n3_c]' (5)

    properties
        N
        dt
        Q
        R
        R_rate
        nx = 9
        nu = 5

        % Ship parameters
        L = 175
        rho = 1025
        nabla = 21222
        D = 6.533
        t_ded = 0.175
        wp = 0.184
        wp_fwd = 0.0552
        x_azi1 = -78.75
        x_azi2 = -78.75
        x_azi3 = 29.75
        y_azi1 = -12.7
        y_azi2 = 12.7
        y_azi3 = 0

        % Actuator limits
        n_max = 160
        n_min = -80
        n_bow_max = 140
        n_bow_min = -80
        alpha_max = pi
        Dn_max = 10
        Dn_bow_max = 8
        alpha_rate_max = 0.21  % Max azimuth rate [rad/s] = 12 deg/s (ABB spec)

        % Obstacle settings
        max_obs = 5
        r_safety = 35

        % Collision geometry (point | oriented-rectangle)
        collision_model = 'oriented-rectangle'
        hull_length_m = 0
        hull_beam_m = 0
        hull_half_length_m = 0
        hull_half_beam_m = 0
        hull_clearance_m = 0
        hull_smooth_eps = 1e-4

        % NEW: Actuator and forward motion control
        actuator_force_weight = 0.05     % Penalty on RPM magnitude (reduces swaying) — increased 25x
        forward_incentive_weight = 2.5   % Penalty on backward motion (encourages forward bow) — increased 5x
        waypoint_heading_weight = 0.0    % Extra heading penalty at waypoints (0=disabled)
        u_min_forward = 0.5              % Minimum forward speed constraint [m/s] (hard constraint)
        max_brake_rate = 0.3             % Max deceleration rate [m/s²] to minimize braking fuel costs
        soft_obs_weight = 2e5            % Large penalty when soft obstacle slack is enabled
        soft_obs_default_max_m = 0.0     % Default obstacle slack cap [m] (0 disables softening)

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
        prev_sol
        prev_u
        prev_lam_x
        prev_lam_g

        enable_diagnostics = false
    end

    methods

        function obj = NMPC_Container_final(cfg)
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);
            obj.Q = padDiag(getOr(cfg, 'Q', diag([2.0, 0.1, 0.8, 3.0, 3.0, 6.0, 0.001, 0.001, 0.001])), 9);
            obj.R = padDiag(getOr(cfg, 'R', diag([0.1, 0.1, 0.01, 0.01, 0.01])), 5);
            obj.R_rate = padDiag(getOr(cfg, 'R_rate', diag([0.05, 0.05, 0.005, 0.005, 0.005])), 5);
            obj.max_obs = getOr(cfg, 'max_obs', 5);
            obj.r_safety = getOr(cfg, 'r_safety', 30);
            obj.collision_model = lower(strtrim(getOr(cfg, 'collision_model', 'point')));
            obj.actuator_force_weight = getOr(cfg, 'actuator_force_weight', 0.05);
            obj.forward_incentive_weight = getOr(cfg, 'forward_incentive_weight', 2.5);
            obj.waypoint_heading_weight = getOr(cfg, 'waypoint_heading_weight', 0.0);
            obj.u_min_forward = getOr(cfg, 'u_min_forward', 0.5);
            obj.max_brake_rate = getOr(cfg, 'max_brake_rate', 0.3);
            obj.soft_obs_weight = getOr(cfg, 'soft_obs_weight', 2e5);
            obj.soft_obs_default_max_m = getOr(cfg, 'soft_obs_default_max_m', 0.0);

            obj.hull_length_m = getOr(cfg, 'hull_length_m', 36);
            obj.hull_beam_m = getOr(cfg, 'hull_beam_m', 12);
            obj.hull_half_length_m = 0.5 * max(1.0, obj.hull_length_m);
            obj.hull_half_beam_m = 0.5 * max(0.5, obj.hull_beam_m);
            obj.hull_clearance_m = getOr(cfg, 'hull_clearance_m', obj.r_safety);
            obj.hull_smooth_eps = getOr(cfg, 'hull_smooth_eps', 1e-4);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));

            fprintf('NMPC_Container_final: N=%d, dt=%.2f, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.max_obs);
            fprintf('  9-state model: [u v r x y psi n1 n2 n3]\n');
            fprintf('  5 controls: [alpha1 alpha2 n1_c n2_c n3_c]\n');
            fprintf('  forward speed constraint: u >= %.2f m/s\n', obj.u_min_forward);
            fprintf('  max brake rate: %.2f m/s²\n', obj.max_brake_rate);
            fprintf('  soft obstacle slack: weight=%.1f, default_max=%.2f m\n', ...
                obj.soft_obs_weight, obj.soft_obs_default_max_m);
            fprintf('  actuator penalty: %.4f | forward incentive: %.2f\n', ...
                obj.actuator_force_weight, obj.forward_incentive_weight);
            if strcmpi(obj.collision_model, 'oriented-rectangle')
                fprintf('  collision: oriented rectangle %.1f x %.1f m (clearance %.1f m)\n', ...
                    2*obj.hull_half_length_m, 2*obj.hull_half_beam_m, obj.hull_clearance_m);
            else
                fprintf('  collision: point model (r_safety %.1f m)\n', obj.r_safety);
            end
        end

        function buildSolver(obj)
            import casadi.*

            N_h = obj.N;
            n_state = obj.nx;
            n_ctrl = obj.nu;
            n_obs = obj.max_obs;

            %  DECISION VARIABLES
            X = SX.sym('X', n_state, N_h+1);
            U = SX.sym('U', n_ctrl, N_h);
            S_obs = SX.sym('S_obs', n_obs, N_h+1); % Soft obstacle slack [m]

            %  PARAMETERS
            P_x0       = SX.sym('P_x0', n_state, 1);
            P_xref     = SX.sym('P_xref', n_state, N_h+1);
            P_uref     = SX.sym('P_uref', n_ctrl, N_h);
            P_n_obs_real = SX.sym('P_n_obs_real', 1, 1);
            P_obs_pos  = SX.sym('P_obs_pos', 2, n_obs);
            P_obs_rad  = SX.sym('P_obs_rad', n_obs, 1);
            P_u_prev   = SX.sym('P_u_prev', n_ctrl, 1);  % NEW: Previous applied control
            P_max_brake_rate = SX.sym('P_max_brake_rate', 1, 1);  % NEW: Max deceleration rate

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_n_obs_real, ...
                            P_obs_pos(:), P_obs_rad, P_u_prev, P_max_brake_rate);
            obj.np_total = size(P_all, 1);

            %  COST FUNCTION
            Q_u   = obj.Q(1,1);  Q_v   = obj.Q(2,2);  Q_r   = obj.Q(3,3);
            Q_x   = obj.Q(4,4);  Q_y   = obj.Q(5,5);  Q_psi = obj.Q(6,6);
            Q_n1  = obj.Q(7,7);  Q_n2  = obj.Q(8,8);  Q_n3  = obj.Q(9,9);

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
                J = J + Q_n3  * (X(9,k) - P_xref(9,k))^2;

                % NEW: Actuator force penalty (reduces thruster swaying)
                % Penalizes the magnitude of RPM commands to discourage excessive actuation
                J = J + obj.actuator_force_weight * (X(7,k)^2 + X(8,k)^2 + X(9,k)^2);

                % NEW: Forward velocity encouragement (makes bow go forward)
                % Penalizes negative forward speed to naturally prefer forward motion
                u_back = min(0, X(1,k));  % Only penalize if going backward
                J = J + obj.forward_incentive_weight * u_back^2;

                du = U(:,k) - P_uref(:,k);
                J = J + du' * obj.R * du;

                % Obstacle-softening penalty (active only if slack bounds permit >0 values).
                for j = 1:n_obs
                    J = J + obj.soft_obs_weight * S_obs(j,k)^2;
                end
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
            % NEW: Terminal actuator force and forward incentive penalties
            J = J + 2 * obj.actuator_force_weight * (X(7,N_h+1)^2 + X(8,N_h+1)^2 + X(9,N_h+1)^2);
            u_back_N = min(0, X(1,N_h+1));
            J = J + 2 * obj.forward_incentive_weight * u_back_N^2;
            for j = 1:n_obs
                J = J + obj.soft_obs_weight * S_obs(j,N_h+1)^2;
            end

            %  CONSTRAINTS
            g = [];

            % --- Initial condition (n_state equalities) ---
            g = vertcat(g, X(:,1) - P_x0);

            % --- Dynamics (n_state * N_h equalities) ---
            for k = 1:N_h
                xdot_k = obj.dynamicsCasADi(X(:,k), U(:,k));
                x_next = X(:,k) + xdot_k * obj.dt;
                g = vertcat(g, X(:,k+1) - x_next);
            end

            % --- Obstacle avoidance (n_obs * (N_h+1) inequalities) ---
            for k = 1:(N_h+1)
                for j = 1:n_obs
                    if strcmpi(obj.collision_model, 'oriented-rectangle')
                        % Obstacle center in ship body frame at prediction step k.
                        rx = P_obs_pos(1,j) - X(4,k);
                        ry = P_obs_pos(2,j) - X(5,k);
                        cpsi = cos(X(6,k));
                        spsi = sin(X(6,k));

                        qx = cpsi * rx + spsi * ry;
                        qy = -spsi * rx + cpsi * ry;

                        ax = abs(qx) - obj.hull_half_length_m;
                        ay = abs(qy) - obj.hull_half_beam_m;
                        dx = if_else(ax > 0, ax, 0);
                        dy = if_else(ay > 0, ay, 0);
                        dist = sqrt(dx^2 + dy^2 + obj.hull_smooth_eps);
                        g = vertcat(g, dist - P_obs_rad(j) - obj.hull_clearance_m + S_obs(j,k));
                    else
                        dx = X(4,k) - P_obs_pos(1,j);
                        dy = X(5,k) - P_obs_pos(2,j);
                        dist = sqrt(dx^2 + dy^2 + 1e-3);
                        g = vertcat(g, dist - P_obs_rad(j) - obj.r_safety + S_obs(j,k));
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

            % SUBSEQUENT STEPS: Constrain consecutive controls
            for k = 1:(N_h-1)
                d_alpha1 = U(1,k+1) - U(1,k);
                g = vertcat(g, d_alpha1 - obj.alpha_rate_max * obj.dt);
                g = vertcat(g, -d_alpha1 - obj.alpha_rate_max * obj.dt);

                d_alpha2 = U(2,k+1) - U(2,k);
                g = vertcat(g, d_alpha2 - obj.alpha_rate_max * obj.dt);
                g = vertcat(g, -d_alpha2 - obj.alpha_rate_max * obj.dt);
            end

            % --- Braking constraint (surge deceleration limit) ---
            % NEW: Constraint to reduce excessive braking (fuel cost minimization).
            % Between consecutive prediction steps, forward speed cannot decrease faster than max_brake_rate.
            % This prevents the optimizer from making unrealistic sharp decelerations.
            % First step: constrain u(k=1) relative to initial state u0
            du_brake_first = X(1,1) - P_x0(1);  % This should be >= -max_brake_rate * dt
            g = vertcat(g, du_brake_first + P_max_brake_rate * obj.dt);
            % Subsequent steps: constrain u(k) relative to u(k-1)
            for k = 1:(N_h)
                du_brake = X(1,k+1) - X(1,k);
                g = vertcat(g, du_brake + P_max_brake_rate * obj.dt);
            end

            %  VARIABLE BOUNDS
            OPT = vertcat(X(:), U(:), S_obs(:));
            n_vars = size(OPT, 1);

            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);

            % State bounds
            for k = 1:(N_h+1)
                base = (k-1)*n_state;
                lbx(base+1) = obj.u_min_forward;  ubx(base+1) = 12;  % NEW: Hard minimum forward speed
                lbx(base+2) = -3;         ubx(base+2) = 3;
                lbx(base+3) = -0.15;      ubx(base+3) = 0.15;
                lbx(base+4) = -1e5;       ubx(base+4) = 1e5;
                lbx(base+5) = -1e5;       ubx(base+5) = 1e5;
                lbx(base+6) = -inf;       ubx(base+6) = inf;
                lbx(base+7) = obj.n_min;  ubx(base+7) = obj.n_max;
                lbx(base+8) = obj.n_min;  ubx(base+8) = obj.n_max;
                lbx(base+9) = obj.n_bow_min;  ubx(base+9) = obj.n_bow_max;
            end

            % Control bounds
            u_off = n_state * (N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*n_ctrl;
                lbx(base+1) = -obj.alpha_max;  ubx(base+1) = obj.alpha_max;
                lbx(base+2) = -obj.alpha_max;  ubx(base+2) = obj.alpha_max;
                lbx(base+3) = obj.n_min;       ubx(base+3) = obj.n_max;
                lbx(base+4) = obj.n_min;       ubx(base+4) = obj.n_max;
            end

            % Obstacle slack bounds (can be tightened to zero per solve call).
            s_off = n_state * (N_h+1) + n_ctrl * N_h;
            for k = 1:(N_h+1)
                for j = 1:n_obs
                    idx_s = s_off + (k-1)*n_obs + j;
                    lbx(idx_s) = 0;
                    ubx(idx_s) = obj.soft_obs_default_max_m;
                end
            end

            %  CONSTRAINT BOUNDS
            n_eq        = n_state + n_state*N_h;
            n_obs_ineq  = n_obs * (N_h+1);
            n_rate_ineq = 4 + 4*(N_h-1);  % 4 for first step + 4*(N-1) for rest
            n_brake_ineq = 1 + N_h;       % NEW: 1 for first step + N for subsequent steps
            n_constraints = n_eq + n_obs_ineq + n_rate_ineq + n_brake_ineq;

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

            % Brake: g >= 0  (du_brake + max_brake_rate * dt >= 0)
            brake_start = rate_end + 1;
            brake_end   = n_constraints;
            lbg(brake_start:brake_end) = 0;
            ubg(brake_start:brake_end) = inf;

            %  BUILD SOLVER
            nlp = struct('f', J, 'x', OPT, 'g', g, 'p', P_all);

            opts = struct;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.max_iter = 120;
            opts.ipopt.tol = 2e-3;
            opts.ipopt.acceptable_tol = 2e-2;
            opts.ipopt.acceptable_iter = 3;
            opts.ipopt.mu_strategy = 'adaptive';
            opts.ipopt.nlp_scaling_method = 'gradient-based';
            opts.ipopt.sb = 'yes';
            opts.ipopt.warm_start_init_point = 'yes';
            opts.ipopt.warm_start_bound_push = 1e-6;
            opts.ipopt.warm_start_mult_bound_push = 1e-6;
            opts.ipopt.warm_start_slack_bound_push = 1e-6;

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
            fprintf('    - Obstacle:   %d (slots × horizon)\n', n_obs_ineq);
            fprintf('    - Rate:       %d (first-step + consecutive)\n', n_rate_ineq);
            fprintf('    - Brake:      %d (first-step + consecutive)\n', n_brake_ineq);
        end


        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles, u_prev, u_min_forward_override, solve_opts)
            % solve  Solve NMPC problem
            %
            %   [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles, u_prev, u_min_forward_override, solve_opts)
            %
            %   Inputs:
            %       x0        - Current state (8x1)
            %       x_ref     - Reference trajectory (8 x N+1)
            %       obstacles - Array of obstacles (optional)
            %       u_prev    - Previous applied control (4x1) - NEW!
            %       u_min_forward_override - Optional per-step lower bound for surge state u
            %       solve_opts - Optional struct:
            %                    .enable_soft_obstacles (logical)
            %                    .soft_obs_max_m (nonnegative scalar)
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
            n_state = obj.nx;
            n_ctrl = obj.nu;
            n_obs = obj.max_obs;

            if nargin < 6 || isempty(u_min_forward_override)
                u_min_local = obj.u_min_forward;
            else
                u_min_local = max(0.0, min(12.0, u_min_forward_override));
            end

            if nargin < 7 || isempty(solve_opts)
                solve_opts = struct();
            end
            soft_enable = logical(getOr(solve_opts, 'enable_soft_obstacles', false));
            if soft_enable
                soft_obs_max_m = max(0.0, getOr(solve_opts, 'soft_obs_max_m', 0.0));
            else
                soft_obs_max_m = 0.0;
            end
            n3_max_local = obj.n_bow_max;
            if isfield(solve_opts, 'n3_max') && ~isempty(solve_opts.n3_max)
                n3_max_local = max(0.0, min(obj.n_bow_max, solve_opts.n3_max));
            end

            % Handle previous control
            if nargin < 5 || isempty(u_prev)
                if ~isempty(obj.prev_u)
                    u_prev = obj.prev_u;
                else
                    u_prev = [0; 0; x0(7); x0(8)];
                end
            end

            % Obstacle setup
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

            % Reference control
            u_ref = zeros(n_ctrl, N_h);
            u_ref(3,:) = x0(7);
            u_ref(4,:) = x0(8);
            u_ref(5,:) = min(max(x0(9), obj.n_bow_min), n3_max_local);

            % Build parameter vector (now includes u_prev and max_brake_rate)
            p_val = [x0(:); x_ref(:); u_ref(:); n_real; obs_pos(:); obs_rad(:); u_prev(:); obj.max_brake_rate];

            % Initial guess (warm start)
            if ~isempty(obj.prev_sol)
                X_prev = reshape(obj.prev_sol(1:n_state*(N_h+1)), n_state, N_h+1);
                u_s = n_state*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s:u_s+n_ctrl*N_h-1), n_ctrl, N_h);
                s_s = n_state*(N_h+1) + n_ctrl*N_h + 1;
                S_prev = reshape(obj.prev_sol(s_s:s_s+n_obs*(N_h+1)-1), n_obs, N_h+1);

                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;
                U_init = [U_prev(:,2:end), U_prev(:,end)];
                S_init = [S_prev(:,2:end), S_prev(:,end)];

                x0_guess = [X_init(:); U_init(:); S_init(:)];
            else
                X_init = repmat(x0, 1, N_h+1);
                U_init = u_ref;
                S_init = zeros(n_obs, N_h+1);

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

                x0_guess = [X_init(:); U_init(:); S_init(:)];
            end

            % Fix initial state bounds
            lbx_local = obj.lbx_vec;
            ubx_local = obj.ubx_vec;
            for k = 2:(N_h+1)
                idx_u = (k-1)*n_state + 1;
                lbx_local(idx_u) = u_min_local;
            end
            lbx_local(9:n_state:end) = obj.n_bow_min;
            ubx_local(9:n_state:end) = n3_max_local;
            for i = 1:n_state
                lbx_local(i) = x0(i);
                ubx_local(i) = x0(i);
            end
            lbx_local(9) = x0(9);
            ubx_local(9) = x0(9);

            u_off = n_state * (N_h+1);
            for k = 1:N_h
                base = u_off + (k-1)*n_ctrl;
                lbx_local(base+5) = obj.n_bow_min;
                ubx_local(base+5) = n3_max_local;
            end

            % Per-step selective soft obstacle constraints.
            s_off = n_state*(N_h+1) + n_ctrl*N_h;
            s_idx = (s_off + 1):(s_off + n_obs*(N_h+1));
            lbx_local(s_idx) = 0;
            ubx_local(s_idx) = soft_obs_max_m;

            % Solve
            try
                use_dual_warm_start = ~isempty(obj.prev_lam_x) && ~isempty(obj.prev_lam_g) && ...
                    numel(obj.prev_lam_x) == numel(x0_guess) && ...
                    numel(obj.prev_lam_g) == numel(obj.lbg_vec);

                solver_args = {'x0', x0_guess, ...
                    'lbx', lbx_local, 'ubx', ubx_local, ...
                    'lbg', obj.lbg_vec, 'ubg', obj.ubg_vec, ...
                    'p', p_val};

                if use_dual_warm_start
                    solver_args = [solver_args, {'lam_x0', obj.prev_lam_x, 'lam_g0', obj.prev_lam_g}]; %#ok<AGROW>
                end

                sol = obj.solver(solver_args{:});

                sol_x = full(sol.x);
                X_sol = reshape(sol_x(1:n_state*(N_h+1)), n_state, N_h+1);
                u_s = n_state*(N_h+1) + 1;
                U_sol = reshape(sol_x(u_s:u_s+n_ctrl*N_h-1), n_ctrl, N_h);
                s_s = n_state*(N_h+1) + n_ctrl*N_h + 1;
                S_sol = reshape(sol_x(s_s:s_s+n_obs*(N_h+1)-1), n_obs, N_h+1);

                u_opt = U_sol(:,1);
                X_pred = X_sol;

                obj.prev_sol = sol_x;
                obj.prev_u = u_opt;
                if isfield(sol, 'lam_x') && isfield(sol, 'lam_g')
                    obj.prev_lam_x = full(sol.lam_x);
                    obj.prev_lam_g = full(sol.lam_g);
                end

                info.success = true;
                info.cost = full(sol.f);
                info.n_obs_real = n_real;
                info.used_dual_warm_start = use_dual_warm_start;
                info.soft_obs_enabled = soft_enable;
                info.soft_obs_max_m = soft_obs_max_m;
                info.max_soft_slack_m = max(S_sol(:));
                info.sum_soft_slack_m = sum(S_sol(:));
                obj.solve_ok = obj.solve_ok + 1;

            catch ME
                u_opt = [0; 0; x0(7); x0(8)];
                X_pred = repmat(x0, 1, N_h+1);

                info.success = false;
                info.error = ME.message;
                info.n_obs_real = n_real;
                info.used_dual_warm_start = false;
                info.soft_obs_enabled = soft_enable;
                info.soft_obs_max_m = soft_obs_max_m;
                info.max_soft_slack_m = nan;
                info.sum_soft_slack_m = nan;
                obj.solve_fail = obj.solve_fail + 1;
                obj.prev_lam_x = [];
                obj.prev_lam_g = [];

                if obj.solve_fail <= 5
                    fprintf('  [NMPC FAIL #%d] %s\n', obj.solve_fail, ME.message);
                end
            end

            info.solve_time = toc(t_start);
        end

        function xdot = dynamicsCasADi(obj, x, u_in)
            import casadi.*

            L_ship = obj.L;
            rho_w = obj.rho;
            nabla_ship = obj.nabla;
            D_stern = obj.D;
            D_bow = 4.5;
            t_stern = obj.t_ded;
            t_bow = 0.15;
            wp_stern = obj.wp;
            stern_port_x = obj.x_azi1;
            stern_starboard_x = obj.x_azi2;
            bow_x = obj.x_azi3;
            stern_port_y = obj.y_azi1;
            stern_starboard_y = obj.y_azi2;
            bow_y = obj.y_azi3;

            m_ship = rho_w * nabla_ship;
            Izz_ship = 0.1 * m_ship * L_ship^2;

            m_nd = 0.00792;  mx = 0.000238;  my = 0.007049;
            Iz_nd = 0.000456;  Jz = 0.000419;

            Xuu = -0.0004226;  Xvr = -0.00311;  Xrr = 0.00020;  Xvv = -0.00386;
            Yv = -0.0116;  Yr = 0.00242;  Yvvv = -0.109;  Yrrr = 0.00177;
            Yvvr = 0.0214;  Yvrr = -0.0405;
            Nv = -0.0038545;  Nr = -0.00222;  Nvvv = 0.001492;  Nrrr = -0.00229;
            Nvvr = -0.0424;  Nvrr = 0.00156;

            m11 = m_nd + mx;  m22 = m_nd + my;  m66 = Iz_nd + Jz;

            u = x(1);  v = x(2);  r = x(3);  psi = x(6);  n1 = x(7);  n2 = x(8);  n3 = x(9);
            alpha1 = u_in(1);  alpha2 = u_in(2);  n1_c = u_in(3);  n2_c = u_in(4);  n3_c = u_in(5);

            U = sqrt(u^2 + v^2);
            U = if_else(U < 0.1, 0.1, U);

            u_nd = u / U;  v_nd = v / U;  r_nd = r * L_ship / U;

            X_hyd = Xuu*u_nd^2 + Xvr*v_nd*r_nd + Xvv*v_nd^2 + Xrr*r_nd^2;
            Y_hyd = Yv*v_nd + Yr*r_nd + Yvvv*v_nd^3 + Yrrr*r_nd^3 + Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2;
            N_hyd = Nv*v_nd + Nr*r_nd + Nvvv*v_nd^3 + Nrrr*r_nd^3 + Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2;

            X_hyd = X_hyd + (m_nd + my) * v_nd * r_nd;
            Y_hyd = Y_hyd - (m_nd + mx) * u_nd * r_nd;

            KT0 = 0.527;  KT1 = -0.455;

            n1_rps = n1 / 60;
            v_local1 = v + r * stern_port_y;
            u_inflow1 = u * cos(alpha1) + v_local1 * sin(alpha1);
            u_a1 = u_inflow1 * (1 - wp_stern);
            n1_abs = if_else(n1_rps >= 0, n1_rps, -n1_rps);
            n1_safe = if_else(n1_abs < 0.01, 0.01, n1_abs);
            J1 = u_a1 / (n1_safe * D_stern);
            J1 = if_else(J1 > 1.2, 1.2, if_else(J1 < -0.5, -0.5, J1));
            KT1_val = KT0 + KT1 * J1;
            KT1_val = if_else(KT1_val < 0.05, 0.05, KT1_val);
            T1_gross = rho_w * D_stern^4 * n1_rps * n1_abs * KT1_val;
            T1 = (1 - t_stern) * T1_gross;

            n2_rps = n2 / 60;
            v_local2 = v + r * stern_starboard_y;
            u_inflow2 = u * cos(alpha2) + v_local2 * sin(alpha2);
            u_a2 = u_inflow2 * (1 - wp_stern);
            n2_abs = if_else(n2_rps >= 0, n2_rps, -n2_rps);
            n2_safe = if_else(n2_abs < 0.01, 0.01, n2_abs);
            J2 = u_a2 / (n2_safe * D_stern);
            J2 = if_else(J2 > 1.2, 1.2, if_else(J2 < -0.5, -0.5, J2));
            KT2_val = KT0 + KT1 * J2;
            KT2_val = if_else(KT2_val < 0.05, 0.05, KT2_val);
            T2_gross = rho_w * D_stern^4 * n2_rps * n2_abs * KT2_val;
            T2 = (1 - t_stern) * T2_gross;

            n3_rps = n3 / 60;
            v_local3 = v - r * bow_x;
            u_inflow3 = v_local3;
            u_a3 = u_inflow3 * (1 - t_bow);
            n3_abs = if_else(n3_rps >= 0, n3_rps, -n3_rps);
            n3_safe = if_else(n3_abs < 0.01, 0.01, n3_abs);
            J3 = u_a3 / (n3_safe * D_bow);
            J3 = if_else(J3 > 1.2, 1.2, if_else(J3 < -0.5, -0.5, J3));
            KT3_val = 0.527 * 0.30 + (-0.455 * 0.30) * J3;
            KT3_val = if_else(KT3_val < 0.05 * 0.30, 0.05 * 0.30, KT3_val);
            speed_decay_factor = max(0.3, 1 - 0.08 * max(0.1, u));
            T3_gross = rho_w * D_bow^4 * n3_rps * n3_abs * KT3_val;
            T3 = (1 - t_bow) * T3_gross * speed_decay_factor;

            Fx1 = T1 * cos(alpha1);  Fy1 = T1 * sin(alpha1);
            Fx2 = T2 * cos(alpha2);  Fy2 = T2 * sin(alpha2);
            Fx3 = 0;
            Fy3 = T3;

            X_thrust = Fx1 + Fx2 + Fx3;
            Y_thrust = Fy1 + Fy2 + Fy3;
            N_thrust = stern_port_x * Fy1 - stern_port_y * Fx1 + stern_starboard_x * Fy2 - stern_starboard_y * Fx2 + bow_x * Fy3 - bow_y * Fx3;

            X_hyd_dim = X_hyd * 0.5 * rho_w * L_ship^2 * U^2;
            Y_hyd_dim = Y_hyd * 0.5 * rho_w * L_ship^2 * U^2;
            N_hyd_dim = N_hyd * 0.5 * rho_w * L_ship^3 * U^2;

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
            n3_abs_rpm = if_else(n3 >= 0, n3, -n3);

            Tm1 = if_else(n1_abs_rpm > 18, 5.65 / (n1_abs_rpm/60 + 1e-6), 18.83);
            Tm2 = if_else(n2_abs_rpm > 18, 5.65 / (n2_abs_rpm/60 + 1e-6), 18.83);
            Tm3 = if_else(n3_abs_rpm > 18, 5.65 / (n3_abs_rpm/60 + 1e-6), 18.83) * 1.5;
            Tm1 = if_else(Tm1 > 20, 20, if_else(Tm1 < 1, 1, Tm1));
            Tm2 = if_else(Tm2 > 20, 20, if_else(Tm2 < 1, 1, Tm2));
            Tm3 = if_else(Tm3 > 20, 20, if_else(Tm3 < 1, 1, Tm3));

            n1_c_sat = if_else(n1_c > obj.n_max, obj.n_max, if_else(n1_c < obj.n_min, obj.n_min, n1_c));
            n2_c_sat = if_else(n2_c > obj.n_max, obj.n_max, if_else(n2_c < obj.n_min, obj.n_min, n2_c));
            n3_c_sat = if_else(n3_c > obj.n_bow_max, obj.n_bow_max, if_else(n3_c < obj.n_bow_min, obj.n_bow_min, n3_c));

            n1_dot = (n1_c_sat - n1) / Tm1;
            n2_dot = (n2_c_sat - n2) / Tm2;
            n3_dot = (n3_c_sat - n3) / Tm3;

            n1_dot = if_else(n1_dot > obj.Dn_max, obj.Dn_max, if_else(n1_dot < -obj.Dn_max, -obj.Dn_max, n1_dot));
            n2_dot = if_else(n2_dot > obj.Dn_max, obj.Dn_max, if_else(n2_dot < -obj.Dn_max, -obj.Dn_max, n2_dot));
            n3_dot = if_else(n3_dot > obj.Dn_bow_max, obj.Dn_bow_max, if_else(n3_dot < -obj.Dn_bow_max, -obj.Dn_bow_max, n3_dot));

            xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot; n1_dot; n2_dot; n3_dot];
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

function M = padDiag(M, n)
    if isempty(M)
        M = zeros(n, n);
        return;
    end
    if size(M,1) == n && size(M,2) == n
        return;
    end
    M_pad = zeros(n, n);
    r = min(n, size(M,1));
    c = min(n, size(M,2));
    M_pad(1:r, 1:c) = M(1:r, 1:c);
    M = M_pad;
end
