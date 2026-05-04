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
        alpha_rate_max = 0.20944  % Max azimuth rate [rad/s] = 12 deg/s (ABB spec)

        % Obstacle settings
        max_obs = 5
        max_halfplanes = 0
        r_safety = 35

        % Collision geometry (point | oriented-rectangle)
        collision_model = 'oriented-rectangle'
        hull_length_m = 0
        hull_beam_m = 0
        hull_half_length_m = 0
        hull_half_beam_m = 0
        hull_clearance_m = 0
        hull_smooth_eps = 1e-4
        azimuth_split_max_default = pi          % Default max azimuth angle split between port/starboard [rad]
        stern_cmd_split_max_default = 40        % Default max RPM split between port/starboard stern [rpm]


        % NEW: Actuator and forward motion control
        actuator_force_weight = 0.01     % Penalty on RPM magnitude (reduced for aggressive response) — was 0.05
        forward_incentive_weight = 2.5   % Penalty on backward motion (encourages forward bow) — unchanged
        waypoint_heading_weight = 0.0    % Extra heading penalty at waypoints (0=disabled)
        u_min_forward = 0.5              % Default surge lower bound [m/s] (forward-only unless overridden)
        max_brake_rate = 0.3             % Max deceleration rate [m/s²] to minimize braking fuel costs
        soft_obs_weight = 2e5            % Large penalty when soft obstacle slack is enabled
        soft_obs_default_max_m = 0.0     % Default obstacle slack cap [m] (0 disables softening)
        terminal_pose_slack_weight = 8e4 % Penalty for terminal-pose soft slack
        terminal_pose_slack_default_max = 0.0 % Default max terminal slack (0 disables by default)
        stage_state_cost_scale = 1.0  % Scale on stage state-tracking penalties
        stage_input_tracking_scale = 1.0 % Scale on stage input-tracking (U-uref) penalties
        terminal_speed_weight = NaN   % Override terminal speed weight (NaN -> use 2*Q_u)
        terminal_pos_weight = NaN     % Override terminal position weight (NaN -> use 2*Q_x and 2*Q_y)
        terminal_heading_weight = NaN % Override terminal heading weight (NaN -> use 2*Q_psi)
        terminal_cost_scale = 1.0     % Global scale on terminal state-tracking cost
        terminal_actuator_cost_scale = 1.0 % Scale on terminal actuator effort penalty
        terminal_forward_cost_scale = 1.0  % Scale on terminal reverse-motion penalty

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

        p_layout          % Struct mapping param blocks -> [offset, length]
        constraint_names  % Cell array of constraint block names
        constraint_block_sizes % Cell array of constraint block sizes
        g_func            % CasADi function for fast constraint evaluation
    end

    methods

        function obj = NMPC_Container_final(cfg)
            if nargin < 1 || isempty(cfg)
                cfg = struct();
            end

            obj.N  = getOr(cfg, 'N',  20);
            obj.dt = getOr(cfg, 'dt', 1.0);
            obj.Q = padDiag(getOr(cfg, 'Q', diag([2.0, 0.1, 3.0, 3.0, 3.0, 6.0, 0.001, 0.001, 0.001])), 9);
            obj.R = padDiag(getOr(cfg, 'R', diag([0.08, 0.08, 0.005, 0.005, 0.005])), 5);
            obj.R_rate = padDiag(getOr(cfg, 'R_rate', diag([0.04, 0.04, 0.002, 0.002, 0.002])), 5);
            obj.max_obs = getOr(cfg, 'max_obs', 5);
            obj.max_halfplanes = max(0, round(getOr(cfg, 'max_halfplanes', 0)));
            obj.r_safety = getOr(cfg, 'r_safety', 30);
            obj.collision_model = lower(strtrim(getOr(cfg, 'collision_model', 'point')));
            obj.actuator_force_weight = getOr(cfg, 'actuator_force_weight', 0.05);
            obj.forward_incentive_weight = getOr(cfg, 'forward_incentive_weight', 2.5);
            obj.waypoint_heading_weight = getOr(cfg, 'waypoint_heading_weight', 0.0);
            obj.u_min_forward = getOr(cfg, 'u_min_forward', 0.5);
            obj.max_brake_rate = getOr(cfg, 'max_brake_rate', 0.3);
            obj.soft_obs_weight = getOr(cfg, 'soft_obs_weight', 2e5);
            obj.soft_obs_default_max_m = getOr(cfg, 'soft_obs_default_max_m', 0.0);
            obj.terminal_pose_slack_weight = getOr(cfg, 'terminal_pose_slack_weight', 8e4);
            obj.terminal_pose_slack_default_max = getOr(cfg, 'terminal_pose_slack_default_max', 0.0);
            obj.stage_state_cost_scale = getOr(cfg, 'stage_state_cost_scale', 1.0);
            obj.stage_input_tracking_scale = getOr(cfg, 'stage_input_tracking_scale', 1.0);
            obj.terminal_speed_weight = getOr(cfg, 'terminal_speed_weight', NaN);
            obj.terminal_pos_weight = getOr(cfg, 'terminal_pos_weight', NaN);
            obj.terminal_heading_weight = getOr(cfg, 'terminal_heading_weight', NaN);
            obj.terminal_cost_scale = getOr(cfg, 'terminal_cost_scale', 1.0);
            obj.terminal_actuator_cost_scale = getOr(cfg, 'terminal_actuator_cost_scale', 1.0);
            obj.terminal_forward_cost_scale = getOr(cfg, 'terminal_forward_cost_scale', 1.0);

            obj.hull_length_m = getOr(cfg, 'hull_length_m', 36);
            obj.hull_beam_m = getOr(cfg, 'hull_beam_m', 12);
            obj.hull_half_length_m = 0.5 * max(1.0, obj.hull_length_m);
            obj.hull_half_beam_m = 0.5 * max(0.5, obj.hull_beam_m);
            obj.hull_clearance_m = getOr(cfg, 'hull_clearance_m', obj.r_safety);
            obj.hull_smooth_eps = getOr(cfg, 'hull_smooth_eps', 1e-4);
            obj.enable_diagnostics = logical(getOr(cfg, 'enable_diagnostics', false));

            fprintf('NMPC_Container_final: N=%d, dt=%.2f, obs_slots=%d\n', ...
                obj.N, obj.dt, obj.max_obs);
            fprintf('  map half-plane slots: %d\n', obj.max_halfplanes);
            fprintf('  9-state model: [u v r x y psi n1 n2 n3]\n');
            fprintf('  5 controls: [alpha1 alpha2 n1_c n2_c n3_c]\n');
            fprintf('  surge lower bound: u >= %.2f m/s\n', obj.u_min_forward);
            fprintf('  azimuth rate limit: %.2f deg/s\n', rad2deg(obj.alpha_rate_max));
            fprintf('  max brake rate: %.2f m/s²\n', obj.max_brake_rate);
            fprintf('  soft obstacle slack: weight=%.1f, default_max=%.2f m\n', ...
                obj.soft_obs_weight, obj.soft_obs_default_max_m);
            fprintf('  terminal-pose slack: weight=%.1f, default_max=%.2f\n', ...
                obj.terminal_pose_slack_weight, obj.terminal_pose_slack_default_max);
            fprintf('  stage scales: state=%.2f, input-track=%.2f\n', ...
                obj.stage_state_cost_scale, obj.stage_input_tracking_scale);
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
            n_hp = obj.max_halfplanes;
            n_term_slack = 6;

            %  DECISION VARIABLES
            X = SX.sym('X', n_state, N_h+1);
            U = SX.sym('U', n_ctrl, N_h);
            S_obs = SX.sym('S_obs', n_obs, N_h+1); % Soft obstacle slack [m]
            S_term = SX.sym('S_term', n_term_slack, 1); % Terminal pose/velocity soft slack

            %  PARAMETERS
            P_x0       = SX.sym('P_x0', n_state, 1);
            P_xref     = SX.sym('P_xref', n_state, N_h+1);
            P_uref     = SX.sym('P_uref', n_ctrl, N_h);
            P_n_obs_real = SX.sym('P_n_obs_real', 1, 1);
            P_obs_pos  = SX.sym('P_obs_pos', 2, n_obs);
            P_obs_rad  = SX.sym('P_obs_rad', n_obs, 1);
            P_n_hp_real = SX.sym('P_n_hp_real', 1, 1);
            P_hp_n = SX.sym('P_hp_n', 2, n_hp);
            P_hp_b = SX.sym('P_hp_b', n_hp, 1);
            P_u_prev   = SX.sym('P_u_prev', n_ctrl, 1);  % NEW: Previous applied control
            P_max_brake_rate = SX.sym('P_max_brake_rate', 1, 1);  % NEW: Max deceleration rate
            P_q_state = SX.sym('P_q_state', n_state, 1);
            P_r_input = SX.sym('P_r_input', n_ctrl, 1);
            P_r_rate = SX.sym('P_r_rate', n_ctrl, 1);
            P_stage_scales = SX.sym('P_stage_scales', 2, 1); % [stage_state_scale stage_input_scale]
            P_terminal_weights = SX.sym('P_terminal_weights', 4, 1); % [u x y psi]
            P_terminal_scales = SX.sym('P_terminal_scales', 3, 1); % [state actuator reverse]
            P_collision_clearance = SX.sym('P_collision_clearance', 1, 1);
            P_term_pose_eps = SX.sym('P_term_pose_eps', 3, 1); % [eps_x eps_y eps_psi]
            P_term_vel_max = SX.sym('P_term_vel_max', 3, 1);   % [|u| |v| |r|] terminal bounds
            P_berth_corr = SX.sym('P_berth_corr', 7, 1); % [x0 y0 psi half_w along_min along_max enable]
            P_sync_limits = SX.sym('P_sync_limits', 2, 1); % [max_azimuth_split_rad max_stern_cmd_split_rpm]

            P_all = vertcat(P_x0, P_xref(:), P_uref(:), P_n_obs_real, ...
                            P_obs_pos(:), P_obs_rad, P_n_hp_real, P_hp_n(:), P_hp_b, ...
                            P_u_prev, P_max_brake_rate, ...
                            P_q_state, P_r_input, P_r_rate, ...
                            P_stage_scales, P_terminal_weights, P_terminal_scales, ...
                            P_collision_clearance, ...
                            P_term_pose_eps, P_term_vel_max, P_berth_corr, P_sync_limits);
            obj.np_total = size(P_all, 1);

            %  COST FUNCTION
            w_stage_state = P_stage_scales(1);
            w_stage_input = P_stage_scales(2);
            w_term_u = P_terminal_weights(1);
            w_term_x = P_terminal_weights(2);
            w_term_y = P_terminal_weights(3);
            w_term_psi = P_terminal_weights(4);
            w_terminal_state = P_terminal_scales(1);
            w_terminal_actuator = P_terminal_scales(2);
            w_terminal_forward = P_terminal_scales(3);

            J = 0;

            % Accumulate each constraint family in a named block so the final
            % vectors stay easy to inspect during debugging.
            g_blocks = {};
            lb_blocks = {};
            ub_blocks = {};
            block_names = {};
            block_sizes = [];

            % Stage cost
            for k = 1:N_h
                dpsi = X(6,k) - P_xref(6,k);
                psi_err = atan2(sin(dpsi), cos(dpsi));

                stage_state_cost = 0;
                stage_state_cost = stage_state_cost + P_q_state(1) * (X(1,k) - P_xref(1,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(2) * (X(2,k) - P_xref(2,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(3) * (X(3,k) - P_xref(3,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(4) * (X(4,k) - P_xref(4,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(5) * (X(5,k) - P_xref(5,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(6) * psi_err^2;
                stage_state_cost = stage_state_cost + P_q_state(7) * (X(7,k) - P_xref(7,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(8) * (X(8,k) - P_xref(8,k))^2;
                stage_state_cost = stage_state_cost + P_q_state(9) * (X(9,k) - P_xref(9,k))^2;
                J = J + w_stage_state * stage_state_cost;
                turn_heading_gate = min(1.0, abs(P_xref(3,k)) / 0.10);
                J = J + obj.waypoint_heading_weight * turn_heading_gate * psi_err^2;

                % NEW: Actuator force penalty (reduces thruster swaying)
                % Penalizes the magnitude of RPM commands to discourage excessive actuation
                J = J + obj.actuator_force_weight * (X(7,k)^2 + X(8,k)^2 + X(9,k)^2);

                % NEW: Forward velocity encouragement (makes bow go forward)
                % Penalizes negative forward speed to naturally prefer forward motion
                u_back = min(0, X(1,k));  % Only penalize if going backward
                J = J + obj.forward_incentive_weight * u_back^2;

                du = U(:,k) - P_uref(:,k);
                J = J + w_stage_input * sum1(P_r_input .* (du.^2));

                % Obstacle-softening penalty (active only if slack bounds permit >0 values).
                for j = 1:n_obs
                    J = J + obj.soft_obs_weight * S_obs(j,k)^2;
                end
            end

            % Control rate cost
            for k = 1:(N_h-1)
                dU = U(:,k+1) - U(:,k);
                J = J + sum1(P_r_rate .* (dU.^2));
            end

            % Terminal cost
            dpsi_N = X(6,N_h+1) - P_xref(6,N_h+1);
            psi_err_N = atan2(sin(dpsi_N), cos(dpsi_N));
            terminal_state_cost = 0;
            terminal_state_cost = terminal_state_cost + w_term_u * (X(1,N_h+1) - P_xref(1,N_h+1))^2;
            terminal_state_cost = terminal_state_cost + w_term_x * (X(4,N_h+1) - P_xref(4,N_h+1))^2;
            terminal_state_cost = terminal_state_cost + w_term_y * (X(5,N_h+1) - P_xref(5,N_h+1))^2;
            terminal_state_cost = terminal_state_cost + w_term_psi * psi_err_N^2;
            J = J + w_terminal_state * terminal_state_cost;
            terminal_heading_gate = min(1.0, abs(P_xref(3,N_h+1)) / 0.10);
            J = J + obj.waypoint_heading_weight * terminal_heading_gate * psi_err_N^2;
            % NEW: Terminal actuator force and forward incentive penalties
            J = J + w_terminal_actuator * 2 * obj.actuator_force_weight * ...
                (X(7,N_h+1)^2 + X(8,N_h+1)^2 + X(9,N_h+1)^2);
            u_back_N = min(0, X(1,N_h+1));
            J = J + w_terminal_forward * 2 * obj.forward_incentive_weight * u_back_N^2;
            for j = 1:n_obs
                J = J + obj.soft_obs_weight * S_obs(j,N_h+1)^2;
            end
            J = J + obj.terminal_pose_slack_weight * (S_term' * S_term);

            %  CONSTRAINTS
            % --- Initial condition (n_state equalities) ---
            g_blocks{end+1} = X(:,1) - P_x0;
            lb_blocks{end+1} = zeros(n_state, 1);
            ub_blocks{end+1} = zeros(n_state, 1);
            block_names{end+1} = 'Initial condition';
            block_sizes(end+1) = n_state;

            % --- Dynamics (n_state * N_h equalities) ---
            g_dyn = [];
            for k = 1:N_h
                xdot_k = obj.dynamicsCasADi(X(:,k), U(:,k));
                x_next = X(:,k) + xdot_k * obj.dt;
                g_dyn = vertcat(g_dyn, X(:,k+1) - x_next);
            end
            g_blocks{end+1} = g_dyn;
            lb_blocks{end+1} = zeros(n_state*N_h, 1);
            ub_blocks{end+1} = zeros(n_state*N_h, 1);
            block_names{end+1} = 'Dynamics';
            block_sizes(end+1) = n_state*N_h;

            % --- Obstacle avoidance (n_obs * (N_h+1) inequalities) ---
            g_obs = [];
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
                        g_obs = vertcat(g_obs, dist - P_obs_rad(j) - P_collision_clearance + S_obs(j,k));
                    else
                        dx = X(4,k) - P_obs_pos(1,j);
                        dy = X(5,k) - P_obs_pos(2,j);
                        dist = sqrt(dx^2 + dy^2 + 1e-3);
                        g_obs = vertcat(g_obs, dist - P_obs_rad(j) - P_collision_clearance + S_obs(j,k));
                    end
                end
            end
            g_blocks{end+1} = g_obs;
            lb_blocks{end+1} = zeros(n_obs*(N_h+1), 1);
            ub_blocks{end+1} = inf(n_obs*(N_h+1), 1);
            block_names{end+1} = 'Obstacle avoidance';
            block_sizes(end+1) = n_obs*(N_h+1);

            % --- Map half-plane avoidance (n_hp * (N_h+1) inequalities) ---
            g_hp = [];
            for k = 1:(N_h+1)
                for h = 1:n_hp
                    signed_dist = P_hp_n(1,h) * X(4,k) + P_hp_n(2,h) * X(5,k) - P_hp_b(h);

                    if strcmpi(obj.collision_model, 'oriented-rectangle')
                        cpsi = cos(X(6,k));
                        spsi = sin(X(6,k));
                        hull_proj = abs(P_hp_n(1,h) * cpsi + P_hp_n(2,h) * spsi) * obj.hull_half_length_m + ...
                            abs(-P_hp_n(1,h) * spsi + P_hp_n(2,h) * cpsi) * obj.hull_half_beam_m;
                        g_hp = vertcat(g_hp, signed_dist - hull_proj - P_collision_clearance);
                    else
                        g_hp = vertcat(g_hp, signed_dist - P_collision_clearance);
                    end
                end
            end
            g_blocks{end+1} = g_hp;
            lb_blocks{end+1} = zeros(n_hp*(N_h+1), 1);
            ub_blocks{end+1} = inf(n_hp*(N_h+1), 1);
            block_names{end+1} = 'Map half-plane avoidance';
            block_sizes(end+1) = n_hp*(N_h+1);

            % --- Azimuth rate constraints ---
            % FIRST STEP: Constrain U(:,1) relative to previous control
            % This prevents the "spiral oscillation" bug!
            g_rate = [];
            d_alpha1_first = U(1,1) - P_u_prev(1);
            g_rate = vertcat(g_rate, d_alpha1_first - obj.alpha_rate_max * obj.dt);
            g_rate = vertcat(g_rate, -d_alpha1_first - obj.alpha_rate_max * obj.dt);

            d_alpha2_first = U(2,1) - P_u_prev(2);
            g_rate = vertcat(g_rate, d_alpha2_first - obj.alpha_rate_max * obj.dt);
            g_rate = vertcat(g_rate, -d_alpha2_first - obj.alpha_rate_max * obj.dt);

            % SUBSEQUENT STEPS: Constrain consecutive controls
            for k = 1:(N_h-1)
                d_alpha1 = U(1,k+1) - U(1,k);
                g_rate = vertcat(g_rate, d_alpha1 - obj.alpha_rate_max * obj.dt);
                g_rate = vertcat(g_rate, -d_alpha1 - obj.alpha_rate_max * obj.dt);

                d_alpha2 = U(2,k+1) - U(2,k);
                g_rate = vertcat(g_rate, d_alpha2 - obj.alpha_rate_max * obj.dt);
                g_rate = vertcat(g_rate, -d_alpha2 - obj.alpha_rate_max * obj.dt);
            end
            g_blocks{end+1} = g_rate;
            lb_blocks{end+1} = -inf(4 + 4*(N_h-1), 1);
            ub_blocks{end+1} = zeros(4 + 4*(N_h-1), 1);
            block_names{end+1} = 'Azimuth rate';
            block_sizes(end+1) = 4 + 4*(N_h-1);

            % --- Braking constraint (surge deceleration limit) ---
            % NEW: Constraint to reduce excessive braking (fuel cost minimization).
            % Between consecutive prediction steps, forward speed cannot decrease faster than max_brake_rate.
            % This prevents the optimizer from making unrealistic sharp decelerations.
            % First step: constrain u(k=1) relative to initial state u0
            g_brake = [];
            du_brake_first = X(1,1) - P_x0(1);  % This should be >= -max_brake_rate * dt
            g_brake = vertcat(g_brake, du_brake_first + P_max_brake_rate * obj.dt);
            % Subsequent steps: constrain u(k) relative to u(k-1)
            for k = 1:(N_h)
                du_brake = X(1,k+1) - X(1,k);
                g_brake = vertcat(g_brake, du_brake + P_max_brake_rate * obj.dt);
            end
            g_blocks{end+1} = g_brake;
            lb_blocks{end+1} = zeros(1 + N_h, 1);
            ub_blocks{end+1} = inf(1 + N_h, 1);
            block_names{end+1} = 'Brake deceleration';
            block_sizes(end+1) = 1 + N_h;

            % --- Terminal pose and velocity envelope (soft inequalities) ---
            % Pose envelope centered on terminal reference at k=N+1.
            g_term = [];
            dx_N = X(4,N_h+1) - P_xref(4,N_h+1);
            dy_N = X(5,N_h+1) - P_xref(5,N_h+1);
            dpsi_N_c = atan2(sin(X(6,N_h+1) - P_xref(6,N_h+1)), cos(X(6,N_h+1) - P_xref(6,N_h+1)));

            g_term = vertcat(g_term, dx_N - P_term_pose_eps(1) - S_term(1));
            g_term = vertcat(g_term, -dx_N - P_term_pose_eps(1) - S_term(1));
            g_term = vertcat(g_term, dy_N - P_term_pose_eps(2) - S_term(2));
            g_term = vertcat(g_term, -dy_N - P_term_pose_eps(2) - S_term(2));
            g_term = vertcat(g_term, dpsi_N_c - P_term_pose_eps(3) - S_term(3));
            g_term = vertcat(g_term, -dpsi_N_c - P_term_pose_eps(3) - S_term(3));

            % Optional terminal velocity envelope (absolute-value bounds).
            g_term = vertcat(g_term, abs(X(1,N_h+1)) - P_term_vel_max(1) - S_term(4));
            g_term = vertcat(g_term, abs(X(2,N_h+1)) - P_term_vel_max(2) - S_term(5));
            g_term = vertcat(g_term, abs(X(3,N_h+1)) - P_term_vel_max(3) - S_term(6));
            g_blocks{end+1} = g_term;
            lb_blocks{end+1} = -inf(9, 1);
            ub_blocks{end+1} = zeros(9, 1);
            block_names{end+1} = 'Terminal pose and velocity';
            block_sizes(end+1) = 9;

            % --- Berth corridor hard constraints in berth-local frame ---
            % Local frame origin is berth point, x-axis aligned with berth heading.
            g_corr = [];
            corr_origin_x = P_berth_corr(1);
            corr_origin_y = P_berth_corr(2);
            corr_psi = P_berth_corr(3);
            corr_half_w = P_berth_corr(4);
            corr_along_min = P_berth_corr(5);
            corr_along_max = P_berth_corr(6);
            corr_enable = P_berth_corr(7);
            corr_big_relax = (1 - corr_enable) * 1e6;

            for k = 1:(N_h+1)
                dx_b = X(4,k) - corr_origin_x;
                dy_b = X(5,k) - corr_origin_y;
                x_local = cos(corr_psi) * dx_b + sin(corr_psi) * dy_b;
                y_local = -sin(corr_psi) * dx_b + cos(corr_psi) * dy_b;

                g_corr = vertcat(g_corr, y_local - corr_half_w - corr_big_relax);
                g_corr = vertcat(g_corr, -y_local - corr_half_w - corr_big_relax);
                g_corr = vertcat(g_corr, corr_along_min - x_local - corr_big_relax);
                g_corr = vertcat(g_corr, x_local - corr_along_max - corr_big_relax);
            end
            g_blocks{end+1} = g_corr;
            lb_blocks{end+1} = -inf(4 * (N_h + 1), 1);
            ub_blocks{end+1} = zeros(4 * (N_h + 1), 1);
            block_names{end+1} = 'Berth corridor';
            block_sizes(end+1) = 4 * (N_h + 1);

            % --- Twin-stern azipod synchrony constraints (phase-tunable) ---
            g_sync = [];
            max_azi_split = P_sync_limits(1);
            max_stern_split = P_sync_limits(2);
            for k = 1:N_h
                d_alpha_sync = U(1,k) - U(2,k);
                d_stern_sync = U(3,k) - U(4,k);
                g_sync = vertcat(g_sync, d_alpha_sync - max_azi_split);
                g_sync = vertcat(g_sync, -d_alpha_sync - max_azi_split);
                g_sync = vertcat(g_sync, d_stern_sync - max_stern_split);
                g_sync = vertcat(g_sync, -d_stern_sync - max_stern_split);
            end
            g_blocks{end+1} = g_sync;
            lb_blocks{end+1} = -inf(4 * N_h, 1);
            ub_blocks{end+1} = zeros(4 * N_h, 1);
            block_names{end+1} = 'Twin-stern sync';
            block_sizes(end+1) = 4 * N_h;

            g = vertcat(g_blocks{:});
            lbg = vertcat(lb_blocks{:});
            ubg = vertcat(ub_blocks{:});
            n_constraints = size(g, 1);

            %  VARIABLE BOUNDS
            OPT = vertcat(X(:), U(:), S_obs(:), S_term(:));
            n_vars = size(OPT, 1);

            lbx = -inf(n_vars, 1);
            ubx =  inf(n_vars, 1);

            % State bounds
            for k = 1:(N_h+1)
                base = (k-1)*n_state;
                lbx(base+1) = obj.u_min_forward;  ubx(base+1) = 12;
                lbx(base+2) = -3;         ubx(base+2) = 3;
                lbx(base+3) = -0.25;      ubx(base+3) = 0.25;
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
                lbx(base+5) = obj.n_bow_min;   ubx(base+5) = obj.n_bow_max;
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

            % Terminal slack bounds (can be relaxed per solve call).
            t_off = s_off + n_obs * (N_h+1);
            for j = 1:n_term_slack
                idx_t = t_off + j;
                lbx(idx_t) = 0;
                ubx(idx_t) = obj.terminal_pose_slack_default_max;
            end

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
            obj.constraint_names = block_names;
            obj.constraint_block_sizes = block_sizes;
            obj.g_func = Function('g_func', {X, U, S_obs, S_term, P_all}, {g});
            obj.solver_built = true;

            fprintf('  nlpsol built: %d vars, %d constraints\n', n_vars, n_constraints);
            for idx = 1:numel(block_names)
                fprintf('    - %s: %d\n', block_names{idx}, block_sizes(idx));
            end
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
            %                    .enable_terminal_pose (logical)
            %                    .term_pose_eps_xy_m (nonnegative scalar)
            %                    .term_pose_eps_psi_rad (nonnegative scalar)
            %                    .term_vel_max_u_mps (nonnegative scalar)
            %                    .term_vel_max_v_mps (nonnegative scalar)
            %                    .term_vel_max_r_radps (nonnegative scalar)
            %                    .term_pose_slack_max (nonnegative scalar)
            %                    .enable_berth_corridor (logical)
            %                    .berth_corridor_origin_xy ([x; y])
            %                    .berth_corridor_heading_rad (scalar)
            %                    .berth_corridor_half_width_m (nonnegative scalar)
            %                    .berth_corridor_along_min_m (scalar)
            %                    .berth_corridor_along_max_m (scalar)
            %                    .map_halfplanes (struct array with fields: normal [2x1], offset scalar)
            %                    .max_azimuth_split (nonnegative scalar, rad)
            %                    .max_stern_cmd_split (nonnegative scalar, rpm)
            %                    .state_weights_diag (9x1 or diagonal matrix)
            %                    .input_weights_diag (5x1 or diagonal matrix)
            %                    .rate_weights_diag (5x1 or diagonal matrix)
            %                    .stage_state_cost_scale (nonnegative scalar)
            %                    .stage_input_tracking_scale (nonnegative scalar)
            %                    .terminal_speed_weight (nonnegative scalar)
            %                    .terminal_pos_weight (nonnegative scalar or [x y])
            %                    .terminal_heading_weight (nonnegative scalar)
            %                    .terminal_cost_scale (nonnegative scalar)
            %                    .terminal_actuator_cost_scale (nonnegative scalar)
            %                    .terminal_forward_cost_scale (nonnegative scalar)
            %                    .collision_clearance_m (nonnegative scalar)
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
            n_hp = obj.max_halfplanes;
            n_term_slack = 6;

            if nargin < 6 || isempty(u_min_forward_override)
                u_min_local = obj.u_min_forward;
            else
                u_min_local = max(-4.0, min(12.0, u_min_forward_override));
            end

            if nargin < 7 || isempty(solve_opts)
                solve_opts = struct();
            end

            q_state_default = diag(obj.Q);
            r_input_default = diag(obj.R);
            r_rate_default = diag(obj.R_rate);
            q_state_diag = padVector(getOr(solve_opts, 'state_weights_diag', q_state_default), n_state, q_state_default);
            r_input_diag = padVector(getOr(solve_opts, 'input_weights_diag', r_input_default), n_ctrl, r_input_default);
            r_rate_diag = padVector(getOr(solve_opts, 'rate_weights_diag', r_rate_default), n_ctrl, r_rate_default);
            q_state_diag = max(0.0, q_state_diag);
            r_input_diag = max(0.0, r_input_diag);
            r_rate_diag = max(0.0, r_rate_diag);

            stage_state_scale = max(0.0, getOr(solve_opts, 'stage_state_cost_scale', obj.stage_state_cost_scale));
            stage_input_scale = max(0.0, getOr(solve_opts, 'stage_input_tracking_scale', obj.stage_input_tracking_scale));

            if isnan(obj.terminal_speed_weight)
                term_speed_default = 2 * q_state_default(1);
            else
                term_speed_default = obj.terminal_speed_weight;
            end
            term_speed_weight = max(0.0, getOr(solve_opts, 'terminal_speed_weight', term_speed_default));

            if isnan(obj.terminal_pos_weight)
                term_pos_default = [2 * q_state_default(4); 2 * q_state_default(5)];
            else
                term_pos_default = [obj.terminal_pos_weight; obj.terminal_pos_weight];
            end
            term_pos_raw = getOr(solve_opts, 'terminal_pos_weight', term_pos_default);
            term_pos_weight = padVector(term_pos_raw, 2, term_pos_default);
            term_pos_weight = max(0.0, term_pos_weight);

            if isnan(obj.terminal_heading_weight)
                term_heading_default = 2 * q_state_default(6);
            else
                term_heading_default = obj.terminal_heading_weight;
            end
            term_heading_weight = max(0.0, getOr(solve_opts, 'terminal_heading_weight', term_heading_default));

            terminal_state_scale = max(0.0, getOr(solve_opts, 'terminal_cost_scale', obj.terminal_cost_scale));
            terminal_actuator_scale = max(0.0, getOr(solve_opts, 'terminal_actuator_cost_scale', obj.terminal_actuator_cost_scale));
            terminal_forward_scale = max(0.0, getOr(solve_opts, 'terminal_forward_cost_scale', obj.terminal_forward_cost_scale));

            if strcmpi(obj.collision_model, 'oriented-rectangle')
                collision_clearance_default = obj.hull_clearance_m;
            else
                collision_clearance_default = obj.r_safety;
            end
            collision_clearance = max(0.0, getOr(solve_opts, 'collision_clearance_m', collision_clearance_default));

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

            term_enable = logical(getOr(solve_opts, 'enable_terminal_pose', false));
            if term_enable
                term_eps_xy_raw = getOr(solve_opts, 'term_pose_eps_xy_m', 8.0);
                if isscalar(term_eps_xy_raw)
                    term_eps_x = max(0.0, term_eps_xy_raw);
                    term_eps_y = term_eps_x;
                else
                    term_eps_x = max(0.0, term_eps_xy_raw(1));
                    term_eps_y = max(0.0, term_eps_xy_raw(min(2, numel(term_eps_xy_raw))));
                end
                term_eps_psi = max(0.0, getOr(solve_opts, 'term_pose_eps_psi_rad', deg2rad(8.0)));
                term_vel_u = max(0.0, getOr(solve_opts, 'term_vel_max_u_mps', inf));
                term_vel_v = max(0.0, getOr(solve_opts, 'term_vel_max_v_mps', inf));
                term_vel_r = max(0.0, getOr(solve_opts, 'term_vel_max_r_radps', inf));
                term_slack_max = max(0.0, getOr(solve_opts, 'term_pose_slack_max', obj.terminal_pose_slack_default_max));
            else
                term_eps_x = 1e6;
                term_eps_y = 1e6;
                term_eps_psi = pi;
                term_vel_u = 1e6;
                term_vel_v = 1e6;
                term_vel_r = 1e6;
                term_slack_max = 0.0;
            end

            term_pose_eps_vec = [term_eps_x; term_eps_y; term_eps_psi];
            term_vel_max_vec = [term_vel_u; term_vel_v; term_vel_r];

            corr_enable = logical(getOr(solve_opts, 'enable_berth_corridor', false));
            if corr_enable
                corr_origin_raw = getOr(solve_opts, 'berth_corridor_origin_xy', [0; 0]);
                if numel(corr_origin_raw) < 2
                    corr_origin_raw = [0; 0];
                end
                corr_origin_x = corr_origin_raw(1);
                corr_origin_y = corr_origin_raw(2);
                corr_psi = getOr(solve_opts, 'berth_corridor_heading_rad', 0.0);
                corr_half_w = max(0.0, getOr(solve_opts, 'berth_corridor_half_width_m', 1e6));
                corr_along_min = getOr(solve_opts, 'berth_corridor_along_min_m', -1e6);
                corr_along_max = getOr(solve_opts, 'berth_corridor_along_max_m', 1e6);
                if corr_along_max < corr_along_min
                    tmp = corr_along_min;
                    corr_along_min = corr_along_max;
                    corr_along_max = tmp;
                end
                corr_enable_scalar = 1.0;
            else
                corr_origin_x = 0.0;
                corr_origin_y = 0.0;
                corr_psi = 0.0;
                corr_half_w = 1e6;
                corr_along_min = -1e6;
                corr_along_max = 1e6;
                corr_enable_scalar = 0.0;
            end
            berth_corridor_vec = [corr_origin_x; corr_origin_y; corr_psi; corr_half_w; ...
                corr_along_min; corr_along_max; corr_enable_scalar];

            % Twin-stern azipod synchrony limits (enforced as NLP inequalities)
            max_az_split_local = max(0.0, min(2*obj.alpha_max, ...
                getOr(solve_opts, 'max_azimuth_split', obj.azimuth_split_max_default)));
            max_stern_split_local = max(0.0, min(obj.n_max - obj.n_min, ...
                getOr(solve_opts, 'max_stern_cmd_split', obj.stern_cmd_split_max_default)));
            sync_limits_vec = [max_az_split_local; max_stern_split_local];

            % Handle previous control
            if nargin < 5 || isempty(u_prev)
                if ~isempty(obj.prev_u)
                    u_prev = obj.prev_u;
                else
                    u_prev = [0; 0; x0(7); x0(8); x0(9)];  % Default to current RPMs if no previous control stored
                end
            end

            % Obstacle setup
            obs_pos = 1e8 * ones(2, n_obs);
            obs_rad = zeros(n_obs, 1);
            n_real = 0;
            hp_n = zeros(2, n_hp);
            hp_b = -1e8 * ones(n_hp, 1);
            n_hp_real = 0;

            if nargin >= 4 && ~isempty(obstacles)
                n_real = min(length(obstacles), n_obs);
                for j = 1:n_real
                    obs_pos(:,j) = obstacles(j).position(1:2);
                    obs_rad(j) = obstacles(j).radius;
                end
            end

            if isfield(solve_opts, 'map_halfplanes') && ~isempty(solve_opts.map_halfplanes) && n_hp > 0
                hp_in = solve_opts.map_halfplanes;
                n_hp_real = min(length(hp_in), n_hp);
                for h = 1:n_hp_real
                    n_h = hp_in(h).normal(:);
                    if numel(n_h) < 2
                        continue;
                    end
                    n_h = n_h(1:2);
                    n_norm = norm(n_h);
                    if n_norm < 1e-9
                        continue;
                    end
                    hp_n(:,h) = n_h / n_norm;
                    hp_b(h) = hp_in(h).offset;
                end
            end

            % Reference control
            u_ref = zeros(n_ctrl, N_h);
            u_ref(3,:) = x0(7);
            u_ref(4,:) = x0(8);
            u_ref(5,:) = min(max(x0(9), obj.n_bow_min), n3_max_local);

            % Build parameter vector (now includes u_prev and max_brake_rate)
            p_val = [x0(:); x_ref(:); u_ref(:); n_real; obs_pos(:); obs_rad(:); n_hp_real; hp_n(:); hp_b(:); ...
                u_prev(:); obj.max_brake_rate; ...
                q_state_diag(:); r_input_diag(:); r_rate_diag(:); ...
                [stage_state_scale; stage_input_scale]; ...
                [term_speed_weight; term_pos_weight(:); term_heading_weight]; ...
                [terminal_state_scale; terminal_actuator_scale; terminal_forward_scale]; ...
                collision_clearance; ...
                term_pose_eps_vec; term_vel_max_vec; berth_corridor_vec; sync_limits_vec];

            % Initial guess (warm start)
            if ~isempty(obj.prev_sol)
                X_prev = reshape(obj.prev_sol(1:n_state*(N_h+1)), n_state, N_h+1);
                u_s = n_state*(N_h+1) + 1;
                U_prev = reshape(obj.prev_sol(u_s:u_s+n_ctrl*N_h-1), n_ctrl, N_h);
                s_s = n_state*(N_h+1) + n_ctrl*N_h + 1;
                S_prev = reshape(obj.prev_sol(s_s:s_s+n_obs*(N_h+1)-1), n_obs, N_h+1);
                t_s = s_s + n_obs*(N_h+1);
                T_prev = obj.prev_sol(t_s:t_s+n_term_slack-1);

                X_init = [X_prev(:,2:end), X_prev(:,end)];
                X_init(:,1) = x0;
                U_init = [U_prev(:,2:end), U_prev(:,end)];
                S_init = [S_prev(:,2:end), S_prev(:,end)];
                T_init = T_prev;

                x0_guess = [X_init(:); U_init(:); S_init(:); T_prev(:)];
            else
                X_init = repmat(x0, 1, N_h+1);
                U_init = u_ref;
                S_init = zeros(n_obs, N_h+1);
                T_init = zeros(n_term_slack, 1);

                for k = 1:N_h
                    xk = X_init(:,k);
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

                x0_guess = [X_init(:); U_init(:); S_init(:); T_init(:)];
            end

            if obj.enable_diagnostics && ~isempty(obj.g_func) && ~isempty(obj.constraint_names) && ...
                    ~isempty(obj.constraint_block_sizes)
                g0 = full(obj.g_func(X_init, U_init, S_init, T_init, p_val));
                block_starts = cumsum([1, obj.constraint_block_sizes(1:end-1)]);
                block_max_violation = -inf(numel(obj.constraint_names), 1);
                for idx = 1:numel(obj.constraint_names)
                    block_len = obj.constraint_block_sizes(idx);
                    if block_len <= 0
                        block_max_violation(idx) = 0;
                        continue;
                    end
                    start_idx = block_starts(idx);
                    stop_idx = start_idx + block_len - 1;
                    block_vals = g0(start_idx:stop_idx);
                    lb_block = obj.lbg_vec(start_idx:stop_idx);
                    ub_block = obj.ubg_vec(start_idx:stop_idx);
                    viol = max([max(lb_block - block_vals), max(block_vals - ub_block)]);
                    block_max_violation(idx) = max(0, full(viol));
                end
                [worst_violation, worst_idx] = max(block_max_violation);
                fprintf('  [NMPC DIAG] initial-guess worst block: %s (violation %.3g)\n', ...
                    obj.constraint_names{worst_idx}, worst_violation);
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

            t_off = s_off + n_obs*(N_h+1);
            t_idx = (t_off + 1):(t_off + n_term_slack);
            lbx_local(t_idx) = 0;
            ubx_local(t_idx) = term_slack_max;

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
                t_s = s_s + n_obs*(N_h+1);
                T_sol = sol_x(t_s:t_s+n_term_slack-1);

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
                info.n_halfplanes_real = n_hp_real;
                info.used_dual_warm_start = use_dual_warm_start;
                info.soft_obs_enabled = soft_enable;
                info.soft_obs_max_m = soft_obs_max_m;
                info.max_soft_slack_m = max(S_sol(:));
                info.sum_soft_slack_m = sum(S_sol(:));
                info.terminal_pose_enabled = term_enable;
                info.max_terminal_slack = max(T_sol(:));
                info.berth_corridor_enabled = corr_enable;
                info.max_azimuth_split_rad = max_az_split_local;
                info.max_stern_cmd_split_rpm = max_stern_split_local;
                info.collision_clearance_m = collision_clearance;
                obj.solve_ok = obj.solve_ok + 1;

            catch ME
                u_opt = [0; 0; x0(7); x0(8); x0(9)];
                X_pred = repmat(x0, 1, N_h+1);

                info.success = false;
                info.error = ME.message;
                info.n_obs_real = n_real;
                info.n_halfplanes_real = n_hp_real;
                info.used_dual_warm_start = false;
                info.soft_obs_enabled = soft_enable;
                info.soft_obs_max_m = soft_obs_max_m;
                info.max_soft_slack_m = nan;
                info.sum_soft_slack_m = nan;
                info.terminal_pose_enabled = term_enable;
                info.max_terminal_slack = nan;
                info.berth_corridor_enabled = corr_enable;
                info.max_azimuth_split_rad = max_az_split_local;
                info.max_stern_cmd_split_rpm = max_stern_split_local;
                info.collision_clearance_m = collision_clearance;
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
            D_stern_local = D_stern;
            D_bow_local = D_bow;
            t_stern = obj.t_ded;
            t_bow = 0.15;
            wp_stern = obj.wp;
            wp_bow = 0.05;
            bow_thrust_factor = 0.30;
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

            %% FIX #1: PORT STERN AZIPOD (n1) - CORRECT LOCAL VELOCITY KINEMATICS
            n1_rps = n1 / 60;
            u_local1 = u - r * stern_port_y;
            v_local1 = v + r * stern_port_x;              % FIX: was stern_port_y
            u_inflow1 = u_local1 * cos(alpha1) + v_local1 * sin(alpha1);  % FIX: use u_local1
            u_a1 = u_inflow1 * (1 - wp_stern);
            n1_abs = if_else(n1_rps >= 0, n1_rps, -n1_rps);
            n1_safe = if_else(n1_abs < 0.01, 0.01, n1_abs);
            J1 = u_a1 / (n1_safe * D_stern);
            J1 = if_else(J1 > 1.2, 1.2, if_else(J1 < -0.5, -0.5, J1));
            KT1_val = KT0 + KT1 * J1;
            KT1_val = if_else(KT1_val < 0.05, 0.05, KT1_val);
            T1_gross = rho_w * D_stern_local^4 * n1_rps * n1_abs * KT1_val;
            T1 = (1 - t_stern) * T1_gross;

            %% FIX #2: STARBOARD STERN AZIPOD (n2) - CORRECT LOCAL VELOCITY KINEMATICS
            n2_rps = n2 / 60;
            u_local2 = u - r * stern_starboard_y;        % FIX: was missing
            v_local2 = v + r * stern_starboard_x;        % FIX: was stern_starboard_y
            u_inflow2 = u_local2 * cos(alpha2) + v_local2 * sin(alpha2);  % FIX: use u_local2
            u_a2 = u_inflow2 * (1 - wp_stern);
            n2_abs = if_else(n2_rps >= 0, n2_rps, -n2_rps);
            n2_safe = if_else(n2_abs < 0.01, 0.01, n2_abs);
            J2 = u_a2 / (n2_safe * D_stern);
            J2 = if_else(J2 > 1.2, 1.2, if_else(J2 < -0.5, -0.5, J2));
            KT2_val = KT0 + KT1 * J2;
            KT2_val = if_else(KT2_val < 0.05, 0.05, KT2_val);
            T2_gross = rho_w * D_stern_local^4 * n2_rps * n2_abs * KT2_val;
            T2 = (1 - t_stern) * T2_gross;

            %% FIX #3: BOW TUNNEL THRUSTER (n3) - CORRECT YAW COUPLING & WAKE FRACTION
            n3_rps = n3 / 60;
            u_local3 = u - r * bow_y;                    % FIX: was missing
            v_local3 = v + r * bow_x;                    % FIX: was minus sign (v - r*bow_x)
            u_inflow3 = v_local3;  % Primarily lateral inflow for tunnel thruster
            u_a3 = u_inflow3 * (1 - wp_bow);             %   CORRECT: uses wp_bow not t_bow
            n3_abs = if_else(n3_rps >= 0, n3_rps, -n3_rps);
            n3_safe = if_else(n3_abs < 0.01, 0.01, n3_abs);
            J3 = u_a3 / (n3_safe * D_bow_local);
            J3 = if_else(J3 > 1.2, 1.2, if_else(J3 < -0.5, -0.5, J3));
            KT3_val = 0.527 * bow_thrust_factor + (-0.455 * bow_thrust_factor) * J3;
            KT3_val = if_else(KT3_val < 0.05 * bow_thrust_factor, 0.05 * bow_thrust_factor, KT3_val);
            speed_decay_factor = max(0.3, 1 - 0.08 * max(0.1, u));
            T3_gross = rho_w * D_bow_local^4 * n3_rps * n3_abs * KT3_val;
            T3 = (1 - t_bow) * T3_gross * speed_decay_factor;

            %% THRUST FORCES AND MOMENTS
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

function v = padVector(v_in, n, default_v)
    if nargin < 3 || isempty(default_v)
        default_v = zeros(n, 1);
    end
    default_v = default_v(:);
    if isempty(v_in)
        v = default_v;
        return;
    end
    if ismatrix(v_in) && size(v_in,1) == size(v_in,2) && size(v_in,1) > 1
        raw_v = diag(v_in);
    else
        raw_v = v_in(:);
    end
    v = default_v;
    m = min(n, numel(raw_v));
    v(1:m) = raw_v(1:m);
end
