classdef NMPC_Container < handle
    % NMPC_Container - NMPC for harbor navigation using container.m dynamics
    %
    % Based on:
    % - Your existing container.m model
    % - Martinsen et al. (2020) convex polytope constraints <hover-popup output_id="feb07809-4e6d-4e79-8dce-d5f4b07054a5" part="14.242" file_ids="5"></hover-popup> 
    % - CasADi for optimization
    %
    % Author: Riccardo Legnini
    % Date: 2026-02-23
    
    properties
        % Horizons
        N           % Prediction horizon
        dt          % Sample time [s]
        obstacle_cells
        
        % Cost weights
        Q           % State error weight (10x10 for full container.m state)
        R           % Control effort weight (2x2 for delta_c, n_c)
        Q_N         % Terminal cost
        
        % State/control dimensions
        nx = 10     % States: [u v r x y psi p phi delta n]
        nu = 2      % Controls: [delta_c n_c]
        
        % Constraints
        delta_max   % Max rudder angle [rad]
        n_max       % Max shaft speed [RPM]
        
        % Obstacle constraints (Slack variable formulation)
        r_safety = 5.0;          % Safety margin [m]
        penalty_slack = 100;     % Slack penalty (reduced for numerical stability)

        % CasADi solver
        opti        % Opti stack
        X           % State variables (10 x N+1)
        U           % Control variables (2 x N)
        slack
        
        % Parameters
        X0          % Initial state
        X_ref       % Reference trajectory
        A_obs       % Obstacle constraint matrix
        b_obs       % Obstacle constraint vector
    end
    
    methods

        function obj = NMPC_Container(config)
            % Constructor
            %
            % config struct with fields:
            %   N: prediction horizon
            %   dt: sample time
            %   Q: state weight matrix (10x10)
            %   R: control weight matrix (2x2)
            
            obj.N = config.N;
            obj.dt = config.dt;
            obj.Q = config.Q;
            obj.R = config.R;
            obj.Q_N = config.Q * 2;  % Terminal cost
            
            % Control limits (from container.m)
            obj.delta_max = 10 * pi/180;  % 10 deg
            obj.n_max = 160;              % 160 RPM
            
            fprintf('✓ NMPC_Container initialized (N=%d, dt=%.2f)\n', obj.N, obj.dt);
            fprintf('  → Optimizer will be built dynamically on first solve()\n');
        end

        
        % function buildOptimizer(obj)
        %     % Build CasADi Opti stack
        % 
        %     import casadi.*
        % 
        %     obj.opti = casadi.Opti();
        % 
        %     %% DECISION VARIABLES
        %     obj.X = obj.opti.variable(obj.nx, obj.N + 1);  % States
        %     obj.U = obj.opti.variable(obj.nu, obj.N);      % Controls
        %     obj.slack = obj.opti.variable();  % Will be resized dynamically based on obstacles
        % 
        %     %% PARAMETERS
        %     obj.X0 = obj.opti.parameter(obj.nx, 1);
        %     obj.X_ref = obj.opti.parameter(obj.nx, obj.N + 1);
        % 
        %     %% OBJECTIVE
        %     % Martinsen Eq. (10): min sum ||x-x_ref||^2_Q + ||u||^2_R
        %     J = 0;
        %     for k = 1:obj.N
        %         % State tracking
        %         e = obj.X(:, k) - obj.X_ref(:, k);
        %         J = J + e' * obj.Q * e;
        % 
        %         % Control effort
        %         J = J + obj.U(:, k)' * obj.R * obj.U(:, k);
        %     end
        % 
        %     % Terminal cost
        %     e_N = obj.X(:, obj.N+1) - obj.X_ref(:, obj.N+1);
        %     J = J + e_N' * obj.Q_N * e_N;
        % 
        %     obj.opti.minimize(J);
        % 
        %     %% DYNAMICS CONSTRAINTS
        %     for k = 1:obj.N
        %         % Call container.m to get state derivative
        %         x_k = obj.X(:, k);
        %         u_k = obj.U(:, k);
        % 
        %         % Get dynamics from container.m
        %         xdot = obj.container_dynamics_casadi(x_k, u_k);
        % 
        %         % Euler integration (can upgrade to RK4)
        %         x_next = x_k + xdot * obj.dt;
        % 
        %         obj.opti.subject_to(obj.X(:, k+1) == x_next);
        %     end
        % 
        %     %% INITIAL CONDITION
        %     obj.opti.subject_to(obj.X(:, 1) == obj.X0);
        % 
        %     %% CONTROL CONSTRAINTS
        %     for k = 1:obj.N
        %         % Rudder angle: -10 deg to +10 deg
        %         obj.opti.subject_to(-obj.delta_max <= obj.U(1, k) <= obj.delta_max);
        % 
        %         % Shaft speed: 0 to 160 RPM
        %         obj.opti.subject_to(0 <= obj.U(2, k) <= obj.n_max);
        %     end
        % 
        %     %% SOLVER OPTIONS
        %     opts = struct;
        %     opts.ipopt.print_level = 0;
        %     opts.print_time = 0;
        %     opts.ipopt.max_iter = 100;
        %     opts.ipopt.tol = 1e-4;
        %     opts.ipopt.warm_start_init_point = 'yes';
        % 
        %     obj.opti.solver('ipopt', opts);
        % 
        %     fprintf('✓ NMPC optimizer built (N=%d, dt=%.2f)\n', obj.N, obj.dt);
        % end
        
        function xdot = container_dynamics_casadi(obj, x, u)
            % CasADi-compatible version of container.m dynamics
            % WITH NUMERICAL SAFEGUARDS to prevent NaN
            
            import casadi.*
            
            % Extract states (same order as container.m)
            u_vel = x(1);  % surge velocity
            v     = x(2);  % sway velocity
            r     = x(3);  % yaw rate
            psi   = x(6);  % yaw angle
            p     = x(7);  % roll rate
            phi   = x(8);  % roll angle
            delta = x(9);  % rudder angle
            n     = x(10); % shaft speed [RPM]
            
            % Extract controls
            delta_c = u(1);  % commanded rudder [rad]
            n_c     = u(2);  % commanded shaft speed [RPM]
            
            % NUMERICAL SAFEGUARDS
            % Prevent division by zero in nondimensionalization
            L = 175;
            U = sqrt(u_vel^2 + v^2);  % Total speed
            
            % CRITICAL: Prevent U from being too small
            U_min = 0.5;  % Minimum speed for nondimensionalization
            U_safe = if_else(U < U_min, U_min, U);
            
            % CRITICAL: Prevent n from being too small
            n_min = 1.0;  % Minimum RPM
            n_safe = if_else(abs(n) < n_min, n_min, abs(n));
            
            % NONDIMENSIONALIZATION (with safe values)
            u_nd = u_vel / U_safe;
            v_nd = v / U_safe;
            r_nd = r * L / U_safe;
            p_nd = p * L / U_safe;
            delta_nd = delta;
            n_nd = n_safe / 60 * L / U_safe;
            n_c_nd = n_c / 60 * L / U_safe;
            
            % PARAMETERS (from container.m)
            m  = 0.00792;    mx     = 0.000238;   my = 0.007049;
            Ix = 0.0000176;  alphay = 0.05;       lx = 0.0313;
            ly = 0.0313;     Iz = 0.000456;       Jx = 0.0000034;
            Jz = 0.000419;   xG = 0;
            
            B     = 25.40;   dF = 8.00;    g     = 9.81;
            dA    = 9.00;    d  = 8.50;    nabla = 21222;
            KM    = 10.39;   KB = 4.6154;  AR    = 33.0376;
            Delta = 1.8219;  D  = 6.533;   GM    = 0.3/L;
            rho   = 1025;    t  = 0.175;
            
            W     = rho*g*nabla/(rho*L^2*U_safe^2/2);
            
            % Hydrodynamic coefficients
            Xuu = -0.0004226; Xvr = -0.00311; Xrr = 0.00020;
            Xphiphi = -0.00020; Xvv = -0.00386;
            
            Kv = 0.0003026; Kr = -0.000063; Kp = -0.0000075;
            Kphi = -0.000021; Kvvv = 0.002843; Krrr = -0.0000462;
            Kvvr = -0.000588; Kvrr = 0.0010565; Kvvphi = -0.0012012;
            Kvphiphi = -0.0000793; Krrphi = -0.000243; Krphiphi = 0.00003569;
            
            Yv = -0.0116; Yr = 0.00242; Yp = 0;
            Yphi = -0.000063; Yvvv = -0.109; Yrrr = 0.00177;
            Yvvr = 0.0214; Yvrr = -0.0405; Yvvphi = 0.04605;
            Yvphiphi = 0.00304; Yrrphi = 0.009325; Yrphiphi = -0.001368;
            
            Nv = -0.0038545; Nr = -0.00222; Np = 0.000213;
            Nphi = -0.0001424; Nvvv = 0.001492; Nrrr = -0.00229;
            Nvvr = -0.0424; Nvrr = 0.00156; Nvvphi = -0.019058;
            Nvphiphi = -0.0053766; Nrrphi = -0.0038592; Nrphiphi = 0.0024195;
            
            kk = 0.631; epsilon = 0.921; xR = -0.5;
            wp = 0.184; tau = 1.09; xp = -0.526;
            cpv = 0.0; cpr = 0.0; ga = 0.088;
            cRr = -0.156; cRrrr = -0.275; cRrrv = 1.96;
            cRX = 0.71; aH = 0.237; zR = 0.033;
            xH = -0.48;
            
            % MASS MATRIX
            m11 = m + mx;
            m22 = m + my;
            m32 = -my * ly;
            m42 = my * alphay;
            m33 = Ix + Jx;
            m44 = Iz + Jz;
            
            % ACTUATOR DYNAMICS
            % Rudder rate
            delta_dot = delta_c - delta;
            
            % Shaft dynamics (with safeguard)
            Tm = if_else(n_safe > 0.3, 5.65/n_safe, 18.83);
            n_dot = (1/Tm) * (n_c_nd - n_nd) * 60;
            
            % PROPELLER AND RUDDER FORCES
            vR = ga*v_nd + cRr*r_nd + cRrrr*r_nd^3 + cRrrv*r_nd^2*v_nd;
            uP = u_nd * ((1 - wp) + tau*((v_nd + xp*r_nd)^2 + cpv*v_nd + cpr*r_nd));
            
            % Advance ratio (with safeguard)
            J_denom = n_nd * D;
            uP_safe = if_else(abs(uP) < 0.01, 0.01, uP);  % Safeguard uP
            J = if_else(abs(J_denom) < 0.01, 0.5, uP_safe*U_safe/J_denom);
            
            KT = 0.527 - 0.455*J;
            
            % Safeguard sqrt argument
            sqrt_arg = 1 + 8*kk*KT/(pi*(J^2 + 0.01));  % Prevent J^2 = 0
            sqrt_arg_safe = if_else(sqrt_arg < 0.01, 0.01, sqrt_arg);
            uR = uP_safe*epsilon*sqrt(sqrt_arg_safe);
            
            % CRITICAL: Safeguard uR before division (add small regularization)
            uR_reg = uR + 0.001;  % Add small regularization to prevent zero division
            alphaR = delta_nd + atan(vR/uR_reg);
            FN = -((6.13*Delta)/(Delta + 2.25))*(AR/L^2)*(uR_reg^2 + vR^2)*sin(alphaR);
            T_thrust = 2*rho*D^4/(U_safe^2*L^2*rho)*KT*n_nd*abs(n_nd);
            
            % FORCES AND MOMENTS
            X = Xuu*u_nd^2 + (1-t)*T_thrust + Xvr*v_nd*r_nd + Xvv*v_nd^2 + ...
                Xrr*r_nd^2 + Xphiphi*phi^2 + cRX*FN*sin(delta_nd) + (m + my)*v_nd*r_nd;
            
            Y = Yv*v_nd + Yr*r_nd + Yp*p_nd + Yphi*phi + Yvvv*v_nd^3 + Yrrr*r_nd^3 + ...
                Yvvr*v_nd^2*r_nd + Yvrr*v_nd*r_nd^2 + Yvvphi*v_nd^2*phi + ...
                Yvphiphi*v_nd*phi^2 + Yrrphi*r_nd^2*phi + Yrphiphi*r_nd*phi^2 + ...
                (1 + aH)*FN*cos(delta_nd) - (m + mx)*u_nd*r_nd;
            
            K = Kv*v_nd + Kr*r_nd + Kp*p_nd + Kphi*phi + Kvvv*v_nd^3 + Krrr*r_nd^3 + ...
                Kvvr*v_nd^2*r_nd + Kvrr*v_nd*r_nd^2 + Kvvphi*v_nd^2*phi + ...
                Kvphiphi*v_nd*phi^2 + Krrphi*r_nd^2*phi + Krphiphi*r_nd*phi^2 - ...
                (1 + aH)*zR*FN*cos(delta_nd) + mx*lx*u_nd*r_nd - W*GM*phi;
            
            N = Nv*v_nd + Nr*r_nd + Np*p_nd + Nphi*phi + Nvvv*v_nd^3 + Nrrr*r_nd^3 + ...
                Nvvr*v_nd^2*r_nd + Nvrr*v_nd*r_nd^2 + Nvvphi*v_nd^2*phi + ...
                Nvphiphi*v_nd*phi^2 + Nrrphi*r_nd^2*phi + Nrphiphi*r_nd*phi^2 + ...
                (xR + aH*xH)*FN*cos(delta_nd);
            
            % STATE DERIVATIVES
            % Add regularization to prevent singular mass matrix
            detM = m22*m33*m44 - m32^2*m44 - m42^2*m33 + 1e-12;  % Regularization
            
            u_dot = X*(U_safe^2/L)/m11;
            v_dot = -((-m33*m44*Y + m32*m44*K + m42*m33*N)/detM)*(U_safe^2/L);
            r_dot = ((-m42*m33*Y + m32*m42*K + N*m22*m33 - N*m32^2)/detM)*(U_safe^2/L^2);
            x_dot = (cos(psi)*u_vel - sin(psi)*cos(phi)*v)*U_safe;
            y_dot = (sin(psi)*u_vel + cos(psi)*cos(phi)*v)*U_safe;
            psi_dot = cos(phi)*r*(U_safe/L);
            p_dot = ((-m32*m44*Y + K*m22*m44 - K*m42^2 + m32*m42*N)/detM)*(U_safe^2/L^2);
            phi_dot = p*(U_safe/L);
            
            % Pack derivatives
            xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot; p_dot; phi_dot; delta_dot; n_dot];
        end

        
        function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles)
            % Solve NMPC problem with dynamic obstacle constraints
            %
            % Inputs:
            %   x0: Current state [10x1]
            %   x_ref: Reference trajectory [10 x N+1]
            %   obstacles: Struct array with fields: position [2x1], radius (scalar)
            %
            % Outputs:
            %   u_opt: Optimal control [2x1] = [delta_c, n_c]
            %   X_pred: Predicted trajectory [10 x N+1]
            %   info: Solve info
            
            tic;
            
            % REBUILD optimizer with current obstacle count
            obj.buildOptimizerWithObstacles(obstacles);
            
            % Set parameters
            obj.opti.set_value(obj.X0, x0);
            obj.opti.set_value(obj.X_ref, x_ref);
            
            % Set initial guess from trajectory propagation
            try
                X_init = zeros(obj.nx, obj.N+1);
                X_init(:, 1) = x0;
                U_init = zeros(obj.nu, obj.N);
                
                for k = 1:obj.N
                    xdot = obj.container_dynamics_casadi(X_init(:, k), U_init(:, k));
                    X_init(:, k+1) = X_init(:, k) + xdot * obj.dt;
                end
                
                obj.opti.set_initial(obj.X, X_init);
                obj.opti.set_initial(obj.U, U_init);
                if ~isempty(obj.slack)
                    obj.opti.set_initial(obj.slack, zeros(size(obj.slack)));
                end
            catch
                % Continue anyway if initialization fails
            end
            
            % Solve
            try
                sol = obj.opti.solve();
                
                U_opt = sol.value(obj.U);
                X_pred = sol.value(obj.X);
                
                u_opt = U_opt(:, 1);
                
                info.success = true;
                info.cost = sol.value(obj.opti.f);
                
            catch ME
                warning(ME.identifier, 'NMPC failed: %s', ME.message);
                
                % Backup: zero rudder, maintain thrust
                u_opt = [0; 70];
                X_pred = repmat(x0, 1, obj.N+1);
                
                info.success = false;
                info.error = ME.message;
            end
            
            info.solve_time = toc;
        end

        function buildOptimizerWithObstacles(obj, obstacles)
            % Rebuild optimizer with obstacle-specific constraints
            % This avoids parameter dimension mismatch in CasADi
            
            import casadi.*
            
            % Clear previous optimization problem
            obj.opti = casadi.Opti();
            
            %% DECISION VARIABLES
            obj.X = obj.opti.variable(obj.nx, obj.N + 1);  % States
            obj.U = obj.opti.variable(obj.nu, obj.N);      % Controls
            
            % Slack variables (one per obstacle per time step)
            num_obstacles = length(obstacles);
            if num_obstacles > 0
                obj.slack = obj.opti.variable(num_obstacles, obj.N + 1);
                % Bounds on slack: [0, r_safety]
                obj.opti.subject_to(0 <= obj.slack(:) <= obj.r_safety);
            end
            
            %% PARAMETERS
            obj.X0 = obj.opti.parameter(obj.nx, 1);
            obj.X_ref = obj.opti.parameter(obj.nx, obj.N + 1);
            
            %% OBJECTIVE
            J = 0;
            for k = 1:obj.N
                % State tracking
                e = obj.X(:, k) - obj.X_ref(:, k);
                J = J + e' * obj.Q * e;
                
                % Control effort
                J = J + obj.U(:, k)' * obj.R * obj.U(:, k);
            end
            
            % Terminal cost
            e_N = obj.X(:, obj.N+1) - obj.X_ref(:, obj.N+1);
            J = J + e_N' * obj.Q_N * e_N;
            
            % Add slack penalty (if obstacles exist)
            if num_obstacles > 0
                J = J + obj.penalty_slack * sumsqr(obj.slack);
            end
            
            obj.opti.minimize(J);
            
            %% DYNAMICS CONSTRAINTS
            for k = 1:obj.N
                x_k = obj.X(:, k);
                u_k = obj.U(:, k);
                
                xdot = obj.container_dynamics_casadi(x_k, u_k);
                x_next = x_k + xdot * obj.dt;
                
                obj.opti.subject_to(obj.X(:, k+1) == x_next);
            end
            
            %% INITIAL CONDITION
            obj.opti.subject_to(obj.X(:, 1) == obj.X0);
            
            %% CONTROL CONSTRAINTS
            for k = 1:obj.N
                obj.opti.subject_to(-obj.delta_max <= obj.U(1, k) <= obj.delta_max);
                obj.opti.subject_to(0 <= obj.U(2, k) <= obj.n_max);
            end
            
            %% OBSTACLE AVOIDANCE CONSTRAINTS (CORRECTED - Squared Distance)
            if num_obstacles > 0
                for k = 1:obj.N+1
                    for j = 1:num_obstacles
                        % Extract obstacle properties
                        obs = obstacles(j);
                        
                        % Handle position vector dimensions
                        if size(obs.position, 1) == 1
                            obs_pos = obs.position';  % Convert row to column
                        else
                            obs_pos = obs.position;
                        end
                        
                        x_obs = obs_pos(1);
                        y_obs = obs_pos(2);
                        r_obs = obs.radius;
                        
                        % Vehicle position from state vector
                        x_veh = obj.X(4, k);
                        y_veh = obj.X(5, k);
                        
                        % Distance squared (avoids sqrt and NaN issues)
                        % Add small epsilon to prevent exact zero
                        dist_sq = (x_veh - x_obs)^2 + (y_veh - y_obs)^2 + 0.01;
                        
                        % Minimum allowed distance
                        min_dist = r_obs + obj.r_safety - obj.slack(j, k);
                        
                        % Constraint: distance^2 >= (r_obs + r_safety - slack)^2
                        obj.opti.subject_to(dist_sq >= (min_dist + 1e-2)^2);
                    end
                end
            end
            
            %% SOLVER OPTIONS
            opts = struct;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.max_iter = 500;
            opts.ipopt.tol = 1e-3;
            opts.ipopt.compl_inf_tol = 1e-2;
            opts.ipopt.constr_viol_tol = 1e-3;
            opts.ipopt.warm_start_init_point = 'no';
            opts.ipopt.mu_strategy = 'monotone';
            opts.ipopt.nlp_scaling_method = 'gradient-based';
            
            obj.opti.solver('ipopt', opts);
        end
    end
end
