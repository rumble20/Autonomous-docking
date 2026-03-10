%% run_nmpc_simple.m
% Minimal-viable NMPC harbor navigation — path following + static obstacles
%
% Architecture (simplified):
%   Simple waypoint steering  ->  NMPC_Container_Lite  ->  container() plant
%   PID heading fallback when NMPC fails.
%
% TEST A — Multi-waypoint path following (no obstacles)
% TEST B — Multi-waypoint + one static obstacle
%
% Dependencies:
%   container.m              - Nonlinear ship dynamics (MSS toolbox)
%   NMPC_Container_Lite.m   - Simplified CasADi NMPC solver
%   animateSimResult.m       - Post-simulation animation
%
% Author: Riccardo Legnini
% Date:   2025 (simplified MVP)

clear; close all; clc;
clear animateSimResult   
fprintf('  NMPC HARBOR NAVIGATION — MVP (Simple Guidance + NMPC)\n\n');

%% ===== Sanity check on container.m ======================================
% Test state: [u=7 m/s, v=0, r=0, x=0, y=0, psi=0] (6-DOF)
% Test input: [alpha1=0, alpha2=0, F1=0, F2=0] (azipod thrusters)
x_test = [7; 0; 0; 0; 0; 0];
u_test = [0; 0; 0; 0];
[xdot_test, ~] = container(x_test, u_test);
fprintf('  container.m check: u_dot=%.6f, psi_dot=%.6f (should be ~0)\n\n', ...
    xdot_test(1), xdot_test(6));

%% ===== NMPC configuration ===============================================
% For 6-DOF azipod model: [u, v, r, x, y, psi] + [alpha1, alpha2, F1, F2]
nmpc_cfg = struct();
nmpc_cfg.N  = 20;
nmpc_cfg.dt = 1.0;
% Q weights (6 states): [u, v, r, x, y, psi]
nmpc_cfg.Q  = diag([2.0, 0, 0.8, 0.03, 0.03, 15]);
% R weights (4 controls): [alpha1, alpha2, F1, F2]
nmpc_cfg.R  = diag([0.1, 0.1, 10, 10]);
nmpc_cfg.R_rate       = diag([0.05, 0.05, 5, 5]);
nmpc_cfg.max_obs      = 15;
nmpc_cfg.r_safety     = 40;
nmpc_cfg.penalty_slack = 15000;
% Set true only when debugging; otherwise it prints every solve() call.
nmpc_cfg.enable_diagnostics = false;

nmpc = NMPC_Container_Lite(nmpc_cfg);

%% ===== CasADi dynamics consistency check =================================
fprintf('--- CasADi consistency check ---\n');
import casadi.*
x_sym   = SX.sym('x', 6, 1);    % 6-DOF model
u_sym   = SX.sym('u', 4, 1);    % 4 azipod controls
xdot_sym = nmpc.containerCasADi(x_sym, u_sym);
f_check  = casadi.Function('f_check', {x_sym, u_sym}, {xdot_sym});

xdot_cas = full(f_check(x_test, u_test));
max_err  = max(abs(xdot_test - xdot_cas));

labels = {'u_dot','v_dot','r_dot','x_dot','y_dot','psi_dot'};
fprintf('  %-10s  %12s  %12s  %12s\n', 'State', 'container()', 'CasADi', '|error|');
for i = 1:6
    fprintf('  %-10s  %+12.6f  %+12.6f  %12.2e\n', ...
        labels{i}, xdot_test(i), xdot_cas(i), abs(xdot_test(i)-xdot_cas(i)));
end
if max_err < 1e-6
    fprintf('  >> PASS (max err = %.2e)\n\n', max_err);
elseif max_err < 1e-2
    fprintf('  >> WARN: small diffs (CasADi safeguards). OK.\n\n');
else
    fprintf('  >> FAIL: Large mismatch!\n\n');
end

%% ===== Build solver ONCE ================================================
fprintf('--- Building nlpsol ---\n');
nmpc.buildSolver();

%% ===== Ship image path (for animation) ==================================
shipImgPath = 'c:\Users\SERILEG\OneDrive - ABB\Autonomous-docking\useful pictures\vessel_top.png';

%% ===== Shared simulation parameters =====================================
T_final = 300;          % seconds
dt = nmpc_cfg.dt;
t  = 0:dt:T_final;

% Simple PID fallback gains (used when NMPC fails)
pid_Kp = 1;
pid_Td = 10;

% Explicit test toggles to avoid stale workspace confusion when running
% individual sections in MATLAB.
if ~exist('run_test_A', 'var'), run_test_A = true;  end
if ~exist('run_test_B', 'var'), run_test_B = true;  end

%% ===== Helsinki harbour map (optional, for plotting only) ================
map = [];
if exist('helsinki_harbour.mat', 'file')
    S = load('helsinki_harbour.mat');
    if isfield(S, 'map'), map = S.map; end
end

harbor_anim = [];
if ~isempty(map)
    harbor_anim = HarborAnimHelper(map);
end

%% ========================================================================
%  TEST A — Multi-waypoint path following (no obstacles)
%% ========================================================================
if run_test_A
fprintf('\n==============================================================\n');
fprintf('  TEST A: Multi-waypoint path following (NMPC)\n');
fprintf('==============================================================\n');

wp_A = [-5800, -2800;
         -5500, -2700;
         -5000, -2500;
         -4450, -2200];
wp_speed_A = [7; 7; 7; 5];

% Initial state (6-DOF model)
x0_heading = atan2(wp_A(2,2)-wp_A(1,2), wp_A(2,1)-wp_A(1,1));
x = [7; 0; 0; wp_A(1,1); wp_A(1,2); x0_heading];

wp_idx = 1;
R_accept = 35;   % Waypoint acceptance radius [m]

% Preallocate logging (6-DOF states + 4 azipod controls)
traj_A     = zeros(6, length(t)+1);
ctrl_A     = zeros(4, length(t));
solve_ok_A = false(1, length(t));
xte_A      = zeros(1, length(t));
fallback_A = false(1, length(t));
traj_A(:,1) = x;
steps_A = 0;

fprintf('  Waypoints: ');
for w = 1:size(wp_A,1), fprintf('(%.0f, %.0f) ', wp_A(w,1), wp_A(w,2)); end
fprintf('\n');

for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_A, wp_speed_A, wp_idx, R_accept);
    xte = computeXTE(x, wp_A, wp_idx);

    % ---- 2) Build reference trajectory for NMPC ------------------------
    x_ref = buildSimpleRef(x, chi_d, U_d, nmpc.N, dt, wp_A, wp_idx);

    % ---- 3) Solve NMPC (no obstacles) -----------------------------------
    [u_opt, ~, info] = nmpc.solve(x, x_ref, []);

    % ---- 4) PID fallback (for azipods) ----------------------------------------
    if ~info.success
        % Simple fallback: point azipods forward and maintain thrust
        psi_err = wrapToPi(x(6) - chi_d);
        
        % Thrust proportional to remaining distance and velocity error
        F_mag = 100 * (U_d - x(1));  % Proportional to velocity error
        F_mag = max(-300, min(300, F_mag));  % Clamp to ±300 kN
        
        % Azimuth angles: simple proportional to heading error
        alpha = -pid_Kp * psi_err;
        alpha = max(-pi/4, min(pi/4, alpha));  % Clamp to ±45°
        
        u_opt = [alpha; -alpha; F_mag; F_mag];  % Diverging azimuth for sway control
        fallback_A(i) = true;
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x_old = x;
    x = rk4Step(x, u_opt, dt);
    
    % Catch position explosion
    if abs(x(4)) > 50000 || abs(x(5)) > 50000 || any(isnan(x)) || any(isinf(x))
        fprintf('\n\n  ❌❌❌ RK4 EXPLOSION AT ITERATION %d\n', i);
        fprintf('     x_before = [u=%.2f, v=%.2f, r=%.4f, x=%.0f, y=%.0f]\n', ...
            x_old(1), x_old(2), x_old(3), x_old(4), x_old(5));
        fprintf('     u_opt = [α₁=%.4f, α₂=%.4f, F₁=%.1f, F₂=%.1f]\n', u_opt(1), u_opt(2), u_opt(3), u_opt(4));
        fprintf('     x_after = [u=%.3e, v=%.3e, r=%.3e, x=%.3e, y=%.3e]\n', ...
            x(1), x(2), x(3), x(4), x(5));
        fprintf('     Δpos = [%.1f, %.1f] (should be ~[%.1f, %.1f])\n', ...
            x(4)-x_old(4), x(5)-x_old(5), x_old(1)*cos(x_old(6))*dt, x_old(1)*sin(x_old(6))*dt);
        break;  % Stop simulation
    end

    % ---- 6) Logging ------------------------------------------------------
    steps_A = i;
    traj_A(:, i+1)  = x;
    ctrl_A(:, i)    = u_opt;
    solve_ok_A(i)   = info.success;
    xte_A(i)        = xte;

    % ---- 7) Progress -----------------------------------------------------
    if i == 1 || mod(i, 30) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) psi=%+5.1fdeg wp=%d xte=%+.1fm ok=%d dt=%.3fs\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), wp_idx, xte, info.success, info.solve_time);
    end

    % ---- 8) Done? --------------------------------------------------------
    if norm(x(4:5) - wp_A(end,:)') < R_accept
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

% Trim logs
traj_A     = traj_A(:, 1:steps_A+1);
ctrl_A     = ctrl_A(:, 1:steps_A);
solve_ok_A = solve_ok_A(1:steps_A);
xte_A      = xte_A(1:steps_A);
fallback_A = fallback_A(1:steps_A);
t_A        = (0:steps_A) * dt;

nA_ok = sum(solve_ok_A);  nA_tot = length(solve_ok_A);
fprintf('  Solver: %d/%d (%.0f%%), fallback=%d, mean XTE: %.1f m, max XTE: %.1f m\n', ...
    nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), sum(fallback_A), mean(abs(xte_A)), max(abs(xte_A)));

% ------ Test A: Plots -----------------------------------------------------
figure(1); clf;

subplot(3,1,1);
plotMapBackground(map);
hold on;
plot(traj_A(5,:), traj_A(4,:), 'b-', 'LineWidth', 2);
plot(wp_A(:,2), wp_A(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
plot(traj_A(5,1), traj_A(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('A: Path following'); grid on; axis equal;

subplot(3,1,2);
t_ctrl_A = (0:size(ctrl_A,2)-1)*dt;
% Plot azipod azimuth angles and thrust magnitudes
yyaxis left;  stairs(t_ctrl_A, rad2deg(ctrl_A(1:2,:))', 'b-'); ylabel('Azimuth angle [deg]');
yyaxis right; stairs(t_ctrl_A, ctrl_A(3:4,:)', 'r-');         ylabel('Thrust [kN]');
xlabel('Time [s]'); title('A: Azipod Controls [α₁ α₂ F₁ F₂]'); grid on;
legend('α₁', 'α₂', 'F₁', 'F₂', 'Location', 'best');

subplot(3,1,3);
t_xte_A = (0:length(xte_A)-1)*dt;
plot(t_xte_A, xte_A, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('A: Cross-track error'); grid on;

% ------ Test A: Animation ------------------------------------------------
cfg_anim_A = struct();
cfg_anim_A.figNo       = 10;
cfg_anim_A.testName    = 'A: Path Following';
cfg_anim_A.shipImgFile = shipImgPath;
cfg_anim_A.shipSize    = 0.08;
cfg_anim_A.maxFrames   = 150;
cfg_anim_A.pauseTime   = 0.05;

animateSimResult(traj_A, wp_A, t_A, harbor_anim, cfg_anim_A);
end


%% ========================================================================
%  TEST B — Multi-waypoint + one static obstacle
%% ========================================================================
if run_test_B
fprintf('\n==============================================================\n');
fprintf('  TEST B: Path following + static obstacle avoidance\n');
fprintf('==============================================================\n');

wp_B = [-3400, -1500;
         -2700, -1300;
         -2400, -1400;
         -1800, -1200];
wp_speed_B = [7; 7; 7; 5];

% One static obstacle on the path
obs_B(1).position = [-3000; -1400];
obs_B(1).radius   = 30;

x0_heading_B = atan2(wp_B(2,2)-wp_B(1,2), wp_B(2,1)-wp_B(1,1));
% 6-DOF model: [u=7, v=0, r=0, x, y, psi]
x = [7; 0; 0; wp_B(1,1); wp_B(1,2); x0_heading_B];

wp_idx = 1;
R_accept_B = 50;  % larger radius for smoother segment transitions

% Sanity check: print initial guidance command
fprintf('  >> Initial Setup Verification:\n');
fprintf('     Start position: (%.1f N, %.1f E)\n', x(4), x(5));
fprintf('     WP1->WP2: (%.1f,%.1f) -> (%.1f,%.1f)\n', ...
    wp_B(1,1), wp_B(1,2), wp_B(2,1), wp_B(2,2));
fprintf('     Delta: (%.1f N, %.1f E)\n', wp_B(2,1)-wp_B(1,1), wp_B(2,2)-wp_B(1,2));
fprintf('     Initial heading x0_heading_B = %.1f° (atan2(%.1f, %.1f))\n', ...
    rad2deg(x0_heading_B), wp_B(2,2)-wp_B(1,2), wp_B(2,1)-wp_B(1,1));

% Test guidance immediately
[chi_d_test, U_d_test, wp_idx_test] = simpleWaypointGuidance(x, wp_B, wp_speed_B, 1, R_accept_B);
fprintf('     First guidance call: chi_d=%.1f°, U_d=%.1f m/s, wp_idx=%d\n', ...
    rad2deg(chi_d_test), U_d_test, wp_idx_test);
fprintf('     Heading error: %.1f°\n\n', rad2deg(wrapToPi(x(6) - chi_d_test)));

% Reset warm-start between tests (A -> B) to avoid cross-test bias
nmpc.prev_sol = [];
nmpc.prev_u   = [];

% Preallocate (6-DOF states + 4 azipod controls)
traj_B     = zeros(6, length(t)+1);
ctrl_B     = zeros(4, length(t));
solve_ok_B = false(1, length(t));
xte_B      = zeros(1, length(t));
fallback_B = false(1, length(t));
traj_B(:,1) = x;
steps_B = 0;

fprintf('  Obstacle at (%.0f, %.0f) r=%.0f m\n', ...
    obs_B(1).position(1), obs_B(1).position(2), obs_B(1).radius);
fprintf('  Waypoints (North, East):\n');
for w = 1:size(wp_B,1)
    fprintf('    WP%d: (%7.1f, %7.1f)\n', w, wp_B(w,1), wp_B(w,2));
end
fprintf('  Initial pos: (%7.1f, %7.1f), heading: %+6.1f deg\n\n', ...
    x(4), x(5), rad2deg(x(6)));

wp_idx_prev = 1;  % Track waypoint changes
consecutive_fails = 0;
for i = 1:length(t)
    % ---- 1) Simple waypoint guidance ------------------------------------
    [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp_B, wp_speed_B, wp_idx, R_accept_B);
    xte = computeXTE(x, wp_B, wp_idx);

    % ---- 2) Build reference  --------------------------------------------
    x_ref = buildSimpleRef(x, chi_d, U_d, nmpc.N, dt, wp_B, wp_idx);
    
    % Debug: check reference trajectory sanity on first iteration and when wp index changes
    if i == 1 || (i > 1 && wp_idx ~= wp_idx_prev)
        fprintf('  >> Reference trajectory check at t=%.1f (wp=%d, chi_d=%.1f°):\n', t(i), wp_idx, rad2deg(chi_d));
        fprintf('     Current state: pos=(%.1f,%.1f) ψ=%.1f°\n', x(4), x(5), rad2deg(x(6)));
        for kk = 1:min(5, size(x_ref,2))
            fprintf('     k=%d: pos=(%.1f,%.1f) ψ_ref=%.1f° u_ref=%.1f\n', ...
                kk-1, x_ref(4,kk), x_ref(5,kk), rad2deg(x_ref(6,kk)), x_ref(1,kk));
        end
        fprintf('\n');
    end
    wp_idx_prev = wp_idx;

    % ---- 3) Solve NMPC (with obstacle) ----------------------------------
    [u_opt, X_pred, info] = nmpc.solve(x, x_ref, obs_B);
    
    % Debug: on waypoint changes, check what NMPC is commanding
    if i > 1 && wp_idx ~= wp_idx_prev
        fprintf('  >> NMPC solution at wp change (t=%.1f, wp %d->%d):\n', t(i), wp_idx_prev, wp_idx);
        fprintf('     Commanded: δ=%.1f°, n=%.1f RPM\n', rad2deg(u_opt(1)), u_opt(2));
        fprintf('     Current ψ=%.1f°, chi_d=%.1f°, error=%.1f°\n', ...
            rad2deg(x(6)), rad2deg(chi_d), rad2deg(wrapToPi(x(6)-chi_d)));
        fprintf('     X_pred headings (first 5): ');
        for kk = 1:min(5, size(X_pred,2))
            fprintf('%.1f° ', rad2deg(X_pred(6,kk)));
        end
        fprintf('\n\n');
    end

    % ---- 4) PID fallback (for azipods) ----------------------------------------
    if ~info.success
        % Simple fallback: point azipods forward and maintain thrust
        % This is a placeholder - in reality you'd want heading control too
        psi_err = wrapToPi(x(6) - chi_d);
        
        % Thrust proportional to remaining distance and velocity error
        F_mag = 100 * (U_d - x(1));  % Proportional to velocity error
        F_mag = max(-300, min(300, F_mag));  % Clamp to ±300 kN
        
        % Azimuth angles: simple proportional to heading error
        alpha = -pid_Kp * psi_err;
        alpha = max(-pi/4, min(pi/4, alpha));  % Clamp to ±45°
        
        u_opt = [alpha; -alpha; F_mag; F_mag];  % Diverging azimuth for sway control
        fallback_B(i) = true;
        consecutive_fails = consecutive_fails + 1;
        
        if consecutive_fails == 1
            fprintf('  !! SOLVER FAILED at t=%.1f: pos=(%7.1f,%7.1f) psi=%+6.1f° chi_d=%+6.1f° wp=%d\n', ...
                t(i), x(4), x(5), rad2deg(x(6)), rad2deg(chi_d), wp_idx);
        end
        if consecutive_fails >= 10
            fprintf('  !! WARNING: 10+ consecutive solver failures! Guidance may be infeasible.\n');
            fprintf('     Current: pos=(%7.1f,%7.1f) wp_idx=%d, active seg: WP%d->WP%d\n', ...
                x(4), x(5), wp_idx, wp_idx, min(wp_idx+1, size(wp_B,1)));
            consecutive_fails = 0;  % reset to avoid spamming
        end
    else
        consecutive_fails = 0;
    end

    % ---- 5) Simulate plant (RK4) ----------------------------------------
    x = rk4Step(x, u_opt, dt);
    
    % ---- 6) Collision checks (AFTER step) -------------------------------
    % 6a) Circular obstacle collision
    d_obs = norm(x(4:5) - obs_B(1).position);
    if d_obs < obs_B(1).radius
        fprintf('  >> CIRCULAR OBSTACLE COLLISION at t=%.1f s, d=%.1f m\n', t(i), d_obs);
        steps_B = i;
        traj_B(:, i+1) = x;
        ctrl_B(:, i)   = u_opt;
        solve_ok_B(i)  = info.success;
        xte_B(i)       = xte;
        break;
    end

    % 6b) Map polygon collision (island / shoreline)
    map_collision = false;
    if ~isempty(map) && isfield(map, 'polygons')
        xN = x(4);  % North
        yE = x(5);  % East
        for j = 1:length(map.polygons)
            if inpolygon(xN, yE, map.polygons(j).X, map.polygons(j).Y)
                map_collision = true;
                break;
            end
        end
    end

    if map_collision
        fprintf('  >> MAP COLLISION (inside polygon %d) at t=%.1f s, pos=(%.1f, %.1f)\n', ...
            j, t(i), x(4), x(5));
        steps_B = i;
        traj_B(:, i+1) = x;
        ctrl_B(:, i)   = u_opt;
        solve_ok_B(i)  = info.success;
        xte_B(i)       = xte;
        break;
    end

    % ---- 7) Logging ------------------------------------------------------
    steps_B = i;
    traj_B(:, i+1)  = x;
    ctrl_B(:, i)    = u_opt;
    solve_ok_B(i)   = info.success;
    xte_B(i)        = xte;

    % ---- 8) Progress -----------------------------------------------------
    if i == 1 || mod(i, 15) == 0
        fprintf('  [t=%5.1f] pos=(%7.1f,%6.1f) ψ=%+6.1f° χ_d=%+6.1f° err=%+5.1f° wp=%d ok=%d δ=%+5.1f° obs_d=%.0fm\n', ...
            t(i), x(4), x(5), rad2deg(x(6)), rad2deg(chi_d), rad2deg(wrapToPi(x(6)-chi_d)), ...
            wp_idx, info.success, rad2deg(u_opt(1)), d_obs);
    end

    % ---- 9) Done? --------------------------------------------------------
    if norm(x(4:5) - wp_B(end,:)') < R_accept_B
        fprintf('  >> FINAL WAYPOINT REACHED at t=%.1f s!\n', t(i));
        break;
    end
end

% Trim logs
traj_B     = traj_B(:, 1:steps_B+1);
ctrl_B     = ctrl_B(:, 1:steps_B);
solve_ok_B = solve_ok_B(1:steps_B);
xte_B      = xte_B(1:steps_B);
fallback_B = fallback_B(1:steps_B);
t_B        = (0:steps_B) * dt;

nB_ok = sum(solve_ok_B);  nB_tot = length(solve_ok_B);
fprintf('  Solver: %d/%d (%.0f%%), fallback=%d, mean XTE: %.1f m\n', ...
    nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), sum(fallback_B), mean(abs(xte_B)));

% ------ Test B: Plots -----------------------------------------------------
figure(2); clf;

subplot(3,1,1);
plotMapBackground(map);
hold on;
plot(traj_B(5,:), traj_B(4,:), 'b-', 'LineWidth', 2);
plot(wp_B(:,2), wp_B(:,1), 'r*-', 'MarkerSize', 12, 'LineWidth', 1);
theta_circ = linspace(0, 2*pi, 50);
fill(obs_B(1).position(2) + obs_B(1).radius*cos(theta_circ), ...
     obs_B(1).position(1) + obs_B(1).radius*sin(theta_circ), ...
     'r', 'FaceAlpha', 0.3);
plot(traj_B(5,1), traj_B(4,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('y [m]'); ylabel('x [m]'); title('B: Obstacle avoidance'); grid on; axis equal;

subplot(3,1,2);
t_ctrl_B = (0:size(ctrl_B,2)-1)*dt;
% Plot azipod azimuth angles and thrust magnitudes
yyaxis left;  stairs(t_ctrl_B, rad2deg(ctrl_B(1:2,:))', 'b-'); ylabel('Azimuth angle [deg]');
yyaxis right; stairs(t_ctrl_B, ctrl_B(3:4,:)', 'r-');         ylabel('Thrust [kN]');
xlabel('Time [s]'); title('B: Azipod Controls [α₁ α₂ F₁ F₂]'); grid on;
legend('α₁', 'α₂', 'F₁', 'F₂', 'Location', 'best');

subplot(3,1,3);
t_xte_B = (0:length(xte_B)-1)*dt;
plot(t_xte_B, xte_B, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time [s]'); ylabel('XTE [m]'); title('B: Cross-track error'); grid on;

% ------ Test B: Animation ------------------------------------------------
cfg_anim_B = struct();
cfg_anim_B.figNo       = 11;
cfg_anim_B.testName    = 'B: Obstacle Avoidance';
cfg_anim_B.shipImgFile = shipImgPath;
cfg_anim_B.shipSize    = 0.08;
cfg_anim_B.maxFrames   = 150;
cfg_anim_B.pauseTime   = 0.05;
cfg_anim_B.circObs(1).position = obs_B(1).position;
cfg_anim_B.circObs(1).radius   = obs_B(1).radius;

animateSimResult(traj_B, wp_B, t_B, harbor_anim, cfg_anim_B);
end


%% ===== Summary ==========================================================
fprintf('\n=== SUMMARY ===\n');
if run_test_A && exist('nA_ok', 'var') && exist('nA_tot', 'var')
    fprintf('  Test A — %d/%d solves (%.0f%%), mean XTE: %.1f m, max XTE: %.1f m\n', ...
        nA_ok, nA_tot, 100*nA_ok/max(nA_tot,1), mean(abs(xte_A)), max(abs(xte_A)));
else
    fprintf('  Test A — skipped\n');
end

if run_test_B && exist('nB_ok', 'var') && exist('nB_tot', 'var')
    fprintf('  Test B — %d/%d solves (%.0f%%), fallback=%d, mean XTE: %.1f m\n', ...
        nB_ok, nB_tot, 100*nB_ok/max(nB_tot,1), sum(fallback_B), mean(abs(xte_B)));
else
    fprintf('  Test B — skipped\n');
end
fprintf('  Total solver: OK=%d, Fail=%d\n', nmpc.solve_ok, nmpc.solve_fail);
fprintf('\nDone. Check figures.\n');


%% ========================================================================
%  LOCAL FUNCTIONS
%% ========================================================================

function [chi_d, U_d, wp_idx] = simpleWaypointGuidance(x, wp, wp_speed, wp_idx, R_accept)
% Simple waypoint steering: heading to next waypoint, switch when close.
%
%   x:         state vector [u v r x y psi ...]
%   wp:        Nx2 waypoints [x, y]
%   wp_speed:  Nx1 speed at each wp [m/s]
%   wp_idx:    current segment index (1-based)
%   R_accept:  acceptance radius [m]
%
% Returns:
%   chi_d:  desired heading [rad]
%   U_d:    desired speed [m/s]
%   wp_idx: (possibly updated) segment index

    n_wps = size(wp, 1);
    pos   = [x(4); x(5)];

    % Clamp index to the active segment [wp_idx -> wp_idx+1]
    wp_idx = min(max(1, wp_idx), max(1, n_wps - 1));

    % Robust segment switching: geometric progress only.
    % Advance when the vessel has projected past segment end, or is within
    % acceptance radius of the segment-end waypoint.
    while wp_idx < n_wps - 1
        p_from = wp(wp_idx, :)';
        p_to   = wp(wp_idx + 1, :)';
        seg    = p_to - p_from;
        seg_l2 = seg' * seg;
        seg_len = sqrt(seg_l2);

        if seg_l2 < 1e-9
            wp_idx = wp_idx + 1;
            continue;
        end

        proj = dot(pos - p_from, seg) / seg_l2;  % unbounded [0..1] on segment
        d_to_waypoint = norm(pos - p_to);
        xte_seg = abs(((pos(1)-p_from(1))*seg(2) - (pos(2)-p_from(2))*seg(1)) / max(seg_len, 1e-6));

        % Only accept projection-based switching if we are still reasonably
        % close to the active segment. This prevents skipping many waypoints
        % when the vessel is far off-track.
        near_segment = xte_seg <= max(2*R_accept, 60);

        if d_to_waypoint <= R_accept || (proj >= 1.0 && near_segment)
            wp_idx = wp_idx + 1;
        else
            break;
        end
    end

    % Active segment for guidance target
    p1 = wp(wp_idx, :)';
    p2 = wp(min(wp_idx + 1, n_wps), :)';
    seg = p2 - p1;
    seg_len = norm(seg);

    if seg_len < 1e-6
        target = p2;
    else
        % Follow-the-carrot target: projection + lookahead on active segment.
        s_proj = dot(pos - p1, seg) / seg_len;
        s_proj = max(0, min(seg_len, s_proj));

        U_now = max(1.0, sqrt(x(1)^2 + x(2)^2));
        lookahead = max(30, 4 * U_now);
        s_target = min(seg_len, s_proj + lookahead);

        target = p1 + (s_target / seg_len) * seg;
    end

    dp = target - pos;
    chi_d = atan2(dp(2), dp(1));

    % Use speed target from the next waypoint in active segment.
    U_d = wp_speed(min(wp_idx + 1, n_wps));

    % Slow down when approaching final waypoint
    d_final = norm(pos - wp(end,:)');
    if d_final < 200
        U_d = max(2, U_d * d_final / 200);
    end
end

function xte = computeXTE(x, wp, wp_idx)
% Cross-track error: signed distance from vessel to current path segment.
    n_wps = size(wp, 1);
    idx_from = max(1, wp_idx);
    idx_to   = min(idx_from + 1, n_wps);

    p1 = wp(idx_from, :)';
    p2 = wp(idx_to,   :)';
    pos = [x(4); x(5)];

    seg = p2 - p1;
    seg_len = norm(seg);
    if seg_len < 1
        xte = norm(pos - p1);
        return;
    end

    % Cross product gives signed distance
    xte = ((pos(1)-p1(1))*seg(2) - (pos(2)-p1(2))*seg(1)) / seg_len;
end

function x_ref = buildSimpleRef(x0, chi_d, U_d, N, dt, wp, wp_idx) %#ok<*INUSD>
% Build reference following waypoint path segments (6-DOF model)
%
%   x_ref: 6 × (N+1) matrix  [u, v, r, x, y, psi]
%   Tracked states: u (speed), r (yaw rate), x/y (position), psi (heading)

    x_ref = zeros(6, N+1);
    x_ref(:, 1) = x0;  % ✅ Explicitly set initial condition
    
    % Heading error -> yaw rate reference
    psi_err = atan2(sin(chi_d - x0(6)), cos(chi_d - x0(6)));
    r_d = 0.5 * psi_err;
    r_d = max(-0.15, min(0.15, r_d));  % Clamp to ±0.15 rad/s
    
    for k = 2:(N+1)  % Start from k=2 since k=1 is x0
        x_ref(1, k) = U_d;      % Speed
        x_ref(2, k) = 0;        % Sway
        x_ref(3, k) = r_d;      % Yaw rate
        x_ref(6, k) = chi_d;    % Heading
        
        % ✅ Position propagation (check coordinate convention!)
        % Assuming x=North, y=East based on your waypoint format
        dx = U_d * dt * cos(chi_d);  % North increment
        dy = U_d * dt * sin(chi_d);  % East increment
        x_ref(4, k) = x_ref(4, k-1) + dx;
        x_ref(5, k) = x_ref(5, k-1) + dy;
    end
    
    % ✅ Sanity check
    if any(isnan(x_ref(:))) || any(isinf(x_ref(:)))
        error('[buildSimpleRef] NaN/Inf detected in reference trajectory!');
    end
end


function x_next = rk4Step(x, u_ctrl, dt_s)
% RK4 integration of container.m for one time step (6-DOF azipod model).
    x(1)  = max(x(1), 0.1);  % Maintain minimum speed to avoid singularities
    [k1, ~] = container(x, u_ctrl);
    
    % ✅ CRITICAL: Enforce u >= 0.1 at all RK4 stages (avoid negative surge velocity)
    x2 = x + k1*dt_s/2;
    x2(1) = max(x2(1), 0.1);  % Clamp surge velocity
    [k2, ~] = container(x2, u_ctrl);
    
    x3 = x + k2*dt_s/2;
    x3(1) = max(x3(1), 0.1);  % Clamp surge velocity
    [k3, ~] = container(x3, u_ctrl);
    
    x4 = x + k3*dt_s;
    x4(1) = max(x4(1), 0.1);  % Clamp surge velocity
    [k4, ~] = container(x4, u_ctrl);
    
    x_next  = x + dt_s/6 * (k1 + 2*k2 + 2*k3 + k4);
    
    % ✅ Final safeguard
    x_next(1) = max(x_next(1), 0.1);
end

function angle = wrapToPi(angle)
% Wrap angle to [-pi, pi].
    angle = mod(angle + pi, 2*pi) - pi;
end

function plotMapBackground(map)
% Plot harbour map polygons (if loaded).
    if isempty(map), return; end
    hold on;
    if isfield(map, 'polygons')
        for kk = 1:length(map.polygons)
            patch(map.polygons(kk).Y, map.polygons(kk).X, 'k', ...
                'FaceColor', [0.9 0.2 0.2], 'FaceAlpha', 0.1, ...
                'EdgeColor', 'r', 'LineWidth', 1.5);
        end
    end
    if isfield(map, 'mapPoly')
        for kk = 1:length(map.mapPoly)
            patch(map.mapPoly(kk).Y, map.mapPoly(kk).X, 'c', ...
                'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineWidth', 2);
        end
    end
end
