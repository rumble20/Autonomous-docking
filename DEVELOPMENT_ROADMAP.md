# NMPC Harbor Navigation — Development Roadmap

## Overview
This document outlines a phased development plan to evolve the current MVP (path following + static obstacles) into a production-capable autonomous docking system with dynamic collision avoidance, emergency fallbacks, and advanced actuation.

**Current Status (MVP):**
- ✓ NMPC with basic obstacle avoidance
- ✓ Simple waypoint guidance
- ✓ RK4 plant integration
- ✓ Static circular obstacles
- Passes Test A & B (path following + one static obstacle)

**End Goal:**
- Full autonomous harbor navigation with dynamic vessel avoidance
- Warm-start path planning (RRTx)
- Azimuth thruster control
- SB-MPC COLAV for dynamic obstacles (with velocity estimates)
- Multi-level fallback hierarchy
- Real-time safety constraints

---

## Recommended Development Phases

### **PHASE 0: Validate MVP (1–2 weeks)**
**Goal:** Confirm Tests A & B pass, measure solver performance, identify bottlenecks.

**Deliverables:**
- Run `run_nmpc_simple.m` on hardware/simulator
- Measure:
  - Solve time per step (target: < 0.5 s for dt=1.0 s)
  - Constraint satisfaction (obstacle clearance margin)
  - Path tracking accuracy (XTE mean/max)
  - Rudder/RPM smoothness
- Document any solver failures or numerical issues
- Verify `container.m` dynamics match expected behavior

**Success Criteria:**
- 95%+ NMPC success rate (Tests A & B)
- Solve time < 0.5 s consistently
- Zero collisions with static obstacles
- Smooth control outputs (no excessive rudder chatter)

**Files:**
- `run_nmpc_simple.m` (existing, no changes)
- New: `val_test_logging.m` (structured output for analysis)

---

### **PHASE 1: Dynamic Obstacle Avoidance (Weeks 2–4)**
**Goal:** Handle moving vessels with known state (position, heading, speed).

**Why this before warm-start?**
- Adds core capability (vessel collision avoidance) needed for harbor scenarios
- Solver already handles obstacles; just add motion prediction
- Warm-start helps speed, but not essential for correctness yet

**Approach:**

#### 1A — Obstacle Prediction
Create a simple module to predict obstacle trajectories:

```matlab
class DynamicObstaclePredictor
  properties
    T_pred            % Prediction horizon [s]
    dt_pred           % Discretization [s]
  end
  
  methods
    function obs_future = predictPositions(obj, obs, t_ahead)
      % For each obstacle, compute positions at t_ahead
      % Assumes constant velocity (constant heading + speed)
      % obs(i) = struct with .position, .velocity, .radius
      
      for k = 1:length(obs)
        psi = atan2(obs(k).velocity(2), obs(k).velocity(1));
        speed = norm(obs(k).velocity);
        time_steps = t_ahead / obj.dt_pred;
        
        obs_future(k).trajectory = [obs(k).position];
        for n = 1:time_steps
          obs_future(k).trajectory = [obs_future(k).trajectory, ...
                obs(k).position + speed*n*obj.dt_pred*...
                [cos(psi); sin(psi)]];
        end
      end
    end
  end
end
```

#### 1B — Modify NMPC to Handle Dynamic Obstacles
Update `NMPC_Container_Lite`:

**Before:**
```matlab
% Constraint for one obstacle at one time step
dx = X(4,k) - P_obs_pos(1,j);
dy = X(5,k) - P_obs_pos(2,j);
dist_sq = dx^2 + dy^2;
g = vertcat(g, dist_sq - (P_obs_rad(j) + r_safety)^2);
```

**After (trajectory-based):**
```matlab
% Constraint: vessel stays clear of obstacle trajectory
for k_pred = 1:N_h         % for each NMPC step
  for k_obs = 1:n_obs_traj  % for each obstacle trajectory point
    % Use interpolated obstacle position along its trajectory
    dt_idx = floor(k_pred * dt / dt_obs_pred) + 1;
    obs_pos_pred = obs_trajectory_at(P_obs_traj, j, dt_idx);
    
    dx = X(4,k_pred) - obs_pos_pred(1);
    dy = X(5,k_pred) - obs_pos_pred(2);
    dist_sq = dx^2 + dy^2;
    g = vertcat(g, dist_sq - (P_obs_rad(j) + r_safety)^2);
  end
end
```

**Note:** This increases constraint count; may need to:
- Sparsify obstacles (keep only closest N per horizon)
- Use coarser trajectory discretization (e.g., every 5 s instead of 1 s)

#### 1C — Interface Update
Modify `nmpc.solve()` to accept dynamic obstacles:

```matlab
function [u_opt, X_pred, info] = solve(obj, x0, x_ref, obstacles, obstacles_dynamic)
  % obstacles:         struct array with .position, .radius, .velocity
  % obstacles_dynamic: (optional) pre-computed trajectories
  
  if nargin < 5 || isempty(obstacles_dynamic)
    % Compute trajectories on-the-fly
    obs_pred = DynamicObstaclePredictor(T_horizon, obj.dt);
    obstacles_dynamic = obs_pred.predictPositions(obstacles, obj.N * obj.dt);
  end
  
  % ... build NLP parameters with dynamic obstacle data ...
end
```

#### 1D — Test
Create `Test_C_dynamic_obstacle.m`:
- One stationary vessel (baseline)
- One vessel moving at constant velocity (crossing scenario)
- One vessel moving parallel (overtaking)
- Verify NMPC predicts their paths and steers clear

**Success Criteria:**
- Dynamic obstacles avoided with same safety margin as static
- Solve time ≤ 1.0 s (acceptable increase)
- No false positives (overly conservative steering)

**Files to create/modify:**
- New: `DynamicObstaclePredictor.m`
- Modify: `NMPC_Container_Lite.m` (add dynamic obstacle constraints, parameters)
- New: `test_dynamic_obstacles.m`

---

### **PHASE 2: RRTx Warm-Start Path Planning (Weeks 4–6)**
**Goal:** Use RRTx pre-computed path to initialize NMPC, reducing solve time.

**Why now?**
- Dynamic obstacles working ✓
- NMPC core validated ✓
- RRTx path provides good initialization → faster convergence

**Approach:**

#### 2A — Interface with Your RRTx Algorithm
Assume RRTx outputs:
```matlab
rrtx_path = struct();
rrtx_path.waypoints     % Nx2 [x, y] matrix (collision-free path)
rrtx_path.costs         % Nx1 cost-to-go from each WP to goal
rrtx_path.tree          % Graph structure (for replanning on obs changes)
rrtx_path.update_time   % How long RRTx took to compute [s]
```

#### 2B — Warm-Start Strategy
Instead of cold-start (propagate with `container.m`), use RRTx:

```matlab
function x0_guess = warmStartFromRRTx(obj, x_curr, rrtx_path, N_horiz, dt)
  % 1. Find nearest WP on RRTx path to current position
  nearest_idx = findNearestWaypoint(x_curr(4:5), rrtx_path.waypoints);
  
  % 2. Extract next N waypoints along path
  wp_subset = rrtx_path.waypoints(nearest_idx:min(nearest_idx+N_horiz, end), :);
  
  % 3. Build reference trajectory along RRTx path
  x_ref_rrtx = buildRefAlongPath(x_curr, wp_subset, N_horiz, dt);
  
  % 4. Propagate vehicle state along this reference (cheaper than obstacle-aware propagation)
  X_init = x_ref_rrtx;  % use reference as state guess
  U_init = zeros(2, N_horiz);
  
  % 5. Generate control guesses (simple: zero rudder, constant RPM)
  U_init(1,:) = 0;
  U_init(2,:) = x_curr(10);
  
  x0_guess = [X_init(:); U_init(:); zeros(n_obs, N_horiz+1)];
end
```

#### 2C — Modify NMPC Solve Loop
```matlab
% Old (cold start)
X_init = container_propagate(x0);

% New (with RRTx warm start)
if exist('rrtx_path', 'var') && ~isempty(rrtx_path)
  x0_guess = warmStartFromRRTx(nmpc, x, rrtx_path, nmpc.N, dt);
else
  x0_guess = coldStartWithContainer(x);
end

sol = nmpc.solver('x0', x0_guess, ..., 'p', p_val);
```

#### 2D — Replanning Trigger
When to recompute RRTx?
- On new goal/task
- When NMPC detects significant deviation from RRTx path
- Periodically (e.g., every 30 s)
- On major obstacle change detected

```matlab
function replan_needed = checkReplanTrigger(x, rrtx_path, nmpc_deviation_threshold)
  % Distance to nearest RRTx waypoint
  d_to_path = min(sqrt(sum((x(4:5)' - rrtx_path.waypoints).^2, 2)));
  replan_needed = (d_to_path > nmpc_deviation_threshold);
end
```

#### 2E — Test
Create `test_rrtx_warmstart.m`:
- Same scenario (Test A)
- Measure solve time: cold vs warm start
- Expected: 30–50% faster convergence

**Success Criteria:**
- Warm-start solve time 30–50% faster than cold start
- Same final cost/solution quality
- Replanning triggers work correctly

**Files to create/modify:**
- New: `RRTxInterface.m` (wrapper for your RRTx algorithm)
- Modify: `NMPC_Container_Lite.m` (accept warm-start guess)
- Modify: `run_nmpc_simple.m` (integrate RRTx calls, replanning checks)
- New: `test_rrtx_warmstart.m`

---

### **PHASE 3: Azimuth Thruster Actuation (Weeks 6–8)**
**Goal:** Replace fixed-heading propeller with independent azimuth thrusters.

**Why after warm-start?**
- Core NMPC validated with conventional rudder/propeller
- Extending to azimuth is straightforward (more DOF, richer control)
- Warm-start helps because more-complex dynamics → slower solver

**Current Model:**
- 1 fixed propeller (RPM control): thrust always in ship direction
- 1 rudder (angle control): side force, rotational moment

**Azimuth Model:**
- 2–4 azimuth thrusters: independent thrust magnitude + azimuth angle per thruster
- Example: 2 thrusters (fore + aft)
  ```
  Control: [u_fwd (RPM), azi_fwd (yaw), u_aft (RPM), azi_aft (yaw)]
           or [F_fwd (N), theta_fwd (rad), F_aft (N), theta_aft (rad)]
  ```

#### 3A — Container Dynamics Extension
Modify `container.m` or create `container_azimuth.m`:

```matlab
function [xdot, U] = container_azimuth(x, u_az)
% x = [u v r x y psi p phi ...]  (same 10 states)
% u_az = [RPM_fwd, azi_fwd, RPM_aft, azi_aft]
%        or [F_fwd, theta_fwd, F_aft, theta_aft]

% 1. Convert azimuth commands to body-fixed forces
%    Each thruster applies force F at angle azi relative to ship
F_fwd_x = F_fwd * cos(azi_fwd);
F_fwd_y = F_fwd * sin(azi_fwd);
F_aft_x  = F_aft  * cos(azi_aft);
F_aft_y  = F_aft  * sin(azi_aft);

% Total X, Y, N (moment)
X_thrust = F_fwd_x + F_aft_x;
Y_thrust = F_fwd_y + F_aft_y;
N_thrust = (F_fwd_y * x_fwd) + (F_aft_y * x_aft);  % lever arms

% 2. Combine with hydrodynamic forces (as before)
% 3. Return xdot with thrust contributions
end
```

#### 3B — NMPC_Container_Lite → NMPC_Container_Azimuth (or extend Lite)
- Increase control dimension: 2 → 4 (or 2 → 4 if force-based)
- Increase NLP variables accordingly
- Update control bounds and dynamics call

```matlab
classdef NMPC_Container_Azimuth < NMPC_Container_Lite
  properties
    n_thrusters = 2     % Number of azimuth thrusters
    thruster_positions  % Longitudinal x-positions for moment calc
  end
  
  methods
    function buildSolver(obj)
      % Similar to NMPC_Container_Lite, but:
      % nu = 2 * n_thrusters  (instead of 2)
      % U dimensions: [F1, azi1, F2, azi2, ...]
      % Dynamics call: xdot_k = obj.containerAzimuthCasADi(X(:,k), U(:,k))
    end
    
    function xdot = containerAzimuthCasADi(obj, x, u_az)
      % CasADi version of container_azimuth.m
      import casadi.*
      
      % Extract thruster commands
      n_u = length(u_az);
      F = u_az(1:2:n_u);
      azi = u_az(2:2:n_u);
      
      % Convert to body forces
      F_x = F .* cos(azi);
      F_y = F .* sin(azi);
      
      % Propulsive forces (summed)
      F_total_x = sum(F_x);
      F_total_y = sum(F_y);
      N_total = ... % moment calculation
      
      % Combine with hydrodynamic model
      % (similar to containerCasADi)
    end
  end
end
```

#### 3C — Test & Validation
Create `test_azimuth_actuation.m`:
- Compare conventional vs azimuth on same maneuver
- Verify faster yaw response with azimuth
- Check solver time increase (expect +20–40% due to more controls)

**Success Criteria:**
- Both conventional and azimuth models produce stable solutions
- Azimuth can execute tighter turns (smaller turning radius)
- No solver instability from expanded control space

**Files to create/modify:**
- New: `container_azimuth.m` (dynamics with azimuth)
- New: `NMPC_Container_Azimuth.m` (extended NMPC)
- New: `test_azimuth_actuation.m`

---

### **PHASE 4: SB-MPC COLAV (COLREGs-Aware Collision Avoidance) (Weeks 8–11)**
**Goal:** Integrate your existing SB-MPC COLAV with dynamic obstacle estimates.

**Why now?**
- Dynamic obstacles working ✓
- NMPC core + azimuth validated ✓
- SB-MPC COLAV is sophisticated; best tested with solid foundation

**Current SB-MPC COLAV Status:**
- 300+ lines, 24 tuning parameters
- Evaluates discrete course/speed scenarios
- Computes collision risk, COLREGs metrics, maneuvering cost
- Returns modified course/speed references

**Integration Points:**

#### 4A — Architecture
```
Input:  x (own ship state), x_ts (target ships), chi_d (LOS heading), U_d (LOS speed)
        
Step 1: SB-MPC COLAV evaluates scenarios
        → outputs chi_m (modified course), U_m (scaling factor)
        
Step 2: NMPC builds reference from (chi_m, U_m)
        → x_ref = buildRef(chi_m, U_m)
        
Step 3: NMPC solves with dynamic obstacles (target ships as obstacles)
        → u_opt
        
Step 4: Execute + animate
```

#### 4B — Target Ship State Management
```matlab
function x_ts = updateTargetShips(x_ts_old, dt)
  % Propagate target ships at constant heading/speed
  for i = 1:length(x_ts_old)
    psi = x_ts_old(i,6);
    u_ts = x_ts_old(i,1);
    v_ts = x_ts_old(i,2);
    
    x_ts(i,4) = x_ts_old(i,4) + (cos(psi)*u_ts - sin(psi)*v_ts)*dt;
    x_ts(i,5) = x_ts_old(i,5) + (sin(psi)*u_ts + cos(psi)*v_ts)*dt;
    x_ts(i,6) = x_ts_old(i,6);  % constant heading
    x_ts(i, 1:5) = x_ts_old(i, 1:5);  % constant velocity
  end
end
```

#### 4C — Modify Main Loop
```matlab
% Old
[chi_d, U_d] = los.computeRef(...);
x_ref = buildSimpleRef(x, chi_d, U_d, nmpc.N, dt);
[u_opt, ~, info] = nmpc.solve(x, x_ref, []);

% New
x_ts = updateTargetShips(x_ts, dt);  % constant velocity estimate
[chi_m, U_m, ~, ~] = colav.run(x, chi_d, U_d, chi_m_last, U_m_last, x_ts);
x_ref = buildSimpleRef(x, chi_m, U_m*U_d, nmpc.N, dt);

% Convert target ships to obstacles for NMPC
obs_ts = [];
for i = 1:length(x_ts)
  obs_ts(i).position = x_ts(i,4:5)';
  obs_ts(i).radius = 30;  % typical ship size
  obs_ts(i).velocity = [x_ts(i,1); x_ts(i,2)];
end

[u_opt, ~, info] = nmpc.solve(x, x_ref, obs_ts);
```

#### 4D — Test Scenarios
Create `test_sb_mpc_colav.m`:
- **Scenario 1:** Crossing encounter (target from port/starboard)
- **Scenario 2:** Overtaking (target ahead, same direction)
- **Scenario 3:** Head-on (target approaching head-on)
- **Scenario 4:** Multiple vessels

Verify:
- Safe distances maintained
- COLREGs-compliant maneuvers
- Smooth transitions (no jittery course changes)

**Success Criteria:**
- All crossing/overtaking scenarios passed
- Minimum distance to target > safety threshold
- No COLAV oscillation (course thrashing)

**Files to create/modify:**
- Modify: `run_nmpc_simple.m` or create `run_nmpc_sbmpc.m` (integrate SB-MPC COLAV loop)
- New: `test_sb_mpc_colav.m`
- Possibly: Tune SB-MPC COLAV parameters for your harbor scale

---

### **PHASE 5: Multi-Level Fallback Hierarchy (Weeks 11–13)**
**Goal:** Robust emergency handling when primary controllers fail.

**Why last?**
- Won't be needed until real deployment
- All nominal systems must work first
- Fallback tuning depends on understanding system capabilities

**Hierarchy (priority order):**

#### Level 1: Normal Operation
- NMPC (with SB-MPC COLAV references)
- Target: optimal path, respect constraints

#### Level 2: NMPC Solver Failure
- Fallback to last-known-good solution
- Or simple PID heading control (what you have now)
- Keep same speed, aim for last desired heading
- **Trigger:** `info.success == false` for N consecutive steps

```matlab
if ~info.success
  consec_nmpc_fails = consec_nmpc_fails + 1;
else
  consec_nmpc_fails = 0;
end

if consec_nmpc_fails >= 3
  % NMPC failed 3 times; switch to Level 2
  u_opt = pidHeading(x, chi_d, pid_params);
  fprintf('  >> NMPC FAIL × 3; PID fallback active\n');
  level = 2;
end
```

#### Level 3: Dynamic Obstacle Too Close (Emergency Stop)
- NMPC predicted collision despite constraints? (should not happen, but safety)
- Emergency maneuver: full rudder + reverse thrust
- **Trigger:** Predicted collision in NMPC output, OR obstacle within `r_emergency`

```matlab
% After NMPC solve, check predicted trajectory
if checkCollisionRisk(X_pred, obstacles, r_emergency)
  % Emergency avoidance
  u_opt = emercencyAvoidance(x, obstacle_pos);
  fprintf('  >> EMERGENCY AVOIDANCE: collision predicted\n');
  level = 3;
end

function u_emergency = emergencyAvoidance(x, obs_pos)
  % Hard turn away from obstacle
  dir_away = (x(4:5)' - obs_pos) / norm(x(4:5)' - obs_pos);
  chi_emergency = atan2(dir_away(2), dir_away(1));
  
  % Maximum rudder, full reverse
  u_emergency = [sign(atan2(sin(chi_emergency-x(6)), cos(chi_emergency-x(6)))) * deg2rad(35);
                 -20];  % reverse RPM
end
```

#### Level 4: Complete System Failure
- Cut power (RPM = 0)
- Dead-stick (coast to stop)
- Activate distress beacon
- **Trigger:** All controllers failed, uneliminated collision detected

```matlab
if level >= 3 && time_at_level3 > 30
  % System failure; emergency stop
  u_opt = [0; 0];
  fprintf('  >> SYSTEM FAILURE; EMERGENCY STOP\n');
  % Trigger alarm, beacon, etc.
  level = 4;
  break;  % exit simulation or switch to manual control
end
```

#### Fallback Structure
```matlab
classdef FallbackController < handle
  properties
    level = 1               % Current fallback level
    level_time = 0          % Time at current level
    prev_u = [0; 70]        % Last valid output
    nmpc_fail_count = 0
  end
  
  methods
    function [u_out, level_out] = computeFallback(obj, x, chi_d, pid_params, obstacles)
      try
        % Try NMPC
        u_out = nmpc_solve(...);
        obj.nmpc_fail_count = 0;
        obj.level = 1;
      catch
        obj.nmpc_fail_count = obj.nmpc_fail_count + 1;
        
        if obj.nmpc_fail_count >= 3
          % PID fallback
          u_out = pidHeading(x, chi_d, pid_params);
          obj.level = 2;
        else
          u_out = obj.prev_u;
          obj.level = 1.5;  % degraded
        end
      end
      
      % Check emergency conditions
      if checkEmergency(x, obstacles)
        u_out = emergencyAvoidance(x, obstacles);
        obj.level = 3;
      end
      
      obj.prev_u = u_out;
      obj.level_time = obj.level_time + dt;
      level_out = obj.level;
    end
  end
end
```

#### 5B — Test Complete Failure Scenarios
Create `test_fallback_hierarchy.m`:
- Inject NMPC solver failures
- Inject dynamic obstacles appearing suddenly
- Verify graceful degradation at each level

**Success Criteria:**
- Level 2 PID maintains safe speed/heading when NMPC fails
- Level 3 emergency maneuvers activate appropriately
- Level 4 stop triggered only when necessary
- All transitions smooth (no control jitter)

**Files to create/modify:**
- New: `FallbackController.m`
- New: `test_fallback_hierarchy.m`
- Modify: `run_nmpc_*m` (integrate fallback checks each step)

---

### **PHASE 6: SensorFusion & Real-Time Integration (Weeks 13–16)**
**Goal:** Connect to real sensors, state estimators, communication systems.

**Key Modules:**

#### 6A — State Estimator (EKF/UKF)
- Fuse IMU (acceleration, angular rate) + GPS (position, heading) + compass
- Output: estimated `x = [u v r x y psi p phi delta n]'`
- Uncertainty quantification for fallback decisions

#### 6B — Target Ship Tracker
- AIS input: other vessel positions, headings, speeds
- Kalman filter per target to smooth noisy estimates
- Predict motion over NMPC horizon

#### 6C — Obstacle Detector (LiDAR/Radar)
- Convert sensor readings to obstacle struct
- Fuse with map data
- Anomaly detection (unexpected obstacles)

#### 6D — Communication Module
- AIS broadcast of own ship state
- Listen for nearby vessel state updates
- VHF/SOTDMA collision avoidance coordination (future)

**Files:**
- New: `StateEstimator.m` (EKF wrapper)
- New: `TargetTracker.m` (per-vessel Kalman filters)
- New: `ObstacleDetector.m` (sensor fusion)
- New: `AIS_Interface.m` (communication)
- New: `test_real_world_integration.m`

---

### **PHASE 7: Optimization & Tuning (Weeks 16–20)**
**Goal:** Performance, robustness, parameter tuning for deployment.

#### 7A — NMPC Tuning
- Sweep cost weights (Q, R, R_rate)
- Optimize for: path tracking accuracy + control smoothness + solver speed
- Use Test A/B results as baseline

#### 7B — SB-MPC COLAV Tuning
- Adjust COLREGs parameters for harbor scale
- Tune collision risk weights
- Test on realistic traffic patterns

#### 7C — Solver Acceleration
- Warm-start from RRTx is first step
- Second-order CasADi options (Hessian computation)
- Constraint pruning (remove inactive obstacles from NLP dynamically)

#### 7D — Real-Time Safety Analysis
- Formal verification (bounded verification of constraint satisfaction)
- Monte Carlo uncertainty propagation
- Stress testing (worst-case scenarios)

**Files:**
- New: `tuningTools/` directory
  - `parameterSweep.m`
  - `performanceAnalysis.m`
  - `robustnessTest.m`

---

## Summary: Recommended Order

| Phase | Duration | Goal | Why This Order |
|-------|----------|------|---|
| 0 | 1–2 wks | Validate MVP | Essential before adding complexity |
| 1 | 2–4 wks | Dynamic obstacles | Core capability; foundational |
| 2 | 4–6 wks | RRTx warm-start | Solver acceleration once core works |
| 3 | 6–8 wks | Azimuth thrusters | Richer control; builds on stable NMPC |
| 4 | 8–11 wks | SB-MPC COLAV | Multi-ship scenarios; complex layer |
| 5 | 11–13 wks | Fallback hierarchy | Safety layer; needed for robustness |
| 6 | 13–16 wks | Real sensors | Integration once control loops proven |
| 7 | 16–20 wks | Optimize & tune | Final polishing for deployment |

**Total:** ~20 weeks (~5 months) for full system from MVP to production-ready.

---

## Key Dependencies & Interfaces

### Assumptions About Your Code
- **RRTx algorithm** returns: `.waypoints` (Nx2), `.tree`, `.update_time`
- **Container dynamics** extendable to azimuth version
- **Animation/plotting** generic enough for any control output

### External Libraries Required
- **CasADi** (already using)
- **Control Systems Toolbox** (for Kalman filtering, Phase 6)
- **Statistics & Machine Learning Toolbox** (uncertainty quant)

### Data Formats (Standardized)
```matlab
%% State
x = [u; v; r; x; y; psi; p; phi; delta; n]  % 10×1

%% Control
u_conv = [delta_c; n_c]                      % 2×1 (conventional)
u_azi = [F1; azi1; F2; azi2]                % 4×1 (azimuth)

%% Obstacle (dynamic or static)
obs(i) = struct(...
  'position', [x_m; y_m], ...                % 2×1 [m]
  'radius', r_m, ...                         % 1×1 [m]
  'velocity', [u_m/s; v_m/s], ...           % 2×1 [m/s]
  'type', 'static|dynamic|map_polygon');     % string

%% Target Ship (full state)
x_ts = [u; v; r; x; y; psi]                 % 6×1
```

---

## Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| RRTx-NMPC mismatch | Test warm-start extensively (Phase 2); have cold-start fallback |
| Azimuth solver slowdown | Start with 2 thrusters; add 3rd/4th only if necessary |
| COLAV over-avoidance | Tune SB-MPC COLREGs weights iteratively; compare to expert pilot behavior |
| Real-time miss | Reduce NMPC horizon if needed; accept slightly suboptimal paths in exchange for speed |
| Sensor noise degradation | Implement state estimator (Phase 6) with uncertainty covariance; propagate to NMPC |

---

## Success Metrics (End of Phase 7)

- ✓ **Functional:** Tests A–D (static/dynamic obstacles, multiple vessels) pass
- ✓ **Real-time:** Solve time < 250 ms @ 4 Hz control loop
- ✓ **Safe:** Zero unplanned collisions, min distance > safety threshold
- ✓ **Efficient:** Path cost ≤ 10% above optimal (RRTx benchmark)
- ✓ **Robust:** 95%+ success rate in adverse conditions (sensor noise, unexpected obstacles)
- ✓ **Compliant:** All maneuvers respect IMO COLREGs
- ✓ **Integrated:** Works with real sensors, AIS, ship systems

---

## Quarterly Milestones

**Q1 (Weeks 1–13):** MVP → Dynamic obstacles → RRTx → Azimuth
- End of Q1: Multi-ship scenarios with conventional propulsion ✓

**Q2 (Weeks 14–20):** SB-MPC COLAV → Fallbacks → Real sensors
- End of Q2: Production-ready system with sensor fusion ✓

**Q3 (Weeks 21–26):** Field testing, tuning, deployment
- End of Q3: Deployed in harbor ✓

---

## Questions to Answer Before Each Phase

**Before Phase 1:** Does the MVP solver run at real-time speeds? Any numerical issues?

**Before Phase 2:** How accurate is target ship velocity estimate? Can we assume constant velocity, or need prediction?

**Before Phase 3:** What's the azimuth thruster configuration? Independent propulsion or coupled?

**Before Phase 4:** Do you want COLREGs compliance mandatory, or guidance-level recommendation?

**Before Phase 5:** What's the risk tolerance for emergency maneuvers? How aggressive can fallback be?

**Before Phase 6:** What sensors are available? GPS/INS, LiDAR, AIS, Compass?

**Before Phase 7:** What are the harbor traffic patterns, traffic density, typical maneuvers?

---

## Next Steps

1. **Confirm Phase 0 success** — Run `run_nmpc_simple.m` on your system
2. **Review this roadmap** — Adjust order/priorities based on your constraints
3. **Schedule Phase 1** — Start dynamic obstacle implementation
4. **Prepare RRTx interface** — Ensure your RRTx outputs the expected structure

Good luck! 🚀
