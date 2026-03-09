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

**Phasing Rationale:**
The order below reflects both technical readiness and computational burden:
1. **Validate MVP** → Establish baseline
2. **Azimuth Thrusters** → Finalize vehicle dynamics before implementing higher-level algorithms
3. **RRTx Warm-Start** → Replace weak LOS guidance with collision-free path planning (lighter computation than obstacle avoidance)
4. **Dynamic Obstacle Avoidance** → Add moving obstacles *atop* proven guidance + actuation
5. **(optional) SB-MPC COLAV** → Add maritime rules as policy layer
6. **Compare results with Giacomo's solution to the obstacle detection problem**

This avoids building obstacle avoidance on top of weak guidance, and staggers computational complexity.

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

### **PHASE 1: Azimuth Thruster Actuation (Weeks 2–4)**
**Goal:** Replace fixed-heading propeller with independent azimuth thrusters.

**Why this phase 1?**
- Core NMPC is validated ✓
- Vehicle dynamics **must be finalized** before implementing higher-level guidance or obstacle avoidance
- Different thruster configuration changes yaw response, turning radius, and solver behavior significantly
- Better to tune NMPC once with final actuation than redesign after realizing actuators changed everything
- Current LOS guidance is weak; no point in tuning obstacle avoidance around it yet

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

#### 1A — Container Dynamics Extension
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

#### 1B — NMPC_Container_Lite → NMPC_Container_Azimuth (or extend Lite)
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

#### 1C — Test & Validation
Create `test_azimuth_actuation.m`:
- Compare conventional vs azimuth on same maneuver (Test A & B equivalents)
- Verify faster yaw response with azimuth
- Check solver time increase (expect +20–40% due to more controls)
- Measure tighter turning radius capability

**Success Criteria:**
- Both conventional and azimuth models produce stable solutions
- Azimuth can execute tighter turns (smaller turning radius)
- No solver instability from expanded control space
- Solve time increase ≤ 50%

**Files to create/modify:**
- New: `container_azimuth.m` (dynamics with azimuth)
- New: `NMPC_Container_Azimuth.m` (extended NMPC)
- New: `test_azimuth_actuation.m`

---

### **PHASE 2: RRTx Warm-Start Path Planning (Weeks 4–6)**
**Goal:** Replace weak LOS guidance with collision-free RRTx path planning.

**Why this phase 2 (before obstacle avoidance)?**
- Azimuth dynamics finalized ✓
- Current LOS guidance is not reliable; NMPC should not try to compensate for bad guidance
- RRTx provides collision-free reference trajectory → NMPC solves faster and more reliably
- RRTx replanning handles static obstacle changes; NMPC focuses on dynamic obstacles and fine guidance
- Computational complexity: RRTx is offline/update-driven; NMPC obstacle constraints are per-step (heavier)

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

#### 2B — Guidance Layer Replacement
Replace LOS guidance with RRTx reference extraction:

```matlab
function [x_ref, chi_d, U_d] = getRRTxRef(obj, x_curr, rrtx_path, N_horiz, dt)
  % 1. Find nearest WP on RRTx path to current position
  nearest_idx = findNearestWaypoint(x_curr(4:5), rrtx_path.waypoints);
  
  % 2. Extract next N waypoints along path (these are collision-free by design)
  wp_subset = rrtx_path.waypoints(nearest_idx:min(nearest_idx+N_horiz, end), :);
  
  % 3. Build reference trajectory along RRTx path
  x_ref = buildRefAlongPath(x_curr, wp_subset, N_horiz, dt);
  
  % 4. Extract course and speed references from x_ref
  chi_d = atan2(x_ref(5,:), x_ref(4,:));  % course from position derivatives
  U_d = sqrt(x_ref(1,:).^2 + x_ref(2,:).^2);  % speed from velocity states
end
```

#### 2C — Warm-Start Strategy for NMPC
Instead of LOS reference, use RRTx-based reference for NMPC initialization:

```matlab
function x0_guess = warmStartFromRRTx(obj, x_curr, x_ref, U_ref, N_horiz, dt)
  % Use RRTx reference trajectory as initial guess for NMPC
  X_init = x_ref;
  
  % Generate control guesses (zero rudder, reference speed)
  U_init(1,:) = 0;                      % rudder angle = 0
  U_init(2,:) = U_ref;                  % RPM = reference speed
  
  % For azimuth: reference azimuth angles from course targets
  U_init(3,:) = atan2(diff([x_curr(5); x_ref(5,:)]), ...  % F thruster azi
                       diff([x_curr(4); x_ref(4,:)]));
  U_init(4,:) = U_init(3,:);            % A thruster azi (symmetric)
  
  x0_guess = [X_init(:); U_init(:); zeros(n_obs, N_horiz+1)];
end
```

#### 2D — Replanning Trigger
When to recompute RRTx?
- On new goal
- When NMPC deviates significantly from RRTx path (LOS-like crosstrack error check)
- When new static obstacles detected
- Periodically (e.g., every 30 s)

```matlab
function replan_needed = checkReplanTrigger(x, rrtx_path, xte_threshold)
  % Crosstrack error: distance to nearest RRTx waypoint
  d_to_path = min(sqrt(sum((x(4:5)' - rrtx_path.waypoints).^2, 2)));
  replan_needed = (d_to_path > xte_threshold);
end
```

#### 2E — Integration with NMPC
Modify your main loop:

```matlab
% Initialization
rrtx = RRTx(...);  % Your existing planner
rrtx_path = rrtx.plan(x_init, goal);  % Compute initial path

for k = 1:n_steps
  % 1. Check if replanning needed
  if checkReplanTrigger(x, rrtx_path, xte_max)
    fprintf('Replanning RRTx path at step %d\n', k);
    rrtx_path = rrtx.plan(x, goal);
  end
  
  % 2. Get RRTx reference
  x_ref = getRRTxRef(x, rrtx_path, nmpc.N, dt);
  
  % 3. Warm-start NMPC with RRTx
  x0_guess = warmStartFromRRTx(nmpc, x, x_ref, N_horiz, dt);
  
  % 4. Solve NMPC (now with good reference, no extra obstacles yet)
  sol = nmpc.solve(x, x0_guess, x_ref);
  
  % 5. Execute + propagate
  u_opt = sol.u_opt(:, 1);
  x = propagate(x, u_opt, dt);
end
```

#### 2F — Test
Create `test_rrtx_guidance.m`:
- Same Test A scenario
- Measure crosstrack error (XTE) with LOS vs RRTx
- Measure NMPC solve time with cold-start (LOS) vs warm-start (RRTx)
- Expected: RRTx XTE ≤ LOS XTE, solve time 30–50% faster

**Success Criteria:**
- RRTx guidance produces lower crosstrack error than LOS
- Warm-start solve time 30–50% faster than cold start
- Consistent path tracking (no oscillations)
- Replanning triggers work without causing instability

**Files to create/modify:**
- New: `RRTxInterface.m` (wrapper for your RRTx algorithm)
- Modify: `NMPC_Container_Azimuth.m` (accept warm-start guess from RRTx)
- Modify: `run_nmpc_simple.m` (replace LOS with RRTx calls, add replanning checks)
- New: `test_rrtx_guidance.m`

---

### **PHASE 3: Dynamic Obstacle Avoidance (Weeks 6–8)**
**Goal:** Handle moving vessels with known state (position, heading, speed).

**Why now (after azimuth + RRTx)?**
- Vehicle actuation finalized ✓
- Guidance proven reliable with RRTx ✓
- NMPC can focus on dynamic obstacles, not compensating for weak guidance
- RRTx handles static obstacles → NMPC adds dynamic collision avoidance

**Approach:**

#### 3A — Obstacle Prediction
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

In this module the circle representing the safe zone around obstacles should be expanded to a dynamic adapting ellipse that adapts based on general speed and the prediction module.

#### 3B — Modify NMPC to Handle Dynamic Obstacles
Update `NMPC_Container_Azimuth`:

**Before (no obstacles):**
```matlab
% dt = 1.0 s, no constraint loop
```

**After (trajectory-based):**
```matlab
% Constraint: vessel stays clear of obstacle trajectory
for k_pred = 1:N_h         % for each NMPC step
  for j = 1:n_obs          % for each obstacle
    for k_obs = 1:n_obs_traj  % for each trajectory discretization
      % Use interpolated obstacle position along its trajectory
      dt_idx = floor(k_pred * dt / dt_obs_pred) + 1;
      obs_pos_pred = obs_trajectory_at(P_obs_traj, j, dt_idx);
      
      dx = X(4,k_pred) - obs_pos_pred(1);
      dy = X(5,k_pred) - obs_pos_pred(2);
      dist_sq = dx^2 + dy^2;
      g = vertcat(g, dist_sq - (P_obs_rad(j) + r_safety)^2);
    end
  end
end
```

**Note:** This increases constraint count; may need to:
- Sparsify obstacles (keep only closest N per horizon)
- Use coarser trajectory discretization (e.g., every 5 s instead of 1 s)

#### 3C — Interface Update
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

#### 3D — Test
Create `test_dynamic_obstacles.m`:
- One stationary vessel (baseline)
- One vessel moving at constant velocity (crossing scenario)
- One vessel moving parallel (overtaking)
- Verify NMPC predicts their paths and steers clear

**Success Criteria:**
- Dynamic obstacles avoided with same safety margin as static
- Solve time ≤ 1.5 s (manageable increase from RRTx phase)
- No false positives (overly conservative steering)

**Files to create/modify:**
- New: `DynamicObstaclePredictor.m`
- Modify: `NMPC_Container_Azimuth.m` (add dynamic obstacle constraints)
- New: `test_dynamic_obstacles.m`

---

### **PHASE 4: SB-MPC COLAV (COLREGs-Aware Collision Avoidance) (Weeks 8–11)**
**Goal:** Integrate your existing SB-MPC COLAV with dynamic obstacle estimates.

**Why now (final phase)?**
- Dynamic obstacles working ✓
- NMPC core + azimuth + RRTx + dynamic obstacles all proven ✓
- SB-MPC COLAV is sophisticated policy layer; best tested with solid foundation
- Acts as high-level decision layer guiding NMPC

**Current SB-MPC COLAV Status:**
- 300+ lines, 24 tuning parameters
- Evaluates discrete course/speed scenarios
- Computes collision risk, COLREGs metrics, maneuvering cost
- Returns modified course/speed references

**Integration Architecture**:
```
Input:  x (own ship state), target_ships (vessel tracker), x_ref (RRTx path)
        
Step 1: RRTx provides collision-free reference path
        → x_ref (static obstacles already considered)
        
Step 2: SB-MPC COLAV evaluates scenarios considering target ships
        → may modify reference (chi_m, U_m) for COLREGs compliance
        
Step 3: NMPC solves with both RRTx reference and dynamic obstacles
        → handles moving vessels, refines guidance
        
Step 4: Execute + measure performance
```

#### 4B — Target Ship State Management & Prediction
Integrate with your AIS/target tracker to get vessel states, predict motion:

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
Replace LOS + NMPC with RRTx + SB-MPC COLAV + NMPC:

```matlab
% Get RRTx reference (replans as needed)
if checkReplanTrigger(x, rrtx_path, xte_max)
  rrtx_path = rrtx.plan(x, goal);
end
[x_ref, chi_d, U_d] = getRRTxRef(x, rrtx_path, nmpc.N, dt);

% Get SB-MPC COLAV modified reference
x_ts = updateTargetShips(x_ts, dt);
[chi_m, U_m] = sbmpc_colav.run(x, chi_d, U_d, x_ts);

% Blend RRTx + SB-MPC references
x_ref_blended = buildRefAlongPath(x, chi_m, U_m*U_d, nmpc.N, dt);

% Convert target ships to obstacles for NMPC
obs_ts = [];
for i = 1:length(x_ts)
  obs_ts(i).position = x_ts(i,4:5)';
  obs_ts(i).radius = 30;  % typical ship size
  obs_ts(i).velocity = [x_ts(i,1); x_ts(i,2)];
end

% Azimuth NMPC with dynamic obstacles
[u_opt, X_pred, info] = nmpc_azimuth.solve(x, x_ref_blended, obs_ts);
```

#### 4D — Test Scenarios
Create `test_sb_mpc_colav.m`:
- **Scenario 1:** Crossing encounter (target from port/starboard)
- **Scenario 2:** Overtaking (target ahead, same direction)
- **Scenario 3:** Head-on (target approaching head-on)
- **Scenario 4:** Multiple vessels (2–3 simultaneous targets)

Verify:
- Safe distances maintained
- COLREGs-compliant maneuvers
- Smooth transitions (no jittery course changes)
- Solver remains fast (< 2 s per step)

**Success Criteria:**
- All crossing/overtaking scenarios passed with safety margin
- Minimum distance to target ≥ safety threshold
- No COLAV oscillation (course thrashing)
- Solve time increase ≤ 30% vs static obstacles

**Files to create/modify:**
- New: `SB_MPC_COLAV_Interface.m` (wrapper for your COLAV algorithm)
- Modify: `run_nmpc_simple.m` or create `run_nmpc_harbor_sbmpc.m` (full integrated loop)
- New: `test_sb_mpc_colav.m`
- Possibly: Tune SB-MPC COLAV parameters for harbor scale

---

## Development Checklist

### Completed Phases
- [ ] PHASE 0: MVP Validation
- [ ] PHASE 1: Azimuth Thrusters
- [ ] PHASE 2: RRTx Warm-Start

### In-Progress / Upcoming
- [ ] PHASE 3: Dynamic Obstacle Avoidance
- [ ] PHASE 4: SB-MPC COLAV Integration

### Future (Post-Harbor Navigation MVP)
- Multi-level fallback hierarchy
- Sensor fusion (EKF state estimation, target tracking)
- Real-time hardware integration
- Performance optimization & tuning
- Formal verification & safety analysis

---

## Key Files to Create/Modify Summary

### By Phase
**PHASE 1: Azimuth**
- [ ] `container_azimuth.m`
- [ ] `NMPC_Container_Azimuth.m`
- [ ] `test_azimuth_actuation.m`

**PHASE 2: RRTx Guidance**
- [ ] `RRTxInterface.m`
- [ ] Modify `NMPC_Container_Azimuth.m` (warm-start support)
- [ ] Modify `run_nmpc_simple.m` (RRTx loop integration)
- [ ] `test_rrtx_guidance.m`

**PHASE 3: Dynamic Obstacles**
- [ ] `DynamicObstaclePredictor.m`
- [ ] Modify `NMPC_Container_Azimuth.m` (dynamic constraint handling)
- [ ] `test_dynamic_obstacles.m`

**PHASE 4: SB-MPC COLAV**
- [ ] `SB_MPC_COLAV_Interface.m` (if not already wrapped)
- [ ] Create `run_nmpc_harbor_sbmpc.m` (integrated main loop)
- [ ] `test_sb_mpc_colav.m`

---

## Notes & Considerations

1. **Computational Complexity**: This progression staggers heavy computation:
   - Phase 1: Heavier dynamics (azimuth), simpler control
   - Phase 2: Path planning (RRTx), initialization benefit
   - Phase 3: Dynamic obstacle constraints (heavier NLP)
   - Phase 4: Policy layer (SB-MPC COLAV) on top of proven foundation

2. **Guidance Improvement**: Replacing weak LOS guidance (Phase 2) before adding dynamic obstacles (Phase 3) ensures NMPC can focus on real problems, not compensating for bad guidance.

3. **Solver Convergence**: By Phase 4, warm-start from RRTx should offset the added complexity from dynamic obstacles and SB-MPC integration.

4. **Testing Strategy**: Each phase has dedicated tests (testA/B/C/D…) that build incrementally. Ensure each phase passes before moving to next.

5. **Fallback Hierarchy** (future, post-harbor MVP): Once all nominal systems work, add emergency controllers:
   - PID heading control fallback
   - Emergency avoidance maneuvers
   - Dead-stick (unpowered drifting)

---

## References & Related Work

- **NMPC**: Model Predictive Control for vessel path following (already implemented)
- **RRTx**: Rapidly-exploring Random Trees with rewiring (your existing code in `/autobargesim/maps/`)
- **SB-MPC COLAV**: Your existing scenario-based MPC collision avoidance (300+ lines, 24 parameters)
- **Azimuth Thrupropeller**: Independent thrust magnitude + angle per thruster; standard on modern DP vessels
- **COLREGs**: International Regulations for Preventing Collisions at Sea (enforced by SB-MPC COLAV)

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
