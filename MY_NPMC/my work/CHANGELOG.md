# CHANGELOG — NMPC Harbor Navigation Refactor

**Author:** Riccardo Legnini  
**Date:** 2025-02-27  
**Sources:** autobargesim-v0.1.2 toolbox, container.m (MSS), previous original code

---

## Overview

Complete refactoring of the NMPC harbor navigation codebase. The previous version had a monolithic MPC with manual reference generation and no proper guidance/collision avoidance layers. The new version implements a modular **LOS Guidance → SB-MPC COLAV → NMPC → Plant** pipeline.

Old files are backed up as `*_old.m`.

---

## NEW FILES

### 1. `NavUtils.m` — Static utility class
- **Source:** autobargesim `colav/utils.m` + `guidance/utils.m`
- **Contents:**
  - `wrap_angle_to_pmpi(angle)` — wrap to [-π, π]
  - `wrap_angle_diff_to_pmpi(a1, a2)` — wrapped difference
  - `wrap_min_max(x, xmin, xmax)` — modular wrapping
  - `normalize_vec(v)` — safe vector normalization
  - `sat(x, xmin, xmax)` — saturation/clamping
  - `getNumSamples(dt, T)` — discrete sample count
  - `pointToSegment(px, py, x1, y1, x2, y2)` — point-to-segment distance (was duplicated in old NMPC_Container and HarborObstacles)
  - `isNavigable(point, map)` — polygon-based navigability check
  - `validatePath(waypoints, map, resolution)` — path validity checker
  - `isInsideMapBoundary(point, map)` — boundary containment check

### 2. `ActuatorModel.m` — Modular actuator dynamics
- **Source:** autobargesim `actuatorClass.m` + container.m saturation logic
- **Why:** Actuator rate limiting was embedded inside `dynamics_casadi()`. Now it's a standalone class for the simulation plant loop.
- **Contents:**
  - `rudderDynamics(delta_c, delta)` — rate-limited rudder
  - `shaftDynamics(n_c, n)` — Tm-based shaft model
  - `applyActuatorResponse(ctrl_cmd, dt)` — full rate-limited response
  - `reset(delta0, n0)` — state reset

### 3. `LOSGuidance.m` — Line-of-Sight guidance with waypoint management
- **Source:** autobargesim `guidance/LOSguidance.m` + `guidance/guidance.m`
- **Replaces:** The manual "carrot" reference generation in the old run script (`bearing = atan2(...)`, linear interpolation toward goal)
- **Contents:**
  - `findActiveSegment(wp_pos, x, wp_idx)` — automatic waypoint switching based on radius-of-acceptance and pass-angle threshold (from autobargesim `find_active_wp_segment`)
  - `computeRef(wp_pos, wp_speed, x, wp_idx, chi_d_prev, iter)` — LOS heading with cross-track error correction, sideslip compensation, and rate-limited heading updates
  - `computeXTE(wp_pos, x, wp_idx)` — cross-track error metric
- **Key parameters:** `K_p` (1/lookahead), `R_a` (acceptance radius), `chi_rate_max`, `update_interval`

### 4. `PIDHeadingController.m` — PID heading controller (fallback)
- **Source:** autobargesim `controlClass.LowLevelPIDCtrl`
- **Why:** When the NMPC solver fails (infeasible), the ship needs a fallback controller to maintain safe heading. Old code just sent `[0; RPM]` (zero rudder).
- **Contents:**
  - `compute(psi_d, r, psi, dt)` — PD/PID heading control with angle wrapping
  - `reset(psi0)` — state reset
- **Fallback logic:** Activated after 2+ consecutive NMPC failures in the run script.

### 5. `SBMPC_COLAV.m` — Scenario-Based MPC for COLREGs collision avoidance
- **Source:** autobargesim `colav/sbmpc.m` + `colav/colav.m`
- **Why:** The old code had NO collision avoidance for moving vessels. NMPC obstacle constraints only handled static circles. SB-MPC adds a deliberative COLAV layer.
- **Architecture:** Sits between LOS guidance and NMPC. Evaluates 13×3 = 39 course/speed scenarios. Picks the one minimizing:
  - **Collision cost** — distance-based, time-weighted, relative-velocity scaled
  - **COLREGs cost** — penalizes violations (head-on, crossing, overtaking rules)
  - **Maneuvering cost** — penalizes deviation from desired course/speed (asymmetric: port turns cost more than starboard)
- **Contents:**
  - `run(x, chi_d, U_d, chi_m_last, U_m_last, x_ts)` — main entry point
  - `calcVesselTraj()` — RK4 kinematic trajectory propagation
  - `calcCostCollision()` — distance-based collision cost
  - `calcCostCOLREGs()` — situation classification + rule violation cost
  - `calcCostManeuvering()` — deviation penalty
- **Returns:** Modified course `chi_c` and speed `U_c` for the NMPC reference.

---

## REWRITTEN FILES

### 6. `NMPC_Container.m` — Complete rewrite

#### Previous version (Opti-based):
- Used `casadi.Opti()` — rebuilt optimizer every time obstacle count changed
- No current/environment effects (container.m has none by default)
- Single dynamics model (full 10-state nonlinear)
- Warm start via `opti.set_initial()`
- Obstacle slots varied at runtime → forced rebuilds

#### New version (nlpsol-based):
| Feature | Old | New |
|---------|-----|-----|
| CasADi interface | `casadi.Opti` | `casadi.nlpsol` (build once) |
| Obstacle slots | Variable (rebuild on count change) | Fixed `max_obs` slots (dummies at 1e8) |
| Current effects | None | `V_c`, `beta_c` → relative velocities `u_r`, `v_r` |
| Dynamics models | Full only | `'full'` or `'simplified'` (linear damping from Lcontainer.m) |
| Parameter passing | `opti.set_value()` | NLP parameter vector `p` |
| Warm start | `opti.set_initial()` | Solution vector shift (`prev_sol`) |
| Solver options | Same | Added `sb='yes'` to suppress banner |
| Utility functions | Inline `pointToSegment` | Delegates to `NavUtils` |
| Build trigger | Every obstacle count change | Once (`buildSolver()`) |

#### Detailed changes in dynamics:
- **`dynamics_full_casadi(x, u, env)`**: Added `env = [V_c; beta_c]` parameter. Computes relative velocities `u_r = u - V_c*cos(beta_c - psi)`, `v_r = v - V_c*sin(beta_c - psi)`. All hydrodynamic forces use relative velocities. Kinematics use absolute velocities (correct for NED frame).
- **`dynamics_simplified_casadi(x, u, env)`**: NEW. Uses linearized M/N/G/b matrices from Lcontainer.m for sway-yaw-roll prediction. Surge still nonlinear (thrust model). Much faster for long horizons.
- **`alphaR` computation**: Changed `atan(vR/uR_safe)` to `atan2(vR, uR_safe)` for CasADi safety.
- **`buildSolver()`**: Single NLP construction with SX symbolics. Obstacle avoidance as inequality constraints `dist² ≥ (r+safety-slack)²`. All bounds pre-computed.
- **`solve()`**: Assembles parameter vector, fills obstacle slots (unused → dummy), shifts warm start, catches failures.

### 7. `HarborObstacles.m` — Enhanced obstacle manager

#### Changes from previous version:
| Feature | Old | New |
|---------|-----|-----|
| Target ships | Not supported | `addTargetShip(state_6dof, radius, name)` with full kinematic state |
| State format | `.position + .velocity` | Also `.state = [u v r x y psi]` for SB-MPC |
| `getTargetShipStates()` | N/A | Returns Mx6 matrix for SB-MPC |
| `getAllCircularObstacles()` | `getAllObstacles()` (same) | Renamed for clarity |
| `getAllObstaclesIncludingMap()` | Used midpoint of closest segments | Uses nearest-point-on-polygon (tighter) |
| `checkMapCollision(pos)` | Inline in run script | Method on class |
| `checkCircularCollision(pos)` | N/A | New method |
| `updateDynamicObstacles(dt)` | Position only | Also propagates target ship states |
| `predictDynamicPositions(t)` | N/A | New (non-mutating prediction) |
| Utility calls | Inline distance calc | Delegates to `NavUtils.pointToSegment` |

### 8. `run_mpc_harbour_navigatin.m` — Complete rewrite

#### Previous version:
- Manual "carrot" reference: `bearing = atan2(goal_y - y, goal_x - x)`, linear interpolation
- Single waypoint per test (point-to-point)
- No guidance system (hardcoded reference generation)
- No collision avoidance for moving objects
- Euler integration for plant simulation
- Tests A (commented out), B (turn), C (one static obstacle)
- Fallback: zero rudder on NMPC fail

#### New version:
| Feature | Old | New |
|---------|-----|-----|
| Reference generation | Manual bearing + linear interp | LOS guidance (`LOSGuidance.computeRef`) |
| Waypoint management | Single goal point | Multi-waypoint with auto-switching |
| Collision avoidance | NMPC obstacle constraints only | SB-MPC COLAV layer + NMPC constraints |
| Plant integration | Euler (`x + xdot*dt`) | RK4 (`simStep` function) |
| Fallback controller | `[0; RPM]` (zero rudder) | PID heading controller after 2 consecutive fails |
| Solver build | Built inside `nmpc.solve()` | Explicit `nmpc.buildSolver()` once before loop |
| Tests | A(commented), B(turn), C(1 obs) | A(multi-wp), B(multi-wp+obstacle), C(multi-wp+target ship) |
| Metrics | Solver success rate | + XTE, + COLAV modifications log |
| Plotting | 2×3 subplots | 3×3: trajectory + controls + XTE/COLAV per test |
| CasADi check | `Opti`-based function eval | `SX`-based `casadi.Function` with env param |

#### Pipeline per timestep:
```
1. LOSGuidance.findActiveSegment()      → wp_idx
2. LOSGuidance.computeRef()             → chi_d, U_d
3. SBMPC_COLAV.run() [Test C only]      → chi_c, U_c (modified course/speed)
4. buildRefTrajectory()                  → x_ref [10 × N+1]
5. NMPC_Container.solve()               → u_opt [delta_c; n_c]
6. PIDHeadingController [if NMPC fails] → u_opt (fallback)
7. simStep() [RK4 container.m]          → x_next
8. HarborObstacles.checkMapCollision()  → collision flag
```

---

## UNCHANGED FILES

- `container.m` — Plant model (not modified; current effects added only in NMPC prediction)
- `Lcontainer.m` — Linearized model (read by simplified dynamics, not modified)
- `helsinki_harbour.mat` — Map data (not modified)

## BACKUP FILES CREATED

- `NMPC_Container_old.m` — Previous Opti-based NMPC
- `HarborObstacles_old.m` — Previous obstacle manager
- `run_mpc_harbour_navigatin_old.m` — Previous run script

---

## SUMMARY OF AUTOBARGESIM MODULES USED

| autobargesim module | Used in | What was ported |
|---|---|---|
| `colav/utils.m` | `NavUtils.m` | Angle wrapping, saturation, normalization |
| `guidance/utils.m` | `NavUtils.m` | Same utilities (deduplicated) |
| `guidance/guidance.m` | `LOSGuidance.m` | Waypoint switching base class logic |
| `guidance/LOSguidance.m` | `LOSGuidance.m` | LOS heading computation + rate limiting |
| `control/controlClass.m` | `NMPC_Container.m`, `PIDHeadingController.m` | nlpsol pattern, PID controller, XTE calculator |
| `model&actuator/actuatorClass.m` | `ActuatorModel.m` | Rate limiting, propeller/rudder response |
| `model&actuator/modelClass.m` | `NMPC_Container.m` | Current effects (relative velocity), control_ref_model concept |
| `colav/colav.m` | `SBMPC_COLAV.m` | Kinematic model, RK4 integration, trajectory propagation |
| `colav/sbmpc.m` | `SBMPC_COLAV.m` | Full SB-MPC algorithm (scenarios, collision/COLREGs/maneuvering costs) |
| `maps/+maps/planner.m` | NOT USED | Requires shapefile preprocessing (incompatible with helsinki_harbour.mat) |

## NOT IMPLEMENTED (Future work)

- **Graph-based path planner** (`maps/+maps/planner.m`): Requires S-57 shapefile layers (`wtwaxs`, `depare`, `lndare`) processed by `maps.processor`. Helsinki harbour .mat only has polygon arrays. Could be implemented with a visibility-graph or Voronoi approach on the existing polygon data.
- **Full actuator force model in simulation**: `ActuatorModel` handles rate limiting but plant still uses `container.m` internally for forces. Could extract propeller/rudder force computation for custom plant models.
