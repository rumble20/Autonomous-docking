## About the distinction between past work and my work

"This work presents an integrated motion planning and control framework for autonomous ship docking. Unlike traditional two-stage approaches where a global planner generates a fixed collision-free path that a separate controller blindly tracks, the proposed method employs a Nonlinear Model Predictive Controller that jointly performs trajectory generation and tracking by optimizing control inputs online while explicitly enforcing obstacle-avoidance constraints. A lightweight guidance layer provides high-level navigation directives (desired heading, speed, and waypoint sequencing), but the NMPC retains authority to deviate from the reference trajectory when necessary to satisfy safety constraints and actuator limitations."

### Technical arguments for the thesis

1. The reference is soft, constraints are hard

| Component | Traditional decoupled architecture | Integrated architecture used here |
|---|---|---|
| Path/reference | Hard requirement (must be tracked) | Soft guidance (deviation is allowed) |
| Obstacle avoidance | Planner responsibility (mostly fixed) | NMPC responsibility (online, receding horizon) |
| Actuator limits | Often ignored in planning layer | Enforced in NMPC constraints |
| Dynamic feasibility | Assumed | Enforced through model constraints |

2. Trajectory is generated online, not pre-computed

At each control step, NMPC computes:

- Optimal state trajectory: $X^* = \{x_0, \dots, x_N\}$
- Optimal control sequence: $U^* = \{u_0, \dots, u_{N-1}\}$

Therefore, the controller is not only tracking; it is also generating a feasible local trajectory online.

3. Guidance is not full planning

The guidance layer currently:

- Selects and advances waypoints (sequential logic)
- Computes desired heading toward the active segment
- Builds a shaped reference for NMPC

It does not solve a global constrained optimization problem. Collision-avoidance feasibility and dynamic feasibility are handled by NMPC.

### Literature support

From the available papers:

- 2022 Autonomous Docking MPC: uses MPC to generate online trajectories to waypoints, not only to track a fixed offline path.
- Martinsen et al. (2020): discusses advantages of direct NMPC in disturbance-rich docking conditions.
- CBF-MPC paper: emphasizes the value of combining planning and control in one optimization framework.
- Former thesis: presents receding-horizon trajectory optimization with online segment generation.

If RRT* is later integrated, a consistent framing is:

"The system follows a hierarchical architecture in which RRT* provides global route guidance and warm-start information, while NMPC performs local trajectory optimization with hard safety constraints. Unlike strictly decoupled hierarchies, the NMPC layer retains authority to deviate from the RRT* path when real-time constraints require it."

This keeps RRT* as an enhancement, not a replacement for NMPC online trajectory generation.

## Thruster allocation

State clearly that thrust allocation is handled inside the NMPC problem through force/moment generation with two azimuth thrusters (as in the MSS-style vessel model abstraction). Detailed motor-drive/RPM-loop implementation is outside the thesis scope.

## Martinsen

The Martinsen method is strong for harbor docking when:

- A known harbor map is available
- Obstacles are mostly static (piers, walls, moored vessels)
- Sensor fusion updates the local occupancy representation

For dynamic circular obstacles (moving vessels in congested zones), the circular/slack-variable-friendly formulation remains practical because it avoids repeated polytope construction.

## Agile development

The software process started with a traditional requirements-driven plan. After early prototyping, it became clear that fixing all requirements upfront was not practical due to iterative solver tuning, model-controller coupling, and scenario-dependent constraint interactions. Implementation therefore followed an incremental agile workflow with frequent integration and testing.

## How computational efficiency was improved

The speed-up came from reducing NLP size and simplifying online workload while preserving closed-loop behavior.

At discretization/problem-size level:

- Prediction horizon is currently $N=25$ with $\Delta t = 1.0$ s.
- Active map-obstacle slots are capped at 3 (`max_map_obstacles = 3`).

At solver level (IPOPT):

- Bounded iteration count (`max_iter = 120`)
- Relaxed convergence settings (`tol = 2e-3`, `acceptable_tol = 2e-2`, `acceptable_iter = 3`)
- Adaptive barrier strategy for robust real-time behavior

At geometric/environment level:

- Harbor map is represented through sampled circular keep-out primitives.
- Dynamic obstacles are packed as circle obstacles, with optional latent-awareness virtual obstacles.

At execution level:

- Obstacle packaging and selection are bounded and deterministic.
- Runtime diagnostics report step time, solve time, and real-time ratio.

Compact summary:

- Decision variables: $n_{var} = n_x(N+1) + n_uN$
- Obstacle inequalities (current formulation): $n_{obs}(N+1)$
- Real-time index: $\rho = t_{plan}/T_s$, feasible when $\rho < 1$

Note on midpoint constraints:

- In the current active solver, obstacle constraints are enforced at prediction nodes.
- Mid-interval obstacle constraints are not currently enabled in the NLP.

## Hard constraints vs CBF

The active implementation is a hard-constrained NMPC formulation.

Hard constraints currently include:

1. Initial-state equality
- The first predicted state is constrained to the measured state.

2. Dynamic equalities
- Euler-discretized model equations are enforced at every horizon step.

3. Obstacle-avoidance inequalities
- In the active mode (`oriented-rectangle`), obstacle clearance is enforced between circular obstacles and a yawed rectangular vessel footprint.
- This supersedes a pure point-distance model in the default configuration.

4. State/input bounds
- Box constraints on surge/sway/yaw rates, azimuth angles, and shaft states/commands are hard.

5. Actuation-rate and braking constraints
- First-step and consecutive azimuth-rate constraints are hard.
- Deceleration-rate constraint is hard and active.

Selective soft constraints are now included for obstacle inequalities in precision-berthing mode. In nominal transit conditions, obstacle constraints remain hard. Near tight terminal situations, bounded slack variables with high penalty are enabled to preserve feasibility without relaxing safety intent globally.

Thesis framing note:

- This is not a full soft-constraint NMPC. It is a phase-dependent feasibility safeguard: hard constraints by default, selective softening only in constrained endgame conditions, with explicit slack logging for transparency.

A natural extension remains CBF integration, for example:

$$
h(x_{k+1}) - (1-\gamma) h(x_k) \ge 0, \quad \gamma \in (0,1]
$$

Practical options:

- Substitution: replace geometric inequalities with CBF constraints.
- Hybrid: keep geometric hard constraints and add CBF constraints.
- Soft-CBF: introduce slacks $s_k \ge 0$ with strong penalty to improve robustness near infeasible bottlenecks.

## Moving and actuating limits

Guidance and NMPC are coordinated, but not identical in how limits are applied. NMPC and plant integration enforce the hard physical floors/ceilings, while guidance shapes requested speed/heading.

Main limits in the current setup:

1. Surge speed (NMPC hard bound)
- $u_{min} = 0.5$ m/s
- $u_{max} = 12$ m/s

2. Sway speed
- $v_{min} = -3$ m/s
- $v_{max} = +3$ m/s

3. Yaw rate
- $r_{min} = -0.15$ rad/s
- $r_{max} = +0.15$ rad/s

4. Shaft speeds (both azipods)
- $n_{min} = -80$ rpm
- $n_{max} = 160$ rpm

5. Azimuth angles
- $\alpha_{min} = -\pi$
- $\alpha_{max} = +\pi$

6. Shaft commands
- Same bounds as shaft states: $[-80,160]$ rpm

Moving/actuation-rate constraints:

1. Azimuth steering rate limit
- $\dot{\alpha}$ limited to $\pm 0.21$ rad/s in NMPC.

2. First-step azimuth continuity
- First control step constrained against previous applied command.

3. Shaft first-order dynamics
- $\dot{n} = (n_{cmd} - n)/T_m$

4. Shaft acceleration hard clipping
- $\dot{n}$ clipped to $[-10, +10]$ rpm/s.

5. Shaft time constant scheduling
- $T_m = 5.65/(|n|/60)$ for $|n| > 18$ rpm, else $T_m = 18.83$ s, bounded to $[1,20]$ s.

6. Heading kinematics
- $\dot{\psi} = r$

Guidance implications:

- Guidance should avoid demanding curvature that conflicts with azimuth-rate limits.
- Near constrained zones, speed reduction supports feasible turning.
- Consistent obstacle-clearance policy across guidance and NMPC remains important.

## Practical run observation (2026-04-10)

In the tested dynamic-obstacle scenario, the vessel did not execute a strict turn-then-translate maneuver. Heading changed while forward translation was already active, allowing residual lateral drift after bypass.

Observed failure mode:

- Obstacle avoidance succeeded, but corridor recapture failed.
- Waypoint progression and reference recapture became misaligned; cross-track error grew and map collision occurred.

Validation outcome:

- The deceleration-rate constraint was active and numerically respected.
- Logged brake-rate margin remained nonnegative in that run.

Interpretation:

- Remaining issue is mainly guidance/reference recapture robustness, not braking-constraint correctness.

## Recent substantial architecture update (2026-04-13)

To improve robustness in the final docking leg, the controller was updated with a phase-aware recovery mechanism and explicit maneuver staging. This is a structural change, not a one-off parameter retune.

1. Explicit transit-to-berth phase logic
- The maneuver is now split into Phase A (transit) and Phase B (precision berth) using a horizon-based switching radius

$$
R_s = T_{hor}\sqrt{u_{max}^2 + v_{max}^2}
$$

- Phase B uses stricter near-berth behavior (collision model and obstacle policy), while Phase A stays lighter for runtime efficiency.

2. Trend-based missed-approach detection and recovery
- Final-leg monitoring now uses the trend of distance-to-final (increase/decrease over consecutive steps), instead of a simple proximity warning.
- If sustained regression is detected, a temporary recovery mode is latched to force direct-goal recapture with bounded speed and higher yaw authority.
- Recovery is automatically released after sustained improvement, preventing permanent mode lock-in.

3. Why this matters for thesis claims
- The update demonstrates supervisory resilience on top of NMPC: when nominal guidance degrades in cluttered terminal geometry, the system applies a controlled fallback policy while keeping the same core optimizer.
- This supports a stronger claim of operational robustness under realistic harbor uncertainty, rather than only nominal tracking performance.

## About the actuation motors mounted

Even though the present azipod-based setup works under the current limits, high-variability yaw-demand scenarios may be better matched by alternative propulsion concepts (for example, ABB Dynafin-class solutions). This can be discussed as a forward-looking engineering observation rather than a core thesis claim.

## Realism-oriented modifications (model and controller)

The following updates improved physical realism and operational plausibility in the controller/model stack.

1. Vessel collision geometry upgraded from point model to oriented rectangular footprint
- Collision checking uses a yawed rectangle aligned with vessel heading.
- Active footprint is a scaled operational hitbox based on nominal $175 \times 25.4$ m geometry (currently 50% scale).
- This switch is a clear runtime breakpoint: the point model used one distance test per obstacle, while the rectangle model adds heading-dependent rotation and nonlinear footprint math at every prediction node, so solve time increases consistently before any later tuning.
- Implemented in: `run_nmpc.m`, `NMPC_Container_final.m`.

2. Forward-motion realism enforced through hard constraints
- NMPC imposes a hard lower bound on surge speed ($u \ge 0.5$ m/s in current active settings).
- The RK4 plant integration step also enforces a matching floor.
- Implemented in: `run_nmpc.m`, `NMPC_Container_final.m`.

3. Actuation realism via effort and reverse-motion penalties
- Actuator effort term penalizes excessive shaft activity.
- Backward-motion penalty discourages reverse bias while retaining feasibility when needed.
- Implemented in: `NMPC_Container_final.m`.

4. Azimuth continuity at first step
- Azimuth-rate limits are enforced both across horizon steps and from previous applied input to first predicted move.
- Implemented in: `NMPC_Container_final.m`.

5. Terminal behavior aligned with large-vessel practice
- Terminal precision behavior, low-speed heading handling, and soft capture gate are implemented.
- Implemented in: `run_nmpc.m`.

6. Harbor-environment coupling retained
- Map-aware sampling and dynamic-obstacle packaging/replay checks are integrated.
- Implemented in: `run_nmpc.m`.

Explicitly excluded from this realism changelog:

- Waypoint coordinate edits
- Test-specific obstacle placements/headings/speeds
- Scenario reshaping performed only for isolated experiments

## Tight corridor mode

The architecture contains tight-corridor detection and corridor-specific behavior (speed capping and reference-shaping adjustments). In the current default run configuration, this mode is present but disabled (`enable_tight_corridor_mode = false`).

This can still be justified as a safety-performance extension provided transitions preserve deterministic behavior and identical hard safety constraints.

## About what I tried with Test 20

The narrow-passage collision is interpreted as a feasibility limitation rather than insufficient aggressiveness. Even with tighter behavior, hard geometric and safety constraints can reduce effective navigable width below required maneuvering envelope. This supports adding supervisory infeasibility handling (slowdown, hold, or replanning) instead of forcing unsafe traversal.

## Reducing solve time and computational complexity

Solve-time reduction was mainly achieved by reducing NLP size and online obstacle-load complexity:

- Horizon reduced to 25 steps.
- Active map-obstacle slots reduced to 3.
- Map sampling density reduced.
- IPOPT stopping relaxed for real-time use.
- Final-leg logic stabilized to reduce control churn.

Combined effect: substantial runtime improvement while preserving closed-loop operation.

### Runtime breakpoint from point to rectangle model

- The earlier point-ship formulation was cheaper because each obstacle only needed a single Euclidean clearance inequality per prediction node.
- The current yawed-rectangle formulation rotates each obstacle into the vessel frame and evaluates a footprint-aware clearance, which adds trigonometric and piecewise nonlinear work to every NLP solve.
- That means the big runtime jump happens at the formulation switch itself; the optimization fixes below were applied afterward and mainly recover time from that heavier baseline.
- After the switch, runtime sensitivity is dominated by horizon length, obstacle count, and hull/clearance size. Increasing any of them compounds the rectangle-model cost.

### Optimization checklist and implementation status

The following runtime-oriented ideas were tracked and should be stated explicitly with implementation status:

1. Horizon/constraint-size reduction (implemented)
- Horizon reduced from larger experimental values to $N=25$ in active runs.
- Active map obstacle slots are capped (`max_map_obstacles = 3`), and total obstacle slots are bounded before NLP build.

2. Persistent solver object (implemented)
- The CasADi/IPOPT solver is built once per run and reused at each control step.
- Rebuilds are avoided unless solver configuration changes; this removes repeated symbolic/NLP construction overhead during the loop.

3. MATLAB preallocation and bounded online packaging (implemented)
- Main trajectory, control, diagnostics, and timing arrays are preallocated.
- Obstacle packaging/selection is bounded and deterministic, reducing dynamic memory churn.

4. Warm start, primal and dual (implemented)
- Primal warm start: previous solution is shifted and reused as next initial guess (`x0`).
- Dual warm start: previous IPOPT multipliers (`lam_x`, `lam_g`) are reused through (`lam_x0`, `lam_g0`) when dimensions match.
- This decreases average iterations when consecutive NMPC problems are similar.

5. Tolerance relaxation for real-time behavior (implemented)
- IPOPT convergence settings are relaxed relative to strict offline optimization (`tol = 2e-3`, `acceptable_tol = 2e-2`, bounded iterations).
- Control quality remained acceptable for tested harbor scenarios.

6. Practical lower bound on runtime (partially addressed, still relevant)
- Even with the above improvements, very low-latency operation (<1 s/step) is generally unlikely without further simplification.
- Remaining levers are mainly algorithmic/model-level: shorter horizon, fewer obstacle constraints, reduced model complexity, or stronger hardware.

## Braking constraint for fuel-cost optimization

A hard deceleration-rate constraint is included to discourage aggressive brake-then-accelerate behavior and improve operational realism/fuel efficiency:

$$
u_k \ge u_{k-1} - \Delta t \, a_{\max,brake}
$$

where $a_{\max,brake}$ is currently set to 0.3 m/s$^2$ by default.

The constraint is applied:

- At the first effective transition relative to the current measured state
- Between consecutive horizon states

This supports smoother longitudinal speed transitions during obstacle avoidance and final approach.

## Explicit two-phase berthing control (implemented)

The controller now uses an explicit two-phase maneuver policy:

1. Phase A: transit
- Objective: efficient progression along route segments.
- NMPC model: lighter obstacle formulation (point-collision constraints) with lower map-obstacle slot count.
- Role: keep compute time low while preserving obstacle-aware behavior.

2. Phase B: precision berth
- Objective: accurate terminal convergence (position and heading) with smooth low-speed control.
- NMPC model: strongest obstacle geometry (oriented rectangular hull) and increased map-obstacle slot count.
- Cost shaping: increased terminal position/heading importance and stronger yaw/control-rate smoothing.

The phase switch is explicit and logged online using a speed-authority radius:

$$
R_s = T_{hor}\sqrt{u_{max}^2 + v_{max}^2}
$$

where $T_{hor}=N\Delta t$.

This switch also includes hysteresis to avoid phase-chattering near the boundary.

### Why this improves berthing

1. Better real-time tradeoff
- Transit keeps a lighter NLP where strict berth-level geometry is not yet required.
- Precision phase spends computation where it matters most: close to terminal constraints.

2. Better terminal precision
- Increasing terminal position/heading priority in Phase B improves final alignment quality.
- Stronger rate smoothing reduces aggressive late-stage heading oscillations.

3. Better safety in confined final approach
- Phase B enforces full-hull-aware obstacle constraints against map and dynamic obstacle packaging.
- This is more realistic than center-point clearance near quays and narrow corridors.

4. Reduced low-speed dithering
- A dedicated Phase B surge floor avoids ultra-slow creeping and repeated stop-go recapture behavior.

## Selective soft obstacle constraints near berth (implemented)

To mitigate local infeasibility events near dock walls and cluttered terminal geometry, obstacle constraints support selective softening via nonnegative slack variables:

$$
h_{obs}(x_k) + s_k \ge 0, \quad s_k \ge 0
$$

with a large quadratic penalty:

$$
J \leftarrow J + w_s \sum_k s_k^2, \quad w_s \gg 1
$$

Activation policy (runtime):

- Enabled only in Phase B and near berth radius, or after a short NMPC failure streak.
- Disabled elsewhere by clamping slack upper bounds to zero.

Safety diagnostics now log:

- Maximum slack used per step
- Cumulative slack usage over run

Interpretation in thesis:

- Near-zero slack over most runs indicates hard-constraint feasibility is retained.
- Nonzero slack bursts indicate localized infeasibility pressure (useful for scenario diagnosis, not for routine operation).

## Validation status for this update

The updated implementation was checked with MATLAB Code Analyzer and executed through MCP in a shortened run configuration. The run confirmed:

- Dual solver build (Phase A and Phase B configurations)
- Explicit phase-switch radius reporting
- Runtime phase labeling in progress logs
- Soft-slack metrics available in summary output
