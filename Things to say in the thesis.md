## About the distinction between past work and my work

"This work presents an integrated motion planning and control framework for autonomous ship docking. Unlike traditional two-stage approaches where a global planner generates a fixed collision-free path that a separate controller blindly tracks, the proposed method employs a Nonlinear Model Predictive Controller that jointly performs trajectory generation and tracking by optimizing control inputs online while explicitly enforcing obstacle-avoidance constraints. A lightweight guidance layer provides waypoint sequencing and geometric direction (segment heading and optional terminal heading); speed shaping is handled inside NMPC costs and constraints, and the NMPC retains authority to deviate from the reference trajectory when necessary to satisfy safety constraints and actuator limitations."

## Important claim

Does the NMPC also do trajectory generation?
Yes — this is actually one of your strongest claims. This is important to articulate clearly.
In a classical pipeline:
Path planner → Trajectory generator → Trajectory tracker → Controller
In your framework:
Waypoints + berth target → NMPC solves optimal X_pred* → Apply u_opt(0)
The predicted state sequence X_pred (N+1 states × N steps) IS the trajectory, generated and optimized online at every step subject to:
Full nonlinear 9-state ship dynamics (not a simplified model)
Tube corridor constraints (geometric path)
CBF obstacle/map safety constraints
Terminal berth pose + velocity envelope
There is no separate trajectory generator. The NLP solution simultaneously decides where to go and how to get there, respecting all physics and constraints. This is genuinely not just "following" — it is trajectory optimization with receding horizon, which collapses planning and control into one problem.
This is arguably the clearest "unified" aspect of your framework, and worth highlighting explicitly in the paper.

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
- Computes the active segment geometry and desired segment heading
- Provides optional terminal heading and soft cruise preference

It does not solve a global constrained optimization problem. Collision-avoidance feasibility, speed shaping, and dynamic feasibility are handled by NMPC.

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

- The prediction horizon is kept moderate for real-time use.
- Active map-obstacle slots are capped to a small, runtime-safe number.

At solver level (IPOPT):

- Bounded iteration count (`max_iter = 120`)
- Relaxed convergence settings (`tol = 2e-3`, `acceptable_tol = 2e-2`, `acceptable_iter = 3`)
- Adaptive barrier strategy for robust real-time behavior

At geometric/environment level:

- Harbor map is represented through bounded primitives; half-plane edges are the default, with optional circle sampling when needed.
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
- In the active mode, obstacle clearance is enforced between circular obstacles and a yawed rectangular vessel footprint, with optional map half-plane constraints.
- This is enforced across the horizon without external override logic.

4. State/input bounds
- Box constraints on surge/sway/yaw rates, azimuth angles, and shaft states/commands are hard.

5. Actuation-rate and braking constraints
- First-step and consecutive azimuth-rate constraints are hard.
- Deceleration-rate constraint is hard and active.

Selective soft constraints can be enabled for obstacle inequalities in terminal or high-clutter scenarios. In nominal transit conditions, obstacle constraints remain hard. When enabled, bounded slack variables with high penalty preserve feasibility without relaxing safety intent globally.

Thesis framing note:

- This is not a full soft-constraint NMPC. It is a configuration-dependent feasibility safeguard: hard constraints by default, selective softening only in constrained endgame conditions, with explicit slack logging for transparency.

A natural extension remains CBF integration, for example:

$$
h(x_{k+1}) - (1-\gamma) h(x_k) \ge 0, \quad \gamma \in (0,1]
$$

Practical options:

- Substitution: replace geometric inequalities with CBF constraints.
- Hybrid: keep geometric hard constraints and add CBF constraints.
- Soft-CBF: introduce slacks $s_k \ge 0$ with strong penalty to improve robustness near infeasible bottlenecks.

## Moving and actuating limits

Guidance and NMPC are coordinated, but not identical in how limits are applied. Guidance provides geometric targets (segment and optional terminal heading), while NMPC and plant integration enforce the hard physical floors/ceilings and the soft speed shaping inside the optimizer.

Main limits in the current setup:

1. Surge speed (NMPC hard bound)
- The lower bound is scenario-configurable: forward-biased by default, with reverse capability enabled when needed near terminal maneuvers.
- The upper bound remains hard.

2. Sway speed
- $v_{min} = -3$ m/s
- $v_{max} = +3$ m/s

3. Yaw rate
- Yaw rate remains hard-bounded in NMPC.

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

## Legacy (pre-May) run observation (2026-04-10)

This observation comes from the pre-May branch when external recapture logic was still present. It is retained here for contrast with the current NMPC-only approach.

In the tested dynamic-obstacle scenario, the vessel did not execute a strict turn-then-translate maneuver. Heading changed while forward translation was already active, allowing residual lateral drift after bypass.

Observed failure mode:

- Obstacle avoidance succeeded, but corridor recapture failed.
- Waypoint progression and reference recapture became misaligned; cross-track error grew and map collision occurred.

Validation outcome:

- The deceleration-rate constraint was active and numerically respected.
- Logged brake-rate margin remained nonnegative in that run.

Interpretation:

- Remaining issue is mainly guidance/reference recapture robustness, not braking-constraint correctness.

## Legacy (pre-May) architecture update (2026-04-13)

In the pre-May branch, the controller was updated with a phase-aware recovery mechanism and explicit maneuver staging. This was a structural change at the time, but these supervisor features are removed in the current NMPC-only approach.

1. Explicit transit-to-berth phase logic
- The maneuver was split into Phase A (transit) and Phase B (precision berth) using a horizon-based switching radius

$$
R_s = T_{hor}\sqrt{u_{max}^2 + v_{max}^2}
$$

- Phase B used stricter near-berth behavior (collision model and obstacle policy), while Phase A stayed lighter for runtime efficiency.

2. Trend-based missed-approach detection and recovery
- Final-leg monitoring used the trend of distance-to-final (increase/decrease over consecutive steps), instead of a simple proximity warning.
- If sustained regression was detected, a temporary recovery mode latched to force direct-goal recapture with bounded speed and higher yaw authority.
- Recovery was automatically released after sustained improvement, preventing permanent mode lock-in.

3. Why this mattered for thesis claims (pre-May)
- The update demonstrated supervisory resilience on top of NMPC: when nominal guidance degraded in cluttered terminal geometry, the system applied a controlled fallback policy while keeping the same core optimizer.
- This supported a stronger claim of operational robustness under realistic harbor uncertainty, rather than only nominal tracking performance.

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
- NMPC uses a configurable surge lower bound: forward-biased by default, reverse-capable when enabled for terminal maneuvers.
- Plant integration can be configured to respect the same bound.
- Implemented in: `run_nmpc.m`, `NMPC_Container_final.m`.

3. Actuation realism via effort and reverse-motion penalties
- Actuator effort term penalizes excessive shaft activity.
- Backward-motion penalty discourages reverse bias while retaining feasibility when needed.
- Implemented in: `NMPC_Container_final.m`.

4. Azimuth continuity at first step
- Azimuth-rate limits are enforced both across horizon steps and from previous applied input to first predicted move.
- Implemented in: `NMPC_Container_final.m`.

5. Terminal behavior aligned with large-vessel practice
- Terminal behavior is now enforced primarily via NMPC terminal costs and constraints rather than external capture gates.
- Implemented in: `NMPC_Container_final.m` (with minimal run-script plumbing).

6. Harbor-environment coupling retained
- Map-aware sampling and dynamic-obstacle packaging/replay checks are integrated.
- Implemented in: `run_nmpc.m`.

Explicitly excluded from this realism changelog:

- Waypoint coordinate edits
- Test-specific obstacle placements/headings/speeds
- Scenario reshaping performed only for isolated experiments

## Legacy (pre-May) tight corridor mode

The pre-May branch contained tight-corridor detection and corridor-specific behavior (speed capping and reference-shaping adjustments). This external mode is not present in the current NMPC-only branch; corridor behavior is now handled through NMPC costs and constraints.

## Legacy (pre-May) note on Test 20

The narrow-passage collision was interpreted as a feasibility limitation rather than insufficient aggressiveness. Even with tighter external behavior, hard geometric and safety constraints can reduce effective navigable width below the required maneuvering envelope. This supported supervisory infeasibility handling in the pre-May branch; the NMPC-only branch relies on internal costs/constraints and does not use external corridor logic.

## Reducing solve time and computational complexity

Solve-time reduction was mainly achieved by reducing NLP size and online obstacle-load complexity.

Pre-May (legacy) levers:

- Horizon and obstacle-slot counts were aggressively reduced (e.g., 25-step horizon, 3 active map slots in some runs).
- Map sampling density was reduced.
- IPOPT stopping was relaxed for real-time use.
- Final-leg external logic was simplified to reduce control churn.

Post-May (NMPC-only) levers:

- Horizon is kept moderate and scenario-tuned (no external logic required for stability).
- Obstacle slots are bounded and packaging is deterministic.
- Warm start (primal/dual) and relaxed tolerances remain central to real-time behavior.

Combined effect: runtime is higher than the legacy branch but is more principled and reproducible across scenarios.

### Runtime breakpoint from point to rectangle model

- The earlier point-ship formulation was cheaper because each obstacle only needed a single Euclidean clearance inequality per prediction node.
- The current yawed-rectangle formulation rotates each obstacle into the vessel frame and evaluates a footprint-aware clearance, which adds trigonometric and piecewise nonlinear work to every NLP solve.
- That means the big runtime jump happens at the formulation switch itself; the optimization fixes below were applied afterward and mainly recover time from that heavier baseline.
- After the switch, runtime sensitivity is dominated by horizon length, obstacle count, and hull/clearance size. Increasing any of them compounds the rectangle-model cost.

### Optimization checklist and implementation status

The following runtime-oriented ideas were tracked and should be stated explicitly with implementation status:

1. Horizon/constraint-size reduction (implemented)
- The horizon is kept moderate for real-time operation.
- Map obstacle slots are bounded before NLP build.

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

## Legacy (pre-May) explicit two-phase berthing control

The pre-May controller used an explicit two-phase maneuver policy implemented outside the core NMPC formulation. The current NMPC-only branch does not use external phase switching; terminal behavior is encoded through NMPC costs and constraints.

1. Phase A: transit
- Objective: efficient progression along route segments.
- NMPC model: lighter obstacle workload for open-water progress.
- Role: keep compute time low while preserving obstacle-aware behavior.

2. Phase B: precision berth
- Objective: accurate terminal convergence (position and heading) with smooth low-speed control.
- NMPC model: stronger terminal focus with oriented rectangular hull constraints.
- Cost shaping: increased terminal position/heading importance and stronger yaw/control-rate smoothing.

3. PHASE_BERTH: terminal docking
- Objective: final capture and heading alignment with a terminal-specific corridor.
- Role: provide a tighter endgame stage when the route reaches the final docking area.

The phase switch was explicit and logged online using a speed-authority radius:

$$
R_s = T_{hor}\sqrt{u_{max}^2 + v_{max}^2}
$$

where $T_{hor}=N\Delta t$.

This switch also included hysteresis to avoid phase-chattering near the boundary.

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

## Selective soft obstacle constraints near berth (current)

To mitigate local infeasibility events near dock walls and cluttered terminal geometry, obstacle constraints support selective softening via nonnegative slack variables:

$$
h_{obs}(x_k) + s_k \ge 0, \quad s_k \ge 0
$$

with a large quadratic penalty:

$$
J \leftarrow J + w_s \sum_k s_k^2, \quad w_s \gg 1
$$

Activation policy (runtime):

- Enabled only under explicitly configured terminal conditions (no external phase gate is required).
- Disabled elsewhere by clamping slack upper bounds to zero.

Safety diagnostics now log:

- Maximum slack used per step
- Cumulative slack usage over run

Interpretation in thesis:

- Near-zero slack over most runs indicates hard-constraint feasibility is retained.
- Nonzero slack bursts indicate localized infeasibility pressure (useful for scenario diagnosis, not for routine operation).

## Legacy (pre-May) validation status for phase logic

The pre-May implementation was checked with MATLAB Code Analyzer and executed through MCP in a shortened run configuration. The run confirmed:

- Multiple solver configurations for transit, berth, and terminal docking
- Explicit phase-switch radius reporting
- Runtime phase labeling in progress logs
- Soft-slack metrics available in summary output

## Legacy (pre-May) updates after initial two-phase rollout (2026-04-16)

This subsection documents two substantial control-architecture updates that were implemented, tested, and iteratively corrected in the pre-May branch.

### Update 1: phase-dependent reverse-motion policy (forward-biased transit, reverse-capable berth)

### Motivation

The previous setup effectively behaved as forward-only in too many situations. This was caused by a combination of lower-bound constraints and integration-level surge clamping. As a result, the vessel could become over-constrained in terminal maneuvers where short reverse actions are physically useful.

### What was changed

1. Hard forward-only behavior was removed from plant integration.
- The RK4 step no longer applies a forced per-step surge floor, so simulated plant dynamics can realize reverse commands when NMPC selects them.

2. Reverse capability was made phase-dependent instead of globally enabled.
- Phase A (transit): a strict forward lower bound is retained ($u_{min}=0.5$ m/s) to preserve navigation progress.
- Phase B (precision berth): per-step surge lower-bound override allows reverse ($u_{min}$ down to approximately $-1.2$ m/s) when needed.

3. Cost shaping was split by phase.
- Transit keeps stronger forward incentive.
- Berth reduces that penalty to allow controlled reverse/lateral repositioning.

4. Guidance remained forward-oriented.
- Reverse authority was intentionally placed in NMPC (actuation-level decision), not in waypoint-guidance geometry.

### Technical rationale for thesis

This is a constrained-hybrid maneuver policy, not a simple "allow reverse" toggle:

- Progress phase uses directional bias for efficiency and predictability.
- Terminal phase relaxes directional bias for feasibility and precision.

This supports the claim that the controller uses context-dependent constraint activation rather than one global motion rule.

### Update 2: explicit terminal pose constraints with soft feasibility and terminal heading sourcing

### Motivation

Distance-only terminal capture was not sufficient for precise docking claims. Final position/orientation regulation needed to be represented directly in the optimization problem.

### What was changed in NMPC formulation

At the terminal prediction node ($k=N+1$), explicit inequalities were added for:

- Position envelope:
	$$|x_N - x_d| \le \epsilon_x, \quad |y_N - y_d| \le \epsilon_y$$
- Heading envelope:
	$$|\mathrm{wrap}(\psi_N-\psi_d)| \le \epsilon_\psi$$
- Optional terminal-velocity envelope:
	$$|u_N|\le u_{max}^{term}, \ |v_N|\le v_{max}^{term}, \ |r_N|\le r_{max}^{term}$$

The constraints are softened through bounded nonnegative slack variables with strong quadratic penalty:

$$
J \leftarrow J + w_{term}\,\|s_{term}\|_2^2,\quad 0 \le s_{term} \le s_{max}
$$

This preserves solver feasibility in tight terminal geometry while still driving the optimizer toward strict terminal compliance.

### Activation strategy

- Terminal-pose constraints are configured as Phase-B-only by default.

## Legacy (pre-May) scenario-adaptive Phase-B triggering (2026-04-17)

### Why this change was needed (pre-May)

In busy harbor scenarios, relying only on "final-leg proximity" to enter precision mode can be too late or inconsistent. Some routes include several intermediate waypoints before the final berth corridor, and constrained navigation may already require berth-grade caution and authority.

### Implemented change (pre-May)

Phase-B activation was extended from purely terminal-distance logic to a hybrid trigger:

1. Final-approach trigger (existing)
- Enter precision mode when final-leg and terminal-distance conditions are met.

2. Tight-scenario trigger (new)
- Enter precision mode when online corridor tightness is detected (local clearance/crowding criterion).
- Keep precision mode latched for a short hold window (hysteresis in steps) to avoid mode flapping.

Conceptually, the phase decision can be expressed as:

$$
	ext{PhaseB} = \text{FinalApproachTrigger} \;\lor\; \text{TightScenarioTrigger}
$$

with a bounded hold counter for robustness against noisy switching.

### Additional control-policy refinements tied to this update (pre-May)

- Reverse allowance in Phase B is no longer globally active at all distances; reverse authority is enabled mainly near terminal distance or under dynamic threat.
- Mid-speed floor policy is retained in clear-water transit to avoid unnecessary crawl.
- Environment override was added for animation recording to improve long-run debugging stability.

### Test evidence (MCP MATLAB runs, pre-May)

Observed across repeated dynamic-obstacle runs:

- Precision mode can now activate earlier in constrained segments (nonzero Phase-B occupancy in runs that previously stayed in Phase A).
- Collision behavior improved in several tests (collision-free runs obtained under the same map/obstacle setup).
- Remaining limitation: final-capture convergence is still scenario-sensitive in some long runs; additional guidance/phase-coupling refinement is still required before claiming fully general terminal convergence.

Thesis interpretation:

- The architecture shifted from terminal-only staging toward context-aware maneuver staging.
- This is a robustness improvement in supervisory logic, but not yet the final convergence solution for all busy-scenario permutations.
- Transit solves keep terminal slack effectively disabled (upper bound zero) to avoid unnecessary burden.

### Terminal heading source integration

Final desired heading is now resolved with explicit precedence:

1. Explicit desired terminal heading override, if enabled.
2. Final waypoint heading in the third column, if provided.
3. Fallback to geometric course-based heading.

Guidance blends into this desired final heading as distance decreases, instead of relying only on course-to-point direction.

### Additional corrective tuning done after validation

During testing, Phase B initially activated too early due to entry-gate logic. The switch condition was corrected so entry is bounded by a final-distance cap, avoiding premature berthing-mode lock and restoring transit behavior before the terminal region.

### Instrumentation added for transparency

To support thesis-level evidence, terminal slack usage is now logged explicitly and summarized in run output:

- per-step maximum terminal slack,
- run-level maximum terminal slack.

This enables objective reporting of when soft feasibility was actually used.

### Validation observations worth reporting

1. The updated implementation consistently solved all NMPC steps in tested runs (no optimizer fallback required in those logs).
2. Terminal slack became active in constrained endgame cases, confirming the new terminal block is engaged.
3. Phase-entry gating strongly affected behavior:
- too-loose gate: early Phase B and unnecessary terminal-mode exposure,
- too-strict gate: delayed/absent Phase B and increased corridor risk,
- corrected capped gate: balanced transition with collision-free 400 s validation run.

### Thesis interpretation (pre-May)

These two updates collectively strengthen the contribution from a control-design perspective:

- Update 1 introduces context-aware directional feasibility (forward-biased transit, reverse-capable berth).
- Update 2 introduces explicit terminal-state regulation (position + heading + optional velocity) with controlled softening and measurable slack diagnostics.

Together, they move the method from "safe waypoint tracking" toward a more rigorous "precision docking under constrained feasibility" formulation.


## Why done in this hierarchical structure and why not MPPF?

"For harbor navigation with dynamic obstacles, berthing corridors, and phase-dependent objectives, a time-parameterized reference provides explicit control over speed profiles, terminal velocity bounds, and corridor constraints. MPPF decouples time from geometry, which complicates dynamic obstacle prediction and berthing phase transitions. Our architecture uses a nominal waypoint reference for trajectory priority, hard NLP constraints for avoidance, and parameterized terminal bounds for berthing—enabling mathematically verifiable priority enforcement while maintaining solver robustness."

## Pre-May / Post-May implementation history and rationale (referring to "revolution" branch)

Note: the following is a consolidated, English translation of development notes and observations. It explains why many outer-loop heuristics were first added and why the project subsequently consolidated behavior into a cleaner NMPC-centric formulation. Early versions relied on heuristic outer guidance to shape heading and speed references.This improved behavior, but it blurred the responsibility split between guidance and control. The final architecture moved most motion generation into the NMPC, leaving only waypoint progression and mode selection outside. This exposed missing elements in the MPC formulation, especially for intermediate waypoint commitment and final berthing geometry. These were then addressed with a waypoint handoff manager, terminal endpoint attraction, and a dedicated berthing mode.

What I verified (summary)

- Step 1 — Point Tracking → Line Tracking
	- Implemented in a line-based NMPC container. New parameters include `P_wp_start`, `P_wp_end` and derived segment geometry (segment vector, length, unit tangent `t_hat`, unit normal `n_hat`). Costs are placed on cross-track error `xte_k`, along-track progress `along_k` and `dist_to_goal`. The time-indexed point reference (`P_xref`) is no longer supplied to the solver.

- Step 2 — Tube MPC
	- Implemented internal tube logic: `xte_penalty = max(0, abs(xte_k) - W_tube)`. XTE penalties apply only outside the tube while progress-along-segment is always active. Thus the corridor is enforced inside the solver (not as an external mechanism).

- Step 3 — Elimination of external speed governor
	- The new clean `run_nmpc` run has no speed governor blocks, no `applySpeedGovernor` calls and no external logic that rewrites `U_d`. The solver receives only `soft_speed_cap_mps` and `soft_speed_cap_weight`. Speed is therefore left to NMPC + CBF/constraints.

What I removed from the main script (clean)

- Removed active dependencies and ad-hoc helpers: `speed_governor`, `avoid_ref_cfg`, `xte_recovery_cfg`, `dynamic_threat_cfg`, `tight_corridor_mode`, `missed_approach_cfg`, `buildSimpleRef8`, `applySpeedGovernor`. These are not present as active logic in the cleaned main script.

How the clean `run_nmpc` now works

- Loop responsibilities are minimal and environment-facing:
	- select the active segment between waypoints
	- build `path_ref.wp_start`, `path_ref.wp_end`, optional `path_ref.goal_heading`
	- collect obstacles and half-planes
	- pass geometry and a `line+tube` cost plus soft cruise cap and terminal cost to the solver
	- integrate the plant, check collisions and mission success

This produces a purer NMPC structure: environment and simulation plumbing remain in the script; motion decision making lives in the optimizer.

Why this migration was done (computational trade-off)

- Moving decisions into NMPC increased compute time substantially. During development pre-computation and outer heuristics were considered to reduce runtime; however those external fixes often conflicted with each other and were only optimal for narrow test cases. Iterative tests showed the heuristics primarily signalled which costs/constraints the NMPC formulation lacked. Consequently the decision was to formulate those behaviors (more fully) inside NMPC and accept the increased solver cost while applying solver-level optimizations (warm starts, bounded obstacle slots, relaxed tolerances).

- Core components were preserved. The one-by-one fixes were not wasted; they diagnosed requirements. The key change is where the intelligence lives: less in `run_nmpc`, more inside the NMPC problem (costs, constraints, terminal conditions).

What stayed good and was kept in spirit:

- Ship model and actuator physics
- Obstacle handling and map / half-plane safety
- Hull collision checks and terminal handling
- Logging, diagnostics, animation and fallback safety plumbing

This is the correct split: environment + simulation in the script, motion decision making in NMPC.

How NMPC now covers previously external behaviors

1) Minimum speed
- No large outer heuristic block. The solver uses `u_min_forward` and a `max_brake_rate` concept; near final approach bounds can be relaxed. Slowing now emerges because obstacle constraints make fast solutions infeasible rather than because an external governor forces a lower speed.

2) "Keeping the bow straight"
- Previously partly implemented from outside; now emerges from the optimizer via path-following objective, actuator effort penalties and rate smoothing. If strict heading lock is required it should be encoded as a terminal/phase objective, otherwise stable alignment is produced by the cost design.

3) Guidance
- Guidance remains but is reduced: it chooses the active segment, advances waypoint index and optionally provides final heading. It no longer performs speed shaping or heavy motion control.

4) Speed planning near obstacles
- Instead of a custom governor, the mechanism is: soft cruise encouragement + hard/soft safety constraints. Braking and slowdown emerge naturally when fast trajectories are infeasible or expensive.

5) Final stopping
- Formerly enforced by outer logic; now represented by terminal position/heading/velocity costs and optional soft terminal constraints with bounded slack.

Honest assessment

- The external heuristics were useful: they made the system work while revealing the optimizer's missing elements. The correct engineering path was:
	- make it work with external fixes
	- observe recurring failure modes
	- reformulate the NMPC (add costs/constraints) so heuristics become unnecessary

Important developer note

- I had to substantially reduce or replace `simpleWaypointGuidance` with only geometric indication of direction to give NMPC full freedom. This limits how tightly external code can control motion and explains why many assistive functions were originally added in the main script.

Suggested code / documentation refactor

- When writing results, separate artifacts and experiments into `previous_to_may` (legacy heuristics, intermediate fixes, experiment logs) and `post_may` (clean NMPC-centric implementation, production experiments). This will make comparisons and plots clearer for the thesis.

What you can see in the current implementation

- A pure line+tube NMPC pipeline: segment selection, tube cost, soft cruise cap, terminal cost, hull-aware collision handling, CBF obstacle constraints, and diagnostics for slack/phase/success. Solver warm-starting and bounded obstacle slots are in place to mitigate the increased solve time.


## lambda update 05-05

\documentclass[11pt]{article}
\usepackage{amsmath, amssymb, geometry, hyperref, booktabs}
\geometry{margin=1in}
\title{NMPC Continuous Geometry-Aware Scheduler \\ \large Update Notes \& Thesis Reference}
\author{Thesis Project}
\date{\today}
\begin{document}
\maketitle

\section*{1. Overview}
Replaced discrete mode-switching logic (\texttt{if berth\_mode\_active / elseif on\_final\_waypoint}) with a \textbf{continuous caution scheduling layer} that smoothly adapts NMPC costs, bounds, and constraints based on real-time geometry, clearance, and berth proximity. The NMPC container remains \textbf{completely unchanged}; all adaptation occurs in \texttt{run\_nmpc.m} before the solver call.

\section*{2. Core Concept: Continuous Activation Scalars}
Four independent scalars $\lambda \in [0,1]$ are computed at each control iteration:
\begin{itemize}
    \item $\lambda_{\text{tight}}$: Map clearance \& channel-width awareness
    \item $\lambda_{\text{stop}}$: Kinematic stopping-margin awareness
    \item $\lambda_{\text{turn}}$: Upcoming route curvature severity
    \item $\lambda_{\text{berth}}$: Proximity to final docking zone
\end{itemize}
Combined via a max-blend (default) or weighted sum:
\[
\lambda_{\text{total}} = \max\!\big(\lambda_{\text{tight}},\; \lambda_{\text{stop}},\; \lambda_{\text{turn}},\; \lambda_{\text{berth}}\big)
\]

\section*{3. Mathematical Formulations}
\subsection*{Helper Functions}
\[
\text{sat}_{01}(x) = \max\!\big(0, \min(1, x)\big), \quad
\text{ramp}_{01}(x; x_0, x_1) = \text{sat}_{01}\!\left(\frac{x - x_0}{x_1 - x_0}\right)
\]
\[
\text{revRamp}_{01}(x; x_{\text{lo}}, x_{\text{hi}}) = \text{sat}_{01}\!\left(\frac{x_{\text{hi}} - x}{x_{\text{hi}} - x_{\text{lo}}}\right)
\]
\textit{Note:} \texttt{revRamp01} returns $1$ when $x \leq x_{\text{lo}}$, and $0$ when $x \geq x_{\text{hi}}$.

\subsection*{Clearance / Tightness}
\[
d_{\text{edge}} = \min_{e \in \mathcal{E}} \text{dist}(\mathbf{p}, e), \quad
\lambda_{\text{tight}} = \max\!\Big(\text{revRamp}_{01}(d_{\text{edge}}; d_{\text{lo}}, d_{\text{hi}}),\;
\text{revRamp}_{01}(2d_{\text{edge}}; w_{\text{lo}}, w_{\text{hi}})\Big)
\]

\subsection*{Stopping-Distance Awareness}
\[
d_{\text{stop}} = \frac{U^2}{2 a_{\text{brake, eff}}} + d_{\text{margin}}, \quad
d_{\text{free}} = \min\!\big(d_{\text{seg\_end}}, d_{\text{berth\_entry}}, d_{\text{corridor\_end}}, d_{\text{edge}}\big)
\]
\[
\lambda_{\text{stop}} = \text{revRamp}_{01}\!\big(d_{\text{free}} - d_{\text{stop}};\; 0,\; d_{\text{buffer}}\big)
\]
\textit{Interpretation:} $\lambda_{\text{stop}} \to 1$ when available path length $\leq$ required braking distance.

\subsection*{Turn Severity}
\[
\lambda_{\text{turn}} = \text{ramp}_{01}(\theta_{\text{next}}; \theta_{\text{lo}}, \theta_{\text{hi}}) \cdot 
\text{revRamp}_{01}(d_{\text{to\_end}}; d_{\text{near}}, d_{\text{far}})
\]

\subsection*{Berth Activation}
\[
\lambda_{\text{berth}} = \text{revRamp}_{01}(d_{\text{berth}};\; d_{\text{full}},\; d_{\text{activate}})
\]

\section*{4. Parameter Scheduling}
All NMPC tuneables are linearly interpolated: $P(\lambda) = (1-\lambda)P_{\text{far}} + \lambda P_{\text{near}}$.
\begin{table}[h]
\centering
\begin{tabular}{@{}lll@{}}
\toprule
\textbf{Parameter} & \textbf{Range} & \textbf{Governing $\lambda$} \\
\midrule
Soft Speed Cap ($U_{\text{cap}}$) & $[U_{\text{berth}},\; U_{\text{cruise}}]$ & $\min(U^{\text{tight}}, U^{\text{stop}}, U^{\text{turn}}, U^{\text{berth}})$ \\
Speed Cost Weight & $[0.03,\; 0.70]$ & $\lambda_{\text{total}}$ \\
Forward Lower Bound ($u_{\min}$) & $[0.02,\; 0.30]$ m/s & $\lambda_{\text{total}}$ \\
Path Tube Half-Width & $[10,\; 20]$ m & $\max(\lambda_{\text{tight}}, \lambda_{\text{turn}}, \lambda_{\text{berth}})$ \\
XTE Weight & $[12,\; 18]$ & $\max(\lambda_{\text{tight}}, \lambda_{\text{turn}}, \lambda_{\text{berth}})$ \\
Terminal Heading Weight & $[0,\; 140]$ & $\max(\lambda_{\text{turn}}W_{\text{turn}}, \lambda_{\text{berth}}W_{\text{berth}})$ \\
Terminal Stop Weights ($u,v,r$) & $[0,\; 120/80/80]$ & $\lambda_{\text{berth}}$ \\
Berth Corridor Width & $[12,\; 22]$ m & $\lambda_{\text{berth}}$ \\
\bottomrule
\end{tabular}
\caption{Continuous scheduling ranges used in \texttt{solve\_opts}.}
\end{table}

\section*{5. Architecture Changes}
\begin{itemize}
    \item \texttt{NMPC\_Container\_final.m}: \textbf{Zero changes}. Already accepts runtime \texttt{solve\_opts}.
    \item \texttt{run\_nmpc.m}: 
    \begin{itemize}
        \item Added \texttt{sched\_cfg} configuration block.
        \item Replaced rigid \texttt{if/elseif} mode block with continuous scheduler \textbf{before} \texttt{nmpc.solve()}.
        \item Retained only \textit{hard structural differences} (actuator limits, reverse permission, final waypoint flag).
        \item Added helper functions: \texttt{sat01}, \texttt{ramp01}, \texttt{revRamp01}, \texttt{lerp}, geometry utilities.
        \item Added logging arrays for $\lambda$ trajectories, $U_{\text{cap}}$, $d_{\text{edge}}$, $d_{\text{stop}}$.
    \end{itemize}
\end{itemize}

\section*{6. Thesis Contributions \& Plotting Notes}
\begin{itemize}
    \item \textbf{No State Machine:} Behavior emerges from continuous optimization, not hand-coded modes.
    \item \textbf{Stopping-Margin Awareness:} Vessel slows kinematically when $d_{\text{stop}} \approx d_{\text{free}}$, not by arbitrary distance thresholds.
    \item \textbf{Berth as a Funnel:} Terminal pose, speed, and heading constraints tighten gradually over a spatial band.
    \item \textbf{Recommended Plots:} Overlay $\lambda_{\text{total}}$, $U_{\text{cap}}$, and $x_1$ (surge) vs. time. Show how $\lambda_{\text{stop}}$ rises before a sharp turn or narrow channel, causing predictive deceleration.
    \item \textbf{Tuning:} Adjust \texttt{sched\_cfg} thresholds to match vessel dynamics \& harbor geometry. The structure is robust to moderate parameter shifts.
\end{itemize}
\end{document}

## and the other big update of 05-05

\documentclass[11pt]{article}
\usepackage{amsmath, amssymb, geometry, hyperref, booktabs, algorithm, algorithmic}
\geometry{margin=1in}
\title{NMPC Dynamic Obstacle Avoidance with TTC-Based Yielding \\ \large Continuous Activation Scheduling Update}
\author{Thesis Project}
\date{\today}
\begin{document}
\maketitle

\section*{1. Overview}
Extended the continuous geometry-aware scheduler to handle \textbf{dynamic obstacle interactions} without discrete mode switching. The key innovation is a \textbf{TTC (Time-to-Collision) and CPA (Closest Point of Approach)} based activation system that makes the vessel:
\begin{itemize}
    \item \textbf{Yield naturally} by slowing down when approaching conflicts
    \item \textbf{Avoid early sidestepping} by reducing progress reward during yield
    \item \textbf{Return smoothly} to the route after passing the obstacle
    \item \textbf{Maintain smooth control} by increasing rate penalties during avoidance
\end{itemize}
All behavior emerges from continuous cost shaping—no procedural "avoidance mode" or state machine.

\section*{2. Core Mathematical Formulations}

\subsection*{2.1 Relative Motion Quantities}
For each dynamic obstacle $j$:
\begin{align}
    \mathbf{r} &= \mathbf{p}_{\text{obs}}^{(j)} - \mathbf{p}_{\text{ship}} \quad &\text{(relative position)} \\
    \mathbf{v}_{\text{rel}} &= \mathbf{v}_{\text{obs}}^{(j)} - \mathbf{v}_{\text{ship}} \quad &\text{(relative velocity)} \\
    d_{\text{rel}} &= \|\mathbf{r}\| + \epsilon \quad &\text{(separation distance)} \\
    \dot{d}_{\text{rel}} &= \frac{\mathbf{r}^\top \mathbf{v}_{\text{rel}}}{d_{\text{rel}}} \quad &\text{(separation rate)} \\
    v_{\text{close}} &= \max(0, -\dot{d}_{\text{rel}}) \quad &\text{(closing speed)}
\end{align}

\subsection*{2.2 CPA (Closest Point of Approach) Calculations}
\begin{align}
    t_{\text{CPA}} &= \max\left(0, -\frac{\mathbf{r}^\top \mathbf{v}_{\text{rel}}}{\|\mathbf{v}_{\text{rel}}\|^2 + \epsilon}\right) \\
    d_{\text{CPA}} &= \|\mathbf{r} + t_{\text{CPA}} \mathbf{v}_{\text{rel}}\|
\end{align}
where $t_{\text{CPA}}$ is time to closest approach and $d_{\text{CPA}}$ is predicted separation at CPA.

\subsection*{2.3 Forward Sector Filter}
Only obstacles in the forward interaction cone influence avoidance:
\begin{align}
    \cos\beta &= \frac{\mathbf{r}^\top \mathbf{h}_{\text{ship}}}{d_{\text{rel}}}, \quad \mathbf{h}_{\text{ship}} = [\cos\psi; \sin\psi] \\
    \lambda_{\text{sector}} &= \text{sat}_{01}\left(\frac{\cos\beta - \cos\beta_{\text{max}}}{1 - \cos\beta_{\text{max}} + \epsilon}\right)
\end{align}
Default: $\beta_{\text{max}} = 120^\circ$ (forward $240^\circ$ sector).

\subsection*{2.4 Base TTC Activation}
\begin{align}
    \lambda_{\text{ttc}}^{(j)} &= \underbrace{\text{sat}_{01}\left(\frac{T_{\text{far}} - t_{\text{CPA}}}{T_{\text{far}} - T_{\text{near}} + \epsilon}\right)}_{\text{TTC urgency}} \times \nonumber \\
    &\quad \underbrace{\text{sat}_{01}\left(\frac{d_{\text{CPA,th}} - d_{\text{CPA}}}{d_{\text{CPA,th}} + \epsilon}\right)}_{\text{poor miss distance}} \times \nonumber \\
    &\quad \underbrace{\text{sat}_{01}\left(\frac{v_{\text{close}}}{v_{\text{close,ref}} + \epsilon}\right)}_{\text{closing motion}} \times \lambda_{\text{sector}}
\end{align}
Default parameters:
\begin{itemize}
    \item $T_{\text{far}} = 20$\,s, $T_{\text{near}} = 5$\,s
    \item $d_{\text{CPA,th}} = 60$\,m
    \item $v_{\text{close,ref}} = 3.0$\,m/s
\end{itemize}

\subsection*{2.5 Yield and Return Activations}
\textbf{Yield phase} (approaching obstacle, separation decreasing):
\begin{equation}
    \lambda_{\text{yield}} = \max_j \left[ \lambda_{\text{ttc}}^{(j)} \cdot \text{sat}_{01}\left(\frac{-\dot{d}_{\text{rel}}^{(j)}}{\dot{d}_{\text{ref}} + \epsilon}\right) \right]
\end{equation}

\textbf{Return phase} (post-CPA, separation increasing):
\begin{equation}
    \lambda_{\text{return}} = \max_j \left[ \lambda_{\text{ttc}}^{(j)} \cdot \text{sat}_{01}\left(\frac{\dot{d}_{\text{rel}}^{(j)}}{\dot{d}_{\text{ref}} + \epsilon}\right) \right]
\end{equation}

Default: $\dot{d}_{\text{ref}} = 2.0$\,m/s.

\subsection*{2.6 Master Caution Variable}
\begin{equation}
    \lambda_{\text{total}} = \max\left(\lambda_{\text{tight}}, \lambda_{\text{stop}}, \lambda_{\text{turn}}, \lambda_{\text{berth}}, \lambda_{\text{yield}}\right)
\end{equation}
Dynamic obstacle yield now \textbf{dominates} when conflict is urgent.

\section*{3. Parameter Scheduling for Dynamic Obstacles}

\subsection*{3.1 Speed Cap Shaping}
\begin{align}
    U_{\text{cap}}^{\text{dyn}} &= (1 - \lambda_{\text{yield}}) U_{\text{cruise}} + \lambda_{\text{yield}} U_{\text{yield}} \\
    U_{\text{cap}} &= \min\left(U_{\text{cap}}^{\text{geom}}, U_{\text{cap}}^{\text{dyn}}\right)
\end{align}
where $U_{\text{yield}} = 0.02$\,m/s (near-stop/crawl).

\subsection*{3.2 Progress Reward Reduction}
\begin{equation}
    w_{\text{along}} = (1 - \lambda_{\text{yield}}) w_{\text{along}}^{\text{nom}} + \lambda_{\text{yield}} w_{\text{along}}^{\text{min}}
\end{equation}
Default: $w_{\text{along}}^{\text{min}} = 0.10$ (vs $w_{\text{along}}^{\text{nom}} = 1.2$).

\textbf{Effect:} During yield, the optimizer no longer insists on "keep making progress"—slowing down becomes cheaper than sidestepping.

\subsection*{3.3 Surge Lower Bound Relaxation}
\begin{equation}
    u_{\min}^{\text{eff}} = (1 - \lambda_{\text{yield}}) u_{\min}^{\text{nom}} + \lambda_{\text{yield}} u_{\min}^{\text{yield}}
\end{equation}
Default: $u_{\min}^{\text{yield}} = 0.02$\,m/s (allows near-stop).

\subsection*{3.4 Control Smoothness Scaling}
\begin{align}
    R_{\Delta}^{\text{scale}} &= 1.0 + \lambda_{\text{yield}} (R_{\Delta}^{\text{yield}} - 1.0) \\
    J_{\text{rate}} &= R_{\Delta}^{\text{scale}} \sum_{k} \Delta \mathbf{u}_k^\top \mathbf{R}_{\text{rate}} \Delta \mathbf{u}_k
\end{align}
Default: $R_{\Delta}^{\text{yield}} = 3.5$.

\textbf{Effect:} During yield, sudden azimuth/thrust changes become expensive—prevents jerky "swerve now" behavior.

\subsection*{3.5 Route Recapture After CPA}
When $\lambda_{\text{return}} > 0$:
\begin{align}
    w_{\text{xte}} &= w_{\text{xte}}^{\text{geom}} + \lambda_{\text{return}} \Delta w_{\text{xte}}^{\text{ret}} \\
    W_{\text{tube}} &= (1 - \lambda_{\text{return}}) W_{\text{tube}}^{\text{avoid}} + \lambda_{\text{return}} W_{\text{tube}}^{\text{tight}} \\
    w_{\psi} &= w_{\psi}^{\text{turn/berth}} + \lambda_{\text{return}} \Delta w_{\psi}^{\text{ret}}
\end{align}
Defaults: $\Delta w_{\text{xte}}^{\text{ret}} = 18.0$, $\Delta w_{\psi}^{\text{ret}} = 15.0$.

\textbf{Effect:} After passing the obstacle, the route becomes "magnetic" again—pulls vessel back instead of continuing the detour arc.

\subsection*{3.6 Soft Map Barrier Weight}
\begin{equation}
    w_{\text{map}} = w_{\text{map}}^{\text{base}} + \max(\lambda_{\text{tight}}, \lambda_{\text{return}}) \Delta w_{\text{map}}
\end{equation}
Default: $\Delta w_{\text{map}} = 18.0$.

\textbf{Effect:} Prevents the optimizer from seeing lateral escape to open water as "cheap"—encourages staying within the navigable corridor.

\section*{4. Architecture Changes}

\subsection*{4.1 Container Modifications (\texttt{NMPC\_Container\_final.m})}
\begin{itemize}
    \item Added parameters: \texttt{P\_map\_barrier\_w}, \texttt{P\_R\_rate\_scale}
    \item Modified stage cost loop to include:
    \begin{align*}
        J &\leftarrow J + \texttt{P\_R\_rate\_scale} \cdot \sum \Delta \mathbf{u}^\top \mathbf{R}_{\text{rate}} \Delta \mathbf{u} \\
        J &\leftarrow J + \texttt{P\_map\_barrier\_w} \cdot \sum_k \Phi_{\text{map}}(\mathbf{p}_k)
    \end{align*}
    where $\Phi_{\text{map}}$ is a soft barrier penalty for proximity to map boundaries.
    \item Passed new parameters via \texttt{solve\_opts}: \texttt{R\_rate\_scale\_obs}, \texttt{map\_barrier\_weight}
\end{itemize}

\subsection*{4.2 Run Script Modifications (\texttt{run\_nmpc.m})}
\begin{enumerate}
    \item \textbf{Added scheduler config} for dynamic obstacles:
    \begin{verbatim}
    sched_cfg.T_cpa_far, sched_cfg.T_cpa_near
    sched_cfg.d_cpa_th, sched_cfg.v_close_ref
    sched_cfg.cos_beta_max (sector filter)
    sched_cfg.u_yield, sched_cfg.w_along_min
    sched_cfg.R_rate_scale_yield
    sched_cfg.map_barrier_w_near
    \end{verbatim}
    
    \item \textbf{Replaced lambda block} to compute:
    \begin{verbatim}
    lambda_ttc, lambda_yield, lambda_return
    (in addition to existing tight/stop/turn/berth)
    \end{verbatim}
    
    \item \textbf{Extended scheduling logic} to apply:
    \begin{itemize}
        \item Dynamic speed cap: $U_{\text{cap}}^{\text{dyn}}$
        \item Progress weight: $w_{\text{along}}(\lambda_{\text{yield}})$
        \item Control rate scaling: $R_{\Delta}^{\text{scale}}(\lambda_{\text{yield}})$
        \item Map barrier weight: $w_{\text{map}}(\lambda_{\text{return}})$
        \item XTE/tube/heading recapture: $(\lambda_{\text{return}})$
    \end{itemize}
    
    \item \textbf{Added logging arrays}:
    \begin{verbatim}
    lambda_yield_log, lambda_return_log, lambda_ttc_log
    \end{verbatim}
\end{enumerate}

\section*{5. Key Behavioral Improvements}

\subsection*{5.1 Before (Distance-Only Avoidance)}
\begin{itemize}
    \item Obstacle at 300\,m $\rightarrow$ immediate lateral sidestep
    \item Vessel maintains high speed during avoidance
    \item After passing, vessel continues detour arc
    \item No natural "yield" behavior
\end{itemize}

\subsection*{5.2 After (TTC-Based Yielding)}
\begin{itemize}
    \item Obstacle far but non-urgent $\rightarrow$ minimal reaction
    \item TTC enters critical window $\rightarrow$ speed drops smoothly
    \item Progress reward relaxes $\rightarrow$ waiting becomes cheaper than swerving
    \item Control smoothness increases $\rightarrow$ no jerky maneuvers
    \item After CPA ($\dot{d}_{\text{rel}} > 0$) $\rightarrow$ route recapture activates
    \item Vessel returns to nominal path instead of continuing detour
\end{itemize}

\section*{6. Thesis Contributions}

\begin{enumerate}
    \item \textbf{TTC-aware collision avoidance} without discrete modes—behavior emerges from continuous cost shaping based on predicted interaction quality (CPA, closing rate, TTC).
    
    \item \textbf{Yield-before-sidestep principle}—by reducing progress reward and allowing near-stop during $\lambda_{\text{yield}}$, the optimizer naturally prefers "slow down and let obstacle pass" over "maintain speed and swerve early."
    
    \item \textbf{Automatic route recapture}—$\lambda_{\text{return}}$ activates when separation increases, tightening the path tube and increasing XTE/heading penalties to pull vessel back to route.
    
    \item \textbf{Smooth control during conflicts}—increased rate penalties ($R_{\Delta}^{\text{scale}} = 3.5\times$) prevent aggressive azimuth/thrust changes during yield, producing calm, deliberate maneuvers.
    
    \item \textbf{Map confinement via soft barriers}—prevents optimizer from exploiting "open water escape" by penalizing proximity to map boundaries during tight/return phases.
    
    \item \textbf{Sector-based relevance filtering}—only obstacles in forward $240^\circ$ cone influence avoidance, eliminating spurious reactions to distant/receding contacts.
\end{enumerate}

\section*{7. Recommended Thesis Plots}

\begin{itemize}
    \item \textbf{Lambda trajectories:} Overlay $\lambda_{\text{ttc}}$, $\lambda_{\text{yield}}$, $\lambda_{\text{return}}$ vs. time to show smooth activation/deactivation.
    
    \item \textbf{Speed profile:} Show $U_{\text{cap}}^{\text{dyn}}$ dropping during yield, then recovering.
    
    \item \textbf{CPA evolution:} Plot $d_{\text{CPA}}$ and $t_{\text{CPA}}$ over time to demonstrate predictive awareness.
    
    \item \textbf{Control smoothness:} Compare azimuth rate $\dot{\alpha}$ with/without $R_{\Delta}^{\text{scale}}$ scaling.
    
    \item \textbf{Route recapture:} Show XTE decreasing after CPA as $\lambda_{\text{return}}$ rises.
\end{itemize}

\section*{8. Tuning Guidelines}

\begin{table}[h]
\centering
\begin{tabular}{@{}lll@{}}
\toprule
\textbf{Parameter} & \textbf{Effect of Increasing} & \textbf{Recommended Range} \\
\midrule
$T_{\text{far}}$ & Earlier activation, more conservative & 15--30\,s \\
$d_{\text{CPA,th}}$ & Stricter miss distance requirement & 40--100\,m \\
$U_{\text{yield}}$ & Minimum speed during yield & 0.0--0.5\,m/s \\
$R_{\Delta}^{\text{yield}}$ & Smoother (less jerky) avoidance & 2.0--5.0 \\
$\dot{d}_{\text{ref}}$ & Faster yield$\rightarrow$return transition & 1.5--3.0\,m/s \\
$\Delta w_{\text{xte}}^{\text{ret}}$ & Stronger route recapture pull & 10--25 \\
\bottomrule
\end{tabular}
\caption{Tuning parameters for dynamic obstacle avoidance.}
\end{table}

\section*{9. Summary}
This update transforms dynamic obstacle avoidance from a \textbf{reactive, distance-based sidestepping} behavior into a \textbf{predictive, TTC-aware yielding} strategy. The vessel now:
\begin{itemize}
    \item Slows down \textit{before} conflicts become urgent
    \item Prefers waiting over swerving
    \item Maintains smooth, calm control during avoidance
    \item Automatically returns to route after passing
\end{itemize}
All without a single discrete mode switch—pure continuous optimization driven by geometry, relative motion, and predicted interaction quality.

\end{document}


## Observation after this big change

The autonomous path selection by the nmpc still make it sway a lot into directions and paths which are not strictly related to the waypoints indicated and the general route. this brings me back to being temptative about restoring a "recovery mode" which in the previous case it was found to be not optimal.

So the cycle goes on and the perfect trade-off between making the dynamic ship follow rougly the waypoints and keeping the solution mostly an autonomous decision based optimal control problem is very difficult to achieve. 

Honestly, you cannot fully escape some form of context-awareness for the transit-to-berth problem. Here's why: the cost landscape for open-water transit and for millimeter-precision berthing are fundamentally incompatible — if you maximize progress at sea, you overshoot the berth; if you apply berthing precision weights globally, the ship crawls. Something must bridge them.