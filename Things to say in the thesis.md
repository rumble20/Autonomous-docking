## About the distinction between past work and my work

"This work presents an integrated motion planning and control framework for autonomous ship docking. Unlike traditional two-stage approaches where a global planner generates a fixed collision-free path that a separate controller blindly tracks, the proposed method employs a Nonlinear Model Predictive Controller that jointly performs trajectory generation and tracking by optimizing control inputs online while explicitly enforcing obstacle-avoidance constraints. A lightweight guidance layer provides high-level navigation directives (desired heading, speed, and waypoint sequencing), but the NMPC retains full authority to deviate from the reference trajectory when necessary to satisfy safety constraints or actuator limitations."
Technical Arguments for Your Thesis
1. The Reference is Soft, Constraints are Hard
Table
Copy
Component	Traditional Decoupled	Your Integrated Approach
Path/Reference	Hard requirement (must follow)	Soft guidance (can deviate)
Obstacle avoidance	Planner's job (fixed)	NMPC's job (online)
Actuator limits	Often ignored by planner	Enforced in NMPC
Dynamic feasibility	Assumed	Guaranteed by dynamics model

2. Trajectory is Generated Online, Not Pre-computed
The NMPC's X_pred output IS a trajectory. Every timestep, it computes:
Optimal state trajectory: 
X∗={x0...xn}}
Optimal control sequence: 
U∗={u0...um}}
This is trajectory generation, not just tracking!
3. Guidance ≠ Planning
Your guidance layer does not solve a planning problem. It:
Selects waypoints (sequential logic)
Computes heading toward waypoint (trigonometry)
Builds a reference (linear propagation)
It does not:
Check collision feasibility
Optimize paths
Consider dynamics
The NMPC does all of that.

Literature Support
From your available papers:

3

 - 2022 Autonomous Docking MPC: Uses similar approach where MPC generates trajectory to reach waypoints, not just track a fixed path.

4

 - Martinsen et al. 2020: Explicitly discusses that using NMPC directly (vs. planner + tracker) allows handling of drift and disturbances through optimization.

6

 - CBF-MPC paper: States that "combining planning with control to find an ideal trajectory" is a key advantage of optimization-based methods.

5

 - Former thesis: Discusses receding horizon trajectory optimization where the trajectory is generated online, segment by segment.


When your coworker's RRT* is added, you can frame it as:
"The proposed system employs a hierarchical architecture where:
RRT provides global path guidance and warm-start initialization*
NMPC performs local trajectory optimization with safety guarantees
Unlike purely hierarchical approaches, the NMPC layer retains authority to deviate from the RRT path when real-time obstacle constraints require it, making the overall system robust to dynamic obstacles and modeling errors."*
This positions RRT* as an enhancement (warm-start, global guidance), not a replacement for the NMPC's trajectory generation capability.

## Thruster allocation

specify that the truster allocation control is implemented in the NMPC by considering forces and 2 azimuth thrusters as can be seen in the MSS example, 
not in RPM control or any further complication since that is not the scope of the thesis

## Martinsen

The Martinsen method is excellent for harbor docking when you have:
A known map of the harbor layout
Static obstacles (piers, walls, moored vessels)
LIDAR/sensor fusion to augment the map
But for dynamic circular obstacles (like in congested harbors with moving vessels), the slack variable formulation is more appropriate because it directly handles circular geometries without requiring complex polytope generation.

## Agile development

The software development process initially followed a traditional, requirements-driven plan.
After early prototyping, it became clear that fixing all requirements upfront was not practical for this project due to the iterative nature of tuning, solver behaviour, and scenario-dependent constraint handling.
Therefore, the implementation was carried out using an agile, incremental workflow with frequent integration and testing.

## How I made it more computationally efficient

The computational speed-up was achieved through a multi-layer strategy: reducing NMPC problem size, tuning the nonlinear solver for real-time operation, compressing obstacle geometry, and optimizing online data flow.

At the discretization level, a fast profile was adopted with a longer control period and an equivalent prediction horizon. This preserved long-horizon look-ahead while reducing the number of optimization decision variables.

At the solver level, IPOPT was retained for robustness, but configured with real-time-oriented stopping settings: reduced maximum iterations, relaxed acceptable tolerances, limited-memory Hessian approximation, and a CPU-time cap per solve.

At the geometric level, the harbor map was converted into a compact set of circular obstacle primitives using merging and capping. This significantly reduced per-step obstacle handling cost while maintaining shoreline representation.

At the online execution level, nearest-obstacle selection was vectorized and cached, reducing non-optimization overhead at each control update.

To preserve safety under coarser temporal discretization, obstacle-distance constraints were enforced both at prediction nodes and at mid-interval points. This prevents trajectory segments from cutting through obstacles between two consecutive nodes.

Finally, local speed modulation was introduced as a function of obstacle clearance. Near tight obstacles, desired speed is reduced to improve yaw authority and turning feasibility; in open water, cruise speed is preserved.

Compact mathematical summary:

Decision variables: nvar=nx(N+1)+nuNnvar; =nx (N+1)+nu;  NObstacle inequalities with midpoint constraints enabled: nineq=nobs[(N+1)+N]nineq =nobs [(N+1)+N]; Real-time feasibility index:ρ=tplanTs,real-time feasible if ρ<1ρ=Tstplan real-time feasible if ρ<1

## Hard constraints vs cbf

In the current NMPC implementation, safety and feasibility are enforced primarily through hard constraints.
A hard constraint is a condition that must be satisfied exactly by every optimizer iterate at every prediction step; if it cannot be satisfied, the nonlinear program becomes infeasible.

The present formulation includes the following hard constraints:

Initial-state equality constraint
The first predicted state is constrained to coincide with the measured vessel state at the current sampling instant.

Dynamic equality constraints
The full prediction horizon is constrained by the discretized vessel model (Euler forward), so each predicted state transition must satisfy the model equations exactly.

Obstacle-avoidance inequality constraints
For each obstacle and each prediction step, the predicted vessel position must remain outside a safety radius:
[
\sqrt{(x_k-x_o)^2 + (y_k-y_o)^2 + \varepsilon} - r_o - r_{\text{safety}} \ge 0.
]
This is currently a strict, non-relaxable geometric separation condition.

State and input box constraints
Physical and operational limits (e.g., speed, yaw rate, azimuth angles, shaft speeds) are imposed as upper/lower bounds and are also hard.

Hence, the active controller is a hard-constrained NMPC with no slack variables in the optimization problem.

A natural extension is to replace or complement geometric obstacle constraints with Control Barrier Function (CBF) constraints.
Define a safety function (h(x)) such that (h(x)\ge 0) represents the safe set (for example, distance-to-obstacle minus safe distance). A discrete-time CBF condition can be imposed as:
[
h(x_{k+1}) - (1-\gamma)h(x_k) \ge 0,\quad \gamma\in(0,1].
]
This enforces forward invariance of the safe set, i.e., once the system is safe, the optimizer must keep it safe over time.

Compared with purely geometric distance constraints, CBF constraints provide a more dynamic notion of safety because they constrain how safety evolves between consecutive steps, not only the instantaneous position. This is especially useful in tight harbor maneuvers where prediction timing and approach dynamics matter.

For practical development, three integration strategies are recommended:

Substitution strategy
Replace the current obstacle-distance inequalities with CBF inequalities based on the same obstacle geometry.

Hybrid strategy
Keep current hard geometric constraints as a conservative baseline and add CBF constraints to improve transient safety behavior.

Soft-CBF strategy (recommended for robustness)
Introduce nonnegative slack variables (s_k) in CBF constraints:
[
h(x_{k+1}) - (1-\gamma)h(x_k) + s_k \ge 0,\quad s_k\ge 0,
]
and penalize (\sum s_k^2) heavily in the objective.
This preserves safety priority while reducing infeasibility risk in congested maps or near-degenerate situations.

Additional features that can be added in the same framework include:

Hard input-rate constraints (not only rate penalties) for actuator smoothness guarantees.
Polygon/shoreline CBFs to avoid approximating coastlines with circles.
Time-varying CBFs for moving obstacles and COLREG-style directional rules.
Hierarchical weighting between path tracking, energy use, and safety slack for controlled degradation under stress.
In summary, the current controller is correctly formulated as hard-constrained NMPC. CBFs are fully compatible with this architecture and can be used either as a replacement for distance constraints or, more safely, as a hybrid/soft extension to improve robustness and future scalability.

## Moving and actuating limits

For the high-level guidance layer, I use the same practical limits as the NMPC and the container + dual-azipod model, so both guidance and control are consistent.

Simple idea:
- The guidance should not ask the vessel to do maneuvers that the low-level model cannot physically track.
- So I pass speed, heading-rate, azimuth, and shaft limits directly from the control model.

Main state and input limits used in my setup:

1) Surge speed limit
- u_min = 0.1 m/s
- u_max = 12 m/s (approximately 0.2 to 23 knots operational range)

2) Sway speed limit
- v_min = -3 m/s
- v_max = +3 m/s (realistic lateral drift bounds)

3) Yaw rate limit
- r_min = -0.15 rad/s
- r_max = +0.15 rad/s (approximately ±8.6 deg/s, realistic for large vessel)

4) Shaft speed limits (both azipods)
- n_min = -80 rpm
- n_max = 160 rpm

5) Azimuth angle limits (both azipods)
- alpha_min = -pi rad
- alpha_max = +pi rad (full rotation capability)

6) Commanded shaft limits (both commands n1_c and n2_c)
- same as shaft state limits: [-80, 160] rpm

Moving (state-dependent) actuation limits that are very important:

1) Azimuth steering rate limit
- alpha_dot is limited to ±0.21 rad/s (approximately 12 deg/s)
- This is a mechanical constraint on how fast the azipod can rotate.
- In the NMPC, this is enforced as: |alpha(k+1) - alpha(k)| <= alpha_rate_max * dt

2) First-step azimuth rate constraint
- The first control step must also respect the rate limit relative to the previous applied control.
- Enforced as: |alpha(1) - alpha_prev| <= alpha_rate_max * dt
- Without this, the NMPC may command azimuth angles that are physically unreachable in one timestep.
- Missing this constraint causes trajectory mismatch and oscillatory behavior (spiral instability).

3) Shaft first-order dynamics
- n_dot = (n_command - n_actual) / Tm
- This means shaft speed cannot jump instantly.

4) Shaft acceleration hard limit
- n_dot is clipped to [-10, +10] rpm/s
- So even if command changes a lot, actual shaft speed changes gradually.

5) Time constant Tm depends on current shaft speed
- Tm = 5.65 / (|n|/60) when |n| > 18 rpm, otherwise Tm = 18.83
- Tm is bounded to [1, 20] seconds for numerical stability.
- This gives slower response at low shaft speeds and faster response at high speeds.

6) Heading kinematics
- psi_dot = r
- So heading change rate is directly limited by yaw-rate feasibility.

What this means for high-level guidance:

- The guidance layer should generate references that respect realistic turn capability at the current speed.
- In narrow areas, it should avoid aggressive curvature requests because azipod steering rate may not track them in time.
- The azimuth rate constraint is particularly important for azipod vessels since they can rotate fully, but the steering rate is mechanically limited.
- The first-step constraint connects the NMPC prediction to the real actuator state, ensuring continuity between planned and executed motion.
- Guidance and NMPC should use the same obstacle clearance policy, otherwise guidance says "feasible" and NMPC says "not feasible."


## About the actuation motors mounted

Even if with the current limits it still works, the high volatile changing yaw now actuated by the control of the azipod would be best suited rather by ABB DynaFin, which could prove useful for this case