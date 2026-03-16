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

For the high-level path planner, I use the same practical limits as the NMPC and the container + dual-azipod model, so both planning and control are consistent.

Simple idea:
- The planner should not ask the vessel to do maneuvers that the low-level model cannot physically track.
- So I pass speed, heading-rate, azimuth, and shaft limits directly from the control model.

Main state and input limits used in my setup:

1) Surge speed limit
- u_min = 0.1 m/s
- u_max = 10 m/s

2) Yaw rate limit
- r_min = -0.25 rad/s
- r_max = 0.25 rad/s

3) Shaft speed limits (both azipods)
- n_min = -80 rpm
- n_max = 160 rpm

4) Azimuth angle limits (both azipods)
- alpha_min = -pi rad
- alpha_max = +pi rad

5) Commanded shaft limits (both commands n1_c and n2_c)
- same as shaft state limits: [-80, 160] rpm

Moving (state-dependent) actuation limits that are very important:

1) Shaft first-order dynamics
- n_dot = (n_command - n_actual) / Tm
- This means shaft speed cannot jump instantly.

2) Shaft acceleration hard limit
- n_dot is clipped to [-10, +10] rpm/s
- So even if command changes a lot, actual shaft speed changes gradually.

3) Time constant Tm depends on current shaft speed
- Tm is not fixed, it changes with operating point (and is bounded in practice).
- This gives different response speed at low and high shaft speeds.

4) Heading kinematics
- psi_dot = r
- So heading change rate is directly limited by yaw-rate feasibility.

What this means for high-level planning:

- The path planner should plan trajectories that respect a realistic turn capability at the current speed.
- In narrow areas, it should avoid aggressive curvature requests because azipod/shaft dynamics may not track them in time.
- It should use the same obstacle clearance policy as NMPC, otherwise the planner says “feasible” and NMPC says “not feasible.”

Recommended planner contract (easy to implement):

- Dynamic bounds passed as rules:
	abs(n_dot) <= 10 rpm/s
	first-order shaft lag with variable Tm

## About the actuation motors mounted

Even if with the current limits it still works, the high volatile changing yaw now actuated by the control of the azipod would be best suited rather by ABB DynaFin, which could prove useful for this case

