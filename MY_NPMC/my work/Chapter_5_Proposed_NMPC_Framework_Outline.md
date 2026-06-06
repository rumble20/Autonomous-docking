# Chapter 5 Proposed NMPC Framework

This file is a machine-friendly outline for drafting Chapter 5 of the thesis. It focuses on the original NMPC architecture implemented in `my work`, the control flow across files, and the modeling/constraint choices that are directly traceable to code.

## Chapter purpose

Chapter 5 should explain how the NMPC framework is built and how it differs from a generic path-following controller. The chapter should connect the methodology from the theory chapters to the actual harbor-navigation implementation:

- line-tracking and tube-based path following on an active segment
- receding-horizon optimal control with a 9-state vessel model
- obstacle avoidance via discrete-time CBF-style inequalities and soft slacks
- berth-aware terminal regulation with corridor and pose envelopes
- adaptive scheduling that changes weights and constraints between transit and final approach

The chapter should read as the bridge between the modeling chapter and the results chapter. It should explain not only what the controller does, but also where each behavior comes from in the code.

## Core files to cite throughout the chapter

- [run_nmpc.m](run_nmpc.m): top-level scenario setup, waypoint handling, map extraction, obstacle packaging, berth scheduling, online parameter adaptation, solver invocation, logging.
- [NMPC_Container_final.m](NMPC_Container_final.m): the NMPC solver class, objective function, constraints, slack variables, and prediction model.
- [container.m](container.m): nonlinear 9-state vessel dynamics used by the simulator and by the NMPC prediction model through `dynamicsCasADi`.
- [RunNmpcHelpers.m](RunNmpcHelpers.m): RK4 integration wrapper, waypoint/geometry helpers, obstacle conversion helpers, map sampling and half-plane extraction helpers, dynamic obstacle helper routines.
- [NavUtils.m](NavUtils.m): geometric utilities, map validation, wrapping and distance helpers.
- [animateSimResult.m](animateSimResult.m): post-run visualization, useful when describing what is logged and how the controller behavior is inspected.
- [testing scripts/verify_nmpc_integration.m](testing%20scripts/verify_nmpc_integration.m): verification that the runtime loop, solver call, and integration path are consistent.
- [testing scripts/run_nmpc_regression.m](testing%20scripts/run_nmpc_regression.m): regression harness for comparative evaluation.
- [testing scripts/diagnose_map_obstacles.m](testing%20scripts/diagnose_map_obstacles.m): map sampling and obstacle conversion diagnostics.
- [testing scripts/diagnose_heading_logic.m](testing%20scripts/diagnose_heading_logic.m): heading preview and turn-anticipation diagnostics.
- [testing scripts/analyze_waypoints.m](testing%20scripts/analyze_waypoints.m): route geometry and turn-angle analysis.

If a short repository-wide summary is needed in the chapter preamble or appendix, `NotebookLM_reference.md` can be cited as the compact project map.

## 5.1 Overall control architecture

### Narrative goal

Explain that the project is not a point-to-point controller, but a closed-loop harbor-navigation stack that combines route following, obstacle avoidance, and final docking preparation.

### What the architecture does

- The main loop in `run_nmpc.m` loads the harbor map, selects waypoints, constructs the active path segment, packages obstacle and map information, solves the NMPC, and advances the plant.
- The NMPC receives only the active segment `[wp_start -> wp_end]`, not a full future path matrix.
- The ship model is 9-state, underactuated, and actuated through two azimuthing stern propulsors plus a bow tunnel thruster.
- The architecture separates runtime concerns:
  - guidance and scheduling in `run_nmpc.m`
  - optimization and constraints in `NMPC_Container_final.m`
  - dynamics and simulation in `container.m` and `RunNmpcHelpers.m`
  - visualization in `animateSimResult.m`

### Key design points to mention

- The framework uses a single NMPC formulation that is reused across transit, obstacle interaction, and final berthing.
- The same code base supports route segments, turn anticipation, map-obstacle handling, dynamic obstacles, and final-approach enforcement.
- The controller is implemented in MATLAB with CasADi and IPOPT.
- The architecture is simulation-oriented; there are no hardware trials in the current repository.

### Important notation and state definition

- State vector `x = [u, v, r, x, y, psi, n1, n2, n3]`.
- Control vector `u = [alpha1, alpha2, n1_c, n2_c, n3_c]`.
- `u`, `v`, and `r` are surge, sway, and yaw rate.
- `x`, `y`, and `psi` are inertial position and heading.
- `n1`, `n2`, `n3` are shaft speed states for the stern and bow thrusters.

### Suggested emphasis in the chapter

- Explain that the controller is geometry-aware rather than waypoint-idling.
- Make clear that the active segment is the basis for cross-track and along-track error.
- Stress that the final approach is a special scheduling mode, not a separate controller.

## 5.2 Receding horizon formulation

### What the chapter should say

Describe the NMPC as a finite-horizon, direct multiple-shooting formulation solved online at every control step.

### Optimization structure in the code

- Decision variables in `NMPC_Container_final.m` include the full state trajectory `X`, control trajectory `U`, obstacle slacks `S_obs`, and terminal slack `S_term`.
- The prediction horizon is `N_h = obj.N`.
- The initial condition is enforced explicitly as a constraint.
- Dynamics are enforced at each shooting node as equality constraints.
- The optimizer is re-solved at every iteration using the latest measured state and updated geometry.

### Discretization details to state explicitly

- NMPC prediction uses explicit Euler on the shooting nodes: `x_{k+1} = x_k + f(x_k,u_k) * dt`.
- The forward simulation used in the main loop uses RK4 through `RunNmpcHelpers.rk4Step9`.
- The chapter should explicitly note this mismatch between prediction and simulation, because it is intentional and should not be described as the same integrator.

### Receding-horizon behavior

- Only the first control action of the optimized sequence is applied.
- The next iteration reinitializes the optimization with the new measured state and a shifted warm start.
- The controller therefore behaves as a standard receding-horizon scheme with online re-optimization.

### Warm-start logic to mention

- The warm-start strategy is simple shifting with constant tail repetition.
- There is no extrapolation beyond the last predicted point.
- This matters because it is a computationally cheap but robust initialization strategy for the nonlinear solver.

### Suggested paragraph content

- State that the optimization is nonlinear because both the vessel model and the obstacle/corridor geometry are nonlinear.
- State that feasibility is handled through slack variables and bounded constraints rather than through a separate fallback controller.

## 5.3 Cost function design

### Main message

The objective is a weighted sum of tracking, progress, heading, effort, and soft-constraint penalties. The stage and terminal parts should be described separately.

### 5.3.1 Progress objective

Use this subsection to explain how the controller advances along the route instead of simply minimizing a positional error to the end waypoint.

- The code uses the active path segment and computes both cross-track error and along-track progress.
- Cross-track error is measured relative to the segment normal.
- Along-track distance is used to reward motion toward the segment endpoint.
- The stage cost combines a tube-like behavior:
  - lower penalty inside the corridor/tube
  - stronger penalty outside the tube
- The path direction can be previewed toward the next segment when the upcoming turn is sharp.

Implementation points:

- `path_cost` in `NMPC_Container_final.m` includes cross-track, along-track, heading, and speed terms.
- Route progress is not a separate planner; it is embedded directly in the cost.
- The active waypoint update and segment switching are handled in `run_nmpc.m`.

### 5.3.2 Terminal pose regulation

Explain that the terminal objective becomes stronger near the goal and in berth mode.

- Terminal penalties include position error to the final target, heading error to a desired final heading, and residual motion penalties on surge, sway, and yaw rate.
- When berth mode is active, the framework also uses terminal corridor logic and a tighter pose envelope.
- Terminal slack variables allow the final envelope to remain feasible in difficult conditions.

Implementation points:

- `P_terminal_path` carries terminal position, heading, and stop weights.
- `P_goal_heading_enable` turns the terminal heading term on or off.
- The terminal slack is penalized quadratically as `S_term' * S_term`.

### 5.3.3 Control effort penalties

Describe how the controller avoids unrealistic or overly aggressive control.

- Input tracking penalizes deviation from the current reference guess `P_uref`.
- Control-rate penalties penalize step-to-step changes in `U`.
- Actuator magnitude penalties discourage large shaft-speed states.
- Reverse motion is discouraged but not completely forbidden.

Implementation points:

- The stage input penalty is `sum1(P_r_input .* (du.^2))`.
- The rate penalty is `sum1(P_r_rate .* (dU.^2)) * P_R_rate_scale`.
- The speed and actuator terms are part of the overall stage/terminal regularization.

### 5.3.4 Soft-constraint penalties

Explain that feasibility recovery is handled by slacks.

- Obstacle slacks are per obstacle and per stage.
- The terminal slack is a separate vector for the terminal envelope.
- Both are penalized quadratically, not with an explicit L1 norm.
- The chapter should emphasize that the slacks are not decorative; they are part of the actual optimization problem.

Implementation points:

- `S_obs(j,k)` is penalized with `obj.soft_obs_weight * S_obs(j,k)^2`.
- `S_term` is penalized with `obj.terminal_pose_slack_weight * (S_term' * S_term)`.
- In the runtime configuration, slack bounds are finite, and the scheduler can tighten them during berth mode.

### 5.3.5 Important scaling note

- Stage terms are summed directly in the objective.
- There is no explicit `dt` multiplier in the objective terms.
- The effect of discretization is therefore absorbed into the tuning weights.
- This should be stated clearly so the reader does not assume a continuous-time integral form.

## 5.4 Constraint formulation

### Chapter goal

This subsection should explain that the controller is not purely cost-driven; it also enforces a mixed set of hard and softened constraints on dynamics, obstacles, map boundaries, actuators, and terminal behavior.

### 5.4.1 Actuator constraints

Describe the physical actuator limitations first.

- Control inputs include azimuth angles and shaft commands.
- The azimuth commands are rate-limited with hard inequality constraints.
- The shaft commands are bounded, and the plant model also includes actuator dynamics.
- The code keeps the actuator dynamics in the model rather than treating the thrusters as ideal instantaneous controls.

Implementation points:

- Azimuth-rate constraints are written directly on differences of successive azimuth commands.
- The first rate step is referenced to the previous applied control.
- Braking and forward-preference terms constrain behavior near stops and in final approach.

### 5.4.2 Collision avoidance

Explain the obstacle model in the actual implementation, not just in generic theory.

- Obstacles are packaged from static obstacles, dynamic obstacles, and map-derived obstacles.
- The obstacle geometry can be represented as circles or as oriented-rectangle footprint checks, depending on the configured collision model.
- Map obstacles can also be represented as local half-planes derived from the harbor polygon.
- The controller uses the obstacle geometry inside the NMPC formulation rather than in a separate local planner.

Implementation points:

- `run_nmpc.m` prepares static obstacles, dynamic obstacles, and map-derived obstacle representations.
- `NMPC_Container_final.m` uses obstacle positions and radii as parameters.
- The obstacle CBFs are formulated on the shooting nodes, using `x_k` and `x_{k+1}`.
- Slack variables soften the obstacle constraints when needed.

### 5.4.3 CBF safety constraints

Describe the discrete-time CBF form that is actually implemented.

- The implementation uses a discrete-time inequality on the shooting nodes.
- The update compares the barrier value at consecutive nodes and includes a slack term.
- The terminal obstacle condition is a special case and does not use the same `(1-gamma)` factor.

Implementation points to state carefully:

- The code builds `h(x_{k+1}) - (1 - P_gamma_obs) h(x_k) + S_obs(j,k) >= 0` in inequality form.
- The obstacle constraints are enforced with lower bound `0` and upper bound `+inf` after the solver transformation.
- `P_gamma_obs` is the discrete safety aggressiveness parameter.

### 5.4.4 Berthing corridor constraints

Explain that the final approach is geometry-constrained.

- The berth configuration defines a final approach corridor aligned with the docking direction.
- The corridor is activated near the final target and can be tightened in the final approach.
- Pose and velocity envelopes are used to represent a reachable and controllable docking region.
- Slack on the terminal pose envelope prevents immediate infeasibility while still forcing convergence.

Implementation points:

- `berth_cfg` in `run_nmpc.m` defines the docking target, final heading, activation distance, corridor width, and terminal pose tolerances.
- The solver has terminal pose and stop costs, plus corridor-related constraints.

### 5.4.5 Dynamics constraint

Mention the motion model constraint as the core equality that binds everything together.

- The dynamics constraint is the single most important hard equality in the NMPC.
- It couples the prediction model to the control and geometry constraints.
- This is what turns the problem from a static geometry problem into a dynamic control problem.

### 5.4.6 Hard vs soft constraints summary

The chapter should explicitly classify constraints:

- hard: initial condition, state propagation, many input bounds, azimuth rate limits, selected map/corridor boundaries
- soft: obstacle avoidance via slacks, terminal pose envelope via slack, map-barrier penalty terms where configured as soft
- mixed: some conditions are enforced directly and also penalized to bias the optimizer toward safe behavior

## 5.5 Adaptive scheduling strategy

### Main story

Explain that the framework is not static. It changes emphasis based on route geometry, distance to target, turn severity, and berth activation.

### Transit vs final approach

- Transit mode emphasizes route progress, tube tracking, and anticipation of the next segment.
- Final approach mode increases the weight on heading, docking precision, braking, corridor adherence, and terminal pose regulation.
- The switch is not a separate controller; it is a scheduler that modifies weights and constraints online.

### Scheduling signals present in the code

- proximity to the final berth target
- last-leg / final-leg activation logic
- turn angle and sharp-turn anticipation
- corridor activation and tightening
- dynamic obstacle proximity and latent awareness horizon
- map boundary / open-water risk
- stopping distance and terminal capture radius

### What to mention from `run_nmpc.m`

- The script builds a berthing configuration block.
- It derives a corridor from the last route segment.
- It increases or decreases weights using route-following and berth-aware schedules.
- It can activate dynamic obstacle awareness when obstacles become relevant.
- It can tighten the soft obstacle slack and terminal pose tolerances near berth completion.

### What to mention from `NMPC_Container_final.m`

- The solver receives runtime parameters for path weights, heading weights, speed floor, rate scaling, map barrier weight, and terminal path weights.
- The current path segment and next segment direction are used to create a previewed heading reference.
- The terminal objective is therefore stateful with respect to route geometry, not a fixed final-point tracker.

### Recommended explanation style

- Present the scheduler as a smooth blending mechanism rather than a hard switch whenever possible.
- If the chapter needs a concise summary, describe it as a navigation-phase decomposition:
  - transit along the harbor route
  - anticipation near sharp turns
  - final approach and docking capture

## 5.6 Connections to the rest of `my work`

This section is optional but useful if the chapter needs a short implementation traceability subsection.

### Map and scenario files

- `helsinki_harbour.mat` and `helsinki_harbour_UPDATED.mat` define the harbor geometry used by the NMPC scenarios.
- The map extraction logic in `run_nmpc.m` converts polygonal harbors into circle samples or half-planes.

### Visualization and diagnostics

- `animateSimResult.m` provides post-run inspection of ship motion, planned route, and obstacle interaction.
- The figures in `plots in development process` and the saved results in `results` are useful for illustrating controller behavior, but they should be referenced only if the exact scenario is identified.

### Testing and verification

- The diagnostic scripts in `testing scripts` are useful for grounding claims about integration, heading anticipation, obstacle conversion, and waypoint analysis.
- If the chapter mentions numerical properties, the relevant script or log should be identified explicitly.

### Backups

- `backups/2026-05-08` stores older solver versions and can be mentioned only if historical comparison is relevant.
- The main chapter should stay focused on the current implementation, not on the backup history.

## 5.7 Suggested notation and terminology for the chapter draft

Use the following terminology consistently:

- NMPC horizon `N`
- sampling time `dt`
- active path segment `[wp_start, wp_end]`
- cross-track error `XTE`
- along-track progress `along_k`
- tube corridor / tube width
- terminal envelope / terminal pose box
- berth mode / final approach mode
- discrete-time CBF inequality
- obstacle slack `S_obs`
- terminal slack `S_term`

Avoid mixing in alternative names unless they are explicitly defined as aliases in the chapter.

## 5.8 Suggested chapter flow

If the section needs to be written as a coherent narrative, use this order:

1. Introduce the objective of the NMPC framework and the role of the chapter.
2. Present the overall control stack and file responsibilities.
3. Define the prediction problem and receding-horizon structure.
4. Describe the objective function in three blocks: progress, terminal regulation, effort/soft penalties.
5. Describe the constraints: dynamics, actuators, CBF safety, map boundaries, berthing corridor, terminal envelope.
6. Explain the adaptive scheduler that transitions from transit to final approach.
7. Close with a short implementation note on simulation, logging, and verification scripts.

## 5.9 Reproducibility notes to preserve in the final chapter

- The NMPC prediction model and the simulation integrator are not the same, so the chapter must not claim they are identical.
- The objective does not explicitly multiply every stage term by `dt`; weights carry the effective scaling.
- Obstacle constraints are discrete-time constraints evaluated on the shooting nodes.
- Slack variables are quadratic-penalized feasibility recovery terms, not linear penalties in the current implementation.
- The chapter should mention whether a constraint is hard, soft, or mixed.

## 5.10 Minimal implementation summary for an AI chapter drafter

This project implements a harbor-navigation NMPC for a 175 m container ship with two azimuthing stern propulsors and one bow thruster. The controller solves a direct multiple-shooting problem in `NMPC_Container_final.m` using an explicit-Euler prediction model, while the plant simulation in `run_nmpc.m` advances the vessel with RK4 through `RunNmpcHelpers.rk4Step9`. The objective combines active-segment progress, tube tracking, heading alignment, speed shaping, input effort, rate smoothing, obstacle slack penalties, and terminal berthing regulation. Constraints cover dynamics, actuator limits, obstacle CBFs, map half-planes or sampled circles, and berth corridor / terminal pose envelopes. `run_nmpc.m` provides the online scheduling logic that shifts the emphasis from transit tracking to final approach and docking.

## 5.11 Items that should probably become figures or tables later

- control architecture diagram showing `run_nmpc.m` -> `NMPC_Container_final.m` -> `container.m` / `RunNmpcHelpers.m`
- table of states, controls, and physical meaning
- table of cost terms and weight roles
- table of constraint classes and whether they are hard/soft/mixed
- phase diagram of transit, turn anticipation, and final approach
- illustration of the active segment, tube corridor, and berth corridor
- schematic of circular obstacles versus half-plane map boundaries
- example of the discrete-time CBF constraint on the shooting nodes

## 5.12 Drafting caution list

- Do not describe the NMPC as point tracking; it is segment-based line following with terminal docking behavior.
- Do not say the optimizer uses RK4 unless you are specifically describing the plant simulation.
- Do not imply the cost is a continuous integral unless you explicitly note that stage terms are summed directly.
- Do not imply that the obstacle constraints are purely geometric; they are embedded into the nonlinear program with slack and CBF structure.
- Do not treat berth mode as a separate controller; it is a scheduled change in terminal and corridor emphasis.

## 5.13 Detailed implementation inventory for the original NMPC

This section is intentionally dense. It is meant to prevent an AI chapter drafter from omitting the implementation details that make the project technically specific.

### 5.13.1 Solver class identity and role

- `NMPC_Container_final.m` is the actual solver class used by the project.
- It is a handle class that stores horizon length, sampling time, tuning weights, obstacle slots, solver state, and warm-start history.
- The solver is built once and then reused online.
- The class name should be mentioned explicitly in the chapter because it is the core implementation artifact.

### 5.13.2 Decision variables and variable ordering

The nonlinear program is not abstract; its variable vector has a fixed structure.

- State trajectory `X` with size `9 x (N+1)`.
- Control trajectory `U` with size `5 x N`.
- Obstacle slack matrix `S_obs` with size `n_obs x (N+1)`.
- Terminal slack vector `S_term` with length `6`.
- The stacked optimization vector is `OPT = [X(:); U(:); S_obs(:); S_term(:)]`.

The chapter should mention that the solver is formulated over a stacked decision vector, because this is the basis for the bounds and warm-start logic.

### 5.13.3 Parameter vector and online data injection

The solver does not hard-code the route and obstacle data; it receives a long parameter vector `P_all`.

The chapter should note that `P_all` contains, in order:

- initial state `x0`
- active path start, end, and next waypoint geometry
- goal heading and goal-heading enable flag
- path weights and tube width
- soft speed-cap and speed-floor parameters
- terminal cost weights and the final-waypoint flag
- reference input guess `u_ref`
- obstacle count, obstacle positions, radii, and velocities
- half-plane count, half-plane normals, and offsets
- forward-preference scaling
- previous control `u_prev`
- braking rate limit
- diagonal state, input, and rate weights
- stage and terminal cost scales
- collision clearance and terminal pose/velocity tolerances
- berth corridor geometry and enable flag
- azimuth and stern synchrony limits
- CBF gamma values for obstacle and half-plane constraints
- map barrier weight, rate-scaling factor for obstacle yield, and map soft margin
- terminal line-recapture weights

This matters because the optimization is not driven by a handful of parameters; it is heavily parameterized and continuously reshaped online by the scheduler.

### 5.13.4 Objective function details beyond the simple chapter summary

The chapter should explain that the cost is assembled in layers.

#### Stage cost layer

- The active-segment path cost uses the path normal and tangent.
- Cross-track error is split into inside-tube and outside-tube parts.
- Outside-tube error is weighted more strongly than inside-tube error.
- Along-track distance is penalized as remaining distance to the segment end.
- Heading error is measured relative to a previewed segment heading, not only the current segment heading.
- Speed terms penalize excess speed and low-speed violation relative to a soft floor.
- Regularization terms penalize sway, yaw rate, and actuator-state magnitudes.
- Stage input tracking penalizes deviation from `u_ref`.
- Obstacle slacks are penalized stage-by-stage inside the objective.

#### Terminal cost layer

- Final position error is measured relative to `wp_end` or the berth target.
- Final heading error is relative to the scheduled goal heading.
- Residual surge, sway, and yaw rate are penalized at the terminal node.
- Terminal actuator magnitudes and reverse-motion preferences are also penalized.
- Terminal slacks are added as a quadratic penalty.
- A final line-recapture term penalizes terminal cross-track and terminal heading to the active line.

The chapter should be explicit that the terminal objective is not just “go to the last point”; it mixes docking-point regulation, line recapture, and motion damping.

### 5.13.5 Exact cost-shaping logic that the chapter should not compress away

- The active-segment heading reference can blend toward the next segment when the route turns sharply.
- A preview gain is computed from the turn angle.
- The scheduler can smoothly smooth goal-heading changes over several steps so that the reference does not jump abruptly.
- A sharp-turn mode can increase heading weights early if the next segment requires it.
- Stop pressure can increase heading anchoring and can lower the forward-speed lower bound.
- Yield pressure can reduce progress urgency and increase smoothness penalties.

This logic should be described because it is one of the distinctive parts of the original NMPC architecture.

### 5.13.6 Constraint block structure in the NLP

The NLP is assembled as a list of named constraint blocks. The chapter can mention the blocks explicitly.

- initial condition equality
- dynamics equality over all shooting steps
- obstacle CBF inequality block
- map half-plane CBF inequality block
- azimuth-rate inequality block
- brake-deceleration inequality block
- terminal pose and velocity inequality block
- berth corridor inequality block
- twin-stern synchrony inequality block

The solver stores block names and block sizes for diagnostics, so the chapter can say that the implementation is structured and debuggable rather than being a monolithic constraint bundle.

### 5.13.7 Collision geometry and obstacle modeling

This is a major implementation detail and should not be treated generically.

- The default collision model is `oriented-rectangle`.
- In that mode, the vessel hull is represented as a rotated rectangle with half-length and half-beam dimensions.
- For circle obstacles, the barrier function uses distance from the ship position or the hull footprint to obstacle center minus radius and clearance.
- For map boundaries, the code can use half-plane constraints derived from the harbor polygon.
- Map obstacle mode can also build circle samples from boundary points when the configured model is `circle`.
- Dynamic obstacles are turned into circle obstacles for NMPC and can be augmented with latent planning-only obstacles.

The chapter should mention that the project supports both local sampled obstacles and boundary-style half-planes, depending on the run configuration.

### 5.13.8 Discrete-time CBF form and sign conventions

- The obstacle safety constraint is written on discrete prediction nodes.
- It uses the barrier at `k` and `k+1`.
- It includes `P_gamma_obs` and a slack term.
- The sign convention in the code is consistent with an inequality lower-bounded at zero.
- The terminal obstacle version is a special case with no `(1-gamma)` term.

This should be stated clearly because the thesis should not present a continuous-time affine CBF decomposition if the code is implementing a discrete-time shooting-node inequality.

### 5.13.9 Actuator and vehicle-model details that matter to the control formulation

The chapter should include enough model detail to show that the controller is actuator-aware.

- The vessel is 175 m class container-ship geometry with twin stern azipods and a bow tunnel thruster.
- The prediction model includes hydrodynamic surge, sway, and yaw terms.
- Thruster thrust is computed with inflow, advance-ratio-like terms, and thrust coefficients.
- The shaft-speed states are first-order actuator states, not ideal controls.
- Command saturation is applied to shaft commands before shaft-speed dynamics.
- Shaft-speed derivatives are rate-limited by `Dn_max` and `Dn_bow_max`.
- The model includes forward motion and reverse motion, but the scheduler can bias toward forward motion depending on phase.

This detail is useful because it explains why the NMPC can produce realistic actuator behavior and why the effort terms matter.

### 5.13.10 Bounds and feasibility structure

The chapter should not forget that the optimization is heavily bounded.

- The first state block is fixed to the measured initial state.
- Surge, sway, yaw rate, positions, heading, and shaft-speed states all have explicit bounds.
- Control inputs have azimuth bounds and shaft command bounds.
- Obstacle slacks are nonnegative and upper-bounded.
- Terminal slacks are nonnegative and upper-bounded.
- Soft obstacle slack upper bounds are reduced in some scheduled phases so the optimizer prefers real avoidance over large violations.
- Terminal slack upper bounds can be reduced or tightened near berth capture.

This is important because the project uses bounded slacks as a feasibility-recovery mechanism rather than leaving the slack space unbounded.

### 5.13.11 Solver settings and warm-start behavior

The chapter should mention the numerical solver choices.

- The NLP is solved with IPOPT through CasADi `nlpsol`.
- The solver uses relatively strict but practical tolerances suitable for online use.
- Warm-start is enabled.
- The code passes both primal and dual warm-start values when available.
- The warm-start state is cleared if the waypoint segment or goal heading changes too much.
- If a previous solution exists, the state, control, and slack trajectories are shifted forward with the last element repeated.
- If no previous solution exists, the initial guess is built from repeated current state and a single-step forward rollout.

This should be mentioned because warm-start is part of the actual online NMPC performance.

### 5.13.12 Online scheduling logic from `run_nmpc.m`

The chapter needs to explain that the controller is not a fixed-weight optimizer; it is constantly scheduled.

#### Route-following and turn anticipation

- The script computes the active segment index and the next waypoint.
- It estimates the current turn angle.
- It can blend the terminal heading toward the next segment before the handoff.
- Sharp turns can increase heading weights and terminal guidance weights.

#### Berthing logic

- Berthing mode is activated near the final target or on the last segment.
- The final target and final heading can replace the generic waypoint endpoint.
- A corridor aligned with the docking attitude can be enabled.
- Berthing can be previewed before full activation.

#### Tightness and safety logic

- Clearance to map edges influences a caution lambda.
- Stopping distance influences a stop lambda.
- Turn severity influences a turn lambda.
- Dynamic obstacles influence TTC/yield/return lambdas.
- Berthing influence combines preview and strict final approach.
- The maximum of these lambdas drives the master caution intensity.

#### Weight adaptation

- Tube width is tightened in cautious phases.
- XTE weight increases in tight geometry and near berth.
- Heading weight rises during turns, yield, and berthing.
- Soft speed-cap and speed-floor weights are increased near caution conditions.
- Forward-preference is reduced in final berth or yield situations.
- Terminal pose and terminal stop weights are ramped up near berth capture.
- Map barrier weight is increased near tight boundaries and return phases.

This deserves explicit coverage because it is one of the key engineering contributions of the implementation.

### 5.13.13 Dynamic obstacle handling

The chapter should make clear how moving obstacles are incorporated.

- Dynamic obstacles can start immediately or activate when the vessel gets close.
- Their positions are propagated with a simple motion model.
- They are packaged into NMPC obstacle structs.
- They can be expanded into latent awareness obstacles for planning conservatism.
- Time-to-collision, closing speed, CPA distance, and sector filters are used to derive yield/return intensities.

This should be mentioned because the framework is not only static-obstacle-aware.

### 5.13.14 Map handling and harbor boundary logic

- The harbor map is loaded from the project map files.
- The implementation can extract local half-planes from map edges.
- It can also sample circle obstacles from the map boundary.
- The lookahead region is based on current speed and configurable distance limits.
- The corridor or boundary checks can change with the vessel’s local situation.

This is important for making the chapter clearly about harbor navigation rather than open-water path tracking.

### 5.13.15 Runtime diagnostics, logs, and output generation

The chapter should mention the data generated by `run_nmpc.m` because it supports validation.

- state trajectory
- control trajectory
- predicted trajectories
- solver success flags
- solver times and step times
- cross-track error history
- waypoint index history
- heading reference history
- obstacle packaging metrics
- lambda histories for tightness, stop, turn, berth, yield, return, and TTC
- slack histories
- collision flags
- brake margin logs
- path weight histories
- map barrier weight history

The main script also writes animation outputs and terminal logs to the `plots in development process` subfolders, which are useful for the results chapter.

### 5.13.16 Mission completion and terminal capture

The chapter should briefly describe the mission termination logic because it is a practical part of the controller behavior.

- The final target can be the last waypoint or the berth target depending on mode.
- There are hard and soft capture radii.
- Capture can depend on position, speed, and heading criteria.
- A safe terminal stop gate can be applied if the final area is map-constrained.
- A hold-time condition prevents premature termination.

This is worth mentioning because the controller is not only trying to minimize a cost; it is being checked against explicit completion conditions.

### 5.13.17 Diagnostics and helper scripts worth mentioning explicitly

If the chapter needs traceability, mention these scripts as supporting evidence:

- `verify_nmpc_integration.m`: verifies the main loop, solver call, and RK4 integration path.
- `run_nmpc_regression.m`: batch comparison across scenarios.
- `diagnose_heading_logic.m`: heading preview and sharp-turn logic.
- `test_heading_logic_detailed.m`: step-by-step heading analysis.
- `diagnose_map_obstacles.m`: map obstacle extraction and half-plane/circle conversion.
- `call_nmpc_once.m`: minimal call to inspect solver packaging.
- `analyze_waypoints.m`: route length and turn-angle analysis.
- `zone_creator.m`: editing map forbidden zones.

The chapter need not explain each script in detail, but it should mention which ones are relevant to verifying each subsystem.

### 5.13.18 Related code artifacts that are not core but still relevant

- `NavUtils.m` provides reusable geometry and map validation utilities.
- `HarborAnimHelper.m` is a small adapter for map plotting in animations.
- `container.m` is the simulation and model reference used both by the plant and the prediction model wrapper.
- `backups/2026-05-08` contains historical versions and can be used if comparison with older formulations is required.

### 5.13.19 What a faithful chapter draft must make obvious

- The controller is a single NMPC with phase-dependent scheduling, not separate controllers stitched together.
- The active segment is the basic path primitive.
- Tube tracking, terminal docking, and obstacle avoidance all coexist in the same optimization problem.
- The optimization is directly constrained, not post-processed.
- The plant simulation and the NMPC prediction model are distinct integrators.
- Slack variables are a deliberate feasibility tool, not a last-minute patch.
- The scheduler is central, not auxiliary.

### 5.13.20 One-line fallback summary for an AI chapter drafter

The original project implements a single MATLAB/CasADi NMPC for a 175 m twin-azipod-plus-bow-thruster vessel, with direct multiple shooting, explicit-Euler prediction, RK4 simulation, active-segment line following, tube-based progress cost, discrete-time CBF obstacle avoidance with slacks, map half-plane or circle obstacles, berth corridor and terminal pose envelopes, warm-started IPOPT solution, and continuous scheduling of weights and limits based on transit, turn, yield, and final-berthing phases.

## 5.14 Helper-layer function inventory that the chapter should not omit

The helper class `RunNmpcHelpers.m` is not a minor utility file; it is part of the implementation architecture. The chapter should mention it whenever the text discusses how the main script constructs geometry, obstacles, corridors, or diagnostics.

### 5.14.1 Waypoint and segment helpers

- `updateWaypointIndexManaged`: advances the active waypoint index based on projection, cross-track error, and turn severity.
- `getWaypointTurnAngleDeg` and `getNextTurnAngleDeg`: estimate the next route bend for preview and scheduling.
- `computeXTE`: computes signed cross-track error to the active segment.

These functions are directly connected to the route-following logic in `run_nmpc.m`.

### 5.14.2 Berth configuration helpers

- `normalizeBerthCfg`: fills defaults, aligns corridor origin, derives target heading defaults, and ensures pose/velocity fields have consistent shapes.
- `distanceToBerthCorridorEntry` and `distanceToBerthCorridorEnd`: support the berth/approach lambda logic.

The chapter should mention that berthing is not hard-coded only in the main script; it is normalized by the helper layer.

### 5.14.3 Map extraction and obstacle packaging helpers

- `buildMapSamplePoints`: samples map polygons into discrete points for circle obstacle generation.
- `buildMapHalfPlaneEdgeSet`: converts map polygons into compact edge records with midpoint, tangent, normal, and length.
- `selectMapObstaclesFromSamples`: selects sampled boundary points into a local circle-obstacle set.
- `selectMapHalfPlanesFromEdges`: selects nearby edges and converts them into local half-plane constraints.
- `nearestDistanceToMapEdges`: computes local map clearance for scheduling.

The chapter should explicitly state that the map is handled through local geometric extraction, not by solving a global path-planning problem online.

### 5.14.4 Dynamic-obstacle lifecycle helpers

- `buildDynamicObstaclesFromConfig`: creates the moving obstacles from raw configuration arrays.
- `configureDynamicStartMode`: sets immediate or proximity-triggered activation.
- `activateDynamicObstaclesByProximity`: toggles motion when the vessel gets close.
- `propagateDynamicObstacles`: advances obstacle positions with deterministic motion and boundary policy.
- `dynamicToCircleObstacles`: converts active moving obstacles into NMPC-compatible circle obstacles.
- `buildLatentDynamicAwarenessObstacles`: creates future occupancy circles for conservative planning.
- `computeDynamicPackagingDrift`: checks that obstacle packing into NMPC remains consistent.
- `runDynamicReplayCheck`: deterministic replay test for obstacle propagation.

These functions are important because dynamic-obstacle behavior is not just a solver constraint; it has its own lifecycle management in the runtime loop.

### 5.14.5 Collision and hull-geometry helpers

- `buildHullFootprintConfig`: converts nominal ship length and beam into a reduced oriented-rectangle footprint and clearance.
- `detectHullCircleHit`: performs post-simulation hull-vs-circle collision checks.
- `detectHullMapHit`: performs post-simulation hull-vs-map collision checks.
- `buildHullPolygon`: creates the world-frame rotated hull polygon used in map collision checks.
- `hullHitsPolygonSet`, `polygonsIntersectOrContain`, `segmentsIntersect2D`, `orient2d`, and `onSegment2D`: geometric primitives supporting map collision detection.

The chapter should note that the online NMPC and the post-run validation both use oriented-rectangle collision logic, even if the specific constraint form differs.

### 5.14.6 General utility helpers

- `getOr`: safe field getter with defaults.
- `normalizeObstacleSchema`: standardizes obstacle structs before concatenation.
- `normalizeStaticObstacles`: converts static-obstacle input into a canonical struct array.
- `sat01`, `ramp01`, `revRamp01`, `lerp`: scalar scheduling primitives used extensively in `run_nmpc.m`.
- `wrapToPi`, `wrapTo180Deg`, `wrapLinear`: angular and linear wrapping utilities.
- `safePercentile`: robust percentile for logging.
- `plotMapBackground`: map plotting helper used in diagnostics and animations.

The chapter should mention these only insofar as they support the scheduling and logging architecture.

## 5.15 Exact run-time loop sequence in `run_nmpc.m`

This section should be represented in the chapter as the online control loop.

### 5.15.1 Initialization phase

- Configure waypoints, static obstacles, dynamic obstacles, berth settings, and scheduler parameters.
- Normalize berth and obstacle structures.
- Set up the hull footprint model and map extraction settings.
- Build helper function handles.
- Preallocate logs, solver histories, and visualization outputs.

### 5.15.2 At each control step

1. Update the waypoint index using `updateWaypointIndexManaged`.
2. Determine the active segment `[wp_start, wp_end]` and next waypoint.
3. Compute active-segment heading and optional preview heading.
4. Activate berth preview or berth mode if distance and route conditions are met.
5. Propagate dynamic obstacles if enabled.
6. Build the local obstacle set and local half-plane set.
7. Assemble `solve_opts` from base tuning, geometry, and scheduler lambdas.
8. Apply additional continuous scheduling for speed, XTE, heading, terminal weights, corridor width, and safety.
9. Call `nmpc.solve(...)` with the measured state, path reference, obstacles, previous input, and solver options.
10. Log solve outputs, slack magnitudes, and timing.
11. Integrate the plant with `rk4Step9`.
12. Run collision checks against static, dynamic, latent, and map obstacles.
13. Update logs, progress printouts, and completion conditions.

The chapter should be written so a reader can reproduce the runtime sequence from these steps alone.

### 5.15.3 Failure handling and fallback behavior

- If the solver fails, the script falls back to a safe input based on the current actuator states.
- Failure counters are tracked.
- Diagnostic messages are printed when slack or runtime thresholds are exceeded.
- The mission can still terminate on safety gates even if berth capture is not perfect.

This is part of the actual implementation and should not be omitted from the chapter.

### 5.15.4 Mission termination logic

- The final capture target can be the generic route endpoint or the berth target.
- Capture can be hard or soft.
- The controller checks distance, speed, heading, and pose envelope conditions.
- A safe-terminal-stop mode is available when the final goal lies in constrained map geometry.
- A hold-time condition prevents accidental completion.

This is relevant because it shows how the controller is judged successful in the simulation.

## 5.16 Additional implementation subtleties worth preserving in the chapter outline

### 5.16.1 The active path is not a global spline

- The controller operates on the current line segment only.
- There is no full-horizon reference trajectory from waypoints.
- The next segment is only used for preview and turn anticipation.

### 5.16.2 Route switching is geometry-aware

- A waypoint handoff occurs only when projection, distance-to-waypoint, and cross-track conditions are satisfied.
- Sharp bends delay handoff until the ship is properly aligned.
- Final approach can override normal route behavior.

### 5.16.3 Heading references are smoothed

- Large heading jumps are smoothed over several steps.
- The solver sees a continuously changing reference rather than a discontinuous one.

### 5.16.4 The scheduler is layered, not single-factor

- Clearance, stopping distance, turning geometry, berth proximity, and dynamic obstacle geometry each contribute a lambda.
- The max or blended combination of lambdas drives final parameter values.
- This is why the same NMPC can behave differently in open water, in a narrow passage, or near docking.

### 5.16.5 Final-approach behavior is partly enforced through weights

- The terminal pose envelope is tightened.
- Terminal heading weight increases.
- Path tube width decreases.
- XTE penalty rises.
- Speed cap and speed floor change.

This should be emphasized so the chapter does not overstate the role of a single hard berth constraint.

## 5.17 Short list of things the chapter still needs to mention explicitly if space permits

- The solver is built on demand and cached after the first build.
- Diagnostic block names are available for constraint debugging.
- The objective uses stage-state, input-tracking, terminal-state, terminal-actuator, and terminal-forward scaling factors.
- `P_q_state`, `P_r_input`, and `P_r_rate` are passed as diagonals and clipped nonnegative in the solve call.
- The map-barrier penalty is an extra soft term separate from the CBF inequalities.
- The terminal line-recapture term is separate from the berth pose envelope.
- `u_min_forward` can be lowered or made negative near berth or turn/yield states.
- The controller can keep the ship in reverse only in tightly controlled phases.
- The plant and solver use the same vessel physics family but different integration wrappers.
- `container.m` and the CasADi prediction model share the same dynamics logic but are expressed in different computational forms.

## 5.18 Ultra-short canonical summary for the beginning of the chapter outline

The original NMPC is a single, heavily scheduled, direct multiple-shooting optimizer for a 175 m container ship with twin azipods and a bow thruster. It follows an active route segment with tube-based progress tracking, uses explicit-Euler prediction in the solver and RK4 in the plant simulation, enforces dynamics, actuator, map, berth, and obstacle CBF constraints, and relies on a layered scheduler in `run_nmpc.m` to transition from transit to turn anticipation to final berthing while keeping the problem feasible through quadratic slack penalties.

## 5.19 Visualization and post-run inspection layer

The chapter should not spend many pages on plotting code, but it should mention the visualization layer because it is how the project was debugged, validated, and communicated.

### 5.19.1 `animateSimResult.m`

- Replays the saved trajectory on a 2-D harbor map.
- Shows the vessel as an image-based icon when available, or a geometric fallback when not.
- Supports a speed subplot alongside the spatial animation.
- Can overlay planned route history, thruster arrows, obstacle circles, hull hitbox outlines, and extra trajectories.
- Supports GIF and MP4 export.
- Can run with light or dark themes.

The chapter should mention this file when talking about how the controller behavior was inspected and how the results figures were produced.

### 5.19.2 `HarborAnimHelper.m`

- Wraps a map struct so `animateSimResult.m` can call `plotMap()` without requiring the full harbor class.
- Draws polygon hazards and map polygons in a consistent way.

### 5.19.3 What the visualization layer reveals about the NMPC

- Whether the route was followed segment by segment.
- Whether the active corridor and berth corridor were respected.
- Whether thruster commands were smooth or oscillatory.
- Whether the ship icon and hull footprint remained inside the navigable region.
- Whether the predicted routes and actual motion aligned over time.

This is relevant because the animation layer is part of the evidence chain for the thesis.

## 5.20 Historical and alternative planning artifacts

These files are not the main NMPC implementation, but they belong in the chapter notes if you want the downstream AI to understand how the project evolved.

### 5.20.1 `giacomo's/DINAMIC_rrtx.m` and `giacomo's/RRTXData.m`

- Implements an RRT-X style non-holonomic naval path planner.
- Uses a handle-class tree container with node arrays, parents, cost values, and neighbor structures.
- Supports dynamic-obstacle-ready data structures and edge-block flags.
- Produces a robot-to-goal path matrix that can be saved to MAT format.

This is not the current NMPC controller, but it shows that the broader workspace also contains a sampling-based planning branch. If the thesis chapter needs to contrast optimization-based control with historical path-planning methods, these files are the relevant reference.

### 5.20.2 `testing scripts/container_old_2azipods.m`

- Legacy 2-azipod vessel model retained for comparison.
- Useful if the chapter briefly explains how the project moved from older dynamics assumptions to the current twin-azipod-plus-bow-thruster model.

### 5.20.3 Why these historical files matter

- They show the project did not start from the final NMPC formulation.
- They provide context for alternative navigation strategies and older vehicle models.
- They should be mentioned only as development history, not as the main implementation.

## 5.21 Final completeness reminder for the chapter drafter

If you want the next AI to write the chapter without missing important implementation details, it should be able to recover the following from this file alone:

- the solver class name and the exact role of each state/control/slack block
- the path-segment-based control philosophy
- the Euler-vs-RK4 distinction
- the actual path, terminal, obstacle, corridor, and actuator terms in the objective
- the discrete-time CBF form on shooting nodes
- the berth scheduling and continuous blending logic
- the map extraction, obstacle conversion, and dynamic obstacle lifecycle
- the helper-function pipeline that supports the main loop
- the visualization and diagnostics layer
- the existence of historical planners and legacy models in the same workspace

If one of those items is missing, the chapter draft will almost certainly become too generic.

## 5.22 Diagnostic subtleties and historical variants that may still matter

This section is for edge cases that are easy to forget but are present in the repository and can affect how the chapter should be written.

### 5.22.1 Heading-reference precedence and possible discontinuities

- `diagnose_heading_logic.m` explicitly checks that `solve_opts.goal_heading_enable` and `solve_opts.goal_heading_rad` override the values carried in `path_ref`.
- The script also flags that berth-preview activation can create a discontinuous heading goal when the berth heading differs strongly from the current segment heading.
- Sharp-turn mode uses a smaller heading weight and shorter activation distance in the current configuration, while berth preview can impose a different goal heading.
- `test_heading_logic_detailed.m` exists to log heading error, goal-heading enable transitions, and goal-heading changes over multiple segments.

The chapter should mention this because heading continuity is part of the scheduler design and not just an incidental implementation detail.

### 5.22.2 Fallback control when NMPC fails

- If the NMPC solve fails, `run_nmpc.m` falls back to a control input built from current actuator states.
- The diagnostic scripts note that this fallback is conceptually a PID-style or simple heading-based recovery path, not a full alternate controller.
- `call_nmpc_once.m` is a minimal harness intended to force a single solver call and inspect parameter packing, map-half-plane injection, and debug prints.

The chapter should acknowledge that online NMPC failure is handled with a safe fallback, even though the fallback is not the thesis contribution.

### 5.22.3 Map editing / forbidden-zone authoring

- `zone_creator.m` is an interactive utility for adding forbidden polygons to the map.
- It allows the user to click polygon vertices, close the polygon, and save an updated MAT file.
- This is useful if the thesis chapter needs to explain how map data can be manually curated before running the NMPC.

### 5.22.4 Backup run configuration differences

- `backups/2026-05-08/run_nmpc.m` shows an earlier or alternative configuration with different waypoint sequence, obstacle positions, berth heading, corridor parameters, sharp-turn threshold, and dynamic-obstacle settings.
- `backups/2026-05-08/NMPC_Container_final.m` is the corresponding solver snapshot for that earlier configuration.

These backups are not the main implementation, but they prove that the project evolved through multiple tuned configurations. The chapter should only refer to them if a historical comparison is needed.

### 5.22.5 Regression harness meaning

- `run_nmpc_regression.m` is not just a test script; it encodes a small scenario matrix with baseline, dynamic-busy, tight-dynamic, and long-stress cases.
- It parses logs for final capture, collisions, solve percentage, mean/max XTE, and phase-B occupancy.
- The pass criteria require successful final capture, zero collisions, and high solver reliability.

This should be mentioned if the chapter discusses how the NMPC behavior was validated across multiple operating conditions.

### 5.22.6 Map-obstacle diagnostic meaning

- `diagnose_map_obstacles.m` re-implements the same map-sampling and half-plane extraction logic locally to verify that the selected circle obstacles and half-planes look sensible.
- It saves both a PNG figure and MAT data for downstream inspection.
- This matters because the chapter may need to explain that the map-derived obstacle set is not arbitrary; it is reproducible from the harbor polygons.

### 5.22.7 Current-vs-legacy dynamics notation

- The current `my work/container.m` implementation uses the 9-state twin-azipod-plus-bow-thruster model.
- The older `testing scripts/container_old_2azipods.m` is only for comparison.
- The thesis should consistently use the current 9-state model and not drift back into a 2-azipod-only description.

### 5.22.8 Final “do not guess” reminder for the drafting AI

If the next AI is to write the chapter correctly, it must not guess any of the following:

- whether the heading reference comes from `path_ref` or `solve_opts` in a given mode
- whether berth preview changes the heading goal abruptly or gradually
- whether the solver or the plant uses RK4
- whether slacks are linear or quadratic
- whether map obstacles are circles, half-planes, or both
- whether the controller uses a separate fallback controller on failure
- whether the ship model is 2-azipod or 3-thruster

All of those points are already encoded in the repository and now in this outline.
