# NotebookLM Project Reference for MY_NPMC / my work

## Project in one sentence
This folder contains a MATLAB-based harbor navigation framework for a 175 m container ship that uses a nonlinear model predictive controller (NMPC) to follow waypoint segments, manage final berthing, avoid static and dynamic obstacles, and generate logs, plots, and animations for analysis.

## What this framework is for
The project is not a generic ship simulator. It is a control and navigation research framework built to study and demonstrate:

- line tracking on harbor routes rather than point-to-point waypoint chasing
- tube-MPC style path following with a corridor around the route
- collision avoidance with static obstacles, dynamic obstacles, and map boundaries
- final-approach and berthing behavior with a dedicated pose and corridor envelope
- actuator-aware ship maneuvering for a twin-azipod plus bow-thruster vessel
- debugging and regression testing for NMPC behavior during route following

The intended use is to support a thesis or literature review about ship guidance, NMPC, tube MPC, collision avoidance, and autonomous berthing in constrained waterways.

## Core idea of the control architecture
The current design centers on a single NMPC formulation that receives only the active path segment, not a full point-tracking reference matrix.

The main control loop does this:

1. Load the harbor map and mission waypoints.
2. Build a path segment from the current waypoint to the next waypoint.
3. Optionally activate berthing mode near the final target.
4. Extract nearby obstacles and map half-planes.
5. Pack the current state, path geometry, and constraints into the NMPC solver.
6. Solve for the optimal control sequence.
7. Integrate the ship model forward with RK4.
8. Log state, control, solver, obstacle, and scheduling data.
9. Animate or save the run for later analysis.

This is important for NotebookLM because most questions about the project should be answered by tracing this flow.

## Main files and their roles

### [run_nmpc.m](run_nmpc.m)
Main experiment script and orchestrator.

It defines the mission, loads the map, configures the ship and NMPC parameters, applies route-following and berthing logic, gathers obstacles, calls the solver, integrates the dynamics, and records outputs.

Key behaviors:

- defines waypoints for the harbor route
- sets static and dynamic obstacles
- creates berthing configuration and final corridor settings
- normalizes map and berth parameters through helper functions
- builds the `NMPC_Container_final` solver
- updates waypoint index and path reference online
- schedules weights and constraints continuously based on geometry, distance, turn severity, and stop conditions
- records trajectory, control, solver timing, and diagnostics
- optionally records animation video and terminal logs

### [NMPC_Container_final.m](NMPC_Container_final.m)
The solver class.

This is the heart of the framework. It uses CasADi and IPOPT to solve a constrained optimal control problem for a 9-state ship model and 5 control inputs.

Important features:

- 9 states: surge, sway, yaw rate, position, heading, and three thruster shaft speeds
- 5 controls: two azimuth angles and three commanded shaft speeds
- line tracking along the active segment `[wp_start -> wp_end]`
- tube cost that penalizes cross-track error strongly outside a corridor
- along-track progress reward or penalty structure
- optional goal heading guidance near sharp turns or final berthing
- obstacle avoidance with soft slack variables and CBF-style constraints
- map half-plane constraints for harbor boundary awareness
- terminal pose and velocity envelope at the final target
- twin-stern synchrony limits for azimuth and stern RPM differences
- braking-rate limit and forward-motion preference
- warm-start support and solver statistics

### [container.m](container.m)
Full nonlinear ship dynamics model.

This file computes the 9-state time derivative for the vessel. It includes:

- simplified 6-DOF-style ship dynamics with roll removed
- hydrodynamic terms based on Son and Nomoto style coefficients
- twin stern azipod thrust models
- bow tunnel thruster model
- actuator dynamics for shaft speeds
- azimuth steering effects and thrust force decomposition

This model is used both in the simulation loop and inside the NMPC prediction model through `dynamicsCasADi`.

### [RunNmpcHelpers.m](RunNmpcHelpers.m)
Shared utility class for the main loop.

This file contains static helper functions for:

- waypoint progression and segment switching
- turn-angle estimation
- berth configuration normalization
- cross-track error computation
- RK4 stepping wrapper for the 9-state model
- map sampling and half-plane extraction
- conversion from map geometry to local obstacle representations
- dynamic obstacle creation, activation, propagation, and packaging
- geometry helpers such as wrapping and interpolation
- collision checks and plotting helpers

### [NavUtils.m](NavUtils.m)
General navigation utility library.

This class provides reusable math and map-validation tools such as:

- angle wrapping
- vector normalization
- saturation and clamping
- point-to-segment distance
- path validation against map polygons
- map-boundary and polygon occupancy checks

### [animateSimResult.m](animateSimResult.m)
Post-run animation and visualization.

This function replays the recorded trajectory on a 2-D harbor map, shows the ship icon or fallback shape, and can display:

- trajectory history
- planned route history
- thruster commands
- collision circles
- hull hitbox outline
- speed subplot
- video or GIF output

### [HarborAnimHelper.m](HarborAnimHelper.m)
Minimal adapter for map plotting in animation.

It wraps a map struct so `animateSimResult` can call `plotMap()` without requiring the full harbor obstacle class.

## Mission and vehicle model used by the framework

### State vector
The main NMPC and simulation model use the same 9-state vessel representation:

- `u` surge velocity
- `v` sway velocity
- `r` yaw rate
- `x` position in the global frame
- `y` position in the global frame
- `psi` heading angle
- `n1` port stern azipod shaft speed
- `n2` starboard stern azipod shaft speed
- `n3` bow tunnel thruster shaft speed

### Control vector
The 5 controls are:

- `alpha1` port stern azimuth angle
- `alpha2` starboard stern azimuth angle
- `n1_c` port stern shaft command
- `n2_c` starboard stern shaft command
- `n3_c` bow thruster shaft command

### Physical ship assumptions
The project uses a 175 m container ship with:

- twin stern azipods that can steer independently
- a bow tunnel thruster for lateral low-speed authority
- hydrodynamic coefficients adapted from a classical container-ship maneuvering model
- actuator rate and magnitude limits
- a forward-motion preference, but with room for reverse motion during docking

## How the NMPC objective is organized
The solver cost in `NMPC_Container_final.m` is built from several interacting terms.

### Stage cost
At each prediction step, the solver penalizes:

- cross-track error relative to the segment line
- distance remaining along the current segment
- heading deviation from the segment direction or preview heading
- speed above or below a soft target window
- thruster and shaft effort
- control tracking error relative to a reference input guess
- soft obstacle slack variables

The cross-track penalty is intentionally tube-like: inside the tube the penalty is lighter, outside the tube it becomes heavier.

### Terminal cost
At the final prediction point, the solver penalizes:

- position error to the waypoint or berth target
- heading error to the goal heading if enabled
- residual surge, sway, and yaw motion
- thruster effort and reverse-motion preference
- terminal slack variables

### Safety and constraint structure
The solver also imposes:

- dynamic constraints from the ship model
- obstacle avoidance via CBF-style inequalities
- half-plane constraints for map boundaries
- azimuth rate constraints
- brake deceleration constraints
- terminal pose and velocity envelopes
- berth corridor constraints when berth mode is active
- twin-stern synchrony limits

## Route-following and berthing logic
The main script does not simply chase each waypoint independently.

Instead it maintains a current segment and updates the segment index using waypoint progress logic. The current active path is always the line from the current waypoint to the next waypoint.

Important details:

- the route is represented as a sequence of 2-D waypoints
- the solver sees the active segment, the next segment, and optionally a goal heading
- sharp turns can trigger extra heading anticipation
- the final waypoint can trigger berthing mode
- berthing mode can replace the terminal target and activate a corridor aligned with the docking attitude

This design is relevant for a literature review because it is a practical example of switching from transit guidance to final-approach guidance without changing the core NMPC formulation.

## Obstacle and map handling
The project supports multiple obstacle representations.

### Static obstacles
Static obstacles are usually encoded as circles or as structs with position and radius.

### Dynamic obstacles
Dynamic obstacles can:

- start immediately or on proximity trigger
- move with heading and speed
- be converted into NMPC circle obstacles
- be expanded into latent planning-only awareness obstacles

### Map obstacles
The harbor map is loaded from `helsinki_harbour_UPDATED.mat` when available.

The framework can use either:

- circle sampling from map edges
- local half-plane constraints built from map polygon edges

This lets the same controller work with either obstacle-style local sampling or boundary-style half-plane constraints.

### Collision model
The NMPC solver uses an oriented-rectangle collision model by default, so the hull footprint is treated as a rotated rectangle rather than a point.

## Scheduling behavior in the main loop
`run_nmpc.m` contains a continuous scheduler that adjusts solver parameters based on context.

It reacts to:

- clearance to map edges
- stopping distance estimates
- upcoming turn severity
- berth activation distance
- route tightness
- dynamic obstacle interaction risk

The scheduler adjusts weights and limits such as:

- cross-track weight
- tube width
- goal heading weight
- terminal stop weights
- berth corridor width
- map barrier weights
- minimum forward speed preference
- control-rate scaling

This is one of the main reasons the framework is useful for literature review discussion: it is a hybrid of geometric guidance and optimization-based control.

## Simulation and logging outputs
The main run script creates rich diagnostics.

Typical logged items include:

- full state trajectory
- control history
- predicted trajectories
- cross-track error
- solver success rate
- solver times
- obstacle packaging metrics
- collision flags
- heading reference values
- waypoint index history
- scheduler lambda values
- terminal and path weight histories

The project also writes animation outputs and terminal logs into the `plots in development process` folder.

## Testing and diagnostic scripts
The `testing scripts` folder contains small utilities for checking specific behaviors.

### [verify_nmpc_integration.m](testing%20scripts/verify_nmpc_integration.m)
Checks that the main loop instantiates the solver correctly, calls `solve()`, and uses `container.m` inside RK4 integration.

### [run_nmpc_regression.m](testing%20scripts/run_nmpc_regression.m)
Batch regression harness that runs multiple scenarios and parses terminal logs for final capture, collisions, solve rate, and XTE metrics.

### [diagnose_heading_logic.m](testing%20scripts/diagnose_heading_logic.m)
Analyzes heading generation, sharp-turn preview behavior, berth-preview activation, and possible reference discontinuities.

### [test_heading_logic_detailed.m](testing%20scripts/test_heading_logic_detailed.m)
Runs a more detailed step-by-step heading analysis over multiple segments and logs discontinuities and mode transitions.

### [diagnose_map_obstacles.m](testing%20scripts/diagnose_map_obstacles.m)
Checks map sample-point generation, circle obstacle extraction, half-plane generation, and saves a diagnostic figure and MAT file.

### [call_nmpc_once.m](testing%20scripts/call_nmpc_once.m)
Minimal solver call used to trigger parameter packing and inspect the solver input payload.

### [analyze_waypoints.m](testing%20scripts/analyze_waypoints.m)
Evaluates route geometry, segment lengths, turn angles, and route feasibility.

### [zone_creator.m](testing%20scripts/zone_creator.m)
Interactive tool for editing map forbidden zones and saving the updated map structure.

### [container_old_2azipods.m](testing%20scripts/container_old_2azipods.m)
Older ship model reference retained for comparison.

## Stored data, figures, and artifacts
The folder also contains non-code artifacts that matter to interpretation.

### Map files
- `helsinki_harbour.mat`
- `helsinki_harbour_UPDATED.mat`

These are central to the harbor geometry and obstacle interpretation.

### Figures and media
- `heading_analysis.fig`
- files in `plots in development process`
- files in `results`

These outputs show how the controller behaved in prior experiments.

### Backups
The `backups` folder stores earlier versions of the main scripts and solver class. These are useful for version comparison and debugging but are not the primary runtime sources.

## How this project connects to a literature review
If NotebookLM is used to answer literature-review questions, this framework should be interpreted as an implementation case study for several research themes:

- nonlinear model predictive control for marine craft
- tube MPC and corridor-based path tracking
- control barrier functions for collision avoidance
- autonomous harbor navigation in constrained waterways
- final approach and berthing of large vessels
- hybrid guidance logic with route segments, turn anticipation, and terminal docking constraints
- actuator allocation for multi-thruster ships
- map-aware motion planning with local obstacle extraction

The project is therefore a practical bridge between theory and implementation. It is not just a simulator, and it is not just a planner. It is a closed-loop control stack that combines guidance, optimization, dynamics, map reasoning, and visualization.

## High-level reading order for NotebookLM
If you want NotebookLM to answer questions well, the most useful reading order is:

1. [run_nmpc.m](run_nmpc.m)
2. [NMPC_Container_final.m](NMPC_Container_final.m)
3. [container.m](container.m)
4. [RunNmpcHelpers.m](RunNmpcHelpers.m)
5. [NavUtils.m](NavUtils.m)
6. [testing scripts/verify_nmpc_integration.m](testing%20scripts/verify_nmpc_integration.m)
7. [testing scripts/diagnose_heading_logic.m](testing%20scripts/diagnose_heading_logic.m)
8. [testing scripts/analyze_waypoints.m](testing%20scripts/analyze_waypoints.m)

## Short glossary of recurring terms
- XTE: cross-track error relative to the active path segment.
- Tube: a corridor around the desired line where tracking is less aggressively penalized.
- CBF: control barrier function, used here to keep the ship away from obstacles and map boundaries.
- Berth mode: final-approach behavior that activates near the destination and adds corridor and terminal constraints.
- Latent dynamic awareness: planning-only expansion of dynamic obstacles to make the NMPC anticipate future conflict.
- Half-plane constraint: a linear boundary used to approximate map limits locally.
- Oriented rectangle: hull collision model that rotates with the ship heading.

## One-line summary for notebook ingestion
This folder implements a harbor-navigation NMPC framework for a 175 m twin-azipod container ship, with line tracking, tube MPC, CBF safety, map-aware obstacle handling, berthing logic, and diagnostics for route-following research.