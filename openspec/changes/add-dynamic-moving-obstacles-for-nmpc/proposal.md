## Why

Current NMPC tests primarily use static obstacles and fixed harbour map zones, which does not exercise the controller against realistic moving hazards. A simple, deterministic moving-obstacle mode is needed now to validate whether the present NMPC stack can proactively avoid forward-moving traffic.

## What Changes

- Add dynamic obstacle generation in map/scenario setup so obstacle objects can move each simulation step.
- Introduce an initial motion model for dynamic obstacles: straight-line forward motion, zero turn rate, constant speed.
- Extend NMPC obstacle ingestion to include dynamic obstacles using the same compatibility pathways used for static and harbour-derived obstacles.
- Ensure moving obstacles are represented consistently in solver inputs, collision checks, and visualization/debug outputs.
- Add scenario configuration switches and defaults so dynamic obstacle tests can be enabled without breaking existing static-only runs.

## Capabilities

### New Capabilities
- `dynamic-obstacle-generation`: Define and update moving obstacles in simulation maps using constant-speed, no-turn kinematics.
- `nmpc-moving-obstacle-interpretation`: Make NMPC and surrounding pipeline components consume moving obstacles with compatibility equivalent to existing static/harbour obstacle handling.

### Modified Capabilities
- None.

## Impact

- Affected code will likely include scenario setup and execution scripts under [MY_NPMC/my work](MY_NPMC/my%20work), including run scripts and obstacle preparation utilities consumed before each NMPC solve.
- Affected interfaces include obstacle data structures passed into NMPC solve calls and collision/termination checks.
- No external dependency changes are expected; this is primarily behavioral and data-flow extension within existing MATLAB code.
