## Why

The current berthing workflow is limited by actuator-angle wrapping, uncertain actuator-limit enforcement, and point-mass vessel geometry assumptions, which together can make precise docking infeasible in constrained environments. This change is needed now to enable reliable, obstacle-aware, direction-constrained docking at a precise berth pose.

## What Changes

- Add support for multi-turn azipod angle representation in NMPC constraints so heading reversals can be achieved through the shortest physically valid rotation direction, not forced wrap-back through a single principal angle.
- Verify and correct NMPC actuator-limit enforcement (azipod angle, azipod rate, thrust magnitude, and any configured actuator saturations) so commanded controls always satisfy configured limits.
- Introduce a rectangular vessel hitbox model (aligned to vessel body frame and projected to world frame) instead of point-only geometry for collision and clearance checks.
- Extend guidance with berth-approach direction and target heading selection, including obstacle-aware approach adaptation, to support precise final docking orientation and position.

## Capabilities

### New Capabilities
- `azipod-multi-rotation-constraints`: Unwrapped azipod-angle state/constraint handling that allows multiple rotations while respecting rate and hard limits.
- `nmpc-actuator-limit-compliance`: Deterministic NMPC control output validation and enforcement against configured actuator bounds.
- `vessel-rectangular-hitbox`: Ship-body rectangular footprint and collision envelope for planning and feasibility checks.
- `berth-directional-approach-guidance`: Guidance logic that selects and tracks docking approach direction and final berth heading under obstacle constraints.

### Modified Capabilities
- None.

## Impact

- Affected subsystems: NMPC formulation, constraint encoding, guidance/berthing logic, and collision-checking geometry.
- Likely affected code areas: `MY_NPMC/container.m`, NMPC setup/solver interfaces, guidance modules, and scenario validation scripts.
- Behavioral impact: changes to control trajectories, actuator command continuity, and final docking approach decisions.
- Validation impact: requires new scenario tests for reverse approach, obstacle-constrained berth entry, and strict actuator-bound compliance.
