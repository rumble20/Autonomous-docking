## ADDED Requirements

### Requirement: Directional berth approach selection
The guidance module SHALL select an approach direction toward the berth that considers target pose constraints and local obstacle feasibility.

#### Scenario: Obstacle-aware direction choice
- **WHEN** multiple feasible approach directions exist for the same berth target
- **THEN** guidance selects the direction with valid obstacle clearance and lowest approach feasibility cost according to configured criteria

### Requirement: Terminal heading target for docking
The system MUST support an explicit terminal heading requirement at berth in addition to terminal position.

#### Scenario: Docking to specified heading
- **WHEN** a mission requests a precise berth heading
- **THEN** generated guidance commands converge to both the target berth position and requested terminal heading within configured tolerances

### Requirement: Dynamic approach adaptation near berth
Guidance SHALL re-evaluate approach direction and heading plan when obstacle conditions invalidate the current berthing path.

#### Scenario: Mid-approach obstacle invalidation
- **WHEN** the active approach corridor becomes infeasible due to obstacle interaction
- **THEN** guidance transitions to an alternate feasible approach plan while preserving berth target and safety constraints
