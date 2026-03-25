## ADDED Requirements

### Requirement: Dynamic obstacle state initialization
The simulation SHALL support initialization of dynamic obstacles with explicit state fields for position, heading, speed, and radius before simulation starts.

#### Scenario: Initialize a valid obstacle set
- **WHEN** a scenario enables dynamic obstacles and provides initial obstacle definitions
- **THEN** the system stores each obstacle with position, heading, speed, and radius values available for runtime updates

#### Scenario: Preserve static-only behavior
- **WHEN** a scenario disables dynamic obstacles
- **THEN** no dynamic obstacle state is created and existing static obstacle behavior remains unchanged

### Requirement: Constant-speed straight-line propagation
The system SHALL update every enabled dynamic obstacle at each simulation step using constant speed and constant heading with zero turn-rate.

#### Scenario: Update obstacle forward position
- **WHEN** simulation advances by one time step dt
- **THEN** each enabled dynamic obstacle position is advanced using its current heading and speed while heading and speed remain unchanged

#### Scenario: Deterministic replay behavior
- **WHEN** the same initial dynamic obstacle states and simulation settings are replayed
- **THEN** the generated obstacle trajectories are identical across runs

### Requirement: Dynamic obstacle lifecycle handling
The system MUST define deterministic handling for dynamic obstacles that move outside configured map bounds.

#### Scenario: Obstacle leaves supported map region
- **WHEN** a propagated dynamic obstacle state exits configured boundary limits
- **THEN** the system applies the configured policy (for example deactivate, remove, or clip) consistently for that run
