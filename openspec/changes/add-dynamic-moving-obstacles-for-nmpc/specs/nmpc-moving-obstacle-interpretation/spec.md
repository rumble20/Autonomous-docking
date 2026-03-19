## ADDED Requirements

### Requirement: Unified NMPC obstacle compatibility for moving obstacles
The NMPC integration SHALL accept dynamic obstacles through the same compatibility and formatting pathway used for static and harbour-derived obstacles.

#### Scenario: Merge static, harbour, and dynamic obstacle sources
- **WHEN** NMPC solve inputs are assembled for a simulation step
- **THEN** dynamic obstacles are converted and merged into the same NMPC obstacle input schema used by existing obstacle sources

#### Scenario: Maintain existing obstacle contracts
- **WHEN** dynamic obstacles are enabled in a scenario
- **THEN** existing static and harbour obstacle ingestion behavior remains compatible and continues to operate without interface breakage

### Requirement: Step-synchronized moving obstacle constraints
The NMPC pipeline MUST use the current step dynamic obstacle states when computing avoidance constraints.

#### Scenario: Dynamic obstacle moves between solves
- **WHEN** a dynamic obstacle position changes between simulation steps
- **THEN** the subsequent NMPC solve uses the updated obstacle position for constraint generation

#### Scenario: Dynamic obstacle disabled mid-run
- **WHEN** a dynamic obstacle is marked inactive by lifecycle policy
- **THEN** NMPC input assembly excludes that obstacle from future solve constraints

### Requirement: Dynamic obstacle observability in validation outputs
The simulation and debugging outputs SHALL expose dynamic obstacle states sufficiently to verify avoidance behavior over time.

#### Scenario: Visual verification of moving obstacle track
- **WHEN** visualization or debug output is enabled for a run with dynamic obstacles
- **THEN** output includes time-progressing obstacle states that allow verification that obstacle motion and NMPC avoidance are synchronized
