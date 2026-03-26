## ADDED Requirements

### Requirement: Rectangular vessel footprint model
The system SHALL represent the vessel occupancy as a rectangular footprint parameterized by vessel length, beam, and configurable safety margin.

#### Scenario: Footprint generated from vessel parameters
- **WHEN** a simulation or control cycle requests current vessel occupancy
- **THEN** the system returns a rectangle-based footprint consistent with configured vessel dimensions and safety margin

### Requirement: World-frame projection of body-frame hitbox
The collision model MUST transform the body-frame rectangular footprint into world coordinates using current vessel pose.

#### Scenario: Pose change updates world footprint
- **WHEN** vessel heading or position changes between control steps
- **THEN** the world-frame rectangle vertices update consistently with the new pose

### Requirement: Obstacle clearance uses rectangular geometry
Guidance and feasibility checks SHALL evaluate obstacle proximity and collision risk against rectangular occupancy rather than point-only position.

#### Scenario: Narrow berth clearance rejection
- **WHEN** a candidate maneuver clears obstacles as a point but intersects with the rectangular footprint
- **THEN** the candidate is rejected as infeasible
