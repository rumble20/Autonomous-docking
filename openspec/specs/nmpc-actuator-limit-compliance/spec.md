## ADDED Requirements

### Requirement: NMPC output SHALL satisfy configured actuator limits
NMPC control outputs MUST satisfy configured actuator bounds at each control update, including azipod angle, azipod rate, thrust magnitude, and any configured actuator-specific limits.

#### Scenario: Nominal control step within bounds
- **WHEN** NMPC computes a new control action in nominal conditions
- **THEN** every commanded actuator quantity is within configured min/max limits before application to the plant

### Requirement: Limit compliance verification and diagnostics
The system SHALL validate solver outputs against actuator limits prior to plant command application and emit diagnostics on any violation handling.

#### Scenario: Violation detection and handling
- **WHEN** a solver output exceeds a configured actuator limit due to numerical or configuration mismatch
- **THEN** the system applies deterministic violation handling and records a diagnostic event with limit, value, and timestamp

### Requirement: Deterministic violation handling policy
The control stack MUST apply a deterministic policy for out-of-bound solver outputs that preserves safety and repeatability.

#### Scenario: Repeatable handling across identical runs
- **WHEN** identical initial conditions and solver outputs produce an out-of-bound command
- **THEN** the same violation handling action is applied and produces identical bounded commands
