## ADDED Requirements

### Requirement: Continuous multi-rotation azipod angle handling
The controller SHALL represent azipod angle with continuity across multiple rotations so optimization is not constrained to a single wrapped interval.

#### Scenario: Heading reversal chooses shortest feasible rotation
- **WHEN** the target maneuver requires a direction reversal across the principal-angle wrap boundary
- **THEN** the optimizer uses an equivalent unwrapped azipod trajectory that minimizes rotation travel while respecting configured rate and hard bounds

### Requirement: Physical-equivalence mapping between unwrapped and physical azipod angle
The system MUST map unwrapped optimization angles to physically equivalent actuator commands without introducing discontinuities in command output.

#### Scenario: Equivalent command emission from unwrapped state
- **WHEN** the internal azipod state exceeds one full turn in either direction
- **THEN** the emitted actuator command remains continuous and physically equivalent for the plant interface

### Requirement: Multi-rotation constraints remain enforceable
The optimization layer SHALL enforce azipod hard-angle and angle-rate limits under the multi-rotation representation.

#### Scenario: Rate limits across wrap boundary
- **WHEN** consecutive control intervals cross a nominal wrap boundary
- **THEN** azipod angle-rate constraints are evaluated on the continuous representation and remain within configured limits
