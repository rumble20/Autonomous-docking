## ADDED Requirements

### Requirement: NMPC step runtime budget enforcement
The NMPC execution loop MUST evaluate each control step against configured runtime budgets and classify the step as on-time, warning, or overrun.

#### Scenario: Step completes within nominal budget
- **WHEN** a control step finishes with elapsed time less than or equal to `runtime_budget_s`
- **THEN** the step is recorded as on-time and normal command application proceeds

#### Scenario: Step exceeds warning threshold
- **WHEN** a control step finishes with elapsed time greater than `runtime_warn_s` but less than or equal to `runtime_hard_limit_s`
- **THEN** the system records a warning-level timing event for diagnostics

### Requirement: Deterministic stage-level runtime instrumentation
The controller MUST emit deterministic timing measurements for major NMPC stages at every control step.

#### Scenario: Stage timing report generated per step
- **WHEN** a control step is executed
- **THEN** timing values for precompute, problem assembly, solver execution, and post-processing are logged with the same stage keys and timestamp format

### Requirement: Configurable runtime optimization controls
The system SHALL provide bounded configuration controls for runtime-performance tuning without changing safety-constraint semantics.

#### Scenario: Runtime tuning configuration applied
- **WHEN** optimization controls are enabled in configuration
- **THEN** the NMPC loop applies configured warm-start, caching, and horizon/grid adaptation settings within validated bounds

### Requirement: Safe deterministic overrun handling
The control stack MUST apply deterministic safe-command handling when runtime hard limit is exceeded.

#### Scenario: Hard-limit overrun handling
- **WHEN** a control step elapsed time is greater than `runtime_hard_limit_s`
- **THEN** the system emits a bounded fallback command according to configured policy and records an overrun event

### Requirement: Runtime regression benchmark reporting
The project SHALL provide repeatable benchmark outputs for NMPC runtime to detect regressions.

#### Scenario: Benchmark run produces runtime metrics
- **WHEN** a runtime benchmark scenario set is executed with fixed seeds
- **THEN** the run reports p50, p95, max step time, and deadline-miss rate for comparison against baseline
