## Why

Current NMPC simulation runtime is too high for practical real-time deployment and iteration speed. Achieving a per-step runtime target near 1 second (with measured slack on laptop hardware) is necessary to make the controller operationally feasible and to de-risk transfer to higher-performance onboard compute.

## What Changes

- Add a runtime-budgeted NMPC execution path with explicit per-step latency targets and acceptance thresholds.
- Add deterministic instrumentation for end-to-end timing and per-stage timing (model evaluation, linearization, solve, post-processing).
- Add optimization measures for computational hot paths, including configurable horizon/control-grid reduction, warm-start strategy, and cached intermediate computations.
- Add fallback/degradation behavior that preserves safe bounded commands when runtime budget is exceeded.
- Add benchmark scenarios and regression checks to prevent runtime regressions.

## Capabilities

### New Capabilities
- `nmpc-real-time-performance`: Runtime budget enforcement, profiling telemetry, optimization controls, and deterministic overrun handling for NMPC execution.

### Modified Capabilities
- None.

## Impact

- Affected code: NMPC setup/solve loop, cost/constraint evaluation path, logging/diagnostics, and scenario runners.
- Affected APIs/config: new runtime-budget and optimization configuration fields; optional telemetry outputs.
- Dependencies/systems: no mandatory external dependency changes expected; optional use of solver and linear algebra settings already available in current stack.
