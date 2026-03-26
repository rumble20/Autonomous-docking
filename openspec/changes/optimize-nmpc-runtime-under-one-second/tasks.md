## 1. Runtime Budget Contract and Instrumentation

- [x] 1.1 Add NMPC runtime configuration fields (`runtime_budget_s`, `runtime_warn_s`, `runtime_hard_limit_s`) with backward-compatible defaults.
- [x] 1.2 Instrument per-step end-to-end timing and stage timing (precompute, assembly, solver, post-processing) with deterministic keys.
- [x] 1.3 Add structured runtime event logging for on-time, warning, and overrun classifications.

## 2. Performance Optimization Controls

- [x] 2.1 Implement warm-start reuse across NMPC steps with guards for invalid initial guesses.
- [x] 2.2 Implement cache/reuse of invariant or slowly varying model terms with explicit invalidation conditions.
- [x] 2.3 Add bounded tuning knobs for horizon/control-grid adaptation and candidate-set reduction.
- [x] 2.4 Add configuration validation to prevent unsafe or numerically unstable optimization settings.

## 3. Overrun Safety Handling

- [x] 3.1 Implement deterministic overrun handling policy for steps exceeding `runtime_hard_limit_s`.
- [x] 3.2 Ensure fallback command path preserves actuator bound/rate compliance.
- [x] 3.3 Add overrun diagnostics counters and timestamps for post-run analysis.

## 4. Benchmarking and Regression Gates

- [x] 4.1 Define fixed-seed benchmark scenario set representative of berthing workloads.
- [x] 4.2 Add benchmark runner output for p50, p95, max step time, and deadline-miss rate.
- [x] 4.3 Capture baseline metrics and add regression checks against baseline thresholds.
- [x] 4.4 Add acceptance check for nominal <= 1.0 s per-step target (with documented laptop slack policy).

## 5. Validation and Documentation

- [x] 5.1 Validate that trajectory quality and safety constraints remain acceptable under optimized settings.
- [x] 5.2 Document optimization tuning guidance and recommended profiles (debug, nominal, aggressive).
- [x] 5.3 Document rollback procedure to disable optimization controls and revert to baseline behavior.
