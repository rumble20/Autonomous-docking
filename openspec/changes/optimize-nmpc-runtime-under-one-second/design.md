## Context

The current NMPC pipeline is functionally valid but too slow for practical closed-loop execution on development hardware, and performance characteristics are not consistently measured per control stage. The project already includes rich guidance, collision checks, and actuator constraints, so optimization must preserve safety and docking behavior while reducing compute cost.

## Goals / Non-Goals

**Goals:**
- Achieve a nominal NMPC per-step runtime target of <= 1.0 s on representative scenarios, with explicit profiling evidence.
- Add deterministic runtime instrumentation at end-to-end and stage level.
- Introduce tunable optimization controls that allow graceful trade-offs between fidelity and runtime.
- Add deterministic overrun handling that keeps commands safe and bounded.
- Prevent regressions with repeatable runtime benchmark checks.

**Non-Goals:**
- Replacing the NMPC framework or solver family.
- Changing core docking objectives or safety constraints semantics.
- Hardware-specific GPU acceleration requirements.

## Decisions

1. Add a runtime budget contract in configuration and execution loop.
- Decision: Define `runtime_budget_s`, `runtime_warn_s`, and `runtime_hard_limit_s` with per-step evaluation.
- Rationale: Converts performance intent into enforceable behavior and measurable acceptance criteria.
- Alternative considered: best-effort logging only.
- Why rejected: does not guarantee predictable control-cycle behavior.

2. Add stage-timing instrumentation around major NMPC phases.
- Decision: Measure at minimum precompute, dynamics/constraint assembly, solver call, and post-processing.
- Rationale: Hotspots can only be optimized reliably if stage-level costs are visible and comparable across runs.
- Alternative considered: total wall-clock only.
- Why rejected: too coarse to direct optimization work.

3. Optimize with bounded knobs before algorithmic redesign.
- Decision: Prioritize warm starts, cached reusable terms, reduced candidate sets, and configurable horizon/control-grid adaptation.
- Rationale: Provides substantial speed gains with lower implementation risk and minimal behavior change.
- Alternative considered: immediate structural rewrite.
- Why rejected: high risk and long lead time.

4. Deterministic overrun handling path.
- Decision: If runtime exceeds hard limit, emit safe bounded fallback command policy (e.g., previous feasible command with rate-limited update), and log event.
- Rationale: Safety and repeatability must be preserved when deadlines are missed.
- Alternative considered: drop command or bypass checks.
- Why rejected: can destabilize behavior or violate actuator limits.

5. Benchmark-first validation.
- Decision: Introduce fixed-seed scenarios and report p50/p95/max step times plus deadline miss rate.
- Rationale: Stable comparisons are required to prove improvement and prevent regressions.

## Risks / Trade-offs

- [Risk] Runtime optimization reduces prediction fidelity under aggressive settings -> Mitigation: expose bounded tuning ranges and include trajectory-quality acceptance checks.
- [Risk] Caching invalidation bugs lead to stale model terms -> Mitigation: strict cache keys and debug assertions in development mode.
- [Risk] Overrun fallback masks persistent performance issues -> Mitigation: dedicated overrun counters and fail-on-threshold in regression tests.
- [Risk] Benchmark variance across laptops makes gates flaky -> Mitigation: compare against baseline deltas and use stable scenario seeds.

## Migration Plan

1. Introduce config defaults that preserve current behavior when optimization knobs are disabled.
2. Land instrumentation first, collect baseline runtime profile, and store reference metrics.
3. Incrementally enable optimization controls and tune for <= 1.0 s target in representative scenarios.
4. Enable overrun policy and runtime regression checks in normal simulation runs.
5. Rollback strategy: disable optimization knobs and overrun policy via config toggles if behavior regressions appear.

## Open Questions

- What exact scenario set should be the canonical runtime benchmark for acceptance?
- Should runtime target gating be absolute on laptop hardware or relative improvement plus capped deadline-miss rate?
- Which solver options currently available in repository deliver the best speed-quality trade-off for docking scenarios?
