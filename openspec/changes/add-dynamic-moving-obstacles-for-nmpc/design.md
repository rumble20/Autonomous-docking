## Context

The current NMPC test workflow in MY_NPMC emphasizes static circular obstacles and fixed harbour exclusion zones. This under-tests temporal collision behavior because the obstacle field is effectively time-invariant. The requested change introduces moving obstacles with deterministic motion (constant speed, no turning) so avoidance behavior can be validated without adding high modeling complexity.

Relevant constraints:
- Existing NMPC integration already consumes obstacle bundles for static and harbour-derived constraints.
- Existing simulation and plotting flows should remain backward compatible for static-only scenarios.
- Initial dynamic behavior should be simple and reproducible for debugging and benchmarking.

## Goals / Non-Goals

**Goals:**
- Add scenario-level support for dynamic obstacles that move every simulation step.
- Define a minimal motion model: straight-line propagation with constant heading and speed.
- Ensure NMPC receives moving-obstacle constraints through the same compatibility pathways used for existing obstacle types.
- Keep static/harbour behavior unchanged when dynamic obstacles are disabled.
- Provide clear observability in logs/plots for dynamic obstacle state over time.

**Non-Goals:**
- Multi-agent interaction models (e.g., reciprocal avoidance).
- Obstacle turning, acceleration profiles, or behavior trees.
- Probabilistic prediction, uncertainty tubes, or multi-hypothesis tracking.
- Major solver architecture changes or replacement of existing NMPC backend.

## Decisions

1. Represent dynamic obstacles with explicit kinematic state.
- Decision: extend obstacle records with position, heading, speed, radius, and enabled flag for dynamic updates.
- Rationale: this is the smallest data extension that supports deterministic motion and NMPC compatibility.
- Alternatives considered:
  - Recompute synthetic circles each step without persistent state: rejected because it reduces traceability/debugging and complicates consistent plotting.
  - Full dynamic object classes: rejected for initial scope due to overhead.

2. Use discrete-time forward Euler propagation for motion update.
- Decision: update each dynamic obstacle per step using x(k+1)=x(k)+v*cos(psi)*dt and y(k+1)=y(k)+v*sin(psi)*dt with constant v and psi.
- Rationale: aligns with current simulation stepping, deterministic, and sufficient for first validation.
- Alternatives considered:
  - Higher-order integration: unnecessary for constant velocity/heading model.
  - Continuous-time substepping: increased complexity without clear benefit for this phase.

3. Unify NMPC obstacle ingestion path.
- Decision: dynamic obstacles will be transformed into the same obstacle constraint format consumed by NMPC solve calls for static/harbour obstacles.
- Rationale: minimizes risk and avoids parallel code paths that could diverge.
- Alternatives considered:
  - Separate dynamic-obstacle API for NMPC: rejected due to duplicated logic and higher maintenance cost.

4. Preserve backward compatibility by feature gating.
- Decision: add configuration toggles/defaults so scenarios without dynamic obstacles behave exactly as before.
- Rationale: prevents regressions in established tests and thesis runs.
- Alternatives considered:
  - Always-on dynamic layer: rejected because it changes baseline behavior unexpectedly.

## Risks / Trade-offs

- [Risk] Coordinate-frame mismatch between map, simulation, and NMPC obstacle interpretation can produce false collisions or missed collisions.
  - Mitigation: enforce one canonical obstacle frame at generation time and reuse existing compatibility conversion utilities.

- [Risk] Dynamic obstacle updates could shift solver load and impact iteration timing.
  - Mitigation: keep obstacle count configurable, start with low default count, and reuse existing pre-filtering of nearby obstacles before solve.

- [Risk] Visual/debug output may become cluttered with moving tracks.
  - Mitigation: keep dynamic overlays optional and avoid legend pollution by default.

- [Trade-off] Simple constant-velocity motion is not behaviorally rich.
  - Mitigation: acceptable for baseline validation; future changes can add turn-rate and acceleration models.

## Migration Plan

1. Introduce dynamic obstacle configuration fields with defaults that keep dynamic mode disabled.
2. Add obstacle initialization logic in map/scenario setup.
3. Add per-step obstacle propagation before each NMPC solve.
4. Route propagated obstacles through existing NMPC-compatible obstacle packaging.
5. Update collision checks/visualization to include dynamic obstacles.
6. Validate with one deterministic scenario where a moving obstacle crosses planned vessel path.
7. Rollback strategy: disable dynamic obstacle flag to restore previous static-only behavior.

## Open Questions

- Should moving obstacles be clipped/removed when they leave map boundaries, or wrapped/re-spawned?
- Should near-horizon prediction use only current dynamic obstacle position initially, or a short projected sequence using constant velocity?
- Which specific run script should host the first canonical validation scenario (run_nmpc.m vs run_nmpc_simple.m variants)?
