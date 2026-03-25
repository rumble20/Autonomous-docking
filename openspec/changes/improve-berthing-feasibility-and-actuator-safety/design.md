## Context

The docking stack currently combines NMPC-based control and guidance logic but has four feasibility gaps for precision berthing: (1) azipod angle wrapping forces non-ideal turns, (2) actuator-limit adherence is not guaranteed end-to-end, (3) vessel collision geometry is effectively point-based for planning/clearance decisions, and (4) guidance does not explicitly optimize berth approach direction and final heading under obstacles.

This change spans guidance, optimization constraints, and geometric collision modeling, so a cross-cutting design is required to keep behavior consistent across modules.

## Goals / Non-Goals

**Goals:**
- Introduce unwrapped azipod-angle handling that allows multi-turn continuity while still enforcing physical angle/rate bounds.
- Guarantee that NMPC-issued controls satisfy configured actuator limits at each control cycle.
- Replace point-only vessel geometry with a rectangular footprint model suitable for obstacle clearance and docking feasibility checks.
- Add guidance-level berth approach direction and final heading selection that is obstacle-aware and improves precise final docking success.
- Keep interfaces clear enough for scenario-based validation and iterative tuning.

**Non-Goals:**
- Replacing the full NMPC solver or optimization backend.
- Building a full high-fidelity contact mechanics model with fender compliance.
- Solving global route planning over full harbor maps beyond local berthing intent.
- Guaranteeing perfect docking in all unmodelled disturbances without further estimator/sensor upgrades.

## Decisions

1. Decision: Use unwrapped internal azipod angle state for optimization continuity.
Rationale: Principal-angle wrapping in [-pi, pi] creates artificial discontinuities and can force long rotations. An unwrapped state (for optimization continuity) with mapped physical constraints allows shortest feasible rotation decisions while preserving actuator realism.
Alternatives considered:
- Keep wrapped state and add special-case branch logic: rejected due to brittle behavior near wrap boundaries.
- Duplicate control channels for clockwise/counterclockwise turn modes: rejected due to unnecessary complexity.

2. Decision: Enforce actuator limits in two layers (hard NMPC constraints + post-solve validation/clamp with diagnostics).
Rationale: Hard constraints are primary enforcement, while a post-solve guard catches model/config mismatch and numerical edge cases.
Alternatives considered:
- Post-solve clamp only: rejected because optimizer may plan infeasible trajectories.
- Hard constraints only with no runtime checks: rejected because violations can still occur through integration/units/config errors.

3. Decision: Model vessel occupancy as a body-fixed rectangle transformed to world frame each step.
Rationale: Rectangular footprint is a strong complexity/benefit trade-off and captures berth clearance constraints much better than a point model.
Alternatives considered:
- Circle footprint: simpler but too conservative or inaccurate for long hull geometry.
- Full polygonal mesh: more accurate but unnecessary complexity for current stage.

4. Decision: Add guidance objective term/logic for approach-direction and terminal-heading selection with obstacle-aware feasibility scoring.
Rationale: Precise berthing requires both pose target and approach orientation to be selected together under local obstacle constraints.
Alternatives considered:
- Fixed approach heading from mission file: rejected as too rigid in cluttered environments.
- Purely reactive heading changes near berth: rejected due to late and unstable maneuver commitments.

## Risks / Trade-offs

- Risk: Unwrapped angle state may drift numerically over long horizons. -> Mitigation: periodic normalization relative to nearest equivalent angle plus continuity-preserving offset bookkeeping.
- Risk: Dual-layer limit enforcement may hide root causes if clipping is frequent. -> Mitigation: log and count clamp events; fail validation scenarios if threshold exceeded.
- Risk: Rectangle footprint can produce conservative blocking near corners. -> Mitigation: parameterize safety margins separately from hull dimensions for tuning.
- Risk: Approach-direction optimization may increase compute time. -> Mitigation: restrict candidate headings/directions and use staged evaluation.
- Risk: Behavior changes may invalidate prior tuning sets. -> Mitigation: provide scenario-based tuning checklist and regression suite.

## Migration Plan

- Introduce new config fields for rectangle dimensions, safety margin, and guidance approach preferences with backward-compatible defaults.
- Keep existing behavior as fallback until new options are enabled per scenario.
- Roll out in phases: angle/actuator compliance checks first, then hitbox constraints, then directional berthing guidance.
- Rollback strategy: disable new feature flags and revert to prior point-geometry/fixed-heading behavior without data migration.

## Open Questions

- Which frame conventions and sign conventions are currently used in all actuator-limit paths, and are they fully consistent?
- Should berth approach direction be selected from a discrete candidate set or treated as a continuous variable in current solver runtime budget?
- What minimum obstacle margin should be enforced in tight-port scenarios for acceptable false-positive/false-negative trade-off?
- Should reverse-in berthing be explicitly preferred in some vessel/berth classes by policy?
