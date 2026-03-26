## 1. Baseline Audit and Configuration

- [x] 1.1 Map current NMPC actuator-limit paths (units, frames, bounds, saturation points) and document mismatches.
- [x] 1.2 Add/confirm configuration fields for azipod multi-rotation behavior, rectangular hitbox dimensions, safety margins, and approach-direction preferences.
- [x] 1.3 Add feature toggles or compatibility defaults so existing scenarios can run before full migration.

## 2. Azipod Multi-Rotation Constraints

- [x] 2.1 Introduce continuous/unwrapped azipod-angle state representation in the NMPC formulation.
- [x] 2.2 Implement mapping from unwrapped internal state to physically equivalent actuator command output.
- [x] 2.3 Enforce hard azipod angle and azipod-rate constraints on the continuous representation, including wrap-boundary transitions.
- [x] 2.4 Add regression scenarios for heading reversal maneuvers that cross principal-angle boundaries.

## 3. Actuator-Limit Compliance Enforcement

- [x] 3.1 Ensure hard NMPC constraints cover all configured actuator limits (angle, rate, thrust, and actuator-specific bounds).
- [x] 3.2 Add deterministic post-solve limit validation and bounded-command handling policy.
- [x] 3.3 Add diagnostics/logging for any limit handling events with commanded value, bound, and timestamp.
- [x] 3.4 Add repeatability tests proving identical out-of-bound inputs produce identical bounded outputs.

## 4. Rectangular Vessel Hitbox Integration

- [x] 4.1 Implement body-frame rectangular footprint generation from vessel length, beam, and margin parameters.
- [x] 4.2 Implement world-frame transformation of rectangle vertices from vessel pose at each control step.
- [x] 4.3 Replace point-only clearance checks with rectangle-based collision and proximity checks in feasibility paths.
- [x] 4.4 Add tight-berth test cases where point-feasible but rectangle-infeasible maneuvers are rejected.

## 5. Directional Berthing Guidance

- [x] 5.1 Implement berth approach-direction selection logic using obstacle-aware feasibility scoring.
- [x] 5.2 Add explicit terminal heading target handling in guidance outputs alongside terminal position.
- [x] 5.3 Implement dynamic re-evaluation of approach direction when active corridor becomes infeasible.
- [x] 5.4 Add scenarios for forward-in and reverse-in berthing with obstacle-induced re-planning.

## 6. Validation, Tuning, and Rollout

- [x] 6.1 Build an end-to-end berthing regression suite covering actuator limits, multi-rotation turns, rectangular collision checks, and precise terminal docking pose.
- [x] 6.2 Tune margins/weights for approach feasibility and terminal heading accuracy on representative harbor scenarios.
- [x] 6.3 Measure runtime impact and optimize candidate heading/direction evaluation to stay within control cycle budget.
- [x] 6.4 Document rollout and rollback procedure using compatibility defaults and feature toggles.
