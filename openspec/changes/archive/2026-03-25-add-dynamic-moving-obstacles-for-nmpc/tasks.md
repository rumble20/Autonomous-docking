## 1. Scenario and data model setup

- [x] 1.1 Add dynamic obstacle configuration flags and defaults (enable/disable, count, speed, heading, radius, boundary policy) in the primary NMPC run configuration path.
- [x] 1.2 Define a dynamic obstacle state structure with required fields (position, heading, speed, radius, active/enabled) and initialize it during scenario/map setup.
- [x] 1.3 Ensure static-only and harbour-only runs preserve existing behavior when dynamic obstacle mode is disabled.

## 2. Dynamic obstacle propagation

- [x] 2.1 Implement per-step straight-line obstacle motion update using constant speed and heading (zero turn-rate).
- [x] 2.2 Apply deterministic lifecycle handling when obstacles leave configured map bounds (e.g., deactivate/remove/clip based on policy).
- [x] 2.3 Add deterministic replay validation checks to confirm identical trajectories for identical initial conditions and settings.

## 3. NMPC compatibility integration

- [x] 3.1 Extend obstacle packaging so dynamic obstacles are converted into the same NMPC obstacle input schema used for static and harbour obstacles.
- [x] 3.2 Merge static, harbour, and dynamic obstacle sources in step-synchronized order before each NMPC solve.
- [x] 3.3 Ensure inactive dynamic obstacles are excluded from NMPC constraints without breaking existing obstacle ingestion contracts.

## 4. Collision checks and observability

- [x] 4.1 Include dynamic obstacles in runtime collision/termination checks using the same geometric consistency assumptions as existing obstacles.
- [x] 4.2 Update plotting/debug output to visualize moving obstacle states over time without cluttering baseline static legends.
- [x] 4.3 Add a deterministic crossing-path validation scenario to verify NMPC avoidance against a forward-moving obstacle.

## 5. Verification and regression safety

- [x] 5.1 Run baseline static/harbour scenarios to confirm no regressions when dynamic obstacle mode is off.
- [x] 5.2 Run dynamic obstacle scenarios and verify solver input updates track propagated obstacle positions each step.
- [x] 5.3 Document operational usage notes for enabling/disabling dynamic obstacles and selecting boundary policy.
