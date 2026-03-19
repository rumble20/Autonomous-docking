# Dynamic Obstacle Usage (run_nmpc.m)

This note explains how to enable and configure moving obstacles in the unified NMPC run script.

## Location

- Primary script: `MY_NPMC/my work/run_nmpc.m`

## Main switches

Set these variables in the USER CONFIGURATION section:

- `enable_dynamic_obstacles`:
  - `true` enables moving obstacles.
  - `false` keeps legacy static/map-only behavior.
- `dynamic_obs_speed_mps`: constant forward speed in m/s.
- `dynamic_obs_radius_m`: obstacle circle radius in meters.
- `dynamic_obs_boundary_policy`:
  - `deactivate` (default): deactivate obstacle after leaving map bounds.
  - `clip`: clip position to bounds.
  - `wrap`: wrap around bounds.
- `dynamic_obs_boundary_margin`: expands map bounds used by lifecycle policy.
- `enable_dynamic_replay_check`: runs deterministic propagation self-check at startup.
- `dynamic_obs_positions_xy`:
  - Required when `enable_dynamic_obstacles = true`.
  - Manual start positions as rows `[x y]`.
- `dynamic_obs_headings_deg`:
  - Required when `enable_dynamic_obstacles = true`.
  - Heading in degrees (0 = +x/North, 90 = +y/East).
  - Can be scalar (replicated to all obstacles) or one per obstacle row.
- `dynamic_obs_speeds_mps`:
  - Optional per-obstacle speed in m/s.
  - `[]` uses `dynamic_obs_speed_mps` for all obstacles.
  - Can be scalar or one per obstacle row.
- `dynamic_obs_start_mode`:
  - `immediate`: obstacle starts moving at simulation start.
  - `proximity`: obstacle remains stationary until ship is close.
- `dynamic_obs_trigger_distance_m`:
  - Trigger distance used when `dynamic_obs_start_mode='proximity'`.
  - Can be scalar or one per obstacle row.

### Manual setup example

```matlab
dynamic_obs_positions_xy = [-3000 -1700; -2920 -1740];
dynamic_obs_headings_deg = [90; 110];
dynamic_obs_speeds_mps   = [8; 6];
dynamic_obs_start_mode = 'proximity';
dynamic_obs_trigger_distance_m = [250; 300];
```

## Scenario behavior

- Obstacles are initialized only from your manual configuration arrays in the USER CONFIGURATION section.
- Motion model is constant speed + constant heading (no turning).
- Active dynamic obstacles are converted to the same obstacle struct schema used by NMPC static/map obstacles: `position` and `radius`.

## Solver compatibility

- Dynamic obstacles are merged into `obs_local` before each `nmpc.solve(...)` call.
- Inactive dynamic obstacles are automatically excluded.
- `Dynamic packaging drift [m]` in summary should remain near zero; this confirms dynamic state and solve input stay synchronized.

## Batch verification helpers

`run_nmpc.m` supports optional environment overrides:

- `NMPC_ENABLE_DYNAMIC_OBS` (`0/1`, `false/true`)
- `NMPC_TFINAL` (simulation length in seconds)
