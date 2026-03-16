# NMPC Tuning Guide (Container Azipod)

This document synthesizes how to tune the full control stack used in:
- run_nmpc_simple.m
- NMPC_Container_Lite.m

It is organized by layers, then by practical symptoms and actions.

## 1. Control Architecture and Where Parameters Live

The behavior comes from three coupled layers:

1. Scenario and simulation setup (run_nmpc_simple.m)
- Test-specific Q (Q_testA, Q_testB)
- Shared N, dt, R, R_rate
- Runtime cap T_final
- Waypoints, acceptance radius, initial conditions

2. NMPC optimization and constraints (NMPC_Container_Lite.m)
- Cost matrices Q, R, R_rate
- Horizon N, sample time dt
- Hard obstacle constraints using sqrt-distance
- Actuator and state bounds
- IPOPT options

3. Guidance and reference shaping (local functions in run_nmpc_simple.m)
- Waypoint guidance lookahead
- buildSimpleRef8 yaw-rate shaping
- buildObstacleAwareRef8 deflection shape and intensity

## 2. Current Tunable Parameters (with Meaning)

### 2.1 Time and horizon

1. base_nmpc_cfg.N
- Meaning: prediction length in steps.
- Increase: better anticipation, higher runtime.
- Decrease: faster solve, less foresight.

2. base_nmpc_cfg.dt
- Meaning: control discretization period.
- Smaller dt: better fidelity, much more compute.
- Larger dt: faster, rougher dynamics representation.

3. T_final
- Meaning: simulation stop time.
- Only affects whether run is cut early, not controller quality.

### 2.2 Cost weights

State order is [u, v, r, x, y, psi, n1, n2].
Control order is [alpha1, alpha2, n1_c, n2_c].

1. Q_testA, Q_testB
- Q_u: speed tracking in surge.
- Q_v: suppress sway drift.
- Q_r: yaw-rate damping.
- Q_x, Q_y: path/position tracking strength.
- Q_psi: heading tracking strength.
- Q_n1, Q_n2: preference on shaft-speed states.

2. R
- Penalizes absolute command effort.
- Higher R(alpha): less aggressive azimuth commands.
- Higher R(n_c): softer rpm commands.

3. R_rate
- Penalizes command increments between steps.
- Primary anti-wobble knob.
- Higher R_rate(alpha): smoother heading behavior.

### 2.3 Obstacle parameters

1. nmpc_cfg_B.r_safety
- Hard clearance margin around obstacle radius.
- Minimum center distance tends to r_obs + r_safety.

2. nmpc_cfg_B.max_obs
- Number of obstacle slots in NLP.
- Keep as small as needed for runtime.

3. buildObstacleAwareRef8 safety_margin
- Reference-level deflection magnitude around obstacles.
- Larger value: earlier/larger bypass.

4. Gaussian width in buildObstacleAwareRef8 (0.22 currently)
- Controls how spread/sudden the deflection is over horizon.
- Larger width: smoother, broader bypass.

### 2.4 Guidance geometry

1. R_accept and R_accept_B
- Waypoint acceptance radius.
- Too small: delayed switching and oscillation near waypoints.
- Too large: corner cutting.

2. lookahead = max(60, 8*U_now)
- Short lookahead: reactive, twitchy.
- Long lookahead: smoother, more anticipative.

3. buildSimpleRef8 r_d gain and clamp
- r_d = k * psi_err, with clamp.
- Higher gain/clamp: more turn authority, more oscillation risk.
- Lower gain/clamp: smoother but can become sluggish.

4. buildObstacleAwareRef8 r_d2 gain and clamp
- Same role as r_d, but for deflected heading in obstacle mode.

### 2.5 Constraints and physical limits (NMPC_Container_Lite)

1. alpha_max
- Max azimuth angle bound.

2. n_max, n_min
- Shaft speed limits.

3. Dn_max
- Max shaft acceleration; key for realism and smoothness.

4. State bounds for u, v, r, x, y, psi, n1, n2
- Feasibility envelope for optimizer.

### 2.6 Solver options (IPOPT)

1. ipopt.max_iter
- More iterations can improve convergence, increases solve time.

2. ipopt.tol, ipopt.acceptable_tol
- Stricter tolerance improves optimality but may be slower.

3. acceptable_iter, mu_strategy, scaling method
- Influence robustness/speed tradeoff.

## 3. Symptom -> Action Map

1. Wobble in heading/path (S-turns, oscillatory azimuth)
- Increase R_rate(alpha1, alpha2) first.
- Increase Q_r moderately.
- Reduce Q_psi or reduce r_d/r_d2 gain/clamp.
- Increase lookahead.

2. Too slow turning / misses waypoint transitions
- Slightly increase Q_x, Q_y.
- Slightly decrease R_rate(alpha).
- Increase r_d/r_d2 clamp a little.

3. Obstacle avoided with too much clearance
- Reduce r_safety first.
- Then reduce obstacle reference safety_margin.
- Optionally reduce Gaussian width if bypass remains too wide.

4. Scrapes obstacle or reacts too late
- Increase r_safety.
- Increase safety_margin in deflected reference.
- Increase horizon N slightly (if runtime allows).

5. Solver is slow
- Reduce N.
- Keep max_obs minimal.
- Relax tolerances slightly or reduce max_iter.
- Preserve warm-start behavior.

6. Route not finished
- Increase T_final.
- Verify R_accept/R_accept_B are not too strict.
- Increase lookahead and reduce wobble to avoid path inefficiency.

## 4. Recommended Tuning Order (Do Not Skip)

1. Set completion budget
- Choose T_final high enough for expected route duration + 20 to 30 percent margin.

2. Tune Test A first (no obstacle)
- Goal: low wobble, acceptable XTE, finish route.
- Prioritize R_rate(alpha), Q_r, Q_psi, lookahead.

3. Freeze Test A settings
- Do not change them while diagnosing obstacle behavior.

4. Tune Test B geometry
- Start with r_safety to set minimum legal clearance.
- Then adjust safety_margin and Gaussian width for path shape.

5. Tune Test B smoothness
- Use r_d2 gain/clamp and R_rate(alpha) to remove post-avoidance wobble.

6. Re-check both tests
- Ensure Test B tuning did not degrade Test A baseline.

## 5. Practical Target Bands (for this project)

Use as safe exploration ranges:

1. N: 30 to 50
2. dt: 1.0 (change only if needed)
3. R_rate(alpha): 0.15 to 0.35
4. Q_psi (Test A): 8 to 12
5. Q_r (Test A): 1.0 to 1.8
6. lookahead min: 50 to 90
7. lookahead speed factor: 6 to 10
8. r_safety (Test B): 18 to 30
9. safety_margin deflection: 70 to 120
10. r_d and r_d2 clamp: 0.08 to 0.12 rad/s

## 6. Minimal Experiment Protocol

For each trial, change only one cluster:

1. Cluster A smoothness:
- R_rate(alpha), Q_r, Q_psi, lookahead.

2. Cluster B clearance:
- r_safety, safety_margin.

3. Cluster B bypass shape:
- Gaussian width, r_d2 clamp.

Log after each run:

1. Solver success rate.
2. Mean and max XTE.
3. Final waypoint reached time (or timeout).
4. Minimum obstacle distance in Test B.
5. Qualitative wobble score from plots.

## 7. Notes Specific to Current Setup

1. You currently use separate NMPC instances for Test A and Test B.
- This is good practice because obstacle constraints and Q priorities differ.

2. You currently use hard obstacle constraints plus deflected reference in Test B.
- This is a robust combination for marine craft with slower yaw dynamics.

3. If obstacle clearance settles near a constant floor, that is expected.
- It is usually r_obs + r_safety (plus small dynamics/reference effects).

---

If needed, extend this guide with a one-page "quick recipes" section (for example: "less wobble", "tighter obstacle pass", "faster solve") and exact parameter deltas for each recipe.
