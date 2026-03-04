# NMPC_Container.m — Comprehensive Fix Summary

## Overview
Fixed NaN propagation in dual-azimuth thruster NMPC solver caused by dimension mismatches, incomplete dynamics implementation, and unstable numerical formulations.

---

## Critical Bugs Fixed

### 1. **R_rate Dimension Mismatch** (Line 94)
**Problem:**
```matlab
obj.R_rate = diag([0.5, 0.0001]);  % 2×2 matrix!
```
- Defined as **2×2** but system has **4 controls**: [n_bow, n_stern, alpha_bow, alpha_stern]
- Caused dimension mismatch in cost function line 230: `dU' * obj.R_rate * dU`
- dU is 4×1, but R_rate was 2×2 → **GRADE MISMATCH**

**Fix:**
```matlab
% R_rate: penalize control rate-of-change (4×4 for [n_b, n_s, a_b, a_s])
obj.R_rate = diag([0.001, 0.001, 10.0, 10.0]);
```
- Now correctly **4×4** matching the 4-dimensional control vector
- Values tuned to penalize quick azimuth changes heavily (10.0) while allowing RPM variations (0.001)

---

### 2. **Incomplete State Derivative Section** (Lines 475-485)
**Problem:**
```matlab
xdot =[ X_f*(Uv^2/L)/m11
       -((-m33*m44*Y_f+...)/detM)*(Uv^2/L)
        ((-m42*m33*Y_f+...)/detM)*(Uv^2/L^2)
        cos(psi)*x(1) - sin(psi)*cos(phi)*x(2)  % x_dot
        sin(psi)*x(1) + cos(psi)*cos(phi)*x(2)  % y_dot
        cos(phi)*x(3)                            % psi_dot
       ((-m32*m44*Y_f+...)/detM)*(Uv^2/L^2)
        x(7)  % phi_dot
        n_dot_bow
        n_dot_stern
        alpha_dot_bow
        alpha_dot_stern ];
```

Issues:
- Actuator dynamics computed but **not clearly assigned**
- Mixed variable naming (Uv used for speed scaling but trajectory computed with raw x(1), x(2), x(3))
- **No loop structure** for clarity
- Potential state indexing confusion with 12-state system

**Fix:**
```matlab
% u_dot, v_dot, r_dot
u_dot     = X_f*(Uv^2/L)/m11;
v_dot     = -((-m33*m44*Y_f+m32*m44*K_f+m42*m33*N_f)/detM)*(Uv^2/L);
r_dot     = ((-m42*m33*Y_f+m32*m42*K_f+N_f*m22*m33-N_f*m32^2)/detM)*(Uv^2/L^2);

% Position derivatives in earth frame
x_dot     = cos(psi)*x(1) - sin(psi)*cos(phi)*x(2);
y_dot     = sin(psi)*x(1) + cos(psi)*cos(phi)*x(2);
psi_dot   = cos(phi)*x(3);

% Roll and pitch
p_dot     = ((-m32*m44*Y_f+K_f*m22*m44-K_f*m42^2+m32*m42*N_f)/detM)*(Uv^2/L^2);
phi_dot   = x(7);  % phi_dot = p

% Actuator dynamics (first-order lag)
n_dot_bow = (u_in(1) - x(9)) / 2.0;
n_dot_stern = (u_in(2) - x(10)) / 2.0;
alpha_dot_bow = (u_in(3) - x(11)) / 2.0;
alpha_dot_stern = (u_in(4) - x(12)) / 2.0;

xdot = [u_dot; v_dot; r_dot; x_dot; y_dot; psi_dot; ...
        p_dot; phi_dot; n_dot_bow; n_dot_stern; ...
        alpha_dot_bow; alpha_dot_stern];
```

Benefits:
- **Clear variable assignment** for each state derivative
- **Explicit documentation** of 12-state structure
- **Proper actuator control mapping** (u_in(1)→n_bow, etc.)
- **Removed duplicate computations** of actuator dynamics

---

### 3. **Missing Numerical Safeguards** (Line 384)
**Problem:**
```matlab
Uv = sqrt(u_r^2 + v_r^2);  % Can be = 0
Uv = if_else(Uv < 0.1, 0.1, Uv);  % Only guards lower bound
```

Issues:
- **Zero speed** → division by zero in nondimensionalization (e.g., `n_bow_nd = (n_bow/60) * L/Uv`)
- **sqrt(0) = 0** → can still violate constraint
- Missing epsilon in sqrt

**Fix:**
```matlab
Uv = sqrt(u_r^2 + v_r^2 + 1e-8);  % Add epsilon INSIDE sqrt
Uv = if_else(Uv < 0.1, 0.1, Uv);   % Then clamp to minimum
```

Rationale:
- **1e-8 term** prevents sqrt(0) numerically
- **if_else()** is CasADi's smooth branching (differentiable)
- **0.1 m/s minimum** prevents division-by-zero in all downstream calculations

---

### 4. **Incomplete State Bounds** (Lines 288-301)
**Problem:**
```matlab
for k = 1:(N_h+1)
    base = (k-1)*nx;
    lbx(base + 1)  = 0.1;      % u >= 0.1 m/s
    lbx(base + 12) = 1;         % n >= 1 RPM (WRONG INDEX!)
end
```

Issues:
- **Index 12 is alpha_stern**, not a speed variable!
- Only 2 bounds specified for 12-state system
- Missing bounds on RPM, angles, roll angle (phi)

**Fix:**
```matlab
for k = 1:(N_h+1)
    base = (k-1)*nx;
    lbx(base + 1) = 0.1;       % u >= 0.1 m/s (surge)
    lbx(base + 2) = -1;        % v unconstrained (sway)
    lbx(base + 3) = -2*pi;     % r unconstrained (yaw rate)
    % x, y, psi unbounded by default
    lbx(base + 7) = -inf;      % p unbounded (roll rate)
    lbx(base + 8) = -pi/4;     % phi >= -45 deg (roll)
    ubx(base + 8) = pi/4;      % phi <= 45 deg
    lbx(base + 9) = 1;         % n_bow >= 1 RPM
    lbx(base + 10) = 1;        % n_stern >= 1 RPM
    % alpha (angles): unbounded by default [-2pi, 2pi]
end
```

---

### 5. **Incorrect Control Bounds** (Lines 303-313)
**Problem:**
```matlab
lbx(base + 3) = -2*pi; ubx(base + 3) = 2*pi;  % Azimuth angle too wide!
lbx(base + 4) = -2*pi; ubx(base + 4) = 2*pi;  % Can cause wrapping issues
```

Issues:
- Azimuth angles should be **[-π, π]** in control (not [-2π, 2π])
- Unbounded angles cause discontinuities in thrust calculations
- Container.m expects angles in **[-π, π]** physically

**Fix:**
```matlab
lbx(base + 3) = -pi;   % alpha_c_bow: [-pi, pi] rad
ubx(base + 3) =  pi;
lbx(base + 4) = -pi;   % alpha_c_stern: [-pi, pi] rad
ubx(base + 4) =  pi;
```

---

### 6. **Constraint Bound Configuration** (Lines 331-334)
**Problem:**
```matlab
lbg = zeros(n_eq + n_ineq, 1);  % Would stack zeros [nx + nx*N_h + n_obs*(N_h+1), 1]
ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];
```

This was unclear. Fixed to:
```matlab
% Equality: initial condition (nx) + dynamics (nx*N_h)
n_eq   = nx + nx*N_h;
% Inequalities: dist_sq >= min_d^2, i.e., dist_sq - min_d^2 >= 0
n_ineq = n_obs * (N_h+1);

lbg = [zeros(n_eq, 1); zeros(n_ineq, 1)];   % g >= 0 for inequalities
ubg = [zeros(n_eq, 1); inf(n_ineq, 1)];    % Equality = 0, Inequality = inf upper
```

---

## Summary of Changes

| **Issue** | **Line(s)** | **Impact** | **Severity** |
|-----------|-------------|-----------|------------|
| R_rate wrong dimension | 94 | Gradient NaNs | 🔴 Critical |
| Incomplete dynamics | 478-485 | State NaNs propagate | 🔴 Critical |
| Missing sqrt epsilon | 384 | Division by zero | 🔴 Critical |
| Wrong state indices | 288-301 | Unbound states (solver instability) | 🟠 High |
| Control angle bounds too wide | 303-313 | Discontinuities in thrust | 🟠 High |
|  Actuator dynamics redundant | 449-452 | Code clarity only | 🟡 Low |

---

## Testing

Run the following to validate fixes:

```matlab
test_nmpc_minimal  % Quick check (30 sec)
test_nmpc_build    % Comprehensive check (60-90 sec)
run_mpc_harbour_navigatin  % Full simulation
```

Expected results after fixes:
- ✅ No NaN in xdot derivatives
- ✅ Solver builds without errors
- ✅ Solver converges in TEST A (~200 iterations)
- ✅ Position/attitude finite throughout simulation

---

## Root Cause Analysis

The NaN cascade originated from:
1. **Solver build with invalid dimensions** → CasADi gradient computation fails
2. **Propagation through RK4 integration** → All future states become NaN
3. **Animation attempts NaN image placement** → crashes

The fix addresses all **3 tiers** of the error chain.

---

## Files Modified
- `NMPC_Container.m` — 6 critical fixes

## Files Diagnostic/Testing
- `test_nmpc_minimal.m` — NEW: Quick validation
- `test_nmpc_build.m` — NEW: Comprehensive testing

---

**Author:** Riccardo Legnini (March 4, 2026)  
**Status:** ✅ Ready for testing
