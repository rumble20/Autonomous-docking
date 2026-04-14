The dominant bottleneck is NLP solving (5–7 s per step), which is CasADi's responsibility, but several MATLAB-side tweaks could help: 



**(1) Solver configuration: Reduce prediction horizon from 60 to 40–50 steps (trades lookahead for \~30% speedup), or cut obstacle slots from 9 to 5–6 (fewer constraints = fewer NLP variables).**



**(2) Persistent solver: Currently, NMPC\_Container\_final is instantiated once but rebuilding the CasADi solver each run is expensive; verify the solver object persists across steps and isn't accidentally regenerated.** 



**(3) Vectorization \& preallocation: In run\_nmpc.m, the reference trajectory and obstacle arrays are grown dynamically (\[traj, x\_new]); preallocate full-size arrays upfront to avoid memory reallocation overhead. Avoid creating intermediate copies in guidance logic.** 



(4) Warm-start: Pass the previous NMPC solution as an initial guess to the next solve via CasADi's lbx0/ubx0 bounds—this can halve solve time if the solution doesn't change much frame-to-frame. 



(5) Tolerance relaxation: Slightly loosen CasADi's optimality/feasibility tolerances (e.g., tol\_optim=1e-4 instead of 1e-6)—often negligible impact on control quality but measurable speedup. 



Realistic expectation: Even with all of these, you'll likely hit 3–4 s per step best case before diminishing returns. True real-time (<1 s) likely requires further algorithmic simplification (shorter horizon, fewer obstacles, or a simpler dynamics model) or hardware upgrade.

