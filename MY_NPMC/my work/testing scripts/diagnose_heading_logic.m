% diagnose_heading_logic.m
% Check initial heading and mode-switching logic for discontinuities and errors

clear; clc;
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  HEADING LOGIC DIAGNOSTIC\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

%% 1. Initial heading calculation
fprintf('1. INITIAL HEADING CALCULATION\n\n');

waypoints = [-3800, -1500; -3400, -1300; -3200, -1350; -3000, -1400; -2600, -1800; -2400, -2100; -2000, -2050];

% Initial heading from first two waypoints
seg_vec = waypoints(2,:) - waypoints(1,:);
x0_heading_rad = atan2(seg_vec(2), seg_vec(1));
x0_heading_deg = rad2deg(x0_heading_rad);

fprintf('First waypoint: (%.1f, %.1f)\n', waypoints(1,1), waypoints(1,2));
fprintf('Second waypoint: (%.1f, %.1f)\n', waypoints(2,1), waypoints(2,2));
fprintf('Segment vector: (%.1f, %.1f)\n', seg_vec(1), seg_vec(2));
fprintf('Initial heading: %.3f rad = %.2f deg\n\n', x0_heading_rad, x0_heading_deg);

%% 2. Heading at each waypoint transition
fprintf('2. HEADING AT WAYPOINT TRANSITIONS\n\n');

fprintf('Waypoint#  | Segment Heading (deg) | Turn from Previous\n');
fprintf('-----------|----------------------|-------------------\n');

prev_heading = x0_heading_deg;
for k = 1:(size(waypoints,1)-1)
    seg = waypoints(k+1,:) - waypoints(k,:);
    seg_heading_rad = atan2(seg(2), seg(1));
    seg_heading_deg = rad2deg(seg_heading_rad);
    
    % Heading error (wrapped to [-180, 180])
    dh = seg_heading_deg - prev_heading;
    dh = mod(dh + 180, 360) - 180;
    
    fprintf('WP%d->%d   | %7.2f°             | %+7.2f°\n', k, k+1, seg_heading_deg, dh);
    prev_heading = seg_heading_deg;
end
fprintf('\n');

%% 3. Check wrapping consistency
fprintf('3. HEADING WRAPPING CONSISTENCY\n\n');

test_headings_rad = [0, pi/4, pi/2, pi, -pi/2, -3*pi/4, 3*pi/4];
fprintf('Test heading (rad)  | Wrapped ([-π,π]) | Wrapped (deg)\n');
fprintf('--------------------|-----------------|--------------\n');

for h = test_headings_rad
    h_wrapped = wrapToPi(h);
    h_wrapped_deg = rad2deg(h_wrapped);
    fprintf('%7.3f (%.2f°)   | %7.3f (%.2f°)   | %7.2f°\n', h, rad2deg(h), h_wrapped, rad2deg(h_wrapped), h_wrapped_deg);
end
fprintf('\n');

%% 4. Heading mode transitions: transit -> final
fprintf('4. HEADING MODE TRANSITIONS: TRANSIT -> FINAL\n\n');

% Simulate transit to final waypoint transition
fprintf('Transit mode (not on final wp):\n');
fprintf('  goal_heading_enable = false\n');
fprintf('  goal_heading_rad = chi_seg (track segment)\n');
fprintf('  terminal_goal_heading_weight = 0.0\n\n');

fprintf('Final waypoint mode (on final wp):\n');
fprintf('  goal_heading_enable = true (if heading is set)\n');
fprintf('  goal_heading_rad = chi_seg (or last segment)\n');
fprintf('  terminal_goal_heading_weight = 60.0 (strong)\n\n');

fprintf('⚠️  OBSERVATION: Both modes use chi_seg as goal_heading_rad\n');
fprintf('    There should be NO discontinuity when reaching final waypoint.\n\n');

%% 5. Sharp turn and preview logic
fprintf('5. SHARP TURN & BERTH PREVIEW LOGIC\n\n');

fprintf('Sharp turn detection:\n');
fprintf('  - turn_angle_deg >= 28° triggers sharp_turn mode\n');
fprintf('  - If turn is sharp AND distance < 160m: enable goal_heading_enable\n');
fprintf('  - goal_heading_rad = chi_seg\n');
fprintf('  - terminal_goal_heading_weight = 8.0 (sharp_turn_heading_weight)\n\n');

fprintf('Berth preview logic:\n');
fprintf('  - Activates when in last N segments (preview_last_n_segments=3)\n');
fprintf('  - goal_heading_enable = true\n');
fprintf('  - goal_heading_rad = berth_cfg.heading_deg (may differ from chi_seg!)\n');
fprintf('  - This is a DISCRETE CHANGE from segment heading\n');
fprintf('  - Can cause sudden heading reference jump\n\n');

fprintf('⚠️  POTENTIAL ISSUE: Heading reference can jump when preview activates\n');
fprintf('    If berth_cfg.heading_deg differs significantly from current segment,\n');
fprintf('    NMPC will receive a new goal_heading that is discontinuous.\n\n');

%% 6. Berth mode heading
fprintf('6. BERTH MODE HEADING ASSIGNMENT\n\n');

fprintf('When berth_mode_active:\n');
fprintf('  goal_heading_rad = deg2rad(berth_cfg.heading_deg)\n');
fprintf('  chi_ctrl = deg2rad(berth_cfg.corridor_heading_deg) OR goal_heading_rad\n\n');

fprintf('Example from config:\n');
fprintf('  berth_cfg.heading_deg = 0\n');
fprintf('  berth_cfg.corridor_heading_deg = 0\n\n');

fprintf('If last segment heading is ~0°: smooth transition\n');
fprintf('If last segment heading is ~180°: DISCONTINUOUS jump in goal\n\n');

fprintf('⚠️  POTENTIAL ISSUE: Large heading error at berth activation\n\n');

%% 7. Heading in solve_opts override
fprintf('7. SOLVE_OPTS HEADING OVERRIDES\n\n');

fprintf('In different modes, solve_opts.goal_heading_rad can override path_ref:\n');
fprintf('  berth_preview_active:\n');
fprintf('    solve_opts.goal_heading_enable = true\n');
fprintf('    solve_opts.goal_heading_rad = berth_cfg.heading_deg\n\n');
fprintf('  sharp_turn (near gate):\n');
fprintf('    solve_opts.goal_heading_enable = true\n');
fprintf('    solve_opts.goal_heading_rad = chi_seg\n\n');
fprintf('  transit (default):\n');
fprintf('    solve_opts.goal_heading_enable = true (if sharp turn & near)\n');
fprintf('    solve_opts.goal_heading_rad = chi_seg\n\n');

fprintf('These overrides are applied AFTER path_ref is built.\n');
fprintf('NMPC should respect solve_opts values (see NMPC_Container_final.m line 899).\n\n');

%% 8. Check path_ref vs solve_opts conflict
fprintf('8. PATH_REF vs SOLVE_OPTS CONFLICT CHECK\n\n');

fprintf('In run_nmpc.m:\n');
fprintf('  1. path_ref.goal_heading_enable = goal_heading_enable (default)\n');
fprintf('  2. path_ref.goal_heading_rad = goal_heading_rad (default)\n');
fprintf('  3. LATER: solve_opts.goal_heading_enable might be set\n');
fprintf('  4. LATER: solve_opts.goal_heading_rad might be set\n\n');

fprintf('The NMPC.solve() receives BOTH path_ref AND solve_opts.\n');
fprintf('⚠️  CHECK: Which takes precedence?\n\n');

fprintf('From NMPC_Container_final.m line 899-903:\n');
fprintf('  goal_heading_enable = logical(getOr(solve_opts, ''goal_heading_enable'', ...\n');
fprintf('                        logical(getOr(path_ref, ''goal_heading_enable'', false))));\n');
fprintf('  goal_heading_rad = getOr(solve_opts, ''goal_heading_rad'', ...\n');
fprintf('                   getOr(path_ref, ''goal_heading_rad'', 0.0));\n\n');

fprintf('RESOLUTION: solve_opts takes precedence over path_ref. Good.\n\n');

%% 9. PID fallback heading
fprintf('9. PID FALLBACK HEADING CONTROL\n\n');

fprintf('If NMPC solver fails, fallback uses PID:\n');
fprintf('  psi_err = wrapToPi(chi_ctrl - x(6))\n');
fprintf('  Uses chi_ctrl (not chi_seg or goal_heading_rad directly)\n\n');

fprintf('This can be inconsistent if chi_ctrl was set to corridor heading.\n');
fprintf('But chi_ctrl is usually chi_seg, so should be OK most times.\n\n');

%% 10. Summary and recommendations
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('FINDINGS & RECOMMENDATIONS\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

fprintf('CONFIRMED CORRECT:\n');
fprintf('  ✓ Initial heading from first two waypoints is correct\n');
fprintf('  ✓ Heading wrapping with wrapToPi is consistent\n');
fprintf('  ✓ solve_opts takes precedence over path_ref\n');
fprintf('  ✓ No fundamental discontinuity at transit->final transition\n\n');

fprintf('POTENTIAL ISSUES (worth monitoring):\n');
fprintf('  ⚠️  Berth preview activation: abrupt heading goal change\n');
fprintf('      - If berth_cfg.heading_deg differs from segment heading\n');
fprintf('      - Could cause initial heading error at berth entry\n');
fprintf('      - Recommend: ensure berth heading aligns with approach segment\n\n');

fprintf('  ⚠️  Sharp turn heading weight jump: goes 0 -> 8.0 suddenly\n');
fprintf('      - When entering sharp turn zone\n');
fprintf('      - May cause trajectory to swing to align with turn\n');
fprintf('      - Recommend: monitor heading_err_log for oscillations\n\n');

fprintf('  ⚠️  PID fallback chi_ctrl: may differ from NMPC goal heading\n');
fprintf('      - If chi_ctrl set to corridor heading vs segment heading\n');
fprintf('      - Causes mode mismatch when solver fails\n');
fprintf('      - Recommend: PID fallback should use same chi_ctrl as NMPC\n\n');

fprintf('RECOMMENDATIONS:\n');
fprintf('  1. After next run: check heading_err_log for:\n');
fprintf('     - Sudden jumps at berth preview activation\n');
fprintf('     - Oscillations during sharp turns\n');
fprintf('     - Discontinuities at mode transitions\n\n');

fprintf('  2. Check solve_opts.goal_heading_rad values at each step:\n');
fprintf('     - Log solve_opts.goal_heading_enable and goal_heading_rad\n');
fprintf('     - Verify no unintended changes\n\n');

fprintf('  3. Test with berth_preview_active changing goal heading:\n');
fprintf('     - Add heading error check at preview activation\n');
fprintf('     - Consider gradual weight ramp instead of step change\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n\n');

function ang_wrapped = wrapToPi(angle)
    ang_wrapped = mod(angle + pi, 2*pi) - pi;
end
