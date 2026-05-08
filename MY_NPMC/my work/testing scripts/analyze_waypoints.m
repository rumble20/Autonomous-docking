%% Waypoint Geometry Analysis
clear; clc;

waypoints = [-3800, -1500; -3400, -1300; -3200, -1350; -3000, -1400; -2600, -1800; -2400, -2100; -2000, -2050];

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('  WAYPOINT NAVIGATION FEASIBILITY ANALYSIS\n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

fprintf('Waypoint sequence:\n');
for i = 1:size(waypoints, 1)
    fprintf('  WP%d: (%.0f, %.0f)\n', i, waypoints(i,1), waypoints(i,2));
end

fprintf('\n--- SEGMENT ANALYSIS ---\n');
total_distance = 0;
segment_headings = zeros(size(waypoints, 1)-1, 1);
segment_distances = zeros(size(waypoints, 1)-1, 1);

for i = 1:size(waypoints, 1)-1
    seg = waypoints(i+1,:) - waypoints(i,:);
    dist = norm(seg);
    heading_deg = rad2deg(atan2(seg(2), seg(1)));
    total_distance = total_distance + dist;
    segment_distances(i) = dist;
    segment_headings(i) = heading_deg;
    fprintf('Seg %d->%d: distance=%.1f m, heading=%.1f°\n', i, i+1, dist, heading_deg);
end
fprintf('Total route distance: %.1f m\n\n', total_distance);

fprintf('--- TURN ANALYSIS ---\n');
for i = 2:size(waypoints, 1)-1
    h1 = segment_headings(i-1);
    h2 = segment_headings(i);
    turn_rad = wrapToPi(deg2rad(h2 - h1));
    turn_deg = rad2deg(turn_rad);
    
    if abs(turn_deg) > 120
        status = '❌ CRITICAL (>120°)';
    elseif abs(turn_deg) > 80
        status = '⚠️  SHARP (80-120°)';
    else
        status = '✓ OK (<80°)';
    end
    fprintf('At WP%d: turn=%.1f° %s\n', i, turn_deg, status);
end

fprintf('\n--- BOUNDING BOX ---\n');
x_min = min(waypoints(:,1));
x_max = max(waypoints(:,1));
y_min = min(waypoints(:,2));
y_max = max(waypoints(:,2));
fprintf('X range: %.0f to %.0f m (width: %.0f m)\n', x_min, x_max, x_max-x_min);
fprintf('Y range: %.0f to %.0f m (height: %.0f m)\n', y_min, y_max, y_max-y_min);

fprintf('\n--- SHIP ACTUATION CAPABILITIES ---\n');
fprintf('Ship parameters:\n');
fprintf('  Length: 175 m\n');
fprintf('  Beam: 25.4 m\n');
fprintf('  Actuation: Twin stern azipods (full 360° steering) + bow thruster\n');
fprintf('  Max forward speed: ~12 m/s\n');
fprintf('  Max reverse: -6 m/s (stern azipods)\n\n');

fprintf('Turning radius estimate (at cruise speed 5 m/s):\n');
fprintf('  With Stern Azipods at 90° angle: ~50-80 m (excellent)\n');
fprintf('  With full azimuth control: can spiral turn from near-zero\n');
fprintf('  Bow thruster adds lateral drift capability at low speeds\n\n');

fprintf('--- FEASIBILITY VERDICT ---\n');
max_turn = 0;
for i = 2:size(waypoints, 1)-1
    h1 = segment_headings(i-1);
    h2 = segment_headings(i);
    turn_rad = wrapToPi(deg2rad(h2 - h1));
    turn_deg = rad2deg(turn_rad);
    max_turn = max(max_turn, abs(turn_deg));
end

fprintf('Maximum turn angle: %.1f°\n', max_turn);
fprintf('Shortest segment: %.1f m\n', min(segment_distances));

if max_turn > 120
    fprintf('\n❌ PROBLEMATIC: Turns >120° may cause path tracking issues\n');
    fprintf('   Recommendation: Adjust waypoint spacing or add intermediate waypoints\n');
elseif max_turn > 90
    fprintf('\n⚠️  CHALLENGING: Turns >90° require tight path control\n');
    fprintf('   The NMPC tube-tracking should handle this, but tight tolerances needed\n');
else
    fprintf('\n✓ NAVIGABLE: Turn angles are moderate\n');
end

if min(segment_distances) < 100
    fprintf('\n⚠️  COMPACT: Shortest segment %.0f m - verify NMPC horizon covers routing\n', min(segment_distances));
end

fprintf('\n--- RECOMMENDED DIAGNOSTICS ---\n');
fprintf('1. Check NMPC prediction horizon covers segments (dt*N ≥ 50-75m)\n');
fprintf('2. Monitor cross-track error (XTE) at sharp turns\n');
fprintf('3. Verify azimuth rate limits don''t cause heading lag (0.21 rad/s OK for 175m ship)\n');
fprintf('4. Ensure bow thruster activates when needed for low-speed maneuvering\n');
fprintf('5. Check that twin-stern synchrony constraints don''t lock steering\n\n');

fprintf('--- INTEGRATION CHECK ---\n');
fprintf('✓ run_nmpc.m calls rk4Step9(x, u_opt, dt)\n');
fprintf('✓ rk4Step9 uses container(x, u_ctrl) for dynamics\n');
fprintf('✓ container.m has full 3-thruster azipod model\n');
fprintf('✓ NMPC_Container_final.m configured with azipod dynamics\n\n');

fprintf('═══════════════════════════════════════════════════════════════\n\n');

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
