%% run_nmpc_regression.m
% Batch regression harness for run_nmpc.m.
% Runs a fixed scenario matrix and parses terminal logs into pass/fail metrics.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
log_dir = fullfile(script_dir, 'plots in development process', 'logs');

if ~exist(log_dir, 'dir')
    mkdir(log_dir);
end

scenarios = [ ...
    struct('name', 'baseline_clear', 'tfinal', 320, 'dyn', 0, 'tight', 0), ...
    struct('name', 'dynamic_busy',  'tfinal', 420, 'dyn', 1, 'tight', 0), ...
    struct('name', 'tight_dynamic', 'tfinal', 420, 'dyn', 1, 'tight', 1), ...
    struct('name', 'long_stress',   'tfinal', 520, 'dyn', 1, 'tight', 1)  ...
];

results = repmat(struct( ...
    'name', '', 'log_file', '', 'final_reached', false, 'collisions', NaN, ...
    'solve_pct', NaN, 'mean_xte_m', NaN, 'max_xte_m', NaN, ...
    'phaseB_pct', NaN, 'pass', false), 1, numel(scenarios));

fprintf('==============================================================\n');
fprintf(' NMPC REGRESSION HARNESS\n');
fprintf('==============================================================\n\n');

for k = 1:numel(scenarios)
    sc = scenarios(k);
    fprintf('[%d/%d] Scenario: %s\n', k, numel(scenarios), sc.name);

    old_log = latestLogFile(log_dir);

    setenv('NMPC_TFINAL', num2str(sc.tfinal));
    setenv('NMPC_ENABLE_DYNAMIC_OBS', num2str(sc.dyn));
    setenv('NMPC_ENABLE_TIGHT_CORRIDOR_MODE', num2str(sc.tight));

    run(fullfile(script_dir, 'run_nmpc.m'));

    new_log = latestLogFile(log_dir);
    if isempty(new_log)
        warning('No terminal log found for scenario %s.', sc.name);
        continue;
    end
    if ~isempty(old_log) && strcmpi(new_log, old_log)
        warning('Log file did not rotate for scenario %s. Parsing latest available log anyway.', sc.name);
    end

    summary = parseNmpcLog(new_log);

    results(k).name = sc.name;
    results(k).log_file = new_log;
    results(k).final_reached = summary.final_reached;
    results(k).collisions = summary.collisions;
    results(k).solve_pct = summary.solve_pct;
    results(k).mean_xte_m = summary.mean_xte_m;
    results(k).max_xte_m = summary.max_xte_m;
    results(k).phaseB_pct = summary.phaseB_pct;

    % Pass criteria: strict safety + convergence + solver reliability.
    results(k).pass = summary.final_reached && ...
        isfinite(summary.collisions) && summary.collisions == 0 && ...
        isfinite(summary.solve_pct) && summary.solve_pct >= 99.0;

    fprintf('  final=%d  collisions=%g  solve=%.1f%%  mean|XTE|=%.1f  max|XTE|=%.1f  phaseB=%.1f%%  pass=%d\n\n', ...
        results(k).final_reached, results(k).collisions, results(k).solve_pct, ...
        results(k).mean_xte_m, results(k).max_xte_m, results(k).phaseB_pct, results(k).pass);
end

% Reset env overrides to avoid leaking into manual runs.
setenv('NMPC_TFINAL', '');
setenv('NMPC_ENABLE_DYNAMIC_OBS', '');
setenv('NMPC_ENABLE_TIGHT_CORRIDOR_MODE', '');

fprintf('================ REGRESSION SUMMARY ================\n');
for k = 1:numel(results)
    fprintf('%-14s pass=%d  final=%d  coll=%g  solve=%.1f%%  meanXTE=%.1f  maxXTE=%.1f\n', ...
        results(k).name, results(k).pass, results(k).final_reached, results(k).collisions, ...
        results(k).solve_pct, results(k).mean_xte_m, results(k).max_xte_m);
end
fprintf('====================================================\n');

function log_file = latestLogFile(log_dir)
    d = dir(fullfile(log_dir, 'nmpc_run_*_log.txt'));
    if isempty(d)
        log_file = '';
        return;
    end
    [~, idx] = max([d.datenum]);
    log_file = fullfile(d(idx).folder, d(idx).name);
end

function out = parseNmpcLog(log_file)
    txt = fileread(log_file);

    out = struct('final_reached', false, 'collisions', NaN, 'solve_pct', NaN, ...
        'mean_xte_m', NaN, 'max_xte_m', NaN, 'phaseB_pct', NaN);

    out.final_reached = ~isempty(regexpi(txt, 'FINAL WAYPOINT (REACHED|CAPTURED|SAFELY CAPTURED)', 'once'));

    tok = regexp(txt, 'Collisions detected:\s*([0-9]+)', 'tokens', 'once');
    if ~isempty(tok), out.collisions = str2double(tok{1}); end

    tok = regexp(txt, 'NMPC solves:\s*([0-9]+)\/([0-9]+)\s*\(([0-9\.]+)%\)', 'tokens', 'once');
    if ~isempty(tok), out.solve_pct = str2double(tok{3}); end

    tok = regexp(txt, 'Mean \|XTE\|:\s*([0-9\.]+)\s*m,\s*Max \|XTE\|:\s*([0-9\.]+)\s*m', 'tokens', 'once');
    if ~isempty(tok)
        out.mean_xte_m = str2double(tok{1});
        out.max_xte_m = str2double(tok{2});
    end

    tok = regexp(txt, 'Phase-B occupancy:\s*([0-9\.]+)%', 'tokens', 'once');
    if ~isempty(tok), out.phaseB_pct = str2double(tok{1}); end
end
