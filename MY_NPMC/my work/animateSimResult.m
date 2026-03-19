function animateSimResult(traj, waypoints, t_vec, harbor, cfg)
% animateSimResult  Generalised post-simulation ship animation
%   Replays a recorded 6-DOF container-ship trajectory on a 2-D map.
%   Call this AFTER the simulation loop, on already-collected trajectory data.
%
%
% Inputs:
%   traj      - 6×N state matrix  (rows: u v r x y psi)      [6-DOF azipod model]
%               x = traj(4,:), y = traj(5,:), psi = traj(6,:) [position & heading]
%   waypoints - W×2 [x, y] matrix  (NED convention), or []
%   t_vec     - 1×N simulation time vector [s], or []
%   harbor    - HarborObstacles object (calls harbor.plotMap() if available),
%               or [] to skip map drawing
%   cfg       - options struct (all fields optional):
%
%     .figNo        Figure number                    (default 200)
%     .testName     String label in figure title     (default '')
%     .shipImgFile  Absolute path to vessel icon     (default: viking.png)
%     .shipSize     Ship width as fraction of range  (default 0.08)
%     .maxFrames    Max frames rendered (auto-skip)  (default 150)
%     .pauseTime    Pause between frames [s]         (default 0.05)
%     .extraPaths   Extra trajectories to overlay:
%                   cell array of {x_array, y_array, linestyle, legendName}
%                   e.g. {ts_x, ts_y, 'm--', 'Target ship'}
%     .circObs      Struct array of circular obstacles to draw:
%                   fields: .position (2×1 [x;y]), .radius (scalar)
%
% Example (Test A):
%   cfg.figNo       = 10;
%   cfg.testName    = 'A: Path Following';
%   cfg.shipImgFile = 'C:\...\vessel_top.jpg';
%   cfg.shipSize    = 0.04;
%   animateSimResult(traj_A, wp_A, t, harbor, cfg);
%
%   add  clear animateSimResult  near the top of your script to reset
%       the persistent cache when you change the image file.
%
% Author: Riccardo Legnini (2026)


%  Persistent image cache (loaded once, fixed orientation)
persistent cachedImgFile shipImg shipAlpha shipWidthPx shipHeightPx

%  0. Parse configuration
if nargin < 5 || isempty(cfg), cfg = struct(); end

figNo      = cfgGet(cfg, 'figNo',       200);
testName   = cfgGet(cfg, 'testName',    '');
shipSize   = cfgGet(cfg, 'shipSize',    0.08);
maxFrames  = cfgGet(cfg, 'maxFrames',  150);
pauseTime  = cfgGet(cfg, 'pauseTime',  0.05);
imgFile    = cfgGet(cfg, 'shipImgFile', 'vessel_top.png');
showLegend = cfgGet(cfg, 'showLegend', false);
showCollisionCircles = cfgGet(cfg, 'showCollisionCircles', true);

%  1. Load image once  (only when file path changes)
useImage = true;  % set false if loading fails

if isempty(cachedImgFile) || ~strcmp(imgFile, cachedImgFile) || isempty(shipImg)

    if isempty(imgFile) || ~isfile(imgFile)
        fprintf('  [animateSimResult] WARNING: image not found: "%s"\n  >> Falling back to triangle icon.\n', imgFile);
        useImage = false;
    else
        fprintf('  [animateSimResult] Loading ship icon: "%s"...\n', imgFile);
        try
            [rawImg, ~, rawAlpha] = imread(imgFile);
            rawImg = im2double(rawImg);

            % Alpha: use PNG channel if present, else fully opaque.
            if isempty(rawAlpha)
                rawAlpha = ones(size(rawImg,1), size(rawImg,2));
            else
                rawAlpha = double(rawAlpha) / 255;
            end

            % flipud: MATLAB image() rows go top-to-bottom but plot y goes upward.
            shipImg   = flipud(rawImg);
            shipAlpha = flipud(rawAlpha);

            shipHeightPx  = size(shipImg, 1);
            shipWidthPx   = size(shipImg, 2);
            cachedImgFile = imgFile;
            fprintf('  [animateSimResult] Icon loaded (%dx%d px).\n', shipWidthPx, shipHeightPx);
        catch ME
            fprintf('  [animateSimResult] WARNING: could not load image: %s\n  >> Falling back to triangle icon.\n', ME.message);
            shipImg  = [];
            useImage = false;
        end
    end
else
    % Cache hit — reuse already-loaded image.
end

%  2. Prepare trajectory data — extract x/y/psi from state matrix, apply frame-skip for animation
if size(traj, 1) ~= 6
    error('animateSimResult:InvalidTrajectorySize', ...
        'Expected traj to be 6xN [u v r x y psi]. Got %dx%d.', size(traj,1), size(traj,2));
end

N     = size(traj, 2);
xPath = traj(4, :);   % North
yPath = traj(5, :);   % East
psi   = traj(6, :);   % heading [rad]

% Downsampling
skip = max(1, floor(N / maxFrames));
idx  = 1 : skip : N;

if isempty(t_vec), t_vec = (0:N-1); end

%  3. Set up figure & static elements — create styled figure, draw harbor map, obstacles, ghost path and waypoints
hFig = figure(figNo);
clf(hFig);
set(hFig, 'Name',        sprintf('NMPC Animation — %s', testName), ...
          'NumberTitle', 'off', ...
          'Color',       [0.09 0.09 0.13]);

ax = axes('Parent', hFig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
ax.Color     = [0.11 0.11 0.16];
ax.XColor    = [0.75 0.75 0.80];
ax.YColor    = [0.75 0.75 0.80];
ax.GridColor = [0.28 0.28 0.38];
ax.GridAlpha = 0.5;
xlabel(ax, 'East / y  [m]',  'Color', [0.90 0.90 0.95], 'FontSize', 12);
ylabel(ax, 'North / x  [m]', 'Color', [0.90 0.90 0.95], 'FontSize', 12);
title( ax, sprintf('Ship Simulation — %s', testName), ...
       'Color', [1 1 1], 'FontSize', 13, 'FontWeight', 'bold');

% plotMap() creates many patch objects; we capture them and clear them from
% the legend (HandleVisibility='off') so they don't appear as 'data1...dataN'.
if ~isempty(harbor) && isobject(harbor) && ismethod(harbor, 'plotMap')
    try
        axes(ax);  % make ax the current axes before plotMap draws into gca
        ch_before = get(ax, 'Children');
        harbor.plotMap();
        ch_after  = get(ax, 'Children');
        new_map_h = setdiff(ch_after, ch_before);
        if ~isempty(new_map_h)
            set(new_map_h, 'HandleVisibility', 'off');
        end
        hold(ax, 'on');
    catch ME
        warning('animateShip:plotMapFailed', '%s', ME.message);
    end
end

% --- Circular obstacles ---------------------------------------------------
if showCollisionCircles && isfield(cfg, 'circObs') && ~isempty(cfg.circObs)
    th = linspace(0, 2*pi, 64);
    for k = 1:length(cfg.circObs)
        ox = cfg.circObs(k).position(2);  % East (plot x-axis)
        oy = cfg.circObs(k).position(1);  % North (plot y-axis)
        r  = cfg.circObs(k).radius;
        plot(ax, ox + r*cos(th), oy + r*sin(th), ...
             'Color', [1.0 0.35 0.35], 'LineWidth', 1.1, ...
             'HandleVisibility', 'off');
    end
end

% --- Ghost trajectory of the executed path (full run, low opacity) -------
plot(ax, yPath, xPath, '-', ...
     'Color', [0.35 0.55 1.00 0.40], 'LineWidth', 1.2, ...
    'DisplayName', 'Executed path (ghost)');

% --- Extra paths (e.g. target ship trajectory) ---------------------------
if isfield(cfg, 'extraPaths') && ~isempty(cfg.extraPaths)
    for ep = cfg.extraPaths(:)'
        e = ep{1};
        % e = {x_array, y_array, linestyle, legend_name}
        plot(ax, e{2}, e{1}, e{3}, 'LineWidth', 1.5, 'DisplayName', e{4});
    end
end

% --- Waypoints ------------------------------------------------------------
if ~isempty(waypoints)
    plot(ax, waypoints(:,2), waypoints(:,1), 'r*-', ...
         'MarkerSize', 10, 'LineWidth', 1.0, 'DisplayName', 'Waypoints');
end

% --- Start marker ---------------------------------------------------------
plot(ax, yPath(1), xPath(1), 'go', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

% --- Axis limits (add margin) --------------------------------------------
marginFrac = 0.08;
xRng = max(xPath) - min(xPath);
yRng = max(yPath) - min(yPath);
axRng = max(max(xRng, yRng), 200);  % at least 200 m visible
xMid  = (max(xPath) + min(xPath)) / 2;
yMid  = (max(yPath) + min(yPath)) / 2;
halfSpan = axRng/2 * (1 + marginFrac);

xlim(ax, [yMid - halfSpan, yMid + halfSpan]);
ylim(ax, [xMid - halfSpan, xMid + halfSpan]);

% Ship display size in axes units
shipWidthAx  = axRng * shipSize;
if useImage
    shipHeightAx = shipWidthAx * (shipHeightPx / shipWidthPx);
else
    shipHeightAx = shipWidthAx * 2.5;
end

% --- Initialise ship handle ----------------------------------------------
cx0 = yPath(1);  cy0 = xPath(1);
if useImage
    hShip = image(ax, ...
        'XData',      [cx0 - shipWidthAx/2,  cx0 + shipWidthAx/2], ...
        'YData',      [cy0 - shipHeightAx/2, cy0 + shipHeightAx/2], ...
        'CData',      shipImg, ...
        'AlphaData',  shipAlpha, ...
        'HandleVisibility', 'off');
    uistack(hShip, 'top');
else
    [tx, ty] = shipTriangle(cx0, cy0, shipWidthAx, 0);
    hShip = fill(ax, tx, ty, [0.20 0.75 1.00], ...
        'EdgeColor', 'w', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

% --- Live trail line ------------------------------------------------------
hTrail = plot(ax, yPath(1), xPath(1), '-', ...
              'Color', [0.20 0.85 0.45], 'LineWidth', 2, ...
              'DisplayName', 'Own ship');

% --- Time stamp -----------------------------------------------------------
xlims = xlim(ax);  ylims = ylim(ax);
hTime = text(ax, xlims(1) + 0.02*diff(xlims), ...
                 ylims(2) - 0.04*diff(ylims), ...
                 't = 0 s', ...
                 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');

% Legend
if showLegend
    legend(ax, 'show', 'Location', 'best', ...
        'TextColor', [0.90 0.90 0.95], 'Color', [0.09 0.09 0.13], ...
        'EdgeColor', [0.35 0.35 0.45]);
end

drawnow;

%  4. Animate — step through downsampled frames, update ship icon position and live trail
trailX = yPath(1);
trailY = xPath(1);

for k = 1:length(idx)
    i = idx(k);

    cx = yPath(i);   % East  → plot x
    cy = xPath(i);   % North → plot y

    % Move ship icon (image orientation is fixed — no rotation needed)
    if useImage
        set(hShip, ...
            'XData', [cx - shipWidthAx/2,  cx + shipWidthAx/2], ...
            'YData', [cy - shipHeightAx/2, cy + shipHeightAx/2]);
    else
        [tx, ty] = shipTriangle(cx, cy, shipWidthAx, psi(i));
        set(hShip, 'XData', tx, 'YData', ty);
    end

    % Update trail
    trailX(end+1) = cx;   %#ok<AGROW>
    trailY(end+1) = cy;   %#ok<AGROW>
    set(hTrail, 'XData', trailX, 'YData', trailY);

    % Update time stamp
    if i <= length(t_vec)
        set(hTime, 'String', sprintf('t = %.0f s', t_vec(i)));
    end

    drawnow;          % flush every frame so the animation is actually visible
    pause(pauseTime); % pacing: default 0.05 s → ~20 fps
end

% Mark final position without the large arrow marker.
plot(ax, yPath(end), xPath(end), 'ro', ...
    'MarkerSize', 6, 'MarkerFaceColor', [1 0.3 0.3], ...
    'DisplayName', 'End', 'LineWidth', 1.0);
drawnow;

end % ---- end of animateSimResult ----------------------------------------

%  Triangle fallback — arrowhead icon pointing in direction psi [rad], used when no image file is loaded
function [tx, ty] = shipTriangle(cx, cy, w, psi)
    % A simple arrow-head: bow at top (North = psi=0)
    h = w * 2.5;
    % Local coords: bow tip at (0, h/2), stern corners at (±w/2, -h/2)
    lx = [0,  w/2, 0, -w/2, 0];
    ly = [h/2, -h/2, -h/4, -h/2, h/2];
    % Rotate by psi CW from North
    c = cos(psi);  s = sin(psi);
    tx = cx + c.*lx - s.*ly;   % East
    ty = cy + s.*lx + c.*ly;   % North
end

%  Local helper — cfgGet returns cfg.(name) if present and non-empty, otherwise returns default
function v = cfgGet(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
