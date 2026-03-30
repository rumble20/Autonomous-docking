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
%     .recordVideo  Save MP4 file                    (default false)
%     .recordGif    Save GIF file                    (default false)
%     .recordFps    Recording frame rate             (default 15)
%     .videoFile    Output MP4 path                  (default 'nmpc_run.mp4')
%     .gifFile      Output GIF path                  (default 'nmpc_run.gif')
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
persistent cachedImgFile shipImg shipAlpha shipWidthPx shipHeightPx shipEffWidthPx shipEffHeightPx

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
dynamicObsHistory = cfgGet(cfg, 'dynamicObsHistory', []);
dynamicObsRadius = cfgGet(cfg, 'dynamicObsRadius', 20);
hullCfg = cfgGet(cfg, 'hullCfg', []);
recordVideo = cfgGet(cfg, 'recordVideo', false);
recordGif = cfgGet(cfg, 'recordGif', false);
recordFps = cfgGet(cfg, 'recordFps', 15);
videoFile = cfgGet(cfg, 'videoFile', 'nmpc_run.mp4');
gifFile = cfgGet(cfg, 'gifFile', 'nmpc_run.gif');

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

            % Effective icon aspect from non-transparent pixels so we do not
            % stretch square canvases with tall/narrow ship silhouettes.
            mask = shipAlpha > 0.05;
            cols = find(any(mask, 1));
            rows = find(any(mask, 2));
            if isempty(cols) || isempty(rows)
                shipEffWidthPx = shipWidthPx;
                shipEffHeightPx = shipHeightPx;
            else
                shipEffWidthPx = cols(end) - cols(1) + 1;
                shipEffHeightPx = rows(end) - rows(1) + 1;
            end

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
    if isempty(shipEffWidthPx) || isempty(shipEffHeightPx)
        shipEffWidthPx = shipWidthPx;
        shipEffHeightPx = shipHeightPx;
    end
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

hDynObs = gobjects(0);
if ~isempty(dynamicObsHistory)
    th_dyn = linspace(0, 2*pi, 40);
    nDyn = size(dynamicObsHistory, 1);
    dynCols = lines(max(1, nDyn));
    hDynObs = gobjects(nDyn, 1);
    for k = 1:nDyn
        xk = dynamicObsHistory(k,2,1);
        yk = dynamicObsHistory(k,1,1);
        hDynObs(k) = plot(ax, xk + dynamicObsRadius*cos(th_dyn), yk + dynamicObsRadius*sin(th_dyn), ...
            '--', 'Color', dynCols(k,:), 'LineWidth', 1.2, 'HandleVisibility', 'off');
    end
end

% --- Ghost trajectory of the executed path (full run, low opacity) -------
plot(ax, yPath, xPath, '-', ...
    'Color', [0.35 0.55 1.00], 'LineWidth', 1.2, ...
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
% When hull config is available, keep icon size physically consistent
% with the same rectangle used by collision/hitbox plotting.
if ~isempty(hullCfg) && isfield(hullCfg, 'half_beam_m') && isfield(hullCfg, 'half_length_m')
    boxWidth  = 2 * hullCfg.half_beam_m;
    boxHeight = 2 * hullCfg.half_length_m;
    if useImage
        effAspect = shipEffHeightPx / max(shipEffWidthPx, 1); % height/width
        shipWidthAx  = min(boxWidth, boxHeight / max(effAspect, 1e-6));
        shipHeightAx = shipWidthAx * effAspect;
    else
        shipWidthAx  = boxWidth;
        shipHeightAx = boxHeight;
    end
else
    shipWidthAx  = axRng * shipSize;
    if useImage
        shipHeightAx = shipWidthAx * (shipHeightPx / shipWidthPx);
    else
        shipHeightAx = shipWidthAx * 2.5;
    end
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

% --- Hull footprint rectangle (if hull_cfg provided) ---------------------
hHullRect = [];
if ~isempty(hullCfg) && isstruct(hullCfg) && isfield(hullCfg, 'half_length_m')
    % Initialize rectangle at starting position
    cx0 = yPath(1);  cy0 = xPath(1);  psi0 = psi(1);
    rectCorners0 = computeHullCorners(cx0, cy0, hullCfg.half_length_m, hullCfg.half_beam_m, psi0);
    hHullRect = patch(ax, rectCorners0(1,:), rectCorners0(2,:), ...
                      [1.0 0.50 0.20], ...
                      'FaceAlpha', 0.15, 'EdgeColor', [1.0 0.50 0.20], ...
                      'LineWidth', 1.5, 'HandleVisibility', 'off');
end

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

% 3.5. Optional recording setup (MP4/GIF)
writerObj = [];
gifInitialized = false;
videoFileSaved = videoFile;
if recordVideo
    [vdir, ~, ~] = fileparts(videoFile);
    if ~isempty(vdir) && ~exist(vdir, 'dir')
        mkdir(vdir);
    end
    profiles = {'MPEG-4', 'Motion JPEG AVI'};
    open_ok = false;
    lastErr = '';
    for p = 1:numel(profiles)
        profileName = profiles{p};
        tryFile = videoFile;
        if ~strcmpi(profileName, 'MPEG-4')
            [vp, vn, ~] = fileparts(videoFile);
            tryFile = fullfile(vp, [vn '.avi']);
        end
        try
            writerObj = VideoWriter(tryFile, profileName);
            writerObj.FrameRate = max(1, recordFps);
            open(writerObj);
            videoFileSaved = tryFile;
            open_ok = true;
            if strcmpi(profileName, 'MPEG-4')
                fprintf('  [animateSimResult] Recording video: "%s"\n', videoFileSaved);
            else
                fprintf('  [animateSimResult] MPEG-4 unavailable, recording with %s: "%s"\n', profileName, videoFileSaved);
            end
            break;
        catch ME
            lastErr = ME.message;
            writerObj = [];
        end
    end
    if ~open_ok
        warning('animateSimResult:VideoOpenFailed', 'Could not open video writer: %s', lastErr);
        recordVideo = false;
    end
end
if recordGif
    [gdir, ~, ~] = fileparts(gifFile);
    if ~isempty(gdir) && ~exist(gdir, 'dir')
        mkdir(gdir);
    end
    fprintf('  [animateSimResult] Recording GIF: "%s"\n', gifFile);
end

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

    % Update hull footprint rectangle
    if ~isempty(hHullRect) && isvalid(hHullRect)
        rectCorners = computeHullCorners(cx, cy, hullCfg.half_length_m, hullCfg.half_beam_m, psi(i));
        set(hHullRect, 'XData', rectCorners(1,:), 'YData', rectCorners(2,:));
    end

    % Update trail
    trailX(end+1) = cx;   %#ok<AGROW>
    trailY(end+1) = cy;   %#ok<AGROW>
    set(hTrail, 'XData', trailX, 'YData', trailY);

    % Update time stamp
    if i <= length(t_vec)
        set(hTime, 'String', sprintf('t = %.0f s', t_vec(i)));
    end

    % Update dynamic obstacles (if provided)
    if ~isempty(hDynObs)
        th_dyn = linspace(0, 2*pi, 40);
        for d = 1:length(hDynObs)
            xk = dynamicObsHistory(d,2,min(i, size(dynamicObsHistory,3)));
            yk = dynamicObsHistory(d,1,min(i, size(dynamicObsHistory,3)));
            if isfinite(xk) && isfinite(yk)
                set(hDynObs(d), 'XData', xk + dynamicObsRadius*cos(th_dyn), ...
                    'YData', yk + dynamicObsRadius*sin(th_dyn), 'Visible', 'on');
            else
                set(hDynObs(d), 'Visible', 'off');
            end
        end
    end

    drawnow;          % flush every frame so the animation is actually visible

    % Write animation frame to outputs (if enabled)
    if recordVideo || recordGif
        try
            frame = getframe(hFig);
            if recordVideo && ~isempty(writerObj)
                writeVideo(writerObj, frame);
            end
            if recordGif
                [imRgb, mapGif] = rgb2ind(frame2im(frame), 256);
                if ~gifInitialized
                    imwrite(imRgb, mapGif, gifFile, 'gif', 'LoopCount', inf, ...
                        'DelayTime', max(0.01, 1/max(1, recordFps)));
                    gifInitialized = true;
                else
                    imwrite(imRgb, mapGif, gifFile, 'gif', 'WriteMode', 'append', ...
                        'DelayTime', max(0.01, 1/max(1, recordFps)));
                end
            end
        catch ME
            warning('animateSimResult:FrameWriteFailed', 'Frame capture/write failed: %s', ME.message);
            recordVideo = false;
            recordGif = false;
        end
    end

    pause(pauseTime); % pacing: default 0.05 s → ~20 fps
end

% Mark final position without the large arrow marker.
plot(ax, yPath(end), xPath(end), 'ro', ...
    'MarkerSize', 6, 'MarkerFaceColor', [1 0.3 0.3], ...
    'DisplayName', 'End', 'LineWidth', 1.0);
drawnow;

% Final frame write + cleanup
if recordVideo || recordGif
    try
        frame = getframe(hFig);
        if recordVideo && ~isempty(writerObj)
            writeVideo(writerObj, frame);
        end
        if recordGif
            [imRgb, mapGif] = rgb2ind(frame2im(frame), 256);
            if ~gifInitialized
                imwrite(imRgb, mapGif, gifFile, 'gif', 'LoopCount', inf, ...
                    'DelayTime', max(0.01, 1/max(1, recordFps)));
            else
                imwrite(imRgb, mapGif, gifFile, 'gif', 'WriteMode', 'append', ...
                    'DelayTime', max(0.01, 1/max(1, recordFps)));
            end
        end
    catch ME
        warning('animateSimResult:FinalFrameWriteFailed', 'Final frame capture/write failed: %s', ME.message);
    end
end
if recordVideo && ~isempty(writerObj)
    try
        close(writerObj);
    catch ME
        warning('animateSimResult:VideoCloseFailed', 'Could not close video writer cleanly: %s', ME.message);
    end
end
if recordVideo
    fprintf('  [animateSimResult] Video saved: "%s"\n', videoFileSaved);
end
if recordGif
    fprintf('  [animateSimResult] GIF saved: "%s"\n', gifFile);
end

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

%  Hull footprint rectangle corner computation — returns 4 corner points (closed polygon)
%  Inputs:
%    cx, cy      - center position in world frame (East, North) [m]
%    half_len    - half-length of rectangle aligned with ship longitudinal axis [m]
%    half_beam   - half-beam of rectangle (perpendicular to length) [m]
%    psi         - ship heading [rad]
%  Output:
%    corners     - 2×5 array: [cx1 cx2 cx3 cx4 cx1; cy1 cy2 cy3 cy4 cy1] (closed loop)
%
function corners = computeHullCorners(cx, cy, half_len, half_beam, psi)
    % Rectangle corners in body frame (ship-aligned), then rotate and translate
    % Bow at +x (length direction), starboard at +y (beam direction)
    local_x = [ half_len,  half_len, -half_len, -half_len,  half_len];
    local_y = [ half_beam, -half_beam, -half_beam,  half_beam,  half_beam];
    
    % Rotation matrix: rotate from body frame to world frame by angle psi
    c = cos(psi);   s = sin(psi);
    world_x = c .* local_x - s .* local_y;  % East
    world_y = s .* local_x + c .* local_y;  % North
    
    % Translate to center position
    corners = [cx + world_x; cy + world_y];
end

