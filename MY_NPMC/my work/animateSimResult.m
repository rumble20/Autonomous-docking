function animateSimResult(traj, waypoints, t_vec, harbor, cfg)
% animateSimResult  Generalised post-simulation ship animation WITH SPEED VISUALIZATION
%   Replays a recorded 6-DOF container-ship trajectory on a 2-D map.
%   Includes a live speed plot subplot for fuel consumption analysis.
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
%     .shipImageScale Display scale vs hitbox size   (default 1.45)
%     .showHullHitbox Show hull hitbox outline       (default false)
%     .maxFrames    Max frames rendered (auto-skip)  (default 150)
%     .pauseTime    Pause between frames [s]         (default 0.05)
%     .recordVideo  Save MP4 file                    (default false)
%     .recordGif    Save GIF file                    (default false)
%     .recordFps    Recording frame rate             (default 15)
%     .videoFile    Output MP4 path                  (default 'nmpc_run.mp4')
%     .gifFile      Output GIF path                  (default 'nmpc_run.gif')
%     .plannedRoutes  Planned route history for each step (optional):
%                   cell array length N with each entry containing either:
%                     - state matrix (>=6 x K) where rows 4/5 are x/y
%                     - 2xK array [x; y] or Kx2 array [x y]
%                   or a 3D array with size 2xKxN or 8xKxN (state history).
%     .plannedRouteMaxHistory  Max predicted routes to show (default 6)
%     .plannedRouteAlphaMin    Oldest route opacity (default 0.10)
%     .plannedRouteAlphaMax    Newest route opacity (default 0.70)
%     .plannedRouteLineWidth   Route line width (default 1.4)
%     .plannedRouteColor       Base RGB color (default [0.95 0.75 0.20])
%     .plannedRouteStride      Update/plot every Nth step (default 1)
%     .thrusterHistory   Control history (5xN): [alpha1 alpha2 n1_c n2_c n3_c]
%     .thrusterCfg       Thruster visualization config (optional):
%                   .pos_body_m   3x2 positions in body frame [x_fwd y_starboard]
%                   .types        cell array (e.g. {'azipod','azipod','bow'})
%                   .n_max        max RPM per thruster
%                   .alpha_idx    control index for azimuth (0 if fixed)
%                   .rpm_idx      control index for rpm command
%                   .alpha_fixed  fixed azimuth when alpha_idx=0 (rad)
%                   .colors       Nx3 RGB rows for thruster arrows
%                   .arrow_max_len_m  max arrow length in meters
%                   .arrow_min_len_m  min arrow length in meters
%     .showThrusters     Enable thruster arrows (default true)
%     .thrusterDeadbandRpm   Hide arrows below this RPM (default 5)
%     .thrusterLineWidth     Arrow line width (default 2.0)
%     .thrusterPowerExponent Length scale exponent vs RPM (default 1.6)
%     .thrusterSmoothing     0..0.95 smoothing for arrow vectors (default 0.60)
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
% Updated: 2026-04-09 - Added speed visualization subplot


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
flipShipImageVertical = cfgGet(cfg, 'flipShipImageVertical', false);
shipImageScale = cfgGet(cfg, 'shipImageScale', 1.5);
showHullHitbox = cfgGet(cfg, 'showHullHitbox', false);
recordVideo = cfgGet(cfg, 'recordVideo', false);
recordGif = cfgGet(cfg, 'recordGif', false);
recordFps = cfgGet(cfg, 'recordFps', 15);
videoFile = cfgGet(cfg, 'videoFile', 'nmpc_run.mp4');
gifFile = cfgGet(cfg, 'gifFile', 'nmpc_run.gif');
plannedRoutes = cfgGet(cfg, 'plannedRoutes', []);
plannedRouteMaxHistory = cfgGet(cfg, 'plannedRouteMaxHistory', 6);
plannedRouteAlphaMin = cfgGet(cfg, 'plannedRouteAlphaMin', 0.10);
plannedRouteAlphaMax = cfgGet(cfg, 'plannedRouteAlphaMax', 0.70);
plannedRouteLineWidth = cfgGet(cfg, 'plannedRouteLineWidth', 1.4);
plannedRouteColor = cfgGet(cfg, 'plannedRouteColor', [0.95 0.75 0.20]);
plannedRouteStride = cfgGet(cfg, 'plannedRouteStride', 1);
thrusterHistory = cfgGet(cfg, 'thrusterHistory', []);
thrusterCfg = cfgGet(cfg, 'thrusterCfg', struct());
showThrusters = cfgGet(cfg, 'showThrusters', true);
thrusterDeadbandRpm = cfgGet(cfg, 'thrusterDeadbandRpm', 5);
thrusterLineWidth = cfgGet(cfg, 'thrusterLineWidth', 2.0);
thrusterPowerExponent = cfgGet(cfg, 'thrusterPowerExponent', 1.6);
thrusterSmoothing = cfgGet(cfg, 'thrusterSmoothing', 0.60);

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

            % Optional vertical flip for icons stored with opposite row orientation.
            % Default false so bow/stern orientation follows the source image directly.
            if flipShipImageVertical
                shipImg   = flipud(rawImg);
                shipAlpha = flipud(rawAlpha);
            else
                shipImg   = rawImg;
                shipAlpha = rawAlpha;
            end

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
if size(traj, 1) < 6
    error('animateSimResult:InvalidTrajectorySize', ...
    'Expected traj to have at least 6 rows [u v r x y psi]. Got %dx%d.', size(traj,1), size(traj,2));
end

N     = size(traj, 2);
u_vel = traj(1, :);   % Surge velocity [m/s] - NEW: for speed visualization
xPath = traj(4, :);   % North
yPath = traj(5, :);   % East
psi   = traj(6, :);   % heading [rad]

% Downsampling
skip = max(1, floor(N / maxFrames));
idx  = 1 : skip : N;

if isempty(t_vec), t_vec = (0:N-1); end

%  3. Set up figure & static elements — create styled figure with SUBPLOT layout
hFig = figure(figNo);
clf(hFig);
set(hFig, 'Name',        sprintf('NMPC Animation — %s', testName), ...
          'NumberTitle', 'off', ...
          'Color',       [0.09 0.09 0.13]);

% Make animation window larger for better readability in live view and recordings.
scr = get(0, 'ScreenSize');
figW = min(1600, max(1200, round(0.85 * scr(3))));
figH = min(980,  max(760,  round(0.85 * scr(4))));
figX = max(20, round((scr(3) - figW) / 2));
figY = max(20, round((scr(4) - figH) / 2));
set(hFig, 'Position', [figX, figY, figW, figH]);

% Create subplots and assign custom proportions for clearer composition.
ax = subplot(2, 1, 1, 'Parent', hFig);
ax_speed = subplot(2, 1, 2, 'Parent', hFig);
hold(ax, 'on');
hold(ax_speed, 'on');

% [left bottom width height] in normalized units
set(ax,       'Position', [0.06, 0.36, 0.91, 0.60]);
set(ax_speed, 'Position', [0.06, 0.08, 0.91, 0.22]);

% Map axes styling
axis(ax, 'equal');
grid(ax, 'on');
ax.Color     = [0.11 0.11 0.16];
ax.XColor    = [0.75 0.75 0.80];
ax.YColor    = [0.75 0.75 0.80];
ax.GridColor = [0.28 0.28 0.38];
ax.GridAlpha = 0.5;
xlabel(ax, 'East / y  [m]',  'Color', [0.90 0.90 0.95], 'FontSize', 11);
ylabel(ax, 'North / x  [m]', 'Color', [0.90 0.90 0.95], 'FontSize', 11);
title(ax, sprintf('Ship Simulation — %s', testName), ...
    'Color', [1 1 1], 'FontSize', 13, 'FontWeight', 'bold');

% NEW: Speed plot axes styling
ax_speed.Color = [0.11 0.11 0.16];
ax_speed.XColor = [0.75 0.75 0.80];
ax_speed.YColor = [0.75 0.75 0.80];
ax_speed.GridColor = [0.28 0.28 0.38];
ax_speed.GridAlpha = 0.5;
grid(ax_speed, 'on');
xlabel(ax_speed, 'Time [s]', 'Color', [0.90 0.90 0.95], 'FontSize', 11);
ylabel(ax_speed, 'Surge velocity [m/s]', 'Color', [0.90 0.90 0.95], 'FontSize', 11);
title(ax_speed, 'Forward Speed Profile', 'Color', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold');
set(ax_speed, 'XLim', [t_vec(1), t_vec(end)]);
set(ax_speed, 'YLim', [min(u_vel)*0.95, max(u_vel)*1.05]);

% NEW: Ghost profile (full trajectory at low opacity)
plot(ax_speed, t_vec, u_vel, '-', 'Color', [0.35 0.55 1.00], 'LineWidth', 1.2, ...
    'DisplayName', 'Full profile (ghost)', 'HandleVisibility', 'on');

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
cx0 = yPath(1);  cy0 = xPath(1);  psi0 = psi(1);
if ~isempty(hullCfg) && isstruct(hullCfg) && isfield(hullCfg, 'half_length_m') && isfield(hullCfg, 'half_beam_m')
    shipHalfLen = hullCfg.half_length_m;
    shipHalfBeam = hullCfg.half_beam_m;
else
    shipHalfLen = shipHeightAx / 2;
    shipHalfBeam = shipWidthAx / 2;
end

% Display-only scaling for the ship icon (does not alter collision hitbox geometry).
shipImgHalfLen = 0.5*shipHalfLen;
shipImgHalfBeam = 0.5*shipHalfBeam;
if useImage
    shipImageScale = max(0.5, shipImageScale);
    shipImgHalfLen = shipHalfLen * shipImageScale;
    shipImgHalfBeam = shipHalfBeam * shipImageScale;
end

if useImage
    [xq0, yq0] = computeShipImageQuad(cx0, cy0, shipImgHalfLen, shipImgHalfBeam, psi0);
    hShip = surface(ax, ...
        xq0, yq0, zeros(2), ...
        'CData', shipImg, ...
        'FaceColor', 'texturemap', ...
        'EdgeColor', 'none', ...
        'AlphaData', shipAlpha, ...
        'FaceAlpha', 'texturemap', ...
        'HandleVisibility', 'off');
    uistack(hShip, 'top');
else
    [tx, ty] = shipTriangle(cx0, cy0, 2 * shipHalfBeam, psi0);
    hShip = fill(ax, tx, ty, [0.20 0.75 1.00], ...
        'EdgeColor', 'w', 'LineWidth', 1.2, 'HandleVisibility', 'off');
end

% Bow-direction arrow removed to reduce visual clutter

% --- Live trail line ------------------------------------------------------
hTrail = plot(ax, yPath(1), xPath(1), '-', ...
              'Color', [0.20 0.85 0.45], 'LineWidth', 2, ...
              'DisplayName', 'Own ship');

% --- Hull footprint rectangle (if enabled) -------------------------------
hHullRect = [];
if showHullHitbox && ~isempty(hullCfg) && isstruct(hullCfg) && isfield(hullCfg, 'half_length_m')
    % Initialize rectangle at starting position
    cx0 = yPath(1);  cy0 = xPath(1);  psi0 = psi(1);
    rectCorners0 = computeHullCorners(cx0, cy0, hullCfg.half_length_m, hullCfg.half_beam_m, psi0);
    hHullRect = patch(ax, rectCorners0(1,:), rectCorners0(2,:), ...
                      [1.0 0.50 0.20], ...
                      'FaceAlpha', 0.04, 'EdgeColor', [1.0 0.62 0.18], ...
                      'LineWidth', 2.0, 'LineStyle', '--', 'HandleVisibility', 'off');
    uistack(hHullRect, 'top');
end

% --- Thruster arrows (optional) ------------------------------------------
thrusterPosBody = cfgGet(thrusterCfg, 'pos_body_m', []);
thrusterTypes = cfgGet(thrusterCfg, 'types', {'azipod', 'azipod', 'bow'});
thrusterMaxRpm = cfgGet(thrusterCfg, 'n_max', [160 160 140]);
thrusterAlphaIdx = cfgGet(thrusterCfg, 'alpha_idx', [1 2 0]);
thrusterRpmIdx = cfgGet(thrusterCfg, 'rpm_idx', [3 4 5]);
thrusterAlphaFixed = cfgGet(thrusterCfg, 'alpha_fixed', [NaN NaN pi/2]);
thrusterColors = cfgGet(thrusterCfg, 'colors', []);
thrusterSmoothing = max(0.0, min(0.95, thrusterSmoothing));
haveThrusters = showThrusters && ~isempty(thrusterHistory) && ~isempty(thrusterPosBody);
hThrusters = gobjects(0);
thrusterVecPrev = [];
thrusterMaxLen = 0;
thrusterMinLen = 0;

if haveThrusters
    nThr = size(thrusterPosBody, 1);
    if size(thrusterHistory, 1) ~= 5 && size(thrusterHistory, 2) == 5
        thrusterHistory = thrusterHistory';
    end
    baseLen = 2 * shipHalfLen;
    if ~isempty(hullCfg) && isfield(hullCfg, 'length_m')
        baseLen = hullCfg.length_m;
    end
    thrusterMaxLen = cfgGet(thrusterCfg, 'arrow_max_len_m', max(6, 0.20 * baseLen));
    thrusterMinLen = cfgGet(thrusterCfg, 'arrow_min_len_m', max(2, 0.12 * baseLen));
    if isempty(thrusterColors)
        thrusterColors = [0.95 0.45 0.20; 0.95 0.45 0.20; 0.35 0.85 1.0];
    end
    hThrusters = gobjects(nThr, 1);
    thrusterVecPrev = zeros(nThr, 2);
    for th = 1:nThr
        colorRow = thrusterColors(min(th, size(thrusterColors, 1)), :);
        hThrusters(th) = quiver(ax, cx0, cy0, 0, 0, 0, ...
            'Color', colorRow, 'LineWidth', thrusterLineWidth, 'MaxHeadSize', 2.2, ...
            'HandleVisibility', 'off');
    end
    uistack(hThrusters, 'top');

end

% --- Time stamp -----------------------------------------------------------
xlims = xlim(ax);  ylims = ylim(ax);
hTime = text(ax, xlims(1) + 0.02*diff(xlims), ...
                 ylims(2) - 0.04*diff(ylims), ...
                 't = 0 s', ...
                 'Color', [1 1 1], 'FontSize', 11, 'FontWeight', 'bold');

% Legend for map
if showLegend
    legend(ax, 'show', 'Location', 'best', ...
        'TextColor', [0.90 0.90 0.95], 'Color', [0.09 0.09 0.13], ...
        'EdgeColor', [0.35 0.35 0.45]);
end

% NEW: Speed plot legend
legend(ax_speed, 'show', 'Location', 'best', ...
    'TextColor', [0.90 0.90 0.95], 'Color', [0.09 0.09 0.13], ...
    'EdgeColor', [0.35 0.35 0.45], 'FontSize', 9);

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
% NEW: Initialize live speed visualization
speedX_live = [t_vec(1)];                    % Time points for live speed
speedY_live = [u_vel(1)];                    % Speed values for live line
hSpeedLine = plot(ax_speed, speedX_live, speedY_live, '-', ...
    'Color', [0.20 0.85 0.45], 'LineWidth', 2.2, ...
    'DisplayName', 'Live speed', 'HandleVisibility', 'on');
hSpeedMarker = plot(ax_speed, t_vec(1), u_vel(1), 'o', ...
    'Color', [1.0 0.95 0.10], 'MarkerSize', 7, 'MarkerFaceColor', [1.0 0.95 0.10], ...
    'HandleVisibility', 'off');

trailX = yPath(1);
trailY = xPath(1);

% Planned route overlay handles (updated each frame when provided).
hPlanRoutes = gobjects(0);
havePlannedRoutes = ~isempty(plannedRoutes);
plannedRouteStride = max(1, round(plannedRouteStride));
plannedRouteMaxHistory = max(1, round(plannedRouteMaxHistory));
plannedRouteAlphaMin = max(0.02, min(1.0, plannedRouteAlphaMin));
plannedRouteAlphaMax = max(plannedRouteAlphaMin, min(1.0, plannedRouteAlphaMax));
planHistLen = getPlanHistoryLength(plannedRoutes);
planHistUsesFrames = planHistLen > 0 && planHistLen == numel(idx) && planHistLen ~= N;

for k = 1:length(idx)
    i = idx(k);

    cx = yPath(i);   % East  → plot x
    cy = xPath(i);   % North → plot y

    % Move ship icon and align orientation to hull heading
    if useImage
        [xq, yq] = computeShipImageQuad(cx, cy, shipImgHalfLen, shipImgHalfBeam, psi(i));
        set(hShip, 'XData', xq, 'YData', yq);
    else
        [tx, ty] = shipTriangle(cx, cy, 2 * shipHalfBeam, psi(i));
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

    % Thruster arrows (if provided)
    if haveThrusters && ~isempty(hThrusters)
        ctrlIdx = min(i, size(thrusterHistory, 2));
        if ctrlIdx >= 1
            u_cmd = thrusterHistory(:, ctrlIdx);
            for th = 1:length(hThrusters)
                alpha_idx = 0;
                rpm_idx = 0;
                if th <= numel(thrusterAlphaIdx)
                    alpha_idx = thrusterAlphaIdx(th);
                end
                if th <= numel(thrusterRpmIdx)
                    rpm_idx = thrusterRpmIdx(th);
                end

                rpm_cmd = 0;
                if rpm_idx > 0 && rpm_idx <= size(u_cmd, 1)
                    rpm_cmd = u_cmd(rpm_idx);
                end
                if abs(rpm_cmd) < thrusterDeadbandRpm
                    set(hThrusters(th), 'Visible', 'off');
                    thrusterVecPrev(th, :) = thrusterSmoothing * thrusterVecPrev(th, :);
                    continue;
                end

                alpha_cmd = NaN;
                if alpha_idx > 0 && alpha_idx <= size(u_cmd, 1)
                    alpha_cmd = u_cmd(alpha_idx);
                elseif th <= numel(thrusterAlphaFixed)
                    alpha_cmd = thrusterAlphaFixed(th);
                end
                if ~isfinite(alpha_cmd)
                    alpha_cmd = 0;
                end

                max_rpm = thrusterMaxRpm(min(th, numel(thrusterMaxRpm)));
                rpm_norm = min(1.0, abs(rpm_cmd) / max(max_rpm, 1e-6));
                len = thrusterMinLen + (thrusterMaxLen - thrusterMinLen) * (rpm_norm ^ thrusterPowerExponent);
                dir_sign = sign(rpm_cmd);
                if dir_sign == 0
                    dir_sign = 1;
                end

                dir_body_x = cos(alpha_cmd) * dir_sign;
                dir_body_y = sin(alpha_cmd) * dir_sign;
                if th <= numel(thrusterTypes) && strcmpi(thrusterTypes{th}, 'bow')
                    dir_body_x = 0;
                    dir_body_y = dir_sign;
                end

                [px, py] = bodyToWorldPoint(cx, cy, thrusterPosBody(th,1), thrusterPosBody(th,2), psi(i));
                [vx, vy] = bodyToWorldVector(dir_body_x * len, dir_body_y * len, psi(i));
                thrusterVecPrev(th, :) = thrusterSmoothing * thrusterVecPrev(th, :) + ...
                    (1 - thrusterSmoothing) * [vx, vy];

                set(hThrusters(th), 'XData', px, 'YData', py, ...
                    'UData', thrusterVecPrev(th,1), 'VData', thrusterVecPrev(th,2), ...
                    'Visible', 'on');
            end
        end
    end

    % Planned route overlay (fade old predictions toward background color)
    if havePlannedRoutes && (mod(k-1, plannedRouteStride) == 0)
        if ~isempty(hPlanRoutes)
            try
                delete(hPlanRoutes(ishandle(hPlanRoutes)));
            catch
                delete(hPlanRoutes);
            end
        end
        hPlanRoutes = gobjects(0);
        stepStart = max(1, i - (plannedRouteMaxHistory-1)*plannedRouteStride);
        stepList = stepStart:plannedRouteStride:i;
        nHist = numel(stepList);
        if nHist > 0
            alphaVals = linspace(plannedRouteAlphaMin, plannedRouteAlphaMax, nHist);
            for s = 1:nHist
                stepIdx = stepList(s);
                planIdx = stepIdx;
                if planHistUsesFrames
                    planIdx = max(1, min(planHistLen, floor((stepIdx - 1) / skip) + 1));
                end
                [px, py] = getPlannedRoute(plannedRoutes, planIdx);
                if isempty(px) || isempty(py)
                    continue;
                end
                % Blend planned route color into the background to mimic opacity.
                cBlend = alphaVals(s) * plannedRouteColor + (1 - alphaVals(s)) * ax.Color;
                hPlanRoutes(end+1,1) = plot(ax, px, py, '-', ...
                    'Color', cBlend, 'LineWidth', plannedRouteLineWidth, ...
                    'HandleVisibility', 'off');
            end
            if exist('hShip', 'var') && isvalid(hShip)
                uistack(hShip, 'top');
            end
            if ~isempty(hHullRect) && isvalid(hHullRect)
                uistack(hHullRect, 'top');
            end
        end
    end

    % NEW: Update live speed plot - append current point to live line and marker
    speedX_live(end+1) = t_vec(i);
    speedY_live(end+1) = u_vel(i);
    set(hSpeedLine, 'XData', speedX_live, 'YData', speedY_live);
    set(hSpeedMarker, 'XData', t_vec(i), 'YData', u_vel(i));

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
    % Map body frame to plot frame (x=East, y=North) with psi referenced from North.
    s = sin(psi);  c = cos(psi);
    tx = cx + s.*lx + c.*ly;   % East
    ty = cy + c.*lx - s.*ly;   % North
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
    
    % Map from body frame to plot frame (East, North) with psi from North.
    s = sin(psi);   c = cos(psi);
    world_x = s .* local_x + c .* local_y;  % East
    world_y = c .* local_x - s .* local_y;  % North
    
    % Translate to center position
    corners = [cx + world_x; cy + world_y];
end

function [xq, yq] = computeShipImageQuad(cx, cy, half_len, half_beam, psi)
% Return a 2x2 textured quad aligned with ship heading and hitbox geometry.
% Corner order (body frame):
%   A = bow-port, B = bow-starboard, C = stern-starboard, D = stern-port
% surface uses [A B; D C] to map image rows bow->stern, cols port->starboard.
    local_x = [ half_len,  half_len, -half_len, -half_len];
    local_y = [ half_beam, -half_beam, -half_beam,  half_beam];

    s = sin(psi);   c = cos(psi);
    world_x = cx + s .* local_x + c .* local_y;
    world_y = cy + c .* local_x - s .* local_y;

    xq = [world_x(1), world_x(2); world_x(4), world_x(3)];
    yq = [world_y(1), world_y(2); world_y(4), world_y(3)];
end

function [px, py] = bodyToWorldPoint(cx, cy, x_body, y_body, psi)
% Transform a body-frame point into the map frame (East, North).
    [dx, dy] = bodyToWorldVector(x_body, y_body, psi);
    px = cx + dx;
    py = cy + dy;
end

function [vx, vy] = bodyToWorldVector(x_body, y_body, psi)
% Transform a body-frame vector into the map frame (East, North).
    s = sin(psi);
    c = cos(psi);
    vx = s * x_body + c * y_body;  % East
    vy = c * x_body - s * y_body;  % North
end

function [px, py] = getPlannedRoute(planHist, stepIdx)
% Extract predicted route at a given step index. Returns map-frame coords
% (px = East, py = North) ready for plotting.
    px = [];
    py = [];
    if isempty(planHist) || stepIdx < 1
        return;
    end

    data = [];
    if iscell(planHist)
        if stepIdx <= numel(planHist)
            data = planHist{stepIdx};
        end
    elseif isstruct(planHist)
        if stepIdx <= numel(planHist)
            data = planHist(stepIdx);
        end
    elseif isnumeric(planHist) && ndims(planHist) == 3
        if stepIdx <= size(planHist, 3)
            data = planHist(:,:,stepIdx);
        end
    end

    if isempty(data)
        return;
    end

    if isstruct(data)
        if isfield(data, 'x') && isfield(data, 'y')
            x = data.x;
            y = data.y;
        elseif isfield(data, 'X_pred')
            Xp = data.X_pred;
            if size(Xp, 1) >= 5
                x = Xp(4, :);
                y = Xp(5, :);
            else
                return;
            end
        else
            return;
        end
    elseif isnumeric(data)
        if size(data, 1) >= 6 && size(data, 2) >= 3
            % State matrix (rows: ... x y ...)
            x = data(4, :);
            y = data(5, :);
        elseif size(data, 1) == 2
            % [x; y]
            x = data(1, :);
            y = data(2, :);
        elseif size(data, 2) == 2
            % [x y]
            x = data(:, 1)';
            y = data(:, 2)';
        else
            return;
        end
    else
        return;
    end

    if isempty(x) || isempty(y)
        return;
    end

    % Map frame: plot X as East (y), Y as North (x).
    px = y(:)';
    py = x(:)';
end

function n = getPlanHistoryLength(planHist)
% Return number of stored plan steps for history indexing.
    n = 0;
    if isempty(planHist)
        return;
    end
    if iscell(planHist) || isstruct(planHist)
        n = numel(planHist);
    elseif isnumeric(planHist) && ndims(planHist) == 3
        n = size(planHist, 3);
    end
end
