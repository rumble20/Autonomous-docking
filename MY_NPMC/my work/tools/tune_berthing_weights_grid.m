function best = tune_berthing_weights_grid()
% tune_berthing_weights_grid
% Lightweight parameter sweep helper for approach feasibility/heading quality tuning.

base = struct();
base.base_margin_m = 75;
base.speed_gain_s = 2.2;
base.obs_radius_gain = 0.45;
base.deflect_sigma = 0.19;
base.r_ref_max = 0.14;

margin_candidates = [60, 75, 90];
heading_blend_candidates = [120, 180, 240];
approach_offset_candidates = [180, 240, 300];

best = struct('score', inf, 'base_margin_m', NaN, 'heading_blend_dist_m', NaN, 'approach_offset_m', NaN);

idx = 0;
for m = 1:numel(margin_candidates)
    for h = 1:numel(heading_blend_candidates)
        for a = 1:numel(approach_offset_candidates)
            idx = idx + 1;
            cfg = base;
            cfg.base_margin_m = margin_candidates(m);
            heading_blend = heading_blend_candidates(h);
            approach_offset = approach_offset_candidates(a);

            % Proxy objective until full harbor scenario automation is plugged in.
            score = 0.8*cfg.base_margin_m + 0.15*heading_blend + 0.05*approach_offset;

            fprintf('Candidate %02d: margin=%g blend=%g offset=%g -> proxy score=%.3f\n', ...
                idx, cfg.base_margin_m, heading_blend, approach_offset, score);

            if score < best.score
                best.score = score;
                best.base_margin_m = cfg.base_margin_m;
                best.heading_blend_dist_m = heading_blend;
                best.approach_offset_m = approach_offset;
            end
        end
    end
end

fprintf('Best candidate (proxy): margin=%g blend=%g offset=%g score=%.3f\n', ...
    best.base_margin_m, best.heading_blend_dist_m, best.approach_offset_m, best.score);
end
