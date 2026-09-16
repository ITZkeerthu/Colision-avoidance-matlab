%% diag_simulink.m - dump diagnostics from a Simulink run
cd('U:\SIH-16-9-26-2-');
cg = fullfile(getenv('TEMP'), 'sl_codegen');
if ~isfolder(cg), mkdir(cg); end
Simulink.fileGenControl('set', 'CacheFolder', cg, 'CodeGenFolder', cg, 'createDir', true);
load_system('adaptive_path_planning.slx');
out = sim('adaptive_path_planning');
ego = out.ego_log; pl = out.planner_log; t = out.tout;
d = sqrt(sum(diff(ego(:,1:2)).^2, 2));
[jmax, jidx] = max(d);
fprintf('max jump %.2f at row %d (t=%.3f): (%.1f,%.1f) -> (%.1f,%.1f)\n', ...
    jmax, jidx, t(min(jidx+1, numel(t))), ego(jidx,1), ego(jidx,2), ...
    ego(jidx+1,1), ego(jidx+1,2));
fprintf('speed min/max: %.1f / %.1f\n', min(ego(:,3)), max(ego(:,3)));
fprintf('state codes present: %s\n', mat2str(unique(pl(:,3)).'));
fprintf('num_wp min/max: %.0f / %.0f\n', min(pl(:,1)), max(pl(:,1)));
fprintf('rows: ego %d, pl %d, t %d\n', size(ego,1), size(pl,1), numel(t));
fprintf('\n   t      ego_x    ego_y      v     state  num_wp  clear_now\n');
for k = 1:100:size(ego,1)
    kp = min(k, size(pl,1));
    fprintf('%6.1f %8.0f %7.0f %7.1f %7.0f %7.0f %9.1f\n', ...
        t(k), ego(k,1), ego(k,2), ego(k,3), pl(kp,3), pl(kp,1), pl(kp,6));
end
