%% run_simulink_demo.m - simulate adaptive_path_planning.slx end-to-end
% Drives the Simulink model (which calls the EXISTING Python A* planner
% through matlab_bridge.py) and validates the STEP 7 / STEP 12 metrics.

clear; clc; close all;

cgdir = fullfile(getenv('TEMP'), 'sl_codegen');
if ~isfolder(cgdir), mkdir(cgdir); end
Simulink.fileGenControl('set', 'CacheFolder', cgdir, ...
    'CodeGenFolder', cgdir, 'createDir', true);

if ~bdIsLoaded('adaptive_path_planning')
    load_system('adaptive_path_planning.slx');
end

fprintf('Simulating adaptive_path_planning (calls Python A* via Simulink)...\n');
tic;
out = sim('adaptive_path_planning');
fprintf('Simulation done in %.1f s (wall).\n', toc);

%% Extract logged signals --------------------------------------------------
ego   = out.ego_log;                    % rows: [x y v heading]
pl    = out.planner_log;                % [num_wp path_m state replan plan_ms clear_now]
t     = out.tout;
PX_PER_M = 10;

% Scenario obstacle trajectories (same as the Scenario Environment State
% block - analytic ground truth, never modified by the planner).
ob1 = @(tt) [700,            340];                 % stopped two-wheeler
ob2 = @(tt) [1500 + 35*tt,   420];                 % moving car
ob3 = @(tt) [1800,           330 + 10*tt];         % pedestrian crossing
obs_r = [11 19.8 3.85];                            % stub radii (px)

%% Validation --------------------------------------------------------------
dt = 0.02;
steps   = diff(ego(:,1:2), 1, 1);
jump    = max(hypot(steps(:,1), steps(:,2)));
forward = min(diff(ego(:,1))) >= -0.5;             % no reverse jumps

% Independent collision check against the analytic obstacle trajectories.
min_clear = inf; hits = 0;
for k = 2:size(ego,1)
    tt = t(k); ex = ego(k,1); ey = ego(k,2);
    p1 = ob1(tt); p2 = ob2(tt); p3 = ob3(tt);
    d1 = hypot(ex - p1(1), ey - p1(2)) - 23 - obs_r(1);
    d2 = hypot(ex - p2(1), ey - p2(2)) - 23 - obs_r(2);
    d3 = hypot(ex - p3(1), ey - p3(2)) - 23 - obs_r(3);
    dc = min([d1 d2 d3]);
    min_clear = min(min_clear, dc);
    if dc < 0, hits = hits + 1; end
end
replans      = sum(diff(pl(:,4)) > 0.5);
plan_ms_mean = mean(pl(pl(:,5) > 0, 5));
path_len_m   = pl(end,2);

fprintf('\n================ SIMULINK VALIDATION ================\n');
fprintf('  collision_count       : %d\n', hits);
fprintf('  minimum_clearance     : %.2f px (%.2f m)\n', min_clear, min_clear/PX_PER_M);
fprintf('  number_of_replans     : %d\n', replans);
fprintf('  path_length           : %.1f m\n', path_len_m);
fprintf('  mean planning_time    : %.2f ms\n', plan_ms_mean);
fprintf('  simulation_time       : %.1f s\n', t(end));
fprintf('  EGO final x           : %.0f px (start 300, goal 2500)\n', ego(end,1));
fprintf('  max step displacement : %.2f px (phys. limit %.2f px @125px/s)\n', ...
    jump, 125*dt);
fprintf('  no reverse jumps      : %d\n', forward);
ok = (hits == 0) && (jump <= 125*dt*1.5) && forward && (ego(end,1) > 1000);
fprintf('  OVERALL               : %s\n', tern(ok, 'PASS', 'CHECK ABOVE'));

%% Plot --------------------------------------------------------------------
fig = figure('Name', 'Simulink validation', 'Position', [50 50 1000 640]);
subplot(2,1,1); hold on; grid on;
plot([10 490], [400 400]/PX_PER_M, '-', 'Color', [.75 .75 .78], ...
    'LineWidth', 15);
plot(ego(:,1)/PX_PER_M, ego(:,2)/PX_PER_M, 'c-', 'LineWidth', 1.8);
plot(700/PX_PER_M, 340/PX_PER_M, 'ks', 'MarkerFaceColor', 'y');
plot(1500/PX_PER_M, 420/PX_PER_M, 'ks', 'MarkerFaceColor', 'y');
plot((1500 + 35*t(end))/PX_PER_M, 420/PX_PER_M, 'ks');
plot(1800/PX_PER_M, 330/PX_PER_M, 'ks', 'MarkerFaceColor', 'y');
legend({'highway (150 px)', 'EGO trajectory (Simulink)', ...
    'two-wheeler (stopped)', 'car start', 'car end', ...
    'pedestrian start'}, 'Location', 'northwest');
xlabel('x [m]'); ylabel('y [m]'); axis equal;
title(sprintf('adaptive\\_path\\_planning.slx - collisions=%d, min clear=%.1f m, replans=%d', ...
    hits, min_clear/PX_PER_M, replans));

subplot(2,1,2); grid on; hold on;
yyaxis left;
plot(t, ego(:,3), 'LineWidth', 1.2); ylabel('EGO speed [px/s]');
yyaxis right;
plot(t, pl(:,6)/PX_PER_M, 'LineWidth', 1.2); ylabel('clearance now [m]');
xlabel('t [s]'); title('EGO speed and instantaneous clearance');
try, exportgraphics(fig, 'simulink_validation.png', 'Resolution', 150);
catch, saveas(fig, 'simulink_validation.png'); end
fprintf('Figure saved: simulink_validation.png\n');

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
