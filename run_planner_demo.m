%% run_planner_demo.m
% MATLAB <-> Python integration demo for:
%   "Adaptive Path Planning and Collision Avoidance for Autonomous
%    Vehicles on Unstructured Indian Roads"
%
% This script does NOT implement a planner. It calls the EXISTING A*
% PathPlanner inside main2_collision_free.py through matlab_bridge.py,
% feeds it EGO / GOAL / OBSTACLE states, receives the planned path, drives
% the EGO with the bridge's bounded-motion step (same dynamics as the
% Pygame simulation), and computes validation metrics in MATLAB.
%
% The Pygame application (main2_collision_free.py) remains the primary
% demonstration; this script is the MATLAB validation layer.

clear; clc; close all;

%% 1. Configure the Python environment -----------------------------------
pe = pyenv;
fprintf('Python environment: %s (%s)\n', pe.Version, pe.Executable);
% If pyenv has not been configured on this machine, do this ONCE (edit the
% path to your python.exe):
%   pyenv('Version', ...
%     'C:\Users\itzke\AppData\Local\Programs\Python\Python312\python.exe');

% Add this folder to the Python module search path.
if count(py.sys.path, pwd) == 0
    insert(py.sys.path, int32(0), pwd);
end

% Import (or reload) the bridge module that wraps the EXISTING PathPlanner.
mb = py.importlib.import_module('matlab_bridge');
py.importlib.reload(mb);
fprintf('matlab_bridge imported. Obstacle type codes:\n');
disp(mb.obstacle_type_codes());

%% 2. Scenario definitions (Indian-road validation set) ------------------
% Obstacle spec: struct('type',..,'x',..,'y',..,'v',..,'h',..)
% (x,y in simulation pixels = 10 px/m; h in radians, 0 = +x).
S = {};

% S1 - Highway cruise: two slower cars ahead -> collision-free overtake path.
S{end+1} = struct('name', 'Highway Cruise', ...
    'ego0', [300, 400, 0, 0], 'goal', [4700, 400], ...
    'obs', [ob('car', 900, 400, 55, 0), ob('truck', 1600, 430, 45, 0)], ...
    'T', 14);

% S2 - Stopped two-wheeler on the lane -> dynamic A* lateral detour.
S{end+1} = struct('name', 'Two-Wheeler Blockage', ...
    'ego0', [300, 400, 0, 0], 'goal', [2500, 400], ...
    'obs', ob('two_wheeler', 700, 395, 0, 0), 'T', 14);

% S3 - Pedestrian crossing ahead -> EGO yields (full stop), waits while the
% pedestrian crosses in front, then resumes. Collision-free with margin.
S{end+1} = struct('name', 'Pedestrian Crossing', ...
    'ego0', [300, 400, 0, 0], 'goal', [2500, 400], ...
    'obs', ob('pedestrian', 800, 330, 10, pi/2), 'T', 24);

% S4 - Rear-end threat: wrong-way car closing on the EGO from behind.
S{end+1} = struct('name', 'Rear-End Evasion', ...
    'ego0', [300, 400, 30, 0], 'goal', [2500, 400], ...
    'obs', ob('car', 120, 400, 120, pi), 'T', 8);

% S5 - Dense mixed traffic incl. static pothole -> adaptive replanning.
S{end+1} = struct('name', 'Dense Mixed Traffic', ...
    'ego0', [300, 400, 0, 0], 'goal', [2500, 400], ...
    'obs', [ob('auto_rickshaw', 650, 420, 35, 0), ...
            ob('car', 1050, 380, 50, 0), ...
            ob('bicycle', 1450, 410, 18, 0), ...
            ob('pothole', 1800, 430, 0, 0), ...
            ob('two_wheeler', 2100, 370, 55, 0)], ...
    'T', 16);

PX_PER_M = 10.0;                     % must match matlab_bridge.PX_PER_M
dt = 1/60;                           % same base step as the Pygame sim
plan_every = 0.25;                   % event window: replan 4x per second
step_plan = round(plan_every/dt);

%% 3. Closed-loop validation over the 5 scenarios -------------------------
metrics = table();
traj_store = cell(size(S));
path_store = cell(size(S));
obs_store  = cell(size(S));

for s = 1:numel(S)
    sc = S{s};
    fprintf('\n=== Scenario %d: %s ===\n', s, sc.name);

    mb.initialize_planner();                    % fresh EXISTING PathPlanner
    mb.set_goal(sc.goal(1), sc.goal(2));
    mb.reset_metrics();

    ego = sc.ego0(:).';                         % [x y v heading]
    nsteps = round(sc.T/dt);
    traj = zeros(nsteps+1, 2); traj(1,:) = ego(1:2);
    static_xy = [];
    active_xy = [];

    for k = 0:nsteps-1
        t = k*dt;
        % Analytic obstacle ground truth (their trajectories are never
        % touched by the planner or the bridge).
        cur = move_obs(sc.obs, t);
        mb.update_obstacles(obs2py(cur));

        % Periodic + event-based replanning through the EXISTING A*.
        if mod(k, step_plan) == 0
            r = mb.plan_path(ego(1), ego(2), ego(4));
            if isempty(static_xy)
                fp = mb.get_path();
                static_xy = [py2vec(fp{'static_path_x'}), ...
                             py2vec(fp{'static_path_y'})];
            end
            active_xy = [py2vec(r{'path_x'}), py2vec(r{'path_y'})];
        end

        % EGO motion: bounded accel / steering, continuous collision gate.
        e = mb.ego_step(ego(1), ego(2), ego(3), ego(4), dt);
        ego = [double(e{'x'}), double(e{'y'}), ...
               double(e{'v'}), double(e{'heading'})];
        traj(k+2, :) = ego(1:2);
    end

    m = mb.get_metrics();
    row = table({sc.name}, double(m{'collision_count'}), ...
        double(m{'minimum_clearance_m'}), double(m{'number_of_replans'}), ...
        double(m{'plan_calls'}), double(m{'last_planning_time'})*1000, ...
        double(m{'simulation_time'}), ...
        'VariableNames', {'scenario','collisions','min_clearance_m', ...
                          'replans','plan_calls','last_plan_ms','sim_time_s'});
    metrics = [metrics; row];                    %#ok<AGROW>
    traj_store{s} = traj;
    path_store{s} = active_xy;
    obs_store{s}  = sc.obs;
    disp(row);
end

fprintf('\n=================== SUMMARY ===================\n');
disp(metrics);
fprintf('Total collisions across all scenarios: %d\n', ...
    sum(metrics.collisions));

%% 4. MATLAB path plots (validation only - Pygame remains the live demo) --
fig1 = figure('Name', 'Static A* route + EGO trajectory', ...
              'Position', [40 60 900 700]);
tl = tiledlayout(fig1, 'flow', 'Padding', 'compact');
for s = 1:numel(S)
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    draw_road(ax, 100, 400, 4900, 400, 150);   % the highway used by all demos
    traj = traj_store{s}/PX_PER_M;
    plot(ax, traj(:,1), traj(:,2), 'c-', 'LineWidth', 1.8);
    plot(ax, traj(1,1), traj(1,2), 'go', 'MarkerFaceColor', 'g');
    plot(ax, S{s}.goal(1)/PX_PER_M, S{s}.goal(2)/PX_PER_M, 'rp', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'r');
    obst = obs_store{s};
    for i = 1:numel(obst)
        plot(ax, obst(i).x/PX_PER_M, obst(i).y/PX_PER_M, 'ks', ...
            'MarkerFaceColor', 'y');
        text(ax, obst(i).x/PX_PER_M, obst(i).y/PX_PER_M, ...
            [' ' obst(i).type], 'FontSize', 7);
    end
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    title(ax, sprintf('%d. %s  (coll: %d, min clear: %.1f m)', s, ...
        metrics.scenario{s}, metrics.collisions(s), ...
        metrics.min_clearance_m(s)));
    axis(ax, 'equal');
end
try, exportgraphics(fig1, 'planner_demo_scenarios.png', 'Resolution', 150);
catch, saveas(fig1, 'planner_demo_scenarios.png'); end

% Detailed view of one dynamic replan (active local A* path vs static route).
if ~isempty(path_store{2})
    fig2 = figure('Name', 'Dynamic replanning detail', ...
                  'Position', [980 60 760 560]);
    ax = axes(fig2); hold(ax, 'on'); grid(ax, 'on');
    draw_road(ax, 100, 400, 4900, 400, 150);
    mb.initialize_planner(); mb.set_goal(S{2}.goal(1), S{2}.goal(2));
    mb.update_obstacles(obs2py(move_obs(S{2}.obs, 0)));
    r = mb.plan_path(300, 400, 0);
    fp = mb.get_path();
    st = [py2vec(fp{'static_path_x'}), py2vec(fp{'static_path_y'})]/PX_PER_M;
    pa = [py2vec(r{'path_x'}), py2vec(r{'path_y'})]/PX_PER_M;
    plot(ax, st(:,1), st(:,2), '--', 'Color', [.6 .6 .6], 'LineWidth', 1.2);
    plot(ax, pa(:,1), pa(:,2), 'g-', 'LineWidth', 2.2);
    obst2 = S{2}.obs;
    plot(ax, obst2.x/PX_PER_M, obst2.y/PX_PER_M, 'ks', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot(ax, 30, 40, 'bo', 'MarkerFaceColor', 'b');
    legend(ax, {'road', 'static A* route', 'dynamic A* detour', ...
        'stopped two-wheeler', 'EGO start'}, 'Location', 'southeast');
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    title(ax, 'Dynamic A* replanning around a two-wheeler');
    axis(ax, 'equal'); xlim(ax, [20 120]); ylim(ax, [32 50]);
    try, exportgraphics(fig2, 'planner_demo_replan.png', 'Resolution', 150);
    catch, saveas(fig2, 'planner_demo_replan.png'); end
end
fprintf('\nFigures saved: planner_demo_scenarios.png, planner_demo_replan.png\n');

%% ---- local helpers ------------------------------------------------------
function o = ob(type, x, y, v, h)
%OB Construct one obstacle spec struct.
o = struct('type', type, 'x', x, 'y', y, 'v', v, 'h', h);
end

function cur = move_obs(obs, t)
%MOVE_OBS Ground-truth obstacle motion (constant velocity, straight line).
cur = obs;
for i = 1:numel(obs)
    cur(i).x = obs(i).x + obs(i).v*cos(obs(i).h)*t;
    cur(i).y = obs(i).y + obs(i).v*sin(obs(i).h)*t;
end
end

function pl = obs2py(cur)
%OBS2PY Convert obstacle struct array -> py.list of py.dict for the bridge.
c = cell(1, numel(cur));
for i = 1:numel(cur)
    c{i} = py.dict(pyargs('type', cur(i).type, 'x', cur(i).x, ...
        'y', cur(i).y, 'velocity', cur(i).v, 'heading', cur(i).h));
end
pl = py.list(c);
end

function v = py2vec(pl)
%PY2VEC Convert a py.list of numbers into a MATLAB double column vector.
v = cellfun(@double, cell(pl)).';
end

function draw_road(ax, x1, y1, x2, y2, w)
%DRAW_ROAD Draw a road segment (scaled to metres) for validation plots.
s = 0.1;                                   % px -> m
hl = plot(ax, [x1 x2]*s, [y1 y2]*s, '-', 'Color', [.72 .72 .75], ...
    'LineWidth', w*s*1.6); %#ok<NASGU>
uistack(findobj(ax, 'Type', 'line'), 'bottom');
end
