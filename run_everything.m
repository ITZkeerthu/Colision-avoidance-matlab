function run_everything()
%RUN_EVERYTHING  One-click integration demo run from run_everything.bat.
%
%   1. Configures pyenv (Python 3.12, InProcess) + a LOCAL codegen folder
%      (mapped drives break the Simulink model build).
%   2. Smoke-tests the Python bridge (imports the EXISTING A* planner).
%   3. Runs the MATLAB 5-scenario closed-loop demo  (run_planner_demo).
%   4. Runs the Simulink end-to-end validation      (run_simulink_demo).
%   5. Opens adaptive_path_planning.slx in the editor (Simulink tab) and
%      starts a live 20 s run so the Display blocks / Scope update.
%
%   The Pygame app itself is launched by the batch file; this script only
%   handles the MATLAB / Simulink side.

fprintf('==================================================\n');
fprintf(' ADAPTIVE PATH PLANNING - FULL INTEGRATION DEMO\n');
fprintf('==================================================\n');

projectdir = fileparts(mfilename('fullpath'));
cd(projectdir);

%% 1) Python interpreter ----------------------------------------------------
pe = pyenv;
fprintf('pyenv : %s | %s\n', pe.Version, pe.Executable);
if pe.Status == "NotLoaded"
    pyenv('Version', ...
        'C:\Users\itzke\AppData\Local\Programs\Python\Python312\python.exe');
end

%% 2) Local codegen folder (mapped/network drives break the cgxe build) -----
cgdir = fullfile(getenv('TEMP'), 'sl_codegen');
if ~isfolder(cgdir), mkdir(cgdir); end
Simulink.fileGenControl('set', 'CacheFolder', cgdir, ...
    'CodeGenFolder', cgdir, 'createDir', true);

%% 3) Bridge smoke test ------------------------------------------------------
if count(py.sys.path, pwd) == 0, insert(py.sys.path, int32(0), pwd); end
try
    mb = py.importlib.import_module('matlab_bridge');
    mb.initialize_planner();
    fprintf('Python bridge OK - existing A* planner imported.\n\n');
catch ME
    warning(['Python bridge failed: %s\nFix pyenv (see README section 4), ' ...
        'then re-run.'], ME.message);
    return;
end

%% 4) MATLAB 5-scenario demo (runs in BASE workspace: it starts with CLEAR) -
ok1 = safe_eval('run_planner_demo', 'MATLAB 5-scenario demo');

%% 5) Simulink end-to-end validation -----------------------------------------
ok2 = safe_eval('run_simulink_demo', 'Simulink validation');

%% 6) Open the Simulink model tab and start a live run -----------------------
model = 'adaptive_path_planning';
try
    if ~bdIsLoaded(model), load_system([model '.slx']); end
    open_system(model);                          % Simulink editor tab
    set_param(model, 'SimulationCommand', 'update');   % compile check first
    fprintf(['\nSimulink model open. Starting a live 20 s run so the ' ...
        'Display blocks and Scope update...\n']);
    set_param(model, 'SimulationCommand', 'start');
catch ME
    warning('Could not open/start the Simulink model: %s', ME.message);
end

%% 7) Summary ----------------------------------------------------------------
fprintf('\n======================================================\n');
fprintf(' MATLAB demo          : %s\n', tern(ok1, 'OK',  'SEE WARNINGS ABOVE'));
fprintf(' Simulink validation  : %s\n', tern(ok2, 'PASS', 'SEE WARNINGS ABOVE'));
fprintf(' Simulink model       : open in editor, run started\n');
fprintf(' Pygame app           : launched by run_everything.bat\n');
fprintf('======================================================\n');
end

% =========================================================================
function ok = safe_eval(scriptname, label)
ok = true;
try
    fprintf('\n--- %s (%s) ---\n', label, scriptname);
    evalin('base', scriptname);   % base ws: script's own CLEAR cannot hurt us
catch ME
    ok = false;
    warning('%s failed: %s', scriptname, ME.message);
end
end

function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
