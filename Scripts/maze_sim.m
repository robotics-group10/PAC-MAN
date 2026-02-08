%% ================================
% DETERMINISTIC RANDOM MAZE WITH AUTOMATIC TRAJECTORY
% ================================
clear; close all; clc;

%% Add Simulink folder to path
simulink_folder = fullfile('..', 'Simulink');
if exist(simulink_folder, 'dir')
    addpath(simulink_folder);
else
    error('Simulink folder not found! Check the path.');
end

%% Add Functions folder to path
functions_folder = fullfile('..','Functions');
if exist(functions_folder,'dir')
    addpath(functions_folder);
else
    error('Functions folder not found!');
end

%% ================================
% MAZE PARAMETERS
%% ================================
nrows = 31;       % Number of maze rows (must be odd)
ncols = 31;       % Number of maze columns (must be odd)
room_size = 7;    % Size of the central empty room

%% Initialize RNG for deterministic maze generation
rng(1234);

%% Generate random maze
maze = generateMaze(nrows,ncols);

%% Create empty central room
center_r = ceil(nrows/2);
center_c = ceil(ncols/2);
half = floor(room_size/2);

r1 = center_r - half;
r2 = center_r + half;
c1 = center_c - half;
c2 = center_c + half;

maze(r1:r2, c1:c2) = 0;

%% ================================
% START AND GOAL DEFINITIONS
%% ================================
startPos = [1, 15];

% Tracking ends at the entrance of the central room
goalPos_tracking   = [r1, center_c];

% Regulation target is the center of the room
goalPos_regulation = [center_r, center_c];

%% ================================
% PATH PLANNING USING BFS
%% ================================
path = bfs_path(maze, startPos, goalPos_tracking);

if isempty(path)
    error('No collision-free path found from start to goal.');
end

%% ================================
% CELL COORDINATES → REAL COORDINATES
%% ================================
x_path = path(:,2) - 0.5;
y_path = nrows - path(:,1) + 0.5;
t_path = (1:length(x_path))';

%% ================================
% TRAJECTORY INTERPOLATION
%% ================================

% Numero di punti di interpolazione (smoothness)
interp_factor = 10;                 % 5–10–20
N_sim = interp_factor * length(x_path);

% Parametro (indice, minimal change)
t_sim = linspace(t_path(1), t_path(end), N_sim)';

% Interpolazione spline
x_d = interp1(t_path, x_path, t_sim, 'spline');
y_d = interp1(t_path, y_path, t_sim, 'spline');

% Forza ultimo punto
x_d(end) = x_path(end);
y_d(end) = y_path(end);

% Orientazione smooth
dt = t_sim(2) - t_sim(1);
dx = gradient(x_d, dt);
dy = gradient(y_d, dt);

theta_d = atan2(dy, dx);
theta_d = unwrap(theta_d);

%% ================================
% ROBOT STOP AT END OF TRAJECTORY
%% ================================
num_stop = 2;   % Number of final points with zero motion
num_stop = min(num_stop, length(x_d));

x_d(end-num_stop+1:end) = x_d(end);
y_d(end-num_stop+1:end) = y_d(end);
theta_d(end-num_stop+1:end) = theta_d(end);

q_d_new = [t_sim, x_d(:), y_d(:), theta_d(:)];
q0 = [x_d(1); y_d(1); theta_d(1)];

assignin('base','q_d_new', q_d_new);
assignin('base','q0', q0);

fprintf('Trajectory ready for Simulink: %d points\n', length(t_sim));

disp('--- TRACKING SETUP ---')
disp(['q0 (tracking) = [', num2str(q0.'), ']'])
disp(['Tracking end  = [', num2str([x_d(end), y_d(end), theta_d(end)]), ']'])

%% ================================
% TRACKING CONTROLLER SIMULATION
%% ================================
model_tracking = 'traj_track_state_error_linearization_ctrl';

if exist([model_tracking,'.slx'], 'file')
    load_system(model_tracking);
    set_param(model_tracking,'SimulationCommand','update');
    simOut = sim(model_tracking,'ReturnWorkspaceOutputs','on');
    disp('Tracking simulation completed successfully.');
else
    warning('Model %s not found.', model_tracking);
end

%% ================================
% REGULATION (POST-TRAJECTORY)
%% ================================
model_reg = 'posture_regulation';

if exist([model_reg,'.slx'],'file')
    load_system(model_reg);
else
    warning('Model %s not found.', model_reg);
end

q0_reg = [x_d(end); y_d(end); theta_d(end)];
assignin('base','q0_reg', q0_reg);

% Regulation goal in real-world coordinates
x_goal = goalPos_regulation(2) - 0.5;
y_goal = nrows - goalPos_regulation(1) + 0.5;
theta_goal = 0;   % Orientation (ignored if regulator is position-only)

T_reg = 5;  % Regulation duration [s]

q_goal = [ ...
    0,      x_goal, y_goal;
    T_reg, x_goal, y_goal
];

assignin('base','q_goal', q_goal);

num_reg_points = 50;
t_reg = linspace(0, T_reg, num_reg_points)';

disp('--- REGULATION SETUP ---')
disp(['q0_reg    = [', num2str(q0_reg.'), ']'])
disp(['q_goal xy = [', num2str(x_goal), ', ', num2str(y_goal), ']'])

simOut_reg = sim(model_reg,'ReturnWorkspaceOutputs','on');
disp('Regulation simulation completed successfully.');

%% ================================
% SAVE FIGURES
%% ================================
figures_folder = fullfile(pwd,'Figures');
if ~exist(figures_folder,'dir')
    mkdir(figures_folder);
end
x_goal_tracking = goalPos_tracking(2) - 0.5;
y_goal_tracking = nrows - goalPos_tracking(1) + 0.5;
goal_tracking = [x_goal_tracking,y_goal_tracking];
goal_regulation = [x_goal, y_goal];
plot_and_save(simOut, 'trajectory_unicycle', figures_folder, goal_tracking);
plot_and_save(simOut_reg, 'regulation_unicycle', figures_folder, goal_regulation);

%% ================================
% EXTRACT TRAJECTORIES FOR PLOTTING
%% ================================

% ---- TRACKING ----
q_tr    = simOut.logsout.getElement('q').Values.Data;   % actual
if ismember('q_d', simOut.logsout.getElementNames)
    q_d_tr = simOut.logsout.getElement('q_d').Values.Data; % desired
else
    q_d_tr = [];
end

% ---- REGULATION ----
q_reg    = simOut_reg.logsout.getElement('q').Values.Data;   % actual
if ismember('q_d', simOut_reg.logsout.getElementNames)
    q_d_reg = simOut_reg.logsout.getElement('q_d').Values.Data; % desired
else
    q_d_reg = [];
end

%% ================================
% SINGLE FINAL PLOT: MAZE + TRAJECTORIES
%% ================================
figure('Color','w'); % figure background white
hold on; axis equal tight;
scale = 2;

ax = gca;             % get current axes
ax.Color = 'w';       % set axes (background) to white

% ---- Draw maze walls ----
for r = 1:nrows
    for c = 1:ncols
        if maze(r,c)==1
            rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
                      'FaceColor','k', 'EdgeColor','none'); % walls black
        end
    end
end

% ---- Central room ----
rectangle('Position', [(c1-1)*scale, (nrows-r2)*scale, room_size*scale, room_size*scale], ...
          'EdgeColor','k','LineWidth',2,'LineStyle','--'); % dashed room outline

xlabel('X'); ylabel('Y');
title('Maze with Desired vs Actual Trajectories (Tracking + Regulation)');
axis([0 ncols*scale 0 nrows*scale]);

% DYNAMIC LEGEND
h = [];
labels = {};

% ---- TRACKING ----
if ~isempty(q_d_tr)
    h(end+1) = plot(q_d_tr(:,1)*scale, q_d_tr(:,2)*scale, 'r--', 'LineWidth',1.8);
    labels{end+1} = 'Tracking desired';
end

h(end+1) = plot(q_tr(:,1)*scale, q_tr(:,2)*scale, 'r', 'LineWidth',2.5);
labels{end+1} = 'Tracking actual';

% ---- REGULATION ----
if ~isempty(q_d_reg)
    h(end+1) = plot(q_d_reg(:,1)*scale, q_d_reg(:,2)*scale, 'b--', 'LineWidth',1.8);
    labels{end+1} = 'Regulation desired';
end

h(end+1) = plot(q_reg(:,1)*scale, q_reg(:,2)*scale, 'b', 'LineWidth',2.5);
labels{end+1} = 'Regulation actual';

% ---- START & GOAL ----
h(end+1) = plot(x_d(1)*scale, y_d(1)*scale, 'go', ...
    'MarkerFaceColor','g', 'MarkerSize',8);
labels{end+1} = 'Start';

h(end+1) = plot(x_goal*scale, y_goal*scale, 'mo', ...
    'MarkerFaceColor','m', 'MarkerSize',8);
labels{end+1} = 'Regulation goal';

legend(h, labels, 'Location','best');

grid off;