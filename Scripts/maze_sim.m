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
goalPos_tracking   = [r1, center_c-3];

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
interp_factor = 100;                 % 5–10–20
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
%model_tracking = 'traj_track_state_error_nonlinear_ctlr';
%model_tracking = 'traj_track_output_error_feedback_ctrl';

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
%model_reg = 'cartesian_regulation_ctrl';
model_reg = 'posture_regulation_ctrl';

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

T_reg = 100;  % Regulation duration [s]

q_goal = [ ...
    0,      x_goal, y_goal;
    T_reg, x_goal, y_goal
];

assignin('base','q_goal', q_goal);

num_reg_points = 500;
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
ax.Color  = 'k';
ax.XColor = 'w';
ax.YColor = 'w';
ax.LineWidth = 1.5;

% ---- Draw maze walls ----
for r = 1:nrows
    for c = 1:ncols
        if maze(r,c)==1
            rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
                      'FaceColor','b', 'EdgeColor','b'); % walls black
        end
    end
end

% ---- Central room ----
rectangle('Position', [(c1-1)*scale, (nrows-r2)*scale, room_size*scale, room_size*scale], ...
          'EdgeColor','w', ...
          'LineWidth',2, ...
          'LineStyle','--');


xlabel('X'); ylabel('Y');
title('Maze with Desired vs Actual Trajectories (Tracking + Regulation)');
axis([0 ncols*scale 0 nrows*scale]);

% ---- PARKING BOX ----
park_width  = 1.2 * scale;   % along x
park_height = 1.2 * scale;   % along y
park_x = x_goal*scale - park_width/2;
park_y = y_goal*scale - park_height/2;
rectangle('Position', [park_x, park_y, park_width, park_height], ...
          'EdgeColor', [0 1 0], ...
          'LineWidth', 2.5, ...
          'LineStyle', '-');

plot(x_goal*scale, y_goal*scale, 'gx', 'MarkerSize', 12, 'LineWidth', 2);

% DYNAMIC LEGEND
h = [];
labels = {};

% ---- TRACKING ----
if ~isempty(q_d_tr)
    h(end+1) = plot(q_d_tr(:,1)*scale, q_d_tr(:,2)*scale, 'r--', 'LineWidth',1.8);
    labels{end+1} = 'Tracking desired';
end

h(end+1) = plot(q_tr(:,1)*scale, q_tr(:,2)*scale, 'w', 'LineWidth',2);
labels{end+1} = 'Tracking actual';

% ---- REGULATION ----

if ~isempty(q_d_reg)
    h(end+1) = plot(q_d_reg(:,1)*scale, q_d_reg(:,2)*scale, 'b--', 'LineWidth',1.8);
    labels{end+1} = 'Regulation desired';
end

h(end+1) = plot(q_reg(:,1)*scale, q_reg(:,2)*scale, 'w', 'LineWidth',2);
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

%% Video Generation
%% ================================
% VIDEO GENERATION (VARIABLE SPEED)
%% ================================




%% Video Generation - Pacman Style
%% ================================

% --- SETUP AUDIO (Opzionale) ---
% Nota: VideoWriter non salva l'audio nel file MP4. 
% Questo blocco riproduce l'audio durante la generazione per "effetto scenico".
audio_file = 'pacman_theme.mp3'; % Assicurati che il file esista
try
    [y_audio, Fs] = audioread(audio_file);
    % Riproduci l'audio (solo se vuoi sentirlo mentre genera)
    sound(y_audio, Fs); 
catch
    disp('File audio non trovato o non supportato. Procedo senza musica.');
end

% --- SETUP VIDEO ---
video_folder = fullfile(pwd,'Video');
if ~exist(video_folder,'dir')
    mkdir(video_folder);
end
video_filename = fullfile(video_folder, 'pacman_trajectory.mp4');
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30; 
v.Quality = 100;
open(v);

% --- SETUP FIGURE ---
f = figure('Color','k', 'Position', [100, 100, 800, 800]); 
scale = 2;

% Setup axes
ax = gca;
ax.Color = 'k';         
ax.XColor = 'w';        
ax.YColor = 'w';        
ax.LineWidth = 1.5;
hold on; axis equal tight;
xlabel('X [m]'); ylabel('Y [m]');
title('Pacman Project - Unicycle Trajectory', 'Color', 'w');
axis([0 ncols*scale 0 nrows*scale]);

% --- DRAW MAZE WALLS ---
% Disegna i muri in blu classico Pacman
for r = 1:nrows
    for c = 1:ncols
        if maze(r,c) == 1
            % Muri blu con bordo blu
            rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
                      'FaceColor', [0, 0, 0.8], 'EdgeColor', 'b', 'LineWidth', 2); 
        end
    end
end

% ---- PARKING BOX ----
rectangle('Position', [park_x, park_y, park_width, park_height], ...
          'EdgeColor', 'g', ...
          'LineWidth', 2, ...
          'LineStyle', '-');

% --- INITIALIZE PLOT OBJECTS ---
% Percorso (pallini bianchi stile "pillole" o linea continua)
h_path = plot(nan, nan, 'w.', 'MarkerSize', 8); % Usa '.' per sembrare pillole, o '-' per linea

% Robot (Pacman): Inizializzato vuoto, verrà aggiornato nel loop
h_robot = patch('XData', [], 'YData', [], 'FaceColor', 'y', 'EdgeColor', 'k'); 

% Occhio (opzionale, per dettaglio)
h_eye = plot(nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);

disp('Generating video...');

% --- PARAMETRI PACMAN ---
pacman_radius = 0.4 * scale; % Raggio del Pacman
pac_res = 30; % Risoluzione del cerchio

% --- 1. TRAJECTORY GENERATION ---
step_tracking = 5; % Salta frame per velocizzare
path_x_history = [];
path_y_history = [];

if exist('q_tr', 'var') && ~isempty(q_tr)
    N_tr = size(q_tr, 1);
    
    for i = 1:step_tracking:N_tr
        % Stato corrente
        x = q_tr(i, 1) * scale;
        y = q_tr(i, 2) * scale;
        th = q_tr(i, 3);
        
        % Aggiorna percorso
        current_path_x = q_tr(1:i, 1) * scale;
        current_path_y = q_tr(1:i, 2) * scale;
        set(h_path, 'XData', current_path_x, 'YData', current_path_y);
        
        % --- CALCOLO GEOMETRIA PACMAN ---
        % Animazione bocca: Oscilla tra 0.05 e 0.8 radianti
        mouth_opening = 0.05 + abs(sin(i/3)) * 0.7; 
        
        % Crea i punti del settore circolare (0 è il centro)
        angles = linspace(mouth_opening, 2*pi - mouth_opening, pac_res);
        % Aggiungi il centro (0,0) all'inizio e alla fine per chiudere la shape
        pac_x_local = [0, cos(angles) * pacman_radius, 0];
        pac_y_local = [0, sin(angles) * pacman_radius, 0];
        
        % Rotazione (Matrice R)
        R = [cos(th), -sin(th); sin(th), cos(th)];
        pac_transformed = (R * [pac_x_local; pac_y_local])';
        
        % Traslazione
        set(h_robot, 'XData', pac_transformed(:,1) + x, ...
                     'YData', pac_transformed(:,2) + y);
                 
        % Posiziona l'occhio (in alto rispetto alla bocca)
        eye_pos_local = [0.1 * pacman_radius; 0.6 * pacman_radius]; % Offset relativo
        eye_transformed = (R * eye_pos_local)';
        set(h_eye, 'XData', eye_transformed(1) + x, 'YData', eye_transformed(2) + y);
        
        writeVideo(v, getframe(f));
    end
    
    % Salva storia completa
    path_x_history = q_tr(:, 1) * scale;
    path_y_history = q_tr(:, 2) * scale;
end

% --- 2. REGULATION GENERATION ---
step_regulation = 1; % Più preciso
if exist('q_reg', 'var') && ~isempty(q_reg)
    N_reg = size(q_reg, 1);
    
    for i = 1:step_regulation:N_reg
        % Stato corrente
        x = q_reg(i, 1) * scale;
        y = q_reg(i, 2) * scale;
        th = q_reg(i, 3);
        
        % Aggiorna percorso (Storia + Attuale)
        curr_reg_x = q_reg(1:i, 1) * scale;
        curr_reg_y = q_reg(1:i, 2) * scale;
        set(h_path, 'XData', [path_x_history; curr_reg_x], ...
                    'YData', [path_y_history; curr_reg_y]);
        
        % --- CALCOLO GEOMETRIA PACMAN ---
        mouth_opening = 0.05 + abs(sin((i + N_tr)/3)) * 0.7; % Continua oscillazione
        
        angles = linspace(mouth_opening, 2*pi - mouth_opening, pac_res);
        pac_x_local = [0, cos(angles) * pacman_radius, 0];
        pac_y_local = [0, sin(angles) * pacman_radius, 0];
        
        R = [cos(th), -sin(th); sin(th), cos(th)];
        pac_transformed = (R * [pac_x_local; pac_y_local])';
        
        set(h_robot, 'XData', pac_transformed(:,1) + x, ...
                     'YData', pac_transformed(:,2) + y);
                 
        eye_pos_local = [0.1 * pacman_radius; 0.6 * pacman_radius];
        eye_transformed = (R * eye_pos_local)';
        set(h_eye, 'XData', eye_transformed(1) + x, 'YData', eye_transformed(2) + y);
        
        writeVideo(v, getframe(f));
    end
end

% Stop audio se sta ancora suonando
clear sound; 

close(v);
close(f);
disp(['Video salvato in: ', video_filename]);
disp('NOTA: Il video salvato è muto. Per aggiungere l''audio, usa un editor video');
disp('oppure ffmpeg da terminale:');
disp(['ffmpeg -i "', video_filename, '" -i pacman_theme.mp3 -c:v copy -c:a aac -shortest output_with_audio.mp4']);
















% 
% 
% % Create video directory and writer
% video_folder = fullfile(pwd,'Video');
% if ~exist(video_folder,'dir')
%     mkdir(video_folder);
% end
% 
% video_filename = fullfile(video_folder, 'pacman_trajectory.mp4');
% v = VideoWriter(video_filename, 'MPEG-4');
% v.FrameRate = 30; 
% v.Quality = 100;
% open(v);
% 
% % Setup figure
% f = figure('Color','k', 'Position', [100, 100, 800, 800]); 
% scale = 2;
% 
% % Setup axes
% ax = gca;
% ax.Color = 'k';         
% ax.XColor = 'w';        
% ax.YColor = 'w';        
% ax.LineWidth = 1.5;
% hold on; axis equal tight;
% xlabel('X [m]'); ylabel('Y [m]');
% title('Pacman Project - Unicycle Trajectory', 'Color', 'w');
% axis([0 ncols*scale 0 nrows*scale]);
% 
% % Draw Maze Walls
% for r = 1:nrows
%     for c = 1:ncols
%         if maze(r,c) == 1
%             rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
%                       'FaceColor', 'b', 'EdgeColor', 'b'); 
%         end
%     end
% end
% % ---- PARKING BOX ----
% rectangle('Position', [park_x, park_y, park_width, park_height], ...
%           'EdgeColor', 'g', ...
%           'LineWidth', 2, ...
%           'LineStyle', '--');
% 
% % Initialize plot objects
% h_path = plot(nan, nan, 'w-', 'LineWidth', 1.5); 
% h_robot = patch('XData', [], 'YData', [], 'FaceColor', 'y', 'EdgeColor', 'none'); 
% 
% % Triangle geometry
% tri_size = 0.6 * scale; 
% base_tri = [ 1.0,  0.0; 
%             -0.5,  0.6; 
%             -0.5, -0.6 ] * tri_size;
% 
% disp('Generating video...');
% 
% % First generate trajectory
% step_tracking = 5; % bigger => faster
% 
% % Initialize history containers
% path_x_history = [];
% path_y_history = [];
% 
% if exist('q_tr', 'var') && ~isempty(q_tr)
%     N_tr = size(q_tr, 1);
% 
%     for i = 1:step_tracking:N_tr
%         % Current state
%         x = q_tr(i, 1) * scale;
%         y = q_tr(i, 2) * scale;
%         th = q_tr(i, 3);
% 
%         % Update path
%         current_path_x = q_tr(1:i, 1) * scale;
%         current_path_y = q_tr(1:i, 2) * scale;
%         set(h_path, 'XData', current_path_x, 'YData', current_path_y);
% 
%         % Update robot
%         R = [cos(th), -sin(th); sin(th), cos(th)];
%         tri = (R * base_tri')';
%         set(h_robot, 'XData', tri(:,1)+x, 'YData', tri(:,2)+y);
% 
%         writeVideo(v, getframe(f));
%     end
% 
%     % Store the final full path of tracking to keep it on screen later
%     path_x_history = q_tr(:, 1) * scale;
%     path_y_history = q_tr(:, 2) * scale;
% end
% 
% % Now generate regulation
% step_regulation = 1; % don't skip frames
% 
% if exist('q_reg', 'var') && ~isempty(q_reg)
%     N_reg = size(q_reg, 1);
% 
%     for i = 1:step_regulation:N_reg
%         % Current state
%         x = q_reg(i, 1) * scale;
%         y = q_reg(i, 2) * scale;
%         th = q_reg(i, 3);
% 
%         % Update path: concatenate history + current regulation path
%         curr_reg_x = q_reg(1:i, 1) * scale;
%         curr_reg_y = q_reg(1:i, 2) * scale;
% 
%         set(h_path, 'XData', [path_x_history; curr_reg_x], ...
%                     'YData', [path_y_history; curr_reg_y]);
% 
%         % Update robot triangle
%         R = [cos(th), -sin(th); sin(th), cos(th)];
%         tri = (R * base_tri')';
%         set(h_robot, 'XData', tri(:,1)+x, 'YData', tri(:,2)+y);
% 
%         writeVideo(v, getframe(f));
%     end
% end
% 
% % Close everything
% close(v);
% close(f);
% disp(['Video saved to: ', video_filename]);