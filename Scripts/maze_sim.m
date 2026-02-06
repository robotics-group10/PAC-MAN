%% ================================
% LABIRINTO -> TRAIETTORIA SIMULINK (Option A)
% ================================
clear; close all; clc;

%% PATH SETUP
simulink_folder = fullfile('..', 'Simulink');
if exist(simulink_folder, 'dir')
    addpath(simulink_folder);
else
    error('Simulink folder not found! Check the path.');
end
% Functions folder
functions_folder = fullfile('..','Functions');

if exist(functions_folder,'dir')
    addpath(functions_folder);
else
    error('Functions folder not found!');
end
%% ================================
% PARAMETRI LABIRINTO
% ================================
nrows = 31;       % numero di righe (dispari)
ncols = 31;       % numero di colonne (dispari)
room_size = 7;    % dimensione stanza centrale (quadrata)

%% ================================
% GENERA LABIRINTO CASUALE
% ================================
maze = generateMaze(nrows,ncols);

%% ================================
% CREA STANZA CENTRALE VUOTA
% ================================
center_r = ceil(nrows/2);
center_c = ceil(ncols/2);
half = floor(room_size/2);
r1 = center_r - half; r2 = center_r + half;
c1 = center_c - half; c2 = center_c + half;
maze(r1:r2, c1:c2) = 0;

%% ================================
% PUNTO INIZIALE E GOAL SUL PERIMETRO DELLA STANZA CENTRALE
% ================================
startPos  = pickFreePerimeterCell(maze);

%% ================================
% TROVA LA CELLA GOAL PIU' VICINA AL PERCORSO
% ================================
% Crea lista celle libere sul perimetro della stanza centrale
perimeter_cells = [];
for c = c1:c2
    if maze(r1,c) == 0, perimeter_cells(end+1,:) = [r1,c]; end
    if maze(r2,c) == 0, perimeter_cells(end+1,:) = [r2,c]; end
end
for r = r1+1:r2-1
    if maze(r,c1) == 0, perimeter_cells(end+1,:) = [r,c1]; end
    if maze(r,c2) == 0, perimeter_cells(end+1,:) = [r,c2]; end
end
if isempty(perimeter_cells)
    error('Nessuna cella libera sul perimetro della stanza centrale!');
end

% Seleziona una cella del perimetro come target iniziale
targetPos = perimeter_cells(randi(size(perimeter_cells,1)), :);


%% ================================
% CALCOLO PATH A* (CELLE)
% ================================
path = astar_path(maze, startPos, targetPos);
if isempty(path)
    error('Percorso non trovato!');
end

% TROVA LA CELLA PIU' VICINA AL PERCORSO
distances = zeros(size(perimeter_cells,1),1);
for k = 1:size(perimeter_cells,1)
    distances(k) = min(sum((path - perimeter_cells(k,:)).^2, 2));
end
[~, idx_min] = min(distances);
targetPos = perimeter_cells(idx_min,:);  

% RICALCOLA IL PATH VERSO IL NUOVO TARGET
path = astar_path(maze, startPos, targetPos);
if isempty(path), error('Percorso non trovato!'); end

%% ================================
% CONVERSIONE CELLE -> COORDINATE REALI
% ================================
x_path = path(:,2) - 0.5;          % colonna -> x
y_path = nrows - path(:,1) + 0.5;  % riga -> y (origine in basso a sinistra)

t_path = (1:length(x_path))';      % tempi originali per ogni punto del path
t_sim  = linspace(1, length(x_path), 5*length(x_path))'; % tempi interpolati
dt = t_sim(2) - t_sim(1);

%% ================================
% INTERPOLAZIONE TRAIETTORIA FINO AL GOAL
% ================================

% Coordinate del goal
goal_x = targetPos(2) - 0.5;
goal_y = nrows - targetPos(1) + 0.5;

% Se il path finale coincide già con il target, non aggiungere nuovamente
if ~isequal(path(end,:), targetPos)
    x_path_aug = [x_path; goal_x];
    y_path_aug = [y_path; goal_y];
    t_path_aug = [t_path; t_path(end)+1];  % passo extra per il goal
else
    x_path_aug = x_path;
    y_path_aug = y_path;
    t_path_aug = t_path;
end


% Interpolazione spline (cubic) sui tempi simulazione
x_spline = spline(t_path_aug, x_path_aug);
y_spline = spline(t_path_aug, y_path_aug);

% Nuovo t_sim che copre esattamente tutto il path fino al goal
t_sim = linspace(t_path_aug(1), t_path_aug(end), 5*length(x_path_aug))';

% Valuta spline sui tempi simulazione
x_d = ppval(x_spline, t_sim);
y_d = ppval(y_spline, t_sim);

% Derivata per orientamento
dx = gradient(x_d, t_sim(2)-t_sim(1));
dy = gradient(y_d, t_sim(2)-t_sim(1));
theta_d = atan2(dy, dx);

% Forza array colonna
x_d     = x_d(:);
y_d     = y_d(:);
theta_d = theta_d(:);
t_sim   = t_sim(:);

% FORZA L'ULTIMO PUNTO ESATTAMENTE SUL GOAL
x_d(end) = goal_x;
y_d(end) = goal_y;
theta_d(end) = theta_d(end-1);  % mantiene orientamento finale

% CREA q_d_new PER SIMULINK
q_d_new = [t_sim, x_d, y_d, theta_d];
q0 = [x_d(1); y_d(1); theta_d(1)];

assignin('base','q_d_new', q_d_new);
assignin('base','q0', q0);

fprintf('Traiettoria pronta per Simulink: %d punti (ultimo punto sul goal)\n', length(t_sim));
assignin('base','q0', q0);

fprintf('Traiettoria pronta per Simulink: %d punti\n', length(t_sim));


%% ================================
% ESEMPIO DI SIMULAZIONE
% ================================
model_tracking = 'trajectory_tracking_linearized_crl';

% Load model without opening GUI
if exist([model_tracking,'.slx'], 'file')
    load_system(model_tracking);
else
    warning('Modello %s non trovato. Controlla il percorso.', model_tracking);
end

% Update and run simulation
if exist(model_tracking,'file')
    set_param(model_tracking,'SimulationCommand','update');
    simOut = sim(model_tracking,'ReturnWorkspaceOutputs','on');
    disp('Simulazione completata con successo.');
end
figures_folder = fullfile(pwd,'figures'); % cartella per salvare i plot
if ~exist(figures_folder,'dir')
    mkdir(figures_folder);
end

plot_and_save(simOut, 'trajectory_unicycle', figures_folder, targetPos);

%% ================================
% VISUALIZZAZIONE LABIRINTO E TRAIETTORIA - CELLE GRANDI
% ================================
figure('Color','w'); hold on; axis equal tight;

scale = 2;  % lato della cella (2x più grandi)

% Imposta sfondo bianco degli assi
ax = gca;
ax.Color = [1 1 1];   % bianco

% Disegna i muri
for r = 1:nrows
    for c = 1:ncols
        if maze(r,c) == 1
            rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
                      'FaceColor','k', 'EdgeColor','none');  % muro = nero
        end
    end
end

% Traiettoria (scala le coordinate)
plot(x_d*scale, y_d*scale, 'r', 'LineWidth', 2);

% Start and goal points
plot(x_d(1)*scale, y_d(1)*scale, 'go', 'MarkerFaceColor','g', 'MarkerSize',8); % start = green
plot(x_d(end)*scale, y_d(end)*scale, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8); % goal = red

% Stanza centrale (solo bordo)
rectangle('Position', [(c1-1)*scale, (nrows-r2)*scale, room_size*scale, room_size*scale], ...
          'EdgeColor','r', 'LineWidth',2);

xlabel('X'); ylabel('Y');
title('Labirinto B/N con Traiettoria (bianco = libero, nero = muro)');
grid off;
axis([0 ncols*scale 0 nrows*scale]);






%% ================================
% FUNZIONI
%% ================================
function maze = generateMaze(nrows,ncols)
    if mod(nrows,2)==0 || mod(ncols,2)==0
        error("nrows e ncols devono essere dispari.");
    end
    maze = ones(nrows,ncols);
    visited = false((nrows+1)/2, (ncols+1)/2);
    stack = [1,1]; visited(1,1) = true;
    maze(1,1) = 0; dirs = [-1 0; 1 0; 0 -1; 0 1];
    while ~isempty(stack)
        cur = stack(end,:); r = cur(1); c = cur(2);
        mazeR = 2*r-1; mazeC = 2*c-1;
        nbrs = [];
        for k=1:4
            nr = r + dirs(k,1); nc = c + dirs(k,2);
            if nr>=1 && nr<=size(visited,1) && nc>=1 && nc<=size(visited,2) && ~visited(nr,nc)
                nbrs(end+1,:) = [nr nc];
            end
        end
        if isempty(nbrs)
            stack(end,:) = [];
        else
            next = nbrs(randi(size(nbrs,1)),:);
            midR = mazeR + (next(1)-r);
            midC = mazeC + (next(2)-c);
            maze(mazeR,mazeC) = 0; maze(midR,midC) = 0;
            maze(2*next(1)-1, 2*next(2)-1) = 0;
            visited(next(1), next(2)) = true;
            stack = [stack; next];
        end
    end
end

function path = astar_path(maze, startPos, goalPos)
    startPos = round(startPos); goalPos  = round(goalPos);
    [nrows, ncols] = size(maze);
    cameFrom = zeros(nrows,ncols,2);
    gScore = inf(nrows,ncols); fScore = inf(nrows,ncols);
    gScore(startPos(1), startPos(2)) = 0;
    fScore(startPos(1), startPos(2)) = heuristic(startPos, goalPos);
    openSet = [startPos, fScore(startPos(1), startPos(2))];
    closedSet = false(nrows,ncols);
    neighbors = [0 1; 1 0; 0 -1; -1 0];
    while ~isempty(openSet)
        [~, idx] = min(openSet(:,3));
        current = openSet(idx,1:2); openSet(idx,:) = [];
        if all(current == goalPos)
            path = current;
            while any(cameFrom(current(1),current(2),:) ~= 0)
                parent = squeeze(cameFrom(current(1),current(2),:))';
                path = [parent; path]; current = parent;
            end
            return;
        end
        closedSet(current(1), current(2)) = true;
        for k = 1:4
            neighbor = current + neighbors(k,:);
            r = neighbor(1); c = neighbor(2);
            if r < 1 || r > nrows || c < 1 || c > ncols, continue; end
            if maze(r,c) == 1 || closedSet(r,c), continue; end
            tentative_g = gScore(current(1), current(2)) + 1;
            if tentative_g < gScore(r,c)
                cameFrom(r,c,:) = current;
                gScore(r,c) = tentative_g;
                fScore(r,c) = tentative_g + heuristic(neighbor, goalPos);
                if isempty(openSet) || ~any(all(openSet(:,1:2) == neighbor,2))
                    openSet = [openSet; neighbor, fScore(r,c)];
                end
            end
        end
    end
    path = [];
    warning('Percorso non trovato!');
end

function h = heuristic(node, goal)
    h = abs(node(1)-goal(1)) + abs(node(2)-goal(2)); % Manhattan distance
end

function startPos = pickFreePerimeterCell(maze)
    [nrows, ncols] = size(maze);
    candidates = [];

    % Lati verticali
    for r = 1:nrows
        if maze(r,1) == 0
            candidates(end+1,:) = [r,1];
        end
        if maze(r,ncols) == 0
            candidates(end+1,:) = [r,ncols];
        end
    end

    % Lati orizzontali
    for c = 1:ncols
        if maze(1,c) == 0
            candidates(end+1,:) = [1,c];
        end
        if maze(nrows,c) == 0
            candidates(end+1,:) = [nrows,c];
        end
    end

    if isempty(candidates)
        error('Nessuna cella libera sul perimetro!');
    end

    startPos = candidates(randi(size(candidates,1)), :);
end
