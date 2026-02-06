%% ================================
% LABIRINTO CASUALE MA DETERMINISTICO CON TRAIETTORIA AUTOMATICA
% ================================
clear; close all; clc;

%% PARAMETRI LABIRINTO
nrows = 31;       
ncols = 31;       
room_size = 7;    

%% INIZIALIZZA RNG PER LABIRINTO FISSO
rng(1234);

%% GENERA LABIRINTO CASUALE
maze = generateMaze(nrows,ncols);

%% Aggiungi stanza centrale vuota
center_r = ceil(nrows/2);
center_c = ceil(ncols/2);
half = floor(room_size/2);
r1 = center_r - half; r2 = center_r + half;
c1 = center_c - half; c2 = center_c + half;
maze(r1:r2, c1:c2) = 0;

%% PUNTO DI PARTENZA E DI ARRIVO
startPos = [1, 15];        % [riga, colonna]
goalPos  = [center_r, center_c];  % centro stanza centrale

%% TROVA PERCORSO LIBERO (BFS)
path = bfs_path(maze, startPos, goalPos);

if isempty(path)
    error('Non esiste percorso dal punto di partenza al goal!');
end

%% CONVERSIONE CELLE -> COORDINATE REALI
x_path = path(:,2) - 0.5;
y_path = nrows - path(:,1) + 0.5;
t_path = (1:length(x_path))';

%% INTERPOLAZIONE TRAIETTORIA
x_spline = spline(t_path, x_path);
y_spline = spline(t_path, y_path);
T_stop = t_path(end);
t_sim = linspace(t_path(1), T_stop, 5*length(x_path))';
x_d = ppval(x_spline, t_sim);
y_d = ppval(y_spline, t_sim);

% Forza il punto finale esattamente sul goal
x_d(end) = x_path(end);
y_d(end) = y_path(end);

dx = gradient(x_d, t_sim(2)-t_sim(1));
dy = gradient(y_d, t_sim(2)-t_sim(1));
theta_d = atan2(dy, dx);

q_d_new = [t_sim, x_d(:), y_d(:), theta_d(:)];
q0 = [x_d(1); y_d(1); theta_d(1)];
assignin('base','q_d_new', q_d_new);
assignin('base','q0', q0);
fprintf('Traiettoria pronta per Simulink: %d punti\n', length(t_sim));

%% ================================
% SIMULAZIONE CONTROLLORO
%% ================================
model_tracking = 'trajectory_tracking_linearized_crl';
if exist([model_tracking,'.slx'], 'file')
    load_system(model_tracking);
    set_param(model_tracking,'SimulationCommand','update');
    simOut = sim(model_tracking,'ReturnWorkspaceOutputs','on');
    disp('Simulazione completata con successo.');
else
    warning('Modello %s non trovato.', model_tracking);
end

%% SALVATAGGIO FIGURE
figures_folder = fullfile(pwd,'figures');
if ~exist(figures_folder,'dir')
    mkdir(figures_folder);
end
goal_real = [goalPos(2)-0.5, nrows-goalPos(1)+0.5];
plot_and_save(simOut, 'trajectory_unicycle', figures_folder, goal_real);

%% VISUALIZZAZIONE LABIRINTO E TRAIETTORIA
figure('Color','w'); hold on; axis equal tight;
scale = 2; set(gca,'Color','w');
for r = 1:nrows
    for c = 1:ncols
        if maze(r,c) == 1
            rectangle('Position', [(c-1)*scale, (nrows-r)*scale, scale, scale], ...
                      'FaceColor','k', 'EdgeColor','none');
        end
    end
end
plot(x_d*scale, y_d*scale, 'r', 'LineWidth',2);
plot(x_d(1)*scale, y_d(1)*scale, 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(x_d(end)*scale, y_d(end)*scale, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
rectangle('Position', [(c1-1)*scale, (nrows-r2)*scale, room_size*scale, room_size*scale], ...
          'EdgeColor','r','LineWidth',2);
xlabel('X'); ylabel('Y');
title('Labirinto con Traiettoria Automatica Non Collidente');
grid off;
axis([0 ncols*scale 0 nrows*scale]);

%% ================================
% FUNZIONE GENERAZIONE LABIRINTO
%% ================================
function maze = generateMaze(nrows,ncols)
    if mod(nrows,2)==0 || mod(ncols,2)==0
        error('nrows e ncols devono essere dispari.');
    end
    maze = ones(nrows,ncols);
    visited = false((nrows+1)/2, (ncols+1)/2);
    stack = [1,1]; visited(1,1)=true; maze(1,1)=0;
    dirs=[-1 0;1 0;0 -1;0 1];
    while ~isempty(stack)
        cur=stack(end,:); r=cur(1); c=cur(2);
        mazeR=2*r-1; mazeC=2*c-1; nbrs=[];
        for k=1:4
            nr=r+dirs(k,1); nc=c+dirs(k,2);
            if nr>=1 && nr<=size(visited,1) && nc>=1 && nc<=size(visited,2) && ~visited(nr,nc)
                nbrs(end+1,:)=[nr nc];
            end
        end
        if isempty(nbrs)
            stack(end,:)=[];
        else
            next=nbrs(randi(size(nbrs,1)),:);
            midR=mazeR+(next(1)-r); midC=mazeC+(next(2)-c);
            maze(mazeR,mazeC)=0; maze(midR,midC)=0;
            maze(2*next(1)-1,2*next(2)-1)=0;
            visited(next(1),next(2))=true;
            stack=[stack; next];
        end
    end
end

%% ================================
% FUNZIONE BFS PATHFINDING
%% ================================
function path = bfs_path(maze, startPos, goalPos)
    nrows=size(maze,1); ncols=size(maze,2);
    visited=false(nrows,ncols);
    parent = zeros(nrows,ncols,2);
    queue = startPos;
    visited(startPos(1), startPos(2))=true;
    dirs=[-1 0;1 0;0 -1;0 1];
    found=false;
    
    while ~isempty(queue)
        cur=queue(1,:); queue(1,:)=[];
        if isequal(cur, goalPos)
            found=true; break;
        end
        for k=1:4
            nr=cur(1)+dirs(k,1); nc=cur(2)+dirs(k,2);
            if nr>=1 && nr<=nrows && nc>=1 && nc<=ncols && maze(nr,nc)==0 && ~visited(nr,nc)
                queue(end+1,:)=[nr nc];
                visited(nr,nc)=true;
                parent(nr,nc,:) = cur;
            end
        end
    end
    
    if ~found
        path=[];
        return;
    end
    
    % ricostruisci path
    path=goalPos;
    while ~isequal(path(1,:), startPos)
        p = squeeze(parent(path(1,1), path(1,2),:))';
        path=[p; path];
    end
end
