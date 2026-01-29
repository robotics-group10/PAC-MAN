%% PAC-MAN CONTROLLER TUNING SCRIPT (TRACKING + REGULATION)
clear; clc; close all;

%% PATH SETUP

% Simulink folder
simulink_folder = fullfile('..', 'Simulink');

if exist(simulink_folder, 'dir')
    addpath(simulink_folder);
else
    error('Simulink folder not found! Check the path.');
end

% Model names (without extension)
model_tracking = 'trajectory_tracking_crl';
model_reg      = 'cartesian_regulation_crl';

% Load models without opening GUI
load_system(model_tracking);
load_system(model_reg);

% Figures folder
figures_folder = fullfile('..', 'Figures');

% Clean existing folder for new results
if exist(figures_folder, 'dir')
    rmdir(figures_folder, 's'); 
end
mkdir(figures_folder);

% Functions folder
functions_folder = fullfile('..','Functions');

if exist(functions_folder,'dir')
    addpath(functions_folder);
else
    error('Functions folder not found!');
end

%% TRAJECTORY TRACKING CONFIGURATION
%PARAMETERS
eps_vals = [0.2 0.5 0.8];
a_vals   = [5 10 15];

% Define multiple trajectories
trajectories = {
    @(t) [2*cos(t), 2*sin(t)];                                      % Circle
    @(t) [t, sin(t)];                                               % X-linear sine wave
    @(t) [cos(t), t];                                               % Y-linear sine wave
    @(t) [ t, 2*tanh(t-5) ];                                        % Step/Lane change trajectory
    @(t) [t, 2*(t>5)];                                              % Pure Step
};

t_sim = linspace(0,10,500);

cost_tracking = @(simOut) tracking_cost(simOut);

%% Loop over trajectories
for k = 1:length(trajectories)
    % Generate trajectory
    t = t_sim(:);
    xy = trajectories{k}(t);
    
    x_d = xy(:,1); y_d = xy(:,2);
    dt = gradient(t);
    dx = gradient(x_d) ./ dt;
    dy = gradient(y_d) ./ dt;
    
    % Smoothing od gradient to decrease noise
    dx = smoothdata(dx,'movmean',5);
    dy = smoothdata(dy,'movmean',5);

    % Epsilon to avoid atan2(0,0)
    theta_d = atan2(dy + 1e-12, dx + 1e-12);
    
    % Create q_d_new = [time, x, y, theta] for "From Workspace"
    
    % TODO if the manual switch are dismissed in simulink files, q_d_new 
    % can be replace with q_d
    q_d_new = [t, x_d, y_d, theta_d];
    assignin('base','q_d_new', q_d_new);
    
    fprintf('\n=== Tuning Trajectory %d ===\n', k);
    
    % Run grid search over a and eps
    [best_tracking_params, best_tracking_err, results_tracking] = ...
        grid_search(model_tracking, {'eps','a'}, {eps_vals, a_vals}, cost_tracking);
    
    fprintf('Trajectory %d -> eps=%.2f, a=%.2f | RMS=%.4f\n', ...
        k, best_tracking_params(1), best_tracking_params(2), best_tracking_err);
    
    % Re-run simulation with optimal parameters and plot
    param_names_tracking = {'eps','a'};
    for i = 1:length(best_tracking_params)
        set_param([model_tracking '/' param_names_tracking{i}], ... 
            'Value', num2str(best_tracking_params(i)));
    end
    
    set_param(model_tracking, 'SimulationCommand', 'update');
    simOut = sim(model_tracking,'ReturnWorkspaceOutputs','on');
    plot_and_save(simOut, sprintf('Trajectory_%d_Tracking', k), figures_folder);
    
end

%% CARTESIAN REGULATION (PARKING) CONFIGURATION
%PARAMETERS
kv_vals = [0.5 1 2];
kw_vals = [2 5 7];

% Define goals [x, y]
goals = [1 1; 3 3; 2 5; 0 4];

%% Loop over goals
for k = 1:size(goals,1)
    current_goal = goals(k,:);
    
    % Create q_goal for "From Workspace"
    q_goal_simulink = [0, current_goal(1), current_goal(2); 
                       100, current_goal(1), current_goal(2)];
    assignin('base', 'q_goal', q_goal_simulink);
    
    cost_parking = @(simOut) parking_cost(simOut, current_goal(1), current_goal(2));
    
    fprintf('\n=== Tuning Parking Goal [%.1f, %.1f] ===\n', current_goal(1), current_goal(2));
    
    % Grid search over Kv, Kw
    [best_parking_params, best_parking_err, results_parking] = ...
        grid_search(model_reg, {'kv','kw'}, {kv_vals, kw_vals}, cost_parking);
    
    fprintf('Goal [%.1f, %.1f] -> Kv=%.2f, Kw=%.2f | Err=%.4f\n', ...
        current_goal(1), current_goal(2), best_parking_params(1), best_parking_params(2), best_parking_err);
    
    % Re-run simulation with optimal parameters and plot
    param_names_parking = {'kv','kw'};
    for i = 1:length(best_parking_params)
        set_param([model_reg '/' param_names_parking{i}], 'Value', num2str(best_parking_params(i)));
    end
    
    set_param(model_reg, 'SimulationCommand', 'update');
    simOut = sim(model_reg, 'ReturnWorkspaceOutputs', 'on');
    plot_and_save(simOut, sprintf('Parking_Goal_%.1f_%.1f', current_goal(1), current_goal(2)), figures_folder, current_goal);
  
end