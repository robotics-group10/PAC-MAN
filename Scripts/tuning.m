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
model_tracking = 'traj_track_state_error_linearization_ctrl';
model_reg_cart = 'cartesian_regulation_crl';
model_reg_post = 'posture_regulation';

% Load models without opening GUI
load_system(model_tracking);
load_system(model_reg_cart);
load_system(model_reg_post);


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
    @(t) [2*cos(t), 2*sin(t)];  % Circle
    @(t) [t, sin(t)];           % X-linear sine wave
    @(t) [cos(t), t];           % Y-linear sine wave
    @(t) [ t, 2*tanh(t-5) ];    % Step/Lane change trajectory
    %@(t) [t, 2*(t>5)];         % PureStep (not twice differentiable, but can be tried with output error controller if implemented)
};

% Simulation time
t_sim = linspace(0,10,1000);

% Tune controller
tuning_trajectory_tracking(model_tracking, trajectories, t_sim, eps_vals, a_vals, figures_folder)

%% CARTESIAN REGULATION (PARKING) CONFIGURATION
%PARAMETERS
kv_vals = [0.5 1 2];
kw_vals = [2 5 7];

% Define goals [x, y]
goals = [1 1; 3 3; 2 5; 15 20];

% Tune controller
tuning_cartesian_regulation(model_reg_cart, goals, kv_vals, kw_vals, figures_folder)

%% CARTESIAN POSTURE (PARKING) CONFIGURATION
k1_vals = [0.5 1 2];
k2_vals = [2 5];
k3_vals = [0.5 1];

% Tune controller
tuning_posture_regulation(model_reg_post, goals, k1_vals, k2_vals, k3_vals, figures_folder)