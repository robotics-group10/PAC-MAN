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
model_tracking_linear= 'traj_track_state_error_linearization_ctrl';
model_tracking_nonlinear = 'traj_track_state_error_nonlinear_ctlr';
model_tracking_output_error_feedback = 'traj_track_output_error_feedback_ctrl';
model_tracking_output = 'traj_track_output_error_feedback_ctrl';
model_reg_cart = 'cartesian_regulation_ctrl';
model_reg_post = 'posture_regulation_ctrl';

% Load models without opening GUI
load_system(model_tracking_linear);
load_system(model_tracking_nonlinear);
load_system(model_tracking_output);
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

% Simulation time
t_sim = linspace(0,10,1000);
% Define multiple trajectories
trajectories = {
    @(t) [2*cos(t), 2*sin(t)];  % Circle
    @(t) [t, sin(t)];           % X-linear sine wave
    @(t) [cos(t), t];           % Y-linear sine wave
    @(t) [ t, 2*tanh(t-5) ];    % Step/Lane change trajectory
    %@(t) [t, 2*(t>5)];         % PureStep (not twice differentiable, but can be tried with output error controller if implemented)
};

%% TRAJECTORY TRACKING LINEARIZED CONFIGURATION (#1)
%PARAMETERS

% Best: eps = 0.9, a = 30

eps_vals = [0.2 0.5 0.8];
%a_vals   = [5 10 15];

% Tune controller
%tuning_trajectory_tracking_linear(model_tracking_linear, trajectories, t_sim, eps_vals, a_vals, figures_folder)


%% TRAJECTORY TRACKING NONLINEAR CONFIGURATION (#2)

%PARAMETERS
b_vals = [0.2 0.5 0.8];
xi_vals   = [5 10 15];

% Tune controller
%tuning_trajectory_tracking_nonlinear(model_tracking_nonlinear, trajectories, t_sim, b_vals, xi_vals, figures_folder)

%% TRAJECTORY TRACKING OUTPUT ERROR FEEDBACK (#3)

%PARAMETERS
a_vals = [0.2 0.5 0.8];
kp1_vals = [5 10 15];
kp2_vals = [5 10 15];

% Tune controller
tuning_trajectory_tracking_output_error_feedback(model_tracking_output_error_feedback, trajectories, t_sim , a_vals, kp1_vals, kp2_vals, figures_folder)


%% CARTESIAN REGULATION (PARKING) CONFIGURATION

%PARAMETERS

% Best: Kv=1.00, Kw=6.00 +STABLE -> FOR LONG DISTANCES
%kv_vals = [0.5 1];
%kw_vals = [4 6];

% Best: Kv=2.00, Kw=8.00 +SPEED -> BETTER IN GENERAL 
%kv_vals = [1 2];
%kw_vals = [6 8];

% Best: Kv=3.00, Kw=9.00 
kv_vals = [0.5 1 1.5 2 2.5 3];
kw_vals = [3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8 8.5 9];

% Define goals [x, y]
goals = [
    % Near
    0.2   0.2
    0.5   0.3
   -0.3   0.4

    % Medium
    1     2
    2     1
   -2     3
    3    -2

    % Far
    10    5
   -8    12
    15   20
   -20  -15

    % Axes
    5     0
   -5     0
    0     5
    0    -5
];


% Tune controller
%tuning_cartesian_regulation(model_reg_cart, goals, kv_vals, kw_vals, figures_folder)

%% CARTESIAN POSTURE (PARKING) CONFIGURATION
k1_vals = [1 1.5 2];
k2_vals = [1 1.5 2];
k3_vals = [1 1.5 2];

% Tune controller
%tuning_posture_regulation(model_reg_post, goals, k1_vals, k2_vals, k3_vals, figures_folder)