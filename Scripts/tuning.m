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
model_reg_cart = 'cartesian_regulation_ctrl';
model_reg_post = 'posture_regulation_ctrl';

% Load models without opening GUI
load_system(model_tracking_linear);
load_system(model_tracking_nonlinear);
load_system(model_tracking_output_error_feedback);
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
a_vals   = [5 10 15];

% Tune controller
tuning_trajectory_tracking_linear(model_tracking_linear, trajectories, t_sim, eps_vals, a_vals, figures_folder)


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
%tuning_trajectory_tracking_output_error_feedback(model_tracking_output_error_feedback, trajectories, t_sim , a_vals, kp1_vals, kp2_vals, figures_folder)


%% CARTESIAN REGULATION (PARKING) CONFIGURATION

%PARAMETERS

% best from second test: Kv=0.50, Kw=4.33
% best considering all goals: Kv=0.50, Kw=3.53
small_k_v = 0.5;
big_k_v = 3;
small_k_w = 2;
big_k_w = 10;
num_k = 4;
kv_vals = linspace(small_k_v, big_k_v, num_k);
kw_vals = linspace(small_k_w, big_k_w, num_k);

% Define goals [x, y]
goals = [
    % Near
    0.2   0.2
    %0.5   0.3
    %-0.3   0.4
 
    % Medium
    1     2
    %2     1
    %-2     3
    %3    -2

    % Far
    10    5
    %-8    12
    %15   20
    %-20  -15

    % Axes
    %5     0
   %-5     0
    %0     5
    %0    -5
];


% Tune controller

[k_optimal, final_avg_error] = tuning_cartesian_regulation(model_reg_cart, goals, kv_vals, kw_vals, figures_folder);

small_k_v = k_optimal(1) - 0.5;
big_k_v = k_optimal(1) +0.5;

small_k_w = k_optimal(2) - 1;
big_k_w = k_optimal(2) + 1;

kv_vals = linspace(small_k_v, big_k_v, num_k);
kw_vals = linspace(small_k_w, big_k_w, num_k);

[k_optimal, final_avg_error] = tuning_cartesian_regulation(model_reg_cart, goals, kv_vals, kw_vals, figures_folder);


%% CARTESIAN POSTURE (PARKING) CONFIGURATION

% best considering all goals: K1=0.93, K2=1.64, K3=5.79

small_k = 2;
big_k = 8;

num_k = 3;
k_range = linspace(small_k, big_k, num_k);

k1_vals = k_range;
k2_vals = k_range;
k3_vals = k_range;

% Define goals [x, y]
posture_goals = [

    % Axes
    5     0

    % Near
    0.2   0.2
    0.5   0.3

    % Medium
    1     2
    2     1

    % Far
    10    5
    15   20

];

% Tune controller
[k_optimal, final_avg_error] = tuning_posture_regulation(model_reg_post, posture_goals, k1_vals, k2_vals, k3_vals, figures_folder);

big_k_1 = k_optimal(1) + 1;
big_k_2 = k_optimal(2) + 1;
big_k_3 = k_optimal(3) + 1;

small_k_1 = k_optimal(1) - 1;
small_k_2 = k_optimal(2) - 1;
small_k_3 = k_optimal(3) - 1;

k1_vals = linspace(small_k_1, big_k_1, num_k);
k2_vals = linspace(small_k_2, big_k_2, num_k);
k3_vals = linspace(small_k_3, big_k_3, num_k);

[k_optimal, final_avg_error] = tuning_posture_regulation(model_reg_post, posture_goals, k1_vals, k2_vals, k3_vals, figures_folder);

big_k_1 = k_optimal(1) + 0.5;
big_k_2 = k_optimal(2) + 0.5;
big_k_3 = k_optimal(3) + 0.5;

small_k_1 = k_optimal(1) - 0.5;
small_k_2 = k_optimal(2) - 0.5;
small_k_3 = k_optimal(3) - 0.5;

k1_vals = linspace(small_k_1, big_k_1, num_k);
k2_vals = linspace(small_k_2, big_k_2, num_k);
k3_vals = linspace(small_k_3, big_k_3, num_k);

[k_optimal, final_avg_error] = tuning_posture_regulation(model_reg_post, posture_goals, k1_vals, k2_vals, k3_vals, figures_folder);
