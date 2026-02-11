%% SELECT TUNING TO RUN
% 1 = Tracking linearizzato
% 2 = Tracking non lineare
% 3 = Tracking output error feedback
% 4 = Cartesian regulation
% 5 = Posture regulation

prompt = "Legend: \n" + ...
    " 1 = Tracking linearizzato \n" + ...
    " 2 = Tracking non lineare \n" + ...
    " 3 = Tracking output error feedback \n" + ...
    " 4 = Cartesian regulation \n" + ...
    " 5 = Posture regulation \n " + ...
    "\nEnter the value for the tuning of a controller (1-5): ";
tuning_id = input(prompt);   % <<< scegli qui quale tuning eseguire

if tuning_id < 1 || tuning_id > 5
    error('Tuning_id must be in interval (1-5)!');
end

%% PATH SETUP

% Simulink folder
simulink_folder = fullfile('..', 'Simulink');

if exist(simulink_folder, 'dir')
    addpath(simulink_folder);
else
    error('Simulink folder not found! Check the path.');
end

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

% Simulation time
t_sim = linspace(0,10,100000);
%t_sim = linspace(0,10,1000);

% Define multiple trajectories
trajectories = {
    @(t) [2*cos(t), 2*sin(t)];   % Circle
    @(t) [t, sin(t)];            % X-linear sine wave
    @(t) [cos(t), t];            % Y-linear sine wave
    @(t) [ t, 2*tanh(t-5) ];     % Step/Lane change trajectory
    @(t) [t, 2*(t>5)];           % PureStep
    @(t) squareTrajectory(t, 2); % Square
};
function p = squareTrajectory(t, L)

    T = 4*L;
    tau = mod(t, T);

    x = zeros(size(t));
    y = zeros(size(t));

    % Lato 1
    idx = tau < L;
    x(idx) = tau(idx);
    y(idx) = 0;

    % Lato 2
    idx = tau >= L & tau < 2*L;
    x(idx) = L;
    y(idx) = tau(idx) - L;

    % Lato 3
    idx = tau >= 2*L & tau < 3*L;
    x(idx) = 3*L - tau(idx);
    y(idx) = L;

    % Lato 4
    idx = tau >= 3*L;
    x(idx) = 0;
    y(idx) = 4*L - tau(idx);

    p = [x(:), y(:)];
end

%% RUN SELECTED TUNING
switch tuning_id

    %% =========================================================
    %% TRAJECTORY TRACKING LINEARIZED CONFIGURATION (#1)
    %% =========================================================
    case 1
        % PARAMETERS
        % Best: eps=0.87, a=20.00, mean_err_optimal= 0.0016

        small_eps = 0.1;
        big_eps = 0.9; 
        small_a = 3;
        big_a = 20;
        num_vals = 8;

        eps_vals = linspace(small_eps, big_eps, num_vals);
        a_vals   = linspace(small_a, big_a, num_vals);

        tuning_trajectory_tracking_linear( ...
            model_tracking_linear, ...
            trajectories, ...
            t_sim, ...
            eps_vals, ...
            a_vals, ...
            figures_folder );


    %% =========================================================
    %% TRAJECTORY TRACKING NONLINEAR CONFIGURATION (#2)
    %% =========================================================
    case 2
        % PARAMETERS
        % Best: b=23.79, xi=0.79, mean_err_optimal= 0.0138

        small_xi = 0.1;
        big_xi = 0.9;
        small_b = 1;
        big_b = 30;
        num_vals = 8;

        xi_vals  = linspace(small_xi, big_xi, num_vals);
        b_vals = linspace(small_b, big_b, num_vals);

        tuning_trajectory_tracking_nonlinear( ...
            model_tracking_nonlinear, ...
            trajectories, ...
            t_sim, ...
            b_vals, ...
            xi_vals, ...
            figures_folder );


    %% =========================================================
    %% TRAJECTORY TRACKING OUTPUT ERROR FEEDBACK (#3)
    %% =========================================================
    case 3
        % PARAMETERS
        % Best: a=0.20, kp1=11.50, kp2=13.52, mean_err_optimal= 0.0099

        small_a = 0.2;
        big_a = 2.0;
        small_kp1 = 3;
        big_kp1 = 20;
        small_kp2 = 3;
        big_kp2 = 20;
        num_vals = 8;

        a_vals   = linspace(small_a, big_a, num_vals);
        kp1_vals = linspace(small_kp1, big_kp1, num_vals);
        kp2_vals = linspace(small_kp2, big_kp2, num_vals);

        tuning_trajectory_tracking_output_error_feedback( ...
            model_tracking_output_error_feedback, ...
            trajectories, ...
            t_sim, ...
            a_vals, ...
            kp1_vals, ...
            kp2_vals, ...
            figures_folder );


    %% =========================================================
    %% CARTESIAN REGULATION (PARKING) CONFIGURATION (#4)
    %% =========================================================
    case 4
        % PARAMETERS
        % best from second test: Kv = 0.50, Kw = 4.33
        % best considering all goals: Kv = 0.50, Kw = 3.53
        % last with 3 phases: Kv=0.83, Kw=3.25

        small_k_v = 2;
        big_k_v   = 10;
        small_k_w = 2;
        big_k_w   = 10;
        num_k     = 4;

        kv_vals = linspace(small_k_v, big_k_v, num_k);
        kw_vals = linspace(small_k_w, big_k_w, num_k);

        goals = [
            5     0
            0     5
            0.2   0.2
            0.5   0.3
            1     2
            2     1
            10    5
            15   20
        ];

        [k_optimal, final_avg_error] = tuning_cartesian_regulation( ...
            model_reg_cart, ...
            goals, ...
            kv_vals, ...
            kw_vals, ...
            figures_folder );


        for delta = [1, 0.5]
            kv_vals = linspace(k_optimal(1)-delta, k_optimal(1)+delta, num_k);
            kw_vals = linspace(k_optimal(2)-delta, k_optimal(2)+delta, num_k);
    
            % Refinement around optimum
            [k_optimal, final_avg_error] = tuning_cartesian_regulation( ...
                model_reg_cart, ...
                goals, ...
                kv_vals, ...
                kw_vals, ...
                figures_folder );
        end





    %% =========================================================
    %% CARTESIAN POSTURE (PARKING) CONFIGURATION (#5)
    %% =========================================================
    case 5
        % best considering all goals:
        % K1 = 0.93, K2 = 1.64, K3 = 5.79
        % best with (2-10) interval: K1=0.86, K2=0.50, K3=5.36

        small_k = 2;
        big_k   = 10;
        num_k   = 3;

        k_range = linspace(small_k, big_k, num_k);

        k1_vals = k_range;
        k2_vals = k_range;
        k3_vals = k_range;

        posture_goals = [
            5     0
            0.2   0.2
            0.5   0.3
            1     2
            2     1
            10    5
            15   20
        ];

        [k_optimal, final_avg_error] = tuning_posture_regulation( ...
            model_reg_post, ...
            posture_goals, ...
            k1_vals, ...
            k2_vals, ...
            k3_vals, ...
            figures_folder );

        % First refinement
        for delta = [1, 0.5]
            k1_vals = linspace(k_optimal(1)-delta, k_optimal(1)+delta, num_k);
            k2_vals = linspace(k_optimal(2)-delta, k_optimal(2)+delta, num_k);
            k3_vals = linspace(k_optimal(3)-delta, k_optimal(3)+delta, num_k);

            [k_optimal, final_avg_error] = tuning_posture_regulation( ...
                model_reg_post, ...
                posture_goals, ...
                k1_vals, ...
                k2_vals, ...
                k3_vals, ...
                figures_folder );
        end

    otherwise
        error('Invalid tuning_id selected');

end
