function tuning_trajectory_tracking_output_error_feedback(model_tracking, trajectories, t_sim, a_vals, kp1_vals,kp2_vals, figures_folder)
% Tuning controller parameters for trajectory tracking
% model_tracking: Simulink model name
% trajectories: cell array of trajectory functions @(t) -> [x,y]
% t_sim: simulation time vector
% b_vals, xi_vals: arrays of values for grid search
% figures_folder: folder to save plots

% Define cost function
cost_tracking = @(simOut) tracking_cost(simOut);

%% Loop over trajectories
for k = 1:length(trajectories)
    % Generate trajectory
    t = t_sim(:);
    xy = trajectories{k}(t);  % trajectories{k} should be a function handle
    
    x_d = xy(:,1); 
    y_d = xy(:,2);
    
    dt = gradient(t);
    dx = gradient(x_d) ./ dt;
    dy = gradient(y_d) ./ dt;
    
    % Smooth derivatives to reduce noise
    dx = smoothdata(dx, 'movmean', 5);
    dy = smoothdata(dy, 'movmean', 5);

    % Compute desired orientation
    theta_d = atan2(dy + 1e-12, dx + 1e-12);
    
    % Create desired trajectory for Simulink "From Workspace"
    q_d_new = [t, x_d, y_d, theta_d];
    assignin('base', 'q_d_new', q_d_new);
    
    % Initial state
    q0 = [x_d(1); y_d(1); theta_d(1)];
    assignin('base', 'q0', q0);

    fprintf('\n=== Tuning Trajectory %d ===\n', k);
    
    % Grid search over a (distance from com) and gains kp1, kp2
    [best_tracking_params, best_tracking_err, results_tracking] = ...
        grid_search(model_tracking, {'a', 'kp1', 'kp2'}, {a_vals, kp1_vals, kp2_vals}, cost_tracking);
    
    fprintf('Trajectory %d -> a=%.2f, kp1=%.2f, kp2=%.2f | RMS=%.4f\n', ...
        k, best_tracking_params(1), best_tracking_params(2), best_tracking_params(3), best_tracking_err);
    
    % Update Simulink parameters with optimal values
    param_names_tracking = {'a','kp1', 'kp2'};
    for i = 1:length(best_tracking_params)
        set_param([model_tracking '/' param_names_tracking{i}], ...
            'Value', num2str(best_tracking_params(i)));
    end
    
    % Update and run simulation
    set_param(model_tracking, 'SimulationCommand', 'update','StopTime', '10');
    simOut = sim(model_tracking, 'ReturnWorkspaceOutputs', 'on');
    
    % Plot and save results
    plot_and_save(simOut, sprintf('Trajectory_%d_Tracking', k), figures_folder);

    % Create error plot
    [~, e] = cost_tracking(simOut);
    time = simOut.logsout.getElement('q').Values.Time;
    hFig = figure('Visible', 'off');
    plot(time, e, 'r-', 'LineWidth', 1.5);
    
    title('Trajectory Tracking Error over Time');
    xlabel('Time [s]');
    ylabel('Position Error [m]');
    grid on;
    
    % Save to disk
    figure_name = sprintf('Trajectory_%d_PosError', k);
    saveas(hFig, fullfile(figures_folder, [figure_name, '.png']));
    
    % Close the figure to free memory
    close(hFig);
    
end
end
