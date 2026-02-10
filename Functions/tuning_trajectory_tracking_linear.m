function tuning_trajectory_tracking_linear(model_tracking, trajectories, t_sim, eps_vals, a_vals, figures_folder)

% Define cost function
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
    
    % Created q_d_new = [time, x, y, theta] for "From Workspace"
    
    % TODO if the manual switch are dismissed in simulink files, q_d_new 
    % can be replace with q_d

    q_d_new = [t, x_d, y_d, theta_d];
    assignin('base','q_d_new', q_d_new);
    
    % Created q0 = [x, y, theta] as initial state of unicycle model
    q0 = [x_d(1); y_d(1); theta_d(1)];
    assignin('base','q0', q0);

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
    simOut = sim(model_tracking,'ReturnWorkspaceOutputs', 'on');
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