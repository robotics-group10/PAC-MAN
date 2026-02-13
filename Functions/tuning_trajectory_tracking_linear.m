function tuning_trajectory_tracking_linear(model_tracking, trajectories, t_sim, eps_vals, a_vals, figures_folder)

% Define cost function
cost_tracking = @(simOut) tracking_cost(simOut);
num_goals = size(trajectories, 1);
best_params_history = zeros(num_goals, 2);

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
    
    best_params_history(k, :) = best_tracking_params;

    fprintf('Trajectory %d -> eps=%.2f, a=%.2f | RMS=%.4f\n', ...
        k, best_tracking_params(1), best_tracking_params(2), best_tracking_err);
end

% Compute mean using the best parameters and print
most_frequent_params = mean(best_params_history, 1);
fprintf(['\n', repmat('=', 1, 30), '\n']);
fprintf('Average best paramenters: eps=%.2f, a=%.2f', ...
    most_frequent_params(1), most_frequent_params(2));
fprintf(['\n', repmat('=', 1, 30), '\n']);

% Set values of the parameters for simulation
param_names = {'eps','a'};
for i = 1:2
    set_param([model_tracking '/' param_names{i}], 'Value', num2str(most_frequent_params(i)));
end

% Re-run the simulations for each goal, using the average parameters 
total_err = 0;
for k = 1:length(trajectories)

    % Computing trajectory
    t = t_sim(:);
    xy = trajectories{k}(t);
    x_d = xy(:,1); y_d = xy(:,2);
    dt = gradient(t);
    dx = gradient(x_d) ./ dt;
    dy = gradient(y_d) ./ dt;
    dx = smoothdata(dx,'movmean',5);
    dy = smoothdata(dy,'movmean',5);
    theta_d = atan2(dy + 1e-12, dx + 1e-12);
    q_d_new = [t, x_d, y_d, theta_d];
    assignin('base','q_d_new', q_d_new);
    q0 = [x_d(1); y_d(1); theta_d(1)];
    assignin('base','q0',q0);

    % Re run simulation on trajectory k with medium params
    set_param(model_tracking, 'SimulationCommand', 'update', 'StopTime', '10');
    simOut = sim(model_tracking, 'ReturnWorkspaceOutputs','on');

    % Compute error
    total_err = total_err + tracking_cost(simOut);

    % Plot and save trajectory to disk
    plot_and_save(simOut, sprintf('Trajectory_%d', k), figures_folder);

    % Plot evolution of error and velocities
    % Extract timeseries objects
    try
        q_ts  = simOut.logsout.getElement('q').Values;
        qd_ts = simOut.logsout.getElement('q_d').Values;
        v_ts = simOut.logsout.getElement('v').Values;
        w_ts = simOut.logsout.getElement('w').Values;
        
        % Get data and time vectors
        time = q_ts.Time;
        q    = q_ts.Data(:, 1:2);   % Actual [x, y]
        qd   = qd_ts.Data(:, 1:2);  % Desired [xd, yd]
        v_data    = v_ts.Data;   % Linear Velocity [m/s]
        w_data    = w_ts.Data;   % Angular Velocity [rad/s]
    catch
        error('Could not extract q or q_d from simOut. Check signal logging.');
    end
    
    % Compute component errors
    error_x = qd(:, 1) - q(:, 1); % x_d - x
    error_y = qd(:, 2) - q(:, 2); % y_d - y
    
    % Create figure
    hFig = figure('Visible', 'off', 'Position', [100, 100, 800, 600]);
    
    % Subplot 1: X Error
    subplot(2, 1, 1);
    plot(time, error_x, 'b-', 'LineWidth', 1.5);
    title(['Trajectory ' num2str(k) ': Position Error X (x_d - x)']);
    ylabel('Error [m]');
    grid on;
    
    % Subplot 2: Y Error
    subplot(2, 1, 2);
    plot(time, error_y, 'r-', 'LineWidth', 1.5);
    title(['Trajectory ' num2str(k) ': Position Error Y (y_d - y)']);
    xlabel('Time [s]');
    ylabel('Error [m]');
    grid on;
    
    % Save to disk
    figure_name = sprintf('Trajectory_%d_PosError', k);
    full_path = fullfile(figures_folder, [figure_name, '.png']);
    
    saveas(hFig, full_path);
    close(hFig);
    
    % Create Figure (Invisible)
    hFig_ctrl = figure('Visible', 'off', 'Position', [100, 100, 800, 600]);
    
    % Subplot 1: Linear Velocity (v)
    subplot(2, 1, 1);
    plot(time, v_data, 'b-', 'LineWidth', 1.5);
    title('Control Input: Linear Velocity (v)');
    ylabel('Velocity [m/s]');
    grid on;
    
    % Subplot 2: Angular Velocity (w)
    subplot(2, 1, 2);
    plot(time, w_data, 'r-', 'LineWidth', 1.5);
    title('Control Input: Angular Velocity (w)');
    xlabel('Time [s]');
    ylabel('Ang. Vel [rad/s]');
    grid on;

    % Create filename
    ctrl_fig_name = sprintf('Trajectory_%d_input_kin', k);
    full_path_ctrl = fullfile(figures_folder, [ctrl_fig_name, '.png']);
    
    saveas(hFig_ctrl, full_path_ctrl);
    close(hFig_ctrl);
end

% Avg error
avg_error = total_err / length(trajectories);
fprintf('\n>>> Avg error with final parameters: %.4f <<<\n', avg_error);

end

