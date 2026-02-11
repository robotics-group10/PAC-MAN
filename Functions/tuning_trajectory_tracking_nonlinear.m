function tuning_trajectory_tracking_nonlinear(model_tracking, trajectories, t_sim, b_vals, xi_vals, figures_folder)
% Tuning controller parameters for trajectory tracking
% model_tracking: Simulink model name
% trajectories: cell array of trajectory functions @(t) -> [x,y]
% t_sim: simulation time vector
% b_vals, xi_vals: arrays of values for grid search
% figures_folder: folder to save plots

% Define cost function
cost_tracking = @(simOut) tracking_cost(simOut);
num_goals = size(trajectories, 1);
best_params_history = zeros(num_goals, 2);

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
    
    % Grid search over b and xi
    [best_tracking_params, best_tracking_err, results_tracking] = ...
        grid_search(model_tracking, {'b', 'xi'}, {b_vals, xi_vals}, cost_tracking);
    
    best_params_history(k, :) = best_tracking_params;

    fprintf('Trajectory %d -> b=%.2f, xi=%.2f | RMS=%.4f\n', ...
        k, best_tracking_params(1), best_tracking_params(2), best_tracking_err);
    
    % Update Simulink parameters with optimal values
    param_names_tracking = {'b','xi'};
    for i = 1:length(best_tracking_params)
        set_param([model_tracking '/' param_names_tracking{i}], ...
            'Value', num2str(best_tracking_params(i)));
    end
    
    % Update and run simulation
    set_param(model_tracking, 'SimulationCommand', 'update','StopTime', '10');
    simOut = sim(model_tracking, 'ReturnWorkspaceOutputs', 'on');
    
    % Plot and save results
    plot_and_save(simOut, sprintf('Trajectory_%d_Tracking', k), figures_folder);

    %{
    % TO REMOVE
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
    %}

    % Extract timeseries objects
    try
        q_ts  = simOut.logsout.getElement('q').Values;
        qd_ts = simOut.logsout.getElement('q_d').Values;
        
        % Get data and time vectors
        time = q_ts.Time;
        q    = q_ts.Data(:, 1:2);   % Actual [x, y]
        qd   = qd_ts.Data(:, 1:2);  % Desired [xd, yd]
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
    
    % Close fig
    close(hFig);
    
end

% Parametri ottimali medi sulle traiettorie
most_frequent_params = mean(best_params_history, 1);
fprintf(['\n', repmat('=', 1, 30), '\n']);
fprintf('COMBINAZIONE OTTIMALE IDENTIFICATA: b=%.2f, xi=%.2f', ...
    most_frequent_params(1), most_frequent_params(2));
fprintf(['\n', repmat('=', 1, 30), '\n']);

% Imposta i parametri sul modello
param_names = {'b','xi'};
for i = 1:2
    set_param([model_tracking '/' param_names{i}], 'Value', num2str(most_frequent_params(i)));
end

% Calcolo errore medio finale usando tracking_cost su tutte le traiettorie
total_err = 0;
for k = 1:length(trajectories)
    % Preparazione della traiettoria
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

    % Calcola errore con tracking_cost
    total_err = total_err + tracking_cost(simOut);

    % Salva grafico per la traiettoria finale con parametri medi
    plot_and_save(simOut, sprintf('Trajectory_%d_FinalTracking', k), figures_folder);
end

avg_error = total_err / length(trajectories);
fprintf('\n>>> ERRORE MEDIO FINALE SU TUTTE LE TRAIETTORIE CON PARAMETRI OTTIMI: %.4f <<<\n', avg_error);

end



