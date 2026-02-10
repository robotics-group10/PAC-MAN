function [best_gains, final_avg_error] = tuning_posture_regulation(model_reg, goals, k1_vals, k2_vals, k3_vals, figures_folder)
num_goals = size(goals, 1);
best_params_history = zeros(num_goals, 3);
%% Loop over goals
for k = 1:size(goals,1)
    current_goal = goals(k,:);
    
    % Create q_goal for "From Workspace"
    q_goal_simulink = [0, current_goal(1), current_goal(2); 
                       100, current_goal(1), current_goal(2)];
    assignin('base', 'q_goal', q_goal_simulink);

    % Set initial pose for regulation
    q0_reg = [0; 0; 0];  % replace with actual start position if needed
    assignin('base', 'q0_reg', q0_reg);
    
    cost_parking = @(simOut) parking_cost(simOut, current_goal(1), current_goal(2));
    
    fprintf('\n=== Tuning Parking Goal [%.1f, %.1f] ===\n', current_goal(1), current_goal(2));
    
    % Grid search over K1, K2, K3
    [best_parking_params, best_parking_err, results_parking] = ...
        grid_search(model_reg, {'k1','k2', 'k3'}, {k1_vals, k2_vals, k3_vals}, cost_parking);

    best_params_history(k, :) = best_parking_params;

    fprintf('Goal [%.1f, %.1f] -> K1=%.2f, K2=%.2f, K3=%.2f| Err=%.4f\n', ...
        current_goal(1), current_goal(2), best_parking_params(1), best_parking_params(2), best_parking_params(3), best_parking_err);
    
    % Re-run simulation with optimal parameters and plot
    param_names_parking = {'k1','k2', 'k3'};
    for i = 1:length(best_parking_params)
        set_param([model_reg '/' param_names_parking{i}], 'Value', num2str(best_parking_params(i)));
    end
    
    set_param(model_reg, 'SimulationCommand', 'update','StopTime', '10');
    simOut = sim(model_reg, 'ReturnWorkspaceOutputs', 'on');
    plot_and_save(simOut, sprintf('Posture_Parking_Goal_%.1f_%.1f', current_goal(1), current_goal(2)), figures_folder, current_goal);
  
end

most_frequent_params = mean(best_params_history, 1);

fprintf(['\n', repmat('=', 1, 30), '\n']);
fprintf('COMBINAZIONE OTTIMALE IDENTIFICATA: K1=%.2f, K2=%.2f, K3=%.2f\n', ...
    most_frequent_params(1), most_frequent_params(2), most_frequent_params(3));
fprintf(['\n', repmat('=', 1, 30), '\n']);

total_err = 0;
param_names = {'k1','k2', 'k3'};

for i = 1:3
    set_param([model_reg '/' param_names{i}], 'Value', num2str(most_frequent_params(i)));
end


for k = 1:num_goals
    current_goal = goals(k,:);
    q_goal_simulink = [0, current_goal(1), current_goal(2); 100, current_goal(1), current_goal(2)];
    assignin('base', 'q_goal', q_goal_simulink);
    
    % Esegui simulazione
    set_param(model_reg, 'SimulationCommand', 'update');
    simOut = sim(model_reg, 'ReturnWorkspaceOutputs', 'on', 'LoggingToFile', 'off');
    
    % Calcola errore per questo goal
    current_err = parking_cost(simOut, current_goal(1), current_goal(2));
    total_err = total_err + current_err;
    
    % Plot e salvataggio finale
    plot_and_save(simOut, sprintf('FINAL_GlobalParams_Goal_%.1f_%.1f', current_goal(1), current_goal(2)), figures_folder, current_goal);
end

avg_error = total_err / num_goals;
fprintf('\n>>> ERRORE MEDIO FINALE CON PARAMETRI OTTIMI: %.4f <<<\n', avg_error);

best_gains = most_frequent_params;
final_avg_error = avg_error;

end