function tuning_posture_regulation(model_reg, goals, k1_vals, k2_vals, k3_vals, figures_folder)

%% Loop over goals
for k = 1:size(goals,1)
    current_goal = goals(k,:);
    
    % Create q_goal for "From Workspace"
    q_goal_simulink = [0, current_goal(1), current_goal(2); 
                       100, current_goal(1), current_goal(2)];
    assignin('base', 'q_goal', q_goal_simulink);
    
    cost_parking = @(simOut) parking_cost(simOut, current_goal(1), current_goal(2));
    
    fprintf('\n=== Tuning Parking Goal [%.1f, %.1f] ===\n', current_goal(1), current_goal(2));
    
    % Grid search over K1, K2, K3
    [best_parking_params, best_parking_err, results_parking] = ...
        grid_search(model_reg, {'k1','k2', 'k3'}, {k1_vals, k2_vals, k3_vals}, cost_parking);
    
    fprintf('Goal [%.1f, %.1f] -> K1=%.2f, K2=%.2f, K3=%.2f| Err=%.4f\n', ...
        current_goal(1), current_goal(2), best_parking_params(1), best_parking_params(2), best_parking_params(3), best_parking_err);
    
    % Re-run simulation with optimal parameters and plot
    param_names_parking = {'k1','k2', 'k3'};
    for i = 1:length(best_parking_params)
        set_param([model_reg '/' param_names_parking{i}], 'Value', num2str(best_parking_params(i)));
    end
    
    set_param(model_reg, 'SimulationCommand', 'update');
    simOut = sim(model_reg, 'ReturnWorkspaceOutputs', 'on');
    plot_and_save(simOut, sprintf('Posture_Parking_Goal_%.1f_%.1f', current_goal(1), current_goal(2)), figures_folder, current_goal);
  
end
end