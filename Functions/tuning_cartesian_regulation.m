function tuning_cartesian_regulation(model_reg, goals, kv_vals, kw_vals, figures_folder)

%% Loop over goals
for k = 1:size(goals,1)
    current_goal = goals(k,:);
    
    % Create q_goal for "From Workspace"
    q_goal_simulink = [0, current_goal(1), current_goal(2); 
                       100, current_goal(1), current_goal(2)];
    assignin('base', 'q_goal', q_goal_simulink);
    
    cost_parking = @(simOut) parking_cost(simOut, current_goal(1), current_goal(2));
    
    fprintf('\n=== Tuning Parking Goal [%.1f, %.1f] ===\n', current_goal(1), current_goal(2));
    
    % Grid search over Kv, Kw
    [best_parking_params, best_parking_err, results_parking] = ...
        grid_search(model_reg, {'kv','kw'}, {kv_vals, kw_vals}, cost_parking);
    
    fprintf('Goal [%.1f, %.1f] -> Kv=%.2f, Kw=%.2f | Err=%.4f\n', ...
        current_goal(1), current_goal(2), best_parking_params(1), best_parking_params(2), best_parking_err);
    
    % Re-run simulation with optimal parameters and plot
    param_names_parking = {'kv','kw'};
    for i = 1:length(best_parking_params)
        set_param([model_reg '/' param_names_parking{i}], 'Value', num2str(best_parking_params(i)));
    end
    
    set_param(model_reg, 'SimulationCommand', 'update');
    simOut = sim(model_reg, 'ReturnWorkspaceOutputs', 'on');
    plot_and_save(simOut, sprintf('Cartesian_Parking_Goal_%.1f_%.1f', current_goal(1), current_goal(2)), figures_folder, current_goal);
  
end
end