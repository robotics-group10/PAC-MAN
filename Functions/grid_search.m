function [best_params, best_error, results] = grid_search(model_name, param_names, param_values, cost_func)
    % Generate all combinations of parameters
    [grid{1:length(param_values)}] = ndgrid(param_values{:});
    param_combos = cellfun(@(x) x(:), grid, 'UniformOutput', false);
    param_combos = [param_combos{:}];
    
    num_combos = size(param_combos,1);
    results = zeros(num_combos, length(param_names)+1);
    
    for i = 1:num_combos
        % Set model parameters
        for j = 1:length(param_names)
            set_param([model_name '/' param_names{j}], 'Value', num2str(param_combos(i,j)));
        end
        
        % Run simulation and evaluate cost
        try
            set_param(model_name, 'SimulationCommand', 'update');

            simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
            err = cost_func(simOut);
        catch
            err = inf; % unstable parameter combination → penalized
        end

        results(i,:) = [param_combos(i,:) err];
        
        % Display progress
        param_str = strjoin(arrayfun(@(n,v) sprintf('%s=%.2f', param_names{n}, v), ...
                        1:length(param_names), param_combos(i,:), 'UniformOutput', false), ', ');
        fprintf('Tried: %s -> Error: %.4f\n', param_str, err);
    end
    
    % Find best combination
    [~, idx] = min(results(:,end));
    best_params = results(idx,1:end-1);
    best_error  = results(idx,end);
end