% Plot and save figure function
% Plots the actual trajectory from simulation and optionally the desired
% trajectory and parking goal. Saves the figure as a PNG in the specified folder.
% 'goal' is optional and only relevant for parking.
function plot_and_save(simOut, figure_name, figures_folder, goal)

    % If 'goal' is not provided, set it to empty
    if nargin < 4
        goal = [];
    end

    % Extract actual position data (x, y) from simulation output
    q_actual = simOut.logsout.getElement('q').Values.Data;

    % Get all signal names in the simulation logs
    logsElements = simOut.logsout.getElementNames;

    % Ensure logsElements is a cell array of strings
    if isstring(logsElements)
        logsElements = cellstr(logsElements);
    end

    % Check if desired trajectory 'q_d' exists, extract if available
    if ismember('q_d', logsElements)
        q_des = simOut.logsout.getElement('q_d').Values.Data;
    else
        q_des = [];
    end

    % Create an invisible figure (won't display on screen)
    h = figure('Visible','off');

    % Plot desired trajectory if it exists (red dashed line)
    if ~isempty(q_des)
        plot(q_des(:,1), q_des(:,2),'r--','LineWidth',1.5); hold on;
    end

    % Plot actual trajectory (blue solid line)
    plot(q_actual(:,1), q_actual(:,2),'b','LineWidth',1.5); hold on;

    % Plot goal position if provided (red X marker)
    if ~isempty(goal)
        plot(goal(1), goal(2), 'rx', 'MarkerSize',12,'LineWidth',2);
    end

    % Formatting the plot
    grid on;               % Show grid
    axis equal;            % Equal scaling on both axes
    xlabel('X [m]');       % X-axis label
    ylabel('Y [m]');       % Y-axis label

    % Build legend dynamically
    legend_entries = {'Actual'};
    if ~isempty(q_des), legend_entries = [{'Desired'}, legend_entries]; end
    if ~isempty(goal), legend_entries = [legend_entries, {'Goal'}]; end
    legend(legend_entries);

    % Set the figure title
    title(figure_name);

    % Save the figure as PNG in the specified folder
    saveas(h, fullfile(figures_folder, [figure_name, '.png']));

    % Close the figure to free memory
    close(h);
end
