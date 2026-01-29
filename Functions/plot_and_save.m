% Plot and save figure function (goal is optional, only parking)
function plot_and_save(simOut, figure_name, figures_folder, goal)

    if nargin < 4
        goal = [];
    end

    q_actual = simOut.logsout.getElement('q').Values.Data;

    logsElements = simOut.logsout.getElementNames;
    
    if isstring(logsElements)
        logsElements = cellstr(logsElements);
    end

    if ismember('q_d', logsElements)
        q_des = simOut.logsout.getElement('q_d').Values.Data;
    else
        q_des = [];
    end

    h = figure('Visible','off');
    if ~isempty(q_des)
        plot(q_des(:,1), q_des(:,2),'r--','LineWidth',1.5); hold on;
    end
    plot(q_actual(:,1), q_actual(:,2),'b','LineWidth',1.5); hold on;

    if ~isempty(goal)
        plot(goal(1), goal(2), 'rx', 'MarkerSize',12,'LineWidth',2);
    end

    grid on; axis equal; xlabel('X [m]'); ylabel('Y [m]');
    legend_entries = {'Actual'};
    if ~isempty(q_des), legend_entries = [{'Desired'}, legend_entries]; end
    if ~isempty(goal), legend_entries = [legend_entries, {'Goal'}]; end
    legend(legend_entries);

    title(figure_name);
    saveas(h, fullfile(figures_folder, [figure_name, '.png']));
    close(h);
end
