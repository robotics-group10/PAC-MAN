% Cost function for parking
function err = parking_cost(simOut, x_goal, y_goal)
    try
        q_data = simOut.logsout.getElement('q').Values.Data;
        final_pos = q_data(end,1:2);
        err = sqrt(sum((final_pos - [x_goal y_goal]).^2));
    catch
        err = inf;
    end
end