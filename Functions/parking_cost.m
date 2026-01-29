% Cost function for parking
% Computes the Euclidean distance between the final position of the vehicle
% and the desired goal position (x_goal, y_goal). Returns Inf if data is unavailable.
function err = parking_cost(simOut, x_goal, y_goal)
    try
        % Extract the 'q' signal data from the simulation output
        q_data = simOut.logsout.getElement('q').Values.Data;
        
        % Get the final position (x, y) of the vehicle from the last row of q_data
        final_pos = q_data(end, 1:2);
        
        % Compute the Euclidean distance between final position and goal
        err = sqrt(sum((final_pos - [x_goal y_goal]).^2));
    catch
        % If any error occurs (e.g., q data not found), return infinity
        err = inf;
    end
end
