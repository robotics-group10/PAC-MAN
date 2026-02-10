% Cost function for tracking
% Computes the average tracking error between the actual trajectory (q)
% and the desired trajectory (q_d) over time.
function [err, e] = tracking_cost(simOut)
    % Extract actual position data (x, y) from simulation output
    q  = simOut.logsout.getElement('q').Values.Data(:, 1:2);
    
    % Extract desired position data (x_d, y_d) from simulation output
    qd = simOut.logsout.getElement('q_d').Values.Data(:, 1:2);

    % Compute the Euclidean distance (error) at each time step
    e = vecnorm(q - qd, 2, 2);  % 2-norm along each row (each timestep)
    
    % Compute the mean error over all timesteps as the cost
    err = mean(e);
end
