function err = parking_cost(simOut, x_goal, y_goal)

% Extract the logged state signal (assumed to be q = [x, y, theta])
q = simOut.logsout.getElement(1).Values.Data;   % first logged signal

% Simulation time vector
t = simOut.tout;

% Extract Cartesian position
x = q(:,1);
y = q(:,2);

% Euclidean distance to the goal at each time instant
rho = sqrt((x - x_goal).^2 + (y - y_goal).^2);

% Cost: integral of the squared distance over time
err = trapz(t, rho.^2)/ norm([x_goal,y_goal]);;
end

