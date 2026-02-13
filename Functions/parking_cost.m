function err = parking_cost(simOut, x_goal, y_goal)

% Extract actual position data (x, y) from simulation output
q = simOut.logsout.getElement('q').Values.Data;   % [x, y, theta]
t = simOut.tout;

x = q(:,1);
y = q(:,2);

% Compute distance to the goal position
rho = sqrt((x - x_goal).^2 + (y - y_goal).^2);

% Extract input velocities from simulation
v     = simOut.logsout.getElement('v').Values.Data;
omega = simOut.logsout.getElement('w').Values.Data;

% Compute norm
norm_goal = norm([x_goal, y_goal]);
if norm_goal < 1e-6
    norm_goal = 1;
end

% Evalueate cost function
err = trapz(t, rho.^2 + 0.1*v.^2 + 0.1*omega.^2) / norm_goal;

end