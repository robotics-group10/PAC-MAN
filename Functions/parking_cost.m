function err = parking_cost(simOut, x_goal, y_goal)

q = simOut.logsout.getElement(1).Values.Data;  % <-- PRIMO segnale
t = simOut.tout;

x = q(:,1);
y = q(:,2);

rho = sqrt((x - x_goal).^2 + (y - y_goal).^2);

err = trapz(t, rho.^2);
end
