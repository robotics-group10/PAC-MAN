function err = parking_cost(simOut, x_goal, y_goal)

% --- Estrai lo stato loggato ---
q = simOut.logsout.getElement('q').Values.Data;   % [x, y, theta]
t = simOut.tout;

x = q(:,1);
y = q(:,2);

% --- Errore di posizione ---
rho = sqrt((x - x_goal).^2 + (y - y_goal).^2);

% --- Estrarre comandi di controllo v e omega ---
v     = simOut.logsout.getElement('v').Values.Data;
omega = simOut.logsout.getElement('w').Values.Data;

% --- Normalizzazione sicura ---
norm_goal = norm([x_goal, y_goal]);
if norm_goal < 1e-6
    norm_goal = 1;
end

% --- Costo combinato semplice ---
% 0.1 è un peso piccolo per non penalizzare troppo i comandi
err = trapz(t, rho.^2 + 0.1*v.^2 + 0.1*omega.^2) / norm_goal;

end
%{
function err = parking_cost(simOut, x_goal, y_goal)

% --- Estrai lo stato loggato ---
q = simOut.logsout.getElement('q').Values.Data;   % [x, y, theta]
t = simOut.tout;

x = q(:,1);
y = q(:,2);

% Euclidean distance to the goal at each time instant 
rho = sqrt((x - x_goal).^2 + (y - y_goal).^2); 
% Cost: integral of the squared distance over time the normalized by 
% initial Eucledian distance 
err = trapz(t, rho.^2)/ norm([x_goal,y_goal]);

end
%}