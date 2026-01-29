% Cost function for tracking
function err = tracking_cost(simOut)
    q  = simOut.logsout.getElement('q').Values.Data(:,1:2);
    qd = simOut.logsout.getElement('q_d').Values.Data(:,1:2);

    e = vecnorm(q - qd, 2, 2);
    err = mean(e);
end