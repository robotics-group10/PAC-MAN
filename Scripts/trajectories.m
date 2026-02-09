    @(t) [2*cos(t), 2*sin(t)];                                      % Circle
    @(t) [t, sin(t)];                                               % X-linear sine wave
    @(t) [cos(t), t];                                               % Y-linear sine wave
    @(t) [ t, 2*tanh(t-5) ];                                        % Step/Lane change trajectory
    %@(t) [t, 2*(t>5)];                                              % PureStep (not twice differentiable, but can be tried with output error controller if implemented)