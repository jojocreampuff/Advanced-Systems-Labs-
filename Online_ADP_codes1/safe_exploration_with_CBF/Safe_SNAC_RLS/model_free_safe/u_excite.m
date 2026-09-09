function out = u_excite(t,amp_mult)

    EPS_FLOOR  = 0.2;
    DECAY_RATE = 1e-4;
    env = amp_mult * (EPS_FLOOR + (1-EPS_FLOOR)*exp(-DECAY_RATE*t));

    u0 = sin(t)^2*cos(t) + sin(2*t)^2*cos(0.1*t) + sin(-1.2*t)^2*cos(0.5*t) ...
       + sin(t)^5 + sin(1.12*t)^2 + cos(2.4*t)*sin(2.4*t)^3;

    FREQ  = [0.1 0.4 0.7 1.3 2.1 2.9 3.5];
    PHASE = pi*[0.0007 0.9129 0.1269 0.2957 0.6854 0.1912 0.9057];
    ms = sum(sin(FREQ*t + PHASE));

    out = env*(u0 + ms);

     %% Noisey signal 
    % eps_floor = 0.2;
    % env = amp_mult*(eps_floor + (1 - eps_floor)*exp(-0.0001*t));
    % 
    % u_prob = (sin(t)^2*cos(t) + sin(2*t)^2*cos(0.1*t) + sin(-1.2*t)^2*cos(0.5*t) + sin(t)^5 + sin(1.12*t)^2 + cos(2.4*t)*sin(2.4*t)^3);
    % 
    % F = [0.1 0.4 0.7  1.3  2.1  2.9 3.5;   % rad/s for tau_x
    %     0.2 0.5 0.9  1.7  2.3  3.1 3.9;   % tau_y
    %     0.3 0.6 1.1  1.5  2.7  3.3 4.1]; % tau_z  (all distinct & incommensurate)
    % 
    % PHI = pi*[0.0007    0.9129    0.1269    0.2957    0.6854    0.1912    0.9057
    % 0.1154    0.0872    0.4547    0.1271    0.0755    0.9909    0.4899
    % 0.9512    0.0978    0.5858    0.4391    0.1859    0.6023    0.1811];
    % % optional orthonormal mixing to reduce residual correlation
    % [Qm,~] = qr([   -0.6581   -0.0909   -0.3155
    % 0.6145    0.2427   -0.3625
    % 0.3684    1.1126   -0.8235]);      % 3x3 with approx. orthonormal columns
    % 
    % s = sum(  sin(F*t + PHI), 2 );  % 3x1
    % out = env * (Qm * (s + u_prob ));

end