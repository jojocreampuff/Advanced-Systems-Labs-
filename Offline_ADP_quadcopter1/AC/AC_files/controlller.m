function sim = controlller(timeFinal, iniPos, iniVel, fx, fy, fz)
    ltic = tic;
    %% SETUP
    sim = {};
    
    
    constants
    time = 0:timeStep:timeFinal;

%     randCoeff = 0.0;
%     o = [1 3 5 2 4 6]; % Better plot order
    
    % Smooth out the trajectory function if the initial position/velocity doesnt match trajectory
    f = [fx(time); fy(time); fz(time)];

    F = smooth(f, iniPos, iniVel, time);

    Fx = F(1,:);
    Fy = F(2,:);
    Fz = F(3,:);
    
    
    
    %% SIM ALPHA
    addpath(genpath('train_alpha/'))
    
    alphaRef = trajectory(Fx, Fy, Fz);  % Reference path for part alpha
    [xk, uk, pos_error, instant_cost_pos, cumulative_cost_pos] = alphaPerformer([iniPos iniVel], alphaRef);
    
    sim.x = xk(1, :);
    sim.y = xk(2, :);
    sim.z = xk(3, :);
    
    sim.u = xk(4, :);
    sim.v = xk(5, :);
    sim.w = xk(6, :);
    
    % Convert accelerations to alpha values
    alpha = getAlpha(uk(1, :), uk(2, :), uk(3, :));
    
    sim.ft = alpha(1, :);
    
    sim.phi = alpha(2, :);
    sim.the = alpha(3, :);
    sim.psi = alpha(4, :);
    
    sim.p = alpha(5, :);
    sim.q = alpha(6, :);
    sim.r = alpha(7, :);
    
    
    
    %% SIM BETA
    addpath(genpath('train_beta/'))
    betaRef = alpha(2:7, :); % Reference path for part beta
    iniAng = alpha(2:4, 1)';
    iniWel = alpha(5:7, 1)';
    [xk, uk, att_error, instant_cost_att, cumulative_cost_att] = betaPerformer([iniAng iniWel], betaRef);
    
    sim.taux = uk(1, :);
    sim.tauy = uk(2, :);
    sim.tauz = uk(3, :);
   
    
    %% VARS
    sim.alphaRef = alphaRef;
    sim.betaRef = betaRef;
    sim.pos_error = pos_error;
    sim.att_error = att_error;
    sim.in_cost_pos = instant_cost_pos;
    sim.in_cost_att = instant_cost_att;
    sim.c_cost_pos = cumulative_cost_pos;
    sim.c_cost_att = cumulative_cost_att;
    
    sim.states = [sim.x; sim.y; sim.z; sim.u; sim.v; sim.w; sim.phi; sim.the; sim.psi; sim.p; sim.q; sim.r];
    sim.controls = [sim.ft; sim.taux; sim.tauy; sim.tauz];
    sim.rotors = getRotorSpeed(sim.ft, sim.taux, sim.tauy, sim.tauz);

    sim.time = time;

    sim.f = f;
    sim.F = F;

    ltoc = toc(ltic);
    fprintf("Finished sim\t%0.4f s\n", ltoc)

end