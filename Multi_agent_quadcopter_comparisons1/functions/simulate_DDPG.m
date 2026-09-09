function DDPG_results = simulate_DDPG(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
rng(seed);
R_pos = position.R_pos;
Q_pos = position.Q_pos;
R_att = attitude.R_att;
Q_att = attitude.Q_att;
Ts_Sim      = global_parameters.dt;
g           = global_parameters.g;
Tf       = global_parameters.tf;
m       = global_parameters.m;
Ix      = global_parameters.Ix;
Iy      = global_parameters.Iy;
Iz      = global_parameters.Iz;
W_nominal = global_parameters.W_nominal;
state_noise = sim_params.state_noise;
control_noise =sim_params.control_noise;
saturation_logic = sim_params.saturation_logic;
wind_logic = sim_params.wind_logic; 
r_yaw           = global_parameters.r_yaw; 

thisDir = fileparts(mfilename('fullpath'));

% Folder where your .mat agents live:
ddpgDir = fullfile(thisDir, 'DDPG_Cascaded_function');
addpath(ddpgDir);

% Load agents (from the correct folder no matter where you run from)
Spos = load(fullfile(ddpgDir, "Position.mat"), "agent");
Satt = load(fullfile(ddpgDir, "Attitude.mat"), "agent2");

agent  = Spos.agent;
agent2 = Satt.agent2;

% Set sample times
agent.SampleTime  = Ts_Sim;
agent2.SampleTime = Ts_Sim;

% % Make them visible to Simulink (base workspace)
% assignin('base', 'agent', agent);
% assignin('base', 'agent2', agent2);

% --- model ---
mdl = 'Sim_PA_DDPG';
load_system(mdl);   % better than open_system for programmatic runs

% --- build wind timeseries data ---
T = (0:Ts_Sim:Tf-Ts_Sim)';         % Nx1 time vector
Wxyz_timeseries = [T  Wxyz.'];     % Nx4  (time + 3 signals)

% --- create sim input and pass everything Simulink needs ---
in = Simulink.SimulationInput(mdl);

% Stop time (do NOT leave it as Tf expression inside model)
in = in.setModelParameter('StopTime', num2str(Tf));

% Pass agents
in = in.setVariable('agent',  agent);
in = in.setVariable('agent2', agent2);

in = in.setVariable('Ts_Sim', Ts_Sim);
in = in.setVariable('g', g);
in = in.setVariable('Tf', Tf);
in = in.setVariable('m', m);
in = in.setVariable('Ix', Ix);
in = in.setVariable('Iy', Iy);
in = in.setVariable('Iz', Iz);

in = in.setVariable('W_nominal', W_nominal);
in = in.setVariable('state_noise', state_noise);
in = in.setVariable('control_noise', control_noise);
in = in.setVariable('saturation_logic', saturation_logic);
in = in.setVariable('wind_logic', wind_logic);
in = in.setVariable("seed_for_sim", seed);

% Pass wind signal (if model refers to this variable name)
in = in.setVariable('Wxyz_timeseries', Wxyz_timeseries);

% Pass initial conditions if model uses these symbols by name
in = in.setVariable('x_IC', IC(1));
in = in.setVariable('y_IC', IC(2));
in = in.setVariable('z_IC', IC(3));
in = in.setVariable('u_IC', IC(4));
in = in.setVariable('v_IC', IC(5));
in = in.setVariable('w_IC', IC(6));
in = in.setVariable('phi_IC', IC(7));
in = in.setVariable('theta_IC', IC(8));
in = in.setVariable('psi_IC', IC(9));
in = in.setVariable('p_IC', IC(10));
in = in.setVariable('q_IC', IC(11));
in = in.setVariable('r_IC', IC(12));
in = in.setVariable("psi_dsr",r_yaw(1));

tic
simOut = sim(in);
DDPG_sim_time = toc;

% disp(['Simulation runtime: ', num2str(DDPG_sim_time), ' seconds']) % 17.9222 seconds

X = squeeze(simOut.X);
U = squeeze(simOut.U);
U_pos = squeeze(simOut.U_pos);
U_pos(:,3) = U_pos(:,3) - g;
X_ref = simOut.X_ref';
pos_error = simOut.pos_error';
att_error = simOut.att_error';
att_error(3,:) = att_error(3,:) - [r_yaw, r_yaw(1)];

pos_tracking_cost = Ts_Sim * sum( sum( pos_error .* (Q_pos*pos_error), 1 ) );
pos_control_cost = Ts_Sim  * sum( sum( U_pos .* (R_pos*U_pos), 1 ) );

att_tracking_cost = Ts_Sim * sum( sum( att_error .* (Q_att*att_error), 1 ) );
att_control_cost = Ts_Sim  * sum( sum( U(2:4,:) .* (R_att*U(2:4,:)), 1 ) );

DDPG_results.DDPG_c_cost_total = pos_tracking_cost + pos_control_cost + att_control_cost + att_tracking_cost;

% save all useful info in a struck
DDPG_results.X = X;
DDPG_results.U = U;
DDPG_results.X_ref = X_ref;
DDPG_results.error = [pos_error;att_error];
DDPG_results.simtime = DDPG_sim_time;
DDPG_error = DDPG_results.error;
DDPG_tracking_error = (DDPG_error(1,:).^2 + DDPG_error(2,:).^2 + DDPG_error(3,:).^2).^0.5;
DDPG_results.DDPG_tracking_error = DDPG_tracking_error;
DDPG_results.DDPG_tracking_error_norm = Ts_Sim*sum(DDPG_tracking_error,2);


end

