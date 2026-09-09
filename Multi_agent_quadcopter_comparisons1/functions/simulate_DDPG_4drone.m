function DDPG_results = simulate_DDPG_4drone(seed,comm,position, attitude, global_parameters, ref, IC_all, sim_params, Wxyz)
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
offsets = zeros(3,4);
offsets(:,1) = [ 0;  0; 0];
offsets(:,2) = [-1;  0; 0];
offsets(:,3) = [-1; -1; 0];
offsets(:,4) = [-1;  1; 0];

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

% --- model ---
mdl = 'Sim_PA_DDPG_4drone';
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

% Pass wind signal (if model refers to this variable name)
in = in.setVariable('Wxyz_timeseries', Wxyz_timeseries);

% % Pass initial conditions if model uses these symbols by name
% in = in.setVariable('x_IC', IC(1));
% in = in.setVariable('y_IC', IC(2));
% in = in.setVariable('z_IC', IC(3));
% in = in.setVariable('u_IC', IC(4));
% in = in.setVariable('v_IC', IC(5));
% in = in.setVariable('w_IC', IC(6));
% in = in.setVariable('phi_IC', IC(7));
% in = in.setVariable('theta_IC', IC(8));
% in = in.setVariable('psi_IC', IC(9));
% in = in.setVariable('p_IC', IC(10));
% in = in.setVariable('q_IC', IC(11));
% in = in.setVariable('r_IC', IC(12));
in = in.setVariable('x_IC', IC_all(1,1));
in = in.setVariable('y_IC', IC_all(2,1));
in = in.setVariable('z_IC', IC_all(3,1));
in = in.setVariable('u_IC', IC_all(4,1));
in = in.setVariable('v_IC', IC_all(5,1));
in = in.setVariable('w_IC', IC_all(6,1));
in = in.setVariable('phi_IC', IC_all(7,1));
in = in.setVariable('theta_IC', IC_all(8,1));
in = in.setVariable('psi_IC', IC_all(9,1));
in = in.setVariable('p_IC', IC_all(10,1));
in = in.setVariable('q_IC', IC_all(11,1));
in = in.setVariable('r_IC', IC_all(12,1));
in = in.setVariable("psi_dsr",r_yaw(1));

tic
simOut = sim(in);
DDPG_sim_time = toc;

% disp(['Simulation runtime: ', num2str(DDPG_sim_time), ' seconds']) % 17.9222 seconds

% X = squeeze(simOut.X);
% U = squeeze(simOut.U);
% U_pos = squeeze(simOut.U_pos);
% U_pos(:,3) = U_pos(:,3) - g;
% X_ref = simOut.X_ref';
% pos_error = simOut.pos_error';
% att_error = simOut.att_error';
% att_error(3,:) = att_error(3,:) - [r_yaw, r_yaw(1)];
% 
% pos_tracking_cost = Ts_Sim * sum( sum( pos_error .* (Q_pos*pos_error), 1 ) );
% pos_control_cost = Ts_Sim  * sum( sum( U_pos .* (R_pos*U_pos), 1 ) );
% 
% att_tracking_cost = Ts_Sim * sum( sum( att_error .* (Q_att*att_error), 1 ) );
% att_control_cost = Ts_Sim  * sum( sum( U(2:4,:) .* (R_att*U(2:4,:)), 1 ) );
% 
% DDPG_results.DDPG_c_cost_total = pos_tracking_cost + pos_control_cost + att_control_cost + att_tracking_cost;
% 
% % save all useful info in a struck
% DDPG_results.X = X;
% DDPG_results.U = U;
% DDPG_results.X_ref = X_ref;
% DDPG_results.error = [pos_error;att_error];
% DDPG_results.simtime = DDPG_sim_time;
% DDPG_error = DDPG_results.error;
% DDPG_tracking_error = (DDPG_error(1,:).^2 + DDPG_error(2,:).^2 + DDPG_error(3,:).^2).^0.5;
% DDPG_results.DDPG_tracking_error = DDPG_tracking_error;
% DDPG_results.DDPG_tracking_error_norm = Ts_Sim*sum(DDPG_tracking_error,2);

X1 = squeeze(simOut.X);
X2 = squeeze(simOut.X2);
X3 = squeeze(simOut.X3);
X4 = squeeze(simOut.X4);

U1 = squeeze(simOut.U);
U2 = squeeze(simOut.U2);
U3 = squeeze(simOut.U3);
U4 = squeeze(simOut.U4);

X_ref = simOut.X_ref';

att_error  = squeeze(simOut.att_error');
pos_error  = simOut.pos_error';
att_error(3,:) = att_error(3,:) - [r_yaw, r_yaw(1)];

att_error2 = squeeze(simOut.att_error2');
pos_error2 = simOut.pos_error2';
att_error2(3,:) = att_error2(3,:) - [r_yaw, r_yaw(1)];

att_error3 = squeeze(simOut.att_error3');
pos_error3 = simOut.pos_error3';
att_error3(3,:) = att_error3(3,:) - [r_yaw, r_yaw(1)];

att_error4 = squeeze(simOut.att_error4');
pos_error4 = simOut.pos_error4';
att_error4(3,:) = att_error4(3,:) - [r_yaw, r_yaw(1)];

% Build proper 3D arrays: state/input/error x time x drone
X = cat(3, X1, X2, X3, X4);
U = cat(3, U1, U2, U3, U4);

Pos_error = cat(3, pos_error, pos_error2, pos_error3, pos_error4);
Att_error = cat(3, att_error, att_error2, att_error3, att_error4);
X_ref = cat(3, X_ref, X_ref, X_ref, X_ref);

% Save the 3D versions, not the vertically stacked 2D versions
DDPG_results.X = X;
DDPG_results.U = U;
DDPG_results.X1 = X1;
DDPG_results.X2 = X2;
DDPG_results.X3 = X3;
DDPG_results.X4 = X4;
DDPG_results.U1 = U1;
DDPG_results.U2 = U2;
DDPG_results.U3 = U3;
DDPG_results.U4 = U4;
Nd = size(X,3);
dt = Ts_Sim;
Nt = size(X,2);

tracking_error_xyz = zeros(Nt, Nd);
formation_error = zeros(3, Nt, Nd);
formation_error_norm = zeros(Nt, Nd);

for i = 1:Nt
    for d = 1:Nd
        tracking_error_xyz(i,d) = norm(Pos_error(1:3,i,d));
    end
    for d = 2:Nd
        formation_error(:,i,d) = X(1:3,i,d) - (X(1:3,i,1) + offsets(:,d));
        formation_error_norm(i,d) = norm(formation_error(:,i,d));
    end
end

tracking_error_xyz_norm = zeros(1,Nd);
formation_error_int = zeros(1,Nd);

for d = 1:Nd
    tracking_error_xyz_norm(d) = dt * sum(tracking_error_xyz(:,d));
    if d >= 2
        formation_error_int(d) = dt * sum(formation_error_norm(:,d));
    end
end

DDPG_results.simtime = DDPG_sim_time;
DDPG_results.tracking_error = tracking_error_xyz';
DDPG_results.tracking_error_norm = tracking_error_xyz_norm;
DDPG_results.formation_error = formation_error;
DDPG_results.formation_error_norm = formation_error_norm;
DDPG_results.formation_error_int = formation_error_int;
DDPG_results.X_ref = X_ref;
% DDPG_results.r_smooth1 = squeeze(X_ref(1:6,:,1));
% DDPG_results.r_smooth2 = squeeze(X_ref(1:6,:,2));
% DDPG_results.r_smooth3 = squeeze(X_ref(1:6,:,3));
% DDPG_results.r_smooth4 = squeeze(X_ref(1:6,:,4));

DDPG_results.time = T;
DDPG_results.offsets = offsets;


end

