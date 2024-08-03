

% data_thing = load('trained_weights.mat','Position_W', "Attitude_W")

% Extract and convert the matrices to double

% Position_W = single(data_thing.Position_W);
% Attitude_W = single(data_thing.Attitude_W);
%% turn weights into single and save them as v7.3
Position_W = single(Position_W);
Attitude_W = single(Attitude_W);

save("trained_weights","Attitude_W","Position_W","-v7.3")



% save("z_reference","r_smooth","r_initial","-v7.3")
% data = load("z_reference.mat", "r_smooth");

%% save trajectory as single
ref_states = single(r_smooth.r_smooth_1);

save("reference_SITL","ref_states","-v7.3")

%% save the open loop controls
F_tx_ty_tz = single(u.u_1);

 % F_tx_ty_tz(1,44:77) = 35

u1_u2_u3 = single(uxyz.uxyz_1);

save("open_loop_controls", "u1_u2_u3","F_tx_ty_tz","-v7.3")


