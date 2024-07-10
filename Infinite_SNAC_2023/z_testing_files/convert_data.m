

% data_thing = load('trained_weights.mat','Position_W', "Attitude_W")

% Extract and convert the matrices to double

% Position_W = single(data_thing.Position_W);
% Attitude_W = single(data_thing.Attitude_W);

Position_W = single(Position_W);
Attitude_W = single(Attitude_W);

save("trained_weights","Attitude_W","Position_W","-v7.3")



save("z_reference","r_smooth","r_initial","-v7.3")
data = load("z_reference.mat", "r_smooth");

ref_states = single(r_smooth.r_smooth_1);

save("reference_SITL","ref_states","-v7.3")
