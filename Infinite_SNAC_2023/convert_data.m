clear; clc; close;

data_thing = load('trained_weights.mat','Position_W', "Attitude_W")

% Extract and convert the matrices to double
Position_W = double(data_thing.Position_W);
Attitude_W = double(data_thing.Attitude_W);

% save("trained_weights","Attitude_W","Position_W","-v7.3")