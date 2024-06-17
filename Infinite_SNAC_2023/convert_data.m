clear; clc; close;

load('infinite_Position_SNAC_workspace2.mat','Position_W')
load('infinite_Attitude_SNAC_workspace2.mat','Attitude_W')

save("trained_weights","Attitude_W","Position_W","-v7.3")