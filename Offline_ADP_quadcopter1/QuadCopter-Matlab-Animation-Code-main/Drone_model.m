close all;
clear
clc
 
 %% 1. define the motion coordinates 

load("Drone_shielding.mat")
% X = DCS_ref1_Unsafe.X;
% X_ref = DCS_ref1_Unsafe.X_ref;
X = DCS_ref1_safe.X;
X_ref = DCS_ref1_safe.X_ref;

states = X;
ref = X_ref;
step = 15;
x = states(1,:);
y = states(2,:);
z = -states(3,:);
pitch = states(7,:);
roll = states(8,:);
yaw = states(9,:);
cut = 1000;
x = x(1:step:end-cut);
y = y(1:step:end-cut);
z = z(1:step:end-cut);
pitch = pitch(1:step:end-cut);
roll = roll(1:step:end-cut);
yaw = yaw(1:step:end-cut);

r_x = ref(1,:);
r_y = ref(2,:);
r_z = ref(3,:);
r_x = r_x(1:step:end);
r_y = r_y(1:step:end);
r_z = r_z(1:step:end);



 %% 6. animate by using the function makehgtform
 % Function for ANimation of QuadCopter
moveVector = drone_Animation(x,y,z,roll,pitch,yaw, r_x, r_y, r_z);
 
 
 %% step5: Save the movie
v = VideoWriter('drone_animation', 'Motion JPEG AVI');
% myWriter = VideoWriter('drone_animation1', 'MPEG-4');
v.Quality = 100;
v.FrameRate = 30;
% Open the VideoWriter object, write the movie, and class the file
open(v);
writeVideo(v, moveVector);
close(v); 