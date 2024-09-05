% JITENDRA SINGH
% India 
% this code written for making the animation of Quadcoptor model, all units
% are in meters, in this code of example we are using 'HGtransform'
% function for animate the trajectory of quadcopter
% {Thanks to MATLAB}
close all;
clear
clc
 
 %% 1. define the motion coordinates 
 load("Test_simulation.mat")

    tt    = 0:0.004:6;
    z     = tt/2;
    y     = sin(2*tt);
    x     = cos(2*tt);
    yaw   = 1.2*tt;
    roll  = 5*sin(5*tt);
    pitch = 5*cos(5*tt);
% states = states2;
step = 10;
x = states(1,:);
y = states(2,:);
z = -states(3,:);
pitch = states(7,:);
roll = states(8,:);
yaw = states(9,:);
x = x(1:step:end);
y = y(1:step:end);
z = z(1:step:end);
pitch = pitch(1:step:end);
roll = roll(1:step:end);
yaw = yaw(1:step:end);

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
v.FrameRate = 60;
% Open the VideoWriter object, write the movie, and class the file
open(v);
writeVideo(v, moveVector);
close(v); 