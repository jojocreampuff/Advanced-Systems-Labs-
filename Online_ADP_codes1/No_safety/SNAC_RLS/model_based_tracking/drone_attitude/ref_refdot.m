function [ref, ref_dot] = ref_refdot(t)
%% crown path
% ohm = 0.5;
% amp_x = .15;
% amp_y = .15;
% amp_z = .1;
% z = 0;
% 
%  ref =    [amp_x*cos(ohm*t);          amp_y*sin(ohm*t);         amp_z*sin(.5*ohm*t)-z
%           -amp_x*ohm*sin(ohm*t);      amp_y*ohm*cos(ohm*t);     .5*amp_z*ohm*cos(.5*ohm*t)];
% 
%  ref_dot = [-amp_x*ohm*sin(ohm*t);    amp_y*ohm*cos(ohm*t);     .25*amp_z*ohm*cos(.5*ohm*t);
%            -amp_x*ohm^2*cos(ohm*t);    -amp_y*ohm^2*sin(ohm*t);   -.125*amp_z*ohm^2*sin(.5*ohm*t)];

ohm = 0.5;
amp_x = pi/4;
amp_y = pi/4;
z = .75;

 ref =    [amp_x*cos(ohm*t);          amp_y*sin(ohm*t);         z
          -amp_x*ohm*sin(ohm*t);      amp_y*ohm*cos(ohm*t);     0];

 ref_dot = [-amp_x*ohm*sin(ohm*t);    amp_y*ohm*cos(ohm*t);     0;
           -amp_x*ohm^2*cos(ohm*t);    -amp_y*ohm^2*sin(ohm*t);   0];



end