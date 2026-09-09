function [ref, ref_dot] = ref_refdot(t)
%% crown path
% ohm = 0.5;
% amp = 5;
% amp_z = 1;
% z = 5;
% 
%  ref =    [amp*cos(ohm*t);          amp*sin(ohm*t);         amp_z*sin(2*ohm*t)-z
%           -amp*ohm*sin(ohm*t);      amp*ohm*cos(ohm*t);     2*amp_z*ohm*cos(2*ohm*t)];
% 
%  ref_dot = [-amp*ohm*sin(ohm*t);    amp*ohm*cos(ohm*t);     2*amp_z*ohm*cos(2*ohm*t);
%            -amp*ohm^2*cos(ohm*t);    -amp*ohm^2*sin(ohm*t);   -4*amp_z*ohm^2*sin(2*ohm*t)];

 %% circle path x y
ohm = 0.5;
amp = 5;
amp_z = 1;
z = 5;

 ref =    [amp*cos(ohm*t);          amp*sin(ohm*t);         -z
          -amp*ohm*sin(ohm*t);      amp*ohm*cos(ohm*t);     0];

 ref_dot = [-amp*ohm*sin(ohm*t);    amp*ohm*cos(ohm*t);     0;
           -amp*ohm^2*cos(ohm*t);    -amp*ohm^2*sin(ohm*t);   0];

end