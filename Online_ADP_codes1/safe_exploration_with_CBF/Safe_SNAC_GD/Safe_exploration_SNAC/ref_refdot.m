function [ref, ref_dot] = ref_refdot(t)
    ohm = .5*pi;
    amp = 1;
    ref         = [amp*sin(ohm*t);       amp*ohm*cos(ohm*t)];           % [r, r_dot]'
    ref_dot     = [amp*ohm*cos(ohm*t);   -(ohm)^2*amp*sin(ohm*t)];      % [r_dot, r_ddot]'
end