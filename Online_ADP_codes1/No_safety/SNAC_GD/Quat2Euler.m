function x = Quat2Euler(xQ)
% xQ : [q0; q1; q2; q3; p; q; r]   (scalar-first)
% x  : [phi; theta; psi; p; q; r]  (roll, pitch, yaw)

q0 = xQ(1); q1 = xQ(2); q2 = xQ(3); q3 = xQ(4);
p  = xQ(5); q  = xQ(6); r  = xQ(7);

% normalize to be safe
nq = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0 = q0/nq; q1 = q1/nq; q2 = q2/nq; q3 = q3/nq;

% ZYX (yaw-pitch-roll)
% psi (yaw)
siny_cosp = 2*(q0*q3 + q1*q2);
cosy_cosp = 1 - 2*(q2*q2 + q3*q3);
psi = atan2(siny_cosp, cosy_cosp);

% theta (pitch)
sin_theta = 2*(q0*q2 - q3*q1);
sin_theta = max(-1, min(1, sin_theta));  % clamp for numerical safety
theta = asin(sin_theta);

% phi (roll)
sinx_cosp = 2*(q0*q1 + q2*q3);
cosx_cosp = 1 - 2*(q1*q1 + q2*q2);
phi = atan2(sinx_cosp, cosx_cosp);

% wrap to [-pi, pi] for controller consistency
phi   = wrapToPi(phi);
theta = wrapToPi(theta);
psi   = wrapToPi(psi);

x = [phi; theta; psi; p; q; r];
end