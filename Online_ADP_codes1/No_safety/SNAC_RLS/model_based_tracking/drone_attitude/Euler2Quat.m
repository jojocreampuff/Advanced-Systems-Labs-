function xQ = Euler2Quat(x)
% x    : [phi; theta; psi; p; q; r]    (roll, pitch, yaw)
% xQ   : [q0; q1; q2; q3; p; q; r]     (scalar-first quaternion)

phi   = x(1); theta = x(2); psi = x(3);
p     = x(4); q     = x(5); r   = x(6);

c1 = cos(phi/2);  s1 = sin(phi/2);   % roll
c2 = cos(theta/2);s2 = sin(theta/2); % pitch
c3 = cos(psi/2);  s3 = sin(psi/2);   % yaw

% ZYX (yaw-pitch-roll): R = Rz(psi)*Ry(theta)*Rx(phi)
q0 = c1*c2*c3 + s1*s2*s3;
q1 = s1*c2*c3 - c1*s2*s3;
q2 = c1*s2*c3 + s1*c2*s3;
q3 = c1*c2*s3 - s1*s2*c3;

qv = [q0; q1; q2; q3];
qv = qv / norm(qv);

xQ = [qv; p; q; r];
end