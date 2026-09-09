function out = Quaterion_f(x,Ix,Iy,Iz)
q = x(1:4);  w = x(5:7);

% raw kinematics (scalar-first)
qdot = 0.5 * [ ...
   -q(2)*w(1) - q(3)*w(2) - q(4)*w(3);
    q(1)*w(1) + q(3)*w(3) - q(4)*w(2);
    q(1)*w(2) + q(4)*w(1) - q(2)*w(3);
    q(1)*w(3) + q(2)*w(2) - q(3)*w(1)];

% keep motion on the unit sphere (smooth, no if-statements)
qdot = qdot - (q.'*qdot)*q;                % tangent projection
lambda = 5;                                % 5–20 works; tune
qdot = qdot - lambda*(q.'*q - 1)*q;         % Baumgarte stabilization

p = w(1); qb = w(2); r = w(3);
pdot  = ((Iy - Iz)/Ix) * qb * r;
qbdot = ((Iz - Ix)/Iy) * p  * r;
rdot  = ((Ix - Iy)/Iz) * p  * qb;

out = [qdot; pdot; qbdot; rdot];
end
% function out = Quaterion_f(x)
% Ix = 0.3;
% Iy = 0.4;
% Iz = 0.5;
% m = 1; grav = 9.81;
% Q0 = x(1);
% Q1 = x(2);
% Q2 = x(3);
% Q3 = x(4);
% Q  = [Q0;Q1;Q2;Q3];  % just in case
% Q  = [Q0;Q1;Q2;Q3] / norm([Q0;Q1;Q2;Q3]);  % just in case
% 
% xQ = [Q ; x(5) ; x(6) ; x(7)];
% 
% out = [ ...
%   0.5 * ( -xQ(2)*xQ(5) - xQ(3)*xQ(6) - xQ(4)*xQ(7) );   % q0_dot
%   0.5 * (  xQ(1)*xQ(5) + xQ(3)*xQ(7) - xQ(4)*xQ(6) );   % q1_dot
%   0.5 * (  xQ(1)*xQ(6) + xQ(4)*xQ(5) - xQ(2)*xQ(7) );   % q2_dot
%   0.5 * (  xQ(1)*xQ(7) + xQ(2)*xQ(6) - xQ(3)*xQ(5) );   % q3_dot
%   ((Iy - Iz) / Ix) * xQ(6) * xQ(7);                 % p_dot
%   ((Iz - Ix) / Iy) * xQ(5) * xQ(7);                 % q_dot
%   ((Ix - Iy) / Iz) * xQ(5) * xQ(6)                  % r_dot
% ];
% 
% end