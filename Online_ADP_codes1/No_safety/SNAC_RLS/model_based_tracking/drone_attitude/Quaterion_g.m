function out = Quaterion_g(x,Ix, Iy, Iz)

out = [ ...
  0 0 0;    % q0_dot has no direct torque input
  0 0 0;    % q1_dot
  0 0 0;    % q2_dot
  0 0 0;    % q3_dot
  1/Ix 0 0; % p_dot input τx
  0 1/Iy 0; % q_dot input τy
  0 0 1/Iz  % r_dot input τz
];

end

% out = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];