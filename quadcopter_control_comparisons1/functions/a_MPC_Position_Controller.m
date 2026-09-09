function [pos_control_guess,J_current]=a_MPC_Position_Controller(pos_ref_horizon, X_current, pos_control_guess, Hp, Q, R, dt, pos_iter, decay_rate, alpha, epsilon)
% pos_ref_horizon: the posititon reference in x, y, z, u, v, w for the next Hp timesteps
% X_current: the current state x, y, z, u, v, w
% pos_control_guess: an initial guess on the desired acceleration given as zeros(3,Hp) + [0;0;9.81]
% Hp: the number of timestep in the horizon 
% Q and R: cost function parameters
% Jn: the terminal cost function handle, its x'Qx + u'Ru
% dt: timestep
% pos_iter: the iterations in gradient decent (GD) for the optimization
% decay_rate: is the GD learning rate recay factor
% alpha: is the GD learning rate
% epsilon: the ammount the control is to be perturbed to solving GD (open to this being analyitically done)

f_pos = @(x) [x(4); 
    x(5); 
    x(6); 
    0; 
    0; 
    9.81];

df_dx = [0 0 0 1 0 0
        0 0 0 0 1 0
        0 0 0 0 0 1
        0 0 0 0 0 0
        0 0 0 0 0 0
        0 0 0 0 0 0];

g_pos = [0 0 0; 
    0 0 0; 
    0 0 0; 
    -1 0 0; 
    0 -1 0; 
    0 0 -1];

Q_flat = [Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6)]';
R_flat = [R(1,1), R(2,2), R(3,3)]';

for k = 1:pos_iter

    X = zeros(6,Hp+1);
    X(:,1) = X_current; % start at the current state
    for j = 1:Hp % prop states based on current control guess
        X_dot = f_pos(X(:,j)) + g_pos*pos_control_guess(:,j);
        X(:,j+1) = X(:,j) + dt*X_dot;
    end
    e = X - pos_ref_horizon;
    J_current = dt*(sum(sum(e(:,1:Hp).^2.*Q_flat)) + sum(sum(pos_control_guess.^2.*R_flat))) + e(:,Hp+1)' * Q * e(:,Hp+1);

    lambda = zeros(6,Hp+1);
    grad   = zeros(3,Hp);
    lambda(:,Hp+1) = 2*Q*e(:,Hp+1);
    A = eye(6) + dt *df_dx;
    for j = Hp:-1:1
        lambda(:,j) = 2*dt*Q*e(:,j) + A'*lambda(:,j+1);
        grad(:,j) = 2*dt*R*pos_control_guess(:,j) + (dt*g_pos') * lambda(:,j+1);
    end

    % Gradient update
    alpha_t = alpha / (1 + decay_rate * k);    
    pos_control_guess(:,1:Hp) = pos_control_guess(:,1:Hp) - alpha_t * grad;
end

end