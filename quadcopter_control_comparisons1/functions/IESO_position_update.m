function [ieso_state, d_hat_force, e_obs] = IESO_position_update(ieso_state, X, U_pos, m, dt, w0)
% IESO_POSITION_UPDATE
% 3-axis (x,y,z) 3rd-order ESO for: p_ddot = U_pos + d_f/m
% Estimates disturbance FORCE d_f (N) and returns observation error e_obs = p - p_hat.

% gains (bandwidth method)
beta1 = 3*w0;
beta2 = 3*w0^2;
beta3 = w0^3;

% measurements
p_meas = X(1:3);

% % init if needed
% if isempty(ieso_state)
%     ieso_state.p_hat = p_meas;
%     ieso_state.v_hat = X(4:6);
%     ieso_state.d_hat = zeros(3,1); % disturbance FORCE estimate (N)
% end

% observation error
e_obs = p_meas - ieso_state.p_hat;

% ESO dynamics (vector form)
% p_hat_dot = v_hat + beta1*e
% v_hat_dot = U_pos + (d_hat/m) + beta2*e
% d_hat_dot = beta3*m*e   (NOTE: d_hat is FORCE so multiply by m)
ieso_state.p_hat = ieso_state.p_hat + dt*(ieso_state.v_hat + beta1*e_obs);
ieso_state.v_hat = ieso_state.v_hat + dt*(U_pos + (ieso_state.d_hat./m) + beta2*e_obs);
ieso_state.d_hat = ieso_state.d_hat + dt*(beta3*m*e_obs);

d_hat_force = ieso_state.d_hat;
end