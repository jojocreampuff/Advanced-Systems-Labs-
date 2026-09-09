function [eso_att_state, d_hat_att, e_obs_att] = IESO_attitude_update( ...
    eso_att_state, X, tau_prev, f_att_rate, g_att_rate, dt, w0_att)
% IESO_ATTITUDE_UPDATE
% 3-axis ESO on body-rate dynamics:
%   omega_dot = f(omega) + g*tau + d_att
% Estimates d_att in rad/s^2.
%
% Inputs:
%   eso_att_state : struct with fields omega_hat (3x1), d_hat (3x1)
%   X             : current state (12x1)
%   tau_prev      : previously applied torque (3x1)
%   f_att_rate    : function handle f(omega)
%   g_att_rate    : 3x3 input matrix (diag(1/Ix,1/Iy,1/Iz))
%   dt            : timestep
%   w0_att        : observer bandwidth (e.g. 20-60 depending on dt)
%
% Outputs:
%   d_hat_att     : estimated disturbance angular acceleration (rad/s^2)
%   e_obs_att     : observation error (omega - omega_hat)

% ESO gains (bandwidth method)
beta1 = 2*w0_att;
beta2 = w0_att^2;

% measured body rates
omega_meas = X(10:12);

% % init safety
% if isempty(eso_att_state) || ~isfield(eso_att_state,'omega_hat')
%     eso_att_state.omega_hat = omega_meas;
%     eso_att_state.d_hat = zeros(3,1);
% end

% observation error
e_obs_att = omega_meas - eso_att_state.omega_hat;

% nominal omega_dot from previous applied torque
omega_hat_dot_nom = f_att_rate(eso_att_state.omega_hat) + g_att_rate * tau_prev;

% 2nd-order ESO (sufficient for rate dynamics)
% omega_hat_dot = omega_dot_nom + d_hat + beta1*e
% d_hat_dot     = beta2*e
eso_att_state.omega_hat = eso_att_state.omega_hat + dt * (omega_hat_dot_nom + eso_att_state.d_hat + beta1*e_obs_att);
eso_att_state.d_hat     = eso_att_state.d_hat     + dt * (beta2 * e_obs_att);

d_hat_att = eso_att_state.d_hat;
end