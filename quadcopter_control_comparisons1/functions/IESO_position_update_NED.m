function [ieso_state, d_hat_force, e_obs] = IESO_position_update_NED_force(ieso_state, X, U_pos, m, dt, w0)
% IESO_POSITION_UPDATE_NED_FORCE
% Improved/Extended State Observer for POSITION in NED (North-East-Down),
% written in the SAME "force-disturbance" form as the SMC IESO version.
%
% NED convention:
%   p = [N; E; D]   (Down positive)
%   v = [vN; vE; vD]
%
% Observer model (force-disturbance form, matches SMC IESO style):
%   p_dot = v
%   v_dot = U_pos + d_f/m
% where:
%   U_pos is the commanded inertial acceleration EXCLUDING gravity (m/s^2) in NED
%   d_f is the lumped disturbance FORCE in NED (N)
%
% Outputs:
%   d_hat_force : estimated disturbance force (N) in NED
%   e_obs       : observation error p_meas - p_hat
%
% IMPORTANT:
%   - DO NOT include gravity in U_pos. Gravity is handled later in your
%     thrust/attitude mapping (system_solve).
%   - X(1:3), X(4:6), U_pos must all be in NED.

% 3rd-order ESO gains (bandwidth method)
beta1 = 3*w0;
beta2 = 3*w0^2;
beta3 = w0^3;

% Measurements (NED)
p_meas = X(1:3);

% Observation error
e_obs = p_meas - ieso_state.p_hat;

% ESO update (Euler discretization)
% p_hat_dot = v_hat + beta1*e
% v_hat_dot = U_pos + (d_hat/m) + beta2*e
% d_hat_dot = beta3*m*e       (because d_hat is FORCE)
ieso_state.p_hat = ieso_state.p_hat + dt*(ieso_state.v_hat + beta1*e_obs);
ieso_state.v_hat = ieso_state.v_hat + dt*(U_pos + (ieso_state.d_hat./m) + beta2*e_obs);
ieso_state.d_hat = ieso_state.d_hat + dt*(beta3*m*e_obs);

% Output (force disturbance estimate)
d_hat_force = ieso_state.d_hat;

end