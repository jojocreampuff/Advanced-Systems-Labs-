function [eso_state, d_hat_pos] = ESO_position(eso_state, X, U_applied, m, dt, w0)
% ESO_POSITION_UPDATE
% Translational Extended State Observer for quadcopter position dynamics.
%
% Inputs:
%   eso_state  : struct with fields:
%                   p_hat (3x1)
%                   v_hat (3x1)
%                   d_hat (3x1)
%   X          : current state vector (12x1)
%   U_applied  : applied control AFTER saturation [ft; tau_x; tau_y; tau_z]
%   m, g       : mass and gravity
%   dt         : timestep
%   w0         : observer bandwidth (5–15 typical)
%
% Outputs:
%   eso_state  : updated ESO state
%   d_hat_pos  : estimated disturbance acceleration (3x1)

    % ESO gains (standard 3rd-order form)
    beta1 = 3*w0;
    beta2 = 3*w0^2;
    beta3 = w0^3;

    % Measured position
    p_meas = X(1:3);

    % Attitude
    phi   = X(7);
    theta = X(8);
    psi   = X(9);

    % Rotation matrix (body → inertial)
    R = eul2rotm_zyx(phi, theta, psi);

    % Nominal acceleration from applied thrust (ft already has g covered)
    ft = U_applied(1);
    a_nom = -(1/m) * (R * [0;0;ft]);%; + [0;0;9.81] ;

    % Position error for observer
    e_p = p_meas - eso_state.p_hat;

    % ESO update (Euler discretization)
    eso_state.p_hat = eso_state.p_hat + dt * (eso_state.v_hat + beta1 * e_p);
    eso_state.v_hat = eso_state.v_hat + dt * (a_nom + eso_state.d_hat + beta2 * e_p);
    eso_state.d_hat = eso_state.d_hat + dt * (beta3 * e_p);

    % Output disturbance estimate
    d_hat_pos = eso_state.d_hat;
end