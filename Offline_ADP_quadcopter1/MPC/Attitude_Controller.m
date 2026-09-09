function [torques,angle_error_old,integral_error]=Attitude_Controller(angles_ref,angles,angle_error_old,m,angles_history,angles_ref_history,integral_error,Kp,Kd,Ki,dt)
% Outputs torque values and error values required for the next call of the function

angle_error = angles_ref - angles;    % Error between desired and current angles

if m==1
    % For the first iteration, set the derivative error to zero
    derivative_error = zeros(3, 1);
else
    % For subsequent iterations, calculate the derivative of the error
    derivative_error = (angle_error - angle_error_old) / (dt / 10);
end
if m>100
    integral_error= integral_error+(angles_ref_history(:,m-1)-angles_history(:, m-1)-angles_ref_history(:,m-100)-angles_history(:, m-100))*dt;
else
    integral_error=0;
end
% PID torque calculation
torques = Kp .* angle_error  + Kd .* derivative_error+Ki.*integral_error;
torques = min(max(torques, -30), 30);  % Cap the torques
angle_error_old=angle_error;