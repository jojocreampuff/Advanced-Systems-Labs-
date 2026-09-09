function acceleration_global=acceleration_conversion(acceleration_change_drone,yaw)
% Inputs= Acceleration in x y z directions without applying yaw
% Outputs= Acceleration in x y z directions after applying yaw (global frame)

% Calculate magnitude of acceleration in x y directions
r=sqrt(acceleration_change_drone(1)^2+acceleration_change_drone(2)^2);          
% Calculate the angle to describe the initial acceleration vector
omega = atan2(acceleration_change_drone(1), acceleration_change_drone(2));
% Use yaw angle and calculated angle to find the magnitude in the global x and y directions
x_global=sin(omega-yaw)*r;
y_global=cos(omega-yaw)*r;
% Create the global acceleration vector
acceleration_global=[x_global;y_global;acceleration_change_drone(3)];