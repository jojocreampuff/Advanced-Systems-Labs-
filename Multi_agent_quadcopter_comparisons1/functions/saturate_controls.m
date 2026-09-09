%% saturation function
function out = saturate_controls(U)
    ft = U(1);
    torques = [U(2); U(3); U(4)];
    m = 1;
    g = 9.81;
    L = 0.2;
    drag = 0.02; % (drag coef)
    max_Ft = 2*m*g;
    Tmin = 0;
    Tmax = max_Ft/4;
    max_torque = Tmax*L*2;
    max_yaw_torque = 0.7*max_torque;

%% saturate the torques to the max torque the drone can produce
    for p = 1:2 % sat the first 2 
        if torques(p) > max_torque
            torques(p) = max_torque;
        elseif torques(p) < -max_torque
            torques(p) = -max_torque;
        end
    end
    if torques(3) > max_yaw_torque
        torques(3) = max_yaw_torque;
    elseif torques(3) < -max_yaw_torque
        torques(3) = -max_yaw_torque;
    end
    % solve for the thrust of each motor given those torques    
    T1 = (ft - torques(1)/(2*L) - torques(2)/(2*L) + torques(3)/(4*drag)) / 4;
    T2 = (ft - torques(1)/(2*L) + torques(2)/(2*L) + torques(3)/(4*drag)) / 4;
    T3 = (ft + torques(1)/(2*L) - torques(2)/(2*L) - torques(3)/(4*drag)) / 4;
    T4 = (ft + torques(1)/(2*L) + torques(2)/(2*L) - torques(3)/(4*drag)) / 4;
    each_motor_thrust = [T1;T2;T3;T4];

    % saturate each motor thrust to its bounded values
    for j = 1:4
        if each_motor_thrust(j) > Tmax
            each_motor_thrust(j) = Tmax;
        elseif each_motor_thrust(j) < Tmin
            each_motor_thrust(j) = Tmin;
        end
    end
    
    ft = sum(each_motor_thrust);

    if ft>max_Ft
        ft = max_Ft;
    elseif ft<0
        ft = 0;
    end

    out = [ft; torques];