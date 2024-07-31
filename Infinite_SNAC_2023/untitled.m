
T = [10,10,-10]

saturation(T)
    function torques = saturation(torques)
    Max_torque = 8.64;
% based on max actuator output
        if torques(1) > Max_torque
            torques(1) = Max_torque;
        elseif torques(1) < -Max_torque
            torques(1) = -Max_torque;
        end

        if torques(2) > Max_torque
            torques(2) = Max_torque;
        elseif torques(2) < -Max_torque
            torques(2) = -Max_torque;
        end

        if torques(3) > 5
            torques(3) = 5;
        elseif torques(3) < -5
            torques(3) = -5;
        end

    end