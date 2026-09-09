function noisy_vector = add_control_noise(state_vector, noise_percent)
    L = 0.2;
    max_Ft = 2*9.81;
    Tmax = max_Ft/4;
    Max_torque = Tmax*L*2;
    actuator_uncert = [0.1*max_Ft, 0.01*Max_torque, 0.01*Max_torque, 0.02*Max_torque]';
    
    noise = (noise_percent) * actuator_uncert .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end
