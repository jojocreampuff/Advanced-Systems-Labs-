function noisy_vector = add_state_noise(state_vector, noise_percent)

    sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
    
    noise = (noise_percent) * sensor_uncert .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end