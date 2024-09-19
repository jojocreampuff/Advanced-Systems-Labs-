
%% for smol jmavsim drone std
control = u.u_1;
ft_std = std(control(1,:));
tau_x_std = std(control(2,:));
tau_y_std = std(control(3,:));
tau_z_std = std(control(4,:));

control_std = [ft_std, tau_x_std, tau_y_std, tau_z_std]

states = x.x_1;
for i= 1:length(states(:,1))
    states_std(:,i) = std(states(i,:));

end
states_std