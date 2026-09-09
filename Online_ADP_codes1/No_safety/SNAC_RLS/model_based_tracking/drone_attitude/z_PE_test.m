
clear; clc;
T = .01:0.01:50;


for i = 1:length(T)

    u(:,i) = u_excite_sq(T(i),.3) + u_excite(T(i),.3);

end

figure; 
subplot(3,1,1);
plot(u(1,:))
subplot(3,1,2);
plot(u(2,:))
subplot(3,1,3);
plot(u(3,:))