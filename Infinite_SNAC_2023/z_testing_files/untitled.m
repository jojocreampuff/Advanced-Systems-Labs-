clear; clc;
dt = 0.001;
N = round(100/dt);
F = 50;
phase_1 = pi/4;
excite = zeros(1,N);
time = 0:dt:(N-1)*dt;  
tic

for i = 1:length(time)

    for f = 1:F

        excite(1,i) = .01*exp(-.001*time(i))*sin( rand(1,1)*f*time(i) + phase_1) + excite(1,i); 

    end
    
end

toc

plot(time, excite); 
xlabel('Time (s)');
ylabel('Excitation');
title('Excitation Signal');

