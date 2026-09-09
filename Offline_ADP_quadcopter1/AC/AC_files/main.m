clc; clear; close all;


%% Simulation Parameters
timeFinal = 50;

% Trajectory
fx = @(t) (1-exp(-0.01*t)).*9.81.*cos(0.2*t); % [m]
fy = @(t) (1-exp(-0.01*t)).*9.81.*sin(0.2*t); % [m]
fz = @(t) -.1*t;        % [m]
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t];
Nsims = 1;

%% Run Controller
simIter = cell(1, Nsims);
gtic = tic;

for si = 1:Nsims
  iniPos = [5 5 0]; % [m]
  iniVel = [0 0 0]; % [m/s]

  sim = controlller(timeFinal, iniPos, iniVel, fx, fy, fz);

  simIter{si} = sim;
end

gtoc = toc(gtic);
fprintf("\nSimulations are done!\n%0.4f s  (%0.4f min)\n", gtoc, gtoc/60)

%% Plot Results
plotter(simIter)
