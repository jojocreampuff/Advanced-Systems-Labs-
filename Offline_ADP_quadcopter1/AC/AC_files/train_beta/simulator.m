clc; clear; close all; addpath(genpath('../')); addpath(genpath('basis_funcs'))

simTime = 10;
NSim = 5;

fx = @(t) t*0;
fy = @(t) t*0;
fz = @(t) t*0;

constants;
NTime = simTime/timeStep;
t = (1:NTime)*timeStep;

NStates = 6;
NInputs = 3;

Fx = fx(t);
Fy = fy(t);
Fz = fz(t);

Fx_d = getDDerivative(Fx);
Fy_d = getDDerivative(Fy);
Fz_d = getDDerivative(Fz);


sr = [Fx; Fy; Fz; Fx_d; Fy_d; Fz_d];

sl = cell(NSim, 1);
ul = cell(NSim, 1);

for i = 1:NSim
    s0 = 2*rand(1, NStates) - 1;

    [sk, uk] = betaPerformer(s0, sr);

    sl{i} = sk;
    ul{i} = uk;
end

plotNames = ["PHI", "THE", "PSI", "P", "Q", "R"];
figure;
for i = 1:NStates
    subplot(3, 2, o(i)); hold on; grid on;

    plot(t, sr(i, :), 'g--')
    for k = 1:NSim
        sk = sl{k};
      plot(t, sk(i, :))
    end
    
    xlabel('Time (s)')
    ylabel(plotNames(i))
end
subplot(3, 2, 1); legend("Reference Path");

plotNames = ["B1", "B2", "B3"];
figure;
for i = 1:NInputs
    subplot(3, 1, i); hold on; grid on;

    for k = 1:NSim
        uk = ul{k};
      plot(t, uk(i, :))
    end

    xlabel('Time (s)')
    ylabel(plotNames(i))
end


function y = o(x)
  t = [1 3 5 2 4 6];
  y = t(x);
end