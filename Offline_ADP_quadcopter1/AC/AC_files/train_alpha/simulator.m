 clc; clear; close all; addpath(genpath('../')); addpath(genpath('basis_funcs'))

simTime = 10;
NSim = 5;

fx = @(t) 0*sin(0.5*t);
fy = @(t) 0*cos(0.5*t);
fz = @(t) 0*t;

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
al = cell(NSim, 1);

for i = 1:NSim
    s0 = 2*rand(1, NStates) - 1;

    [sk, uk] = alphaPerformer(s0, sr);

    sl{i} = sk;
    ul{i} = uk;
end

plotNames = ["X", "Y", "Z", "U", "V", "W"];
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

plotNames = ["A1", "A2", "A3"];
figure;
for i = 1:NInputs
    subplot(3, 1, i); hold on; grid on;

    for k = 1:NSim
      uk = ul{k};
      al{k} = getAlpha(uk(1, :),uk(2, :), uk(3, :));
      plot(t, uk(i, :))
    end

    xlabel('Time (s)')
    ylabel(plotNames(i))
end


figure; hold on; grid on;

for k = 1:NSim
    ak = al{k};
    plot(t, ak(1, :))
end

xlabel('Time (s)')
ylabel('FT')


plotNames = ["PHI", "THE", "PSI", "P", "Q", "R"];
figure;
for i = 1:NStates
    subplot(3, 2, o(i)); hold on; grid on;

    for k = 1:NSim
      ak = al{k};
      plot(t, ak(i + 1, :))
    end

    xlabel('Time (s)')
    ylabel(plotNames(i))
end


function y = o(x)
  t = [1 3 5 2 4 6];
  y = t(x);
end