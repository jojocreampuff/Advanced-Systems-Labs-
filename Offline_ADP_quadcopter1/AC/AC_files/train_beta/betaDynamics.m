addpath(genpath('../')); addpath(genpath('basis_funcs'))
constants

% Enums
PHI = 1; P = 4;
THE = 2; Q = 5;
PSI = 3; R = 6;

TAUX = 1;
TAUY = 2;
TAUZ = 3;


% Dynamics
ze = @(x) zeros(1, size(x, 2)); on = @(x) ones(1, size(x, 2));


    
% Vectorized form of the f and g function, used for training
f_hat = @(x) x + timeStep*[
    x(P, :) + x(R, :).*(c(x(PHI, :)).*t(x(THE, :))) + x(Q, :).*(s(x(PHI, :)).*t(x(THE, :)))  % PHI
    x(Q, :).*(c(x(PHI, :))) - x(R, :).*(s(x(PHI, :)))                                        % THE
    x(R, :).*(c(x(PHI, :))./c(x(THE, :))) + x(Q, :).*(s(x(PHI, :))./c(x(THE, :)))            % PSI
    (Iy - Iz)/Ix*x(Q, :).*x(R, :)  % P
    (Iz - Ix)/Iy*x(P, :).*x(R, :)  % Q
    (Ix - Iy)/Iz*x(P, :).*x(Q, :)  % R

];

%                            PHI     THE     PSI         P          Q          R 
g_hat_1 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);  on(s)/Ix;     ze(s);     ze(s)];  % TAUX
g_hat_2 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);     ze(s);  on(s)/Iy;     ze(s)];  % TAUY
g_hat_3 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);     ze(s);     ze(s);  on(s)/Iz];  % TAUZ

g_hat = @(s) [g_hat_1(s)'; g_hat_2(s)'; g_hat_3(s)']';

mod_sim   = @(x, u) f_hat(x) + g_hat(x)*u;
model_sim = @(x, u, k) mod_sim(x(:, k), u(:, k));