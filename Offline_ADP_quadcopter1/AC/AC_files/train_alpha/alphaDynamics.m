addpath(genpath('../')); addpath(genpath('basis_funcs'))
constants

% Enums
X = 1; U = 4;
Y = 2; V = 5;
Z = 3; W = 6;

A1 = 1;
A2 = 2;
A3 = 3;


% Dynamics
ze = @(x) zeros(1, size(x, 2)); on = @(x) ones(1, size(x, 2));

    
% Vectorized form of the f and g function, used for training
f_hat = @(s) s + timeStep*[
    s(U, :)     % X
    s(V, :)     % Y
    s(W, :)     % Z
    ze(s)       % XD
    ze(s)       % YD
    ze(s)       % ZD
];

%                             X        Y      Z       U       V       W 
g_hat_1 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);  on(s);  ze(s);  ze(s)];  % A1
g_hat_2 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);  ze(s);  on(s);  ze(s)];  % A2
g_hat_3 = @(s) timeStep*[  ze(s);  ze(s);  ze(s);  ze(s);  ze(s);  on(s)];  % A3

g_hat = @(s) [g_hat_1(s)'; g_hat_2(s)'; g_hat_3(s)']';

mod_sim   = @(x, k) f_hat(x) + g_hat(x)*k;
model_sim = @(x, u, k) mod_sim(x(:, k), u(:, k));