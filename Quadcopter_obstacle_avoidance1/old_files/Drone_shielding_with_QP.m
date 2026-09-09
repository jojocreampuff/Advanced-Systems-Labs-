function [U_pos, h, delta] = Drone_shielding_with_QP(u_nom, x_pos, pos_error,W_pos,t)
%% CLF-CBF-QP Tracking for double integrator with input bounds
% Barrier: Bbar = -log( h/(1+h) )  (logarithmic reciprocal barrier)
% Constraints:
%% CLF (soft): LfV + LgV*u + c*V - delta <= 0
    % DQ: format [LgV, -1]*[u; delta] <= -LfV - c*V
    %            A_clf * U  < = B_clf
%% CBF (hard): LfB + LgB*u - gamma/Bbar <= 0 <--- first order barrier method
% second order (relative degree 2 CBF) LgB = 0 for this B(x), need barrier to include controls
        % Lf_Lf_h + Lg_Lf_h*u + k1*Lf_h + k0*h + gamma_cbf/Bbar >= 0 (zeroing barrier function)
        % -Lg_Lf_h*u <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
        % [-Lg_Lf_h, 0]* [u ; delta] <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
        %          A_cbf * U        <=  B_cbf

%% QP format: 
        % Must look like this Ax<=b
        % where x is what is to be minimized, in our case U = [u; delta]
    g = 9.81;
    f_pos = @(x) [x(4); x(5); x(6); 0; 0; g];
    g_pos = [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];
    ref_dot = @(t) [(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000 ;
                (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5;
                                                                            -1/10;
(cos(t/5)*((981*exp(-t/100))/100 - 981/100))/25 - (981*cos(t/5)*exp(-t/100))/1000000 - (981*sin(t/5)*exp(-t/100))/25000
(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/25 + (981*cos(t/5)*exp(-t/100))/25000 - (981*sin(t/5)*exp(-t/100))/1000000
                                                                              0];
    N_pos_states = 6;
    N_pos_control = 3;

    %% barrier function: a 3D sphere
    radius = 2;
    c = [-1;-1.5;-2];
    
    % h(x) is the constraint B(x) is the barrier
    h_x = @(x,c,r) (x(1)-c(1)).^2 + (x(2)-c(2)).^2 + (x(3)-c(3)).^2 - r.^2; % > 0
    B_x = @(h) -log( h/(1+h) );
    
    %% position control CLF-CBF-QP parameters
    % stabilizing terms 
    c_clf     = 1;     % CLF rate 
    p_delta   = 1;     % penalty on delta^2 (big => stability over saftey)
    % safety terms 
    gamma_cbf = .05;     % CBF gamma in  gamma/Bbar
    k0 = 1;                   % stiffness
    k1 = 2*sqrt(k0);           % damping
    
    % max and min control values
    u_min = -20;
    u_max =  20;
    
    % quadprog options
    opts = optimoptions('quadprog', 'Display','off');

    e_i = pos_error;
    f_e = f_pos(x_pos) - ref_dot(t);           % drift in error dynamics (without control)
    % ---- Step 2: CLF V(e)=e'Pe and soft constraint
    V = W_pos'*Basis_Func_pos(x_pos);           % approxminated Value function needed for CLF
    gradV = grad_Basis_func_pos(e_i)'*W_pos;    % approximated dV/de needed for CLF
    LfV = gradV' * (f_e);                       % along f_e
    LgV = gradV' * g_pos;                   % 1x3
    % CLF inequality: [LgV, -1]*[u;delta] <= -LfV - c*V
    A_clf = [LgV, -1];
    b_clf = -LfV - c_clf*V;

    % ---- Step 3: h(x) and cbf
    h = h_x(x_pos,c,radius);
    eps_h = 1e-10;
    if h <= eps_h
        h = eps_h;
    end

    Bbar = B_x(h);
    position = x_pos(1:3);
    velocity = x_pos(4:6);
    Lf_h = 2*(position - c)'*velocity;          % grad_h' * f(x)
    Lf_Lf_h = 2*(velocity'*velocity) + 2*(position - c)'*[0;0;g];           % grad(grad_h' * f(x))' * f(x)
    Lg_Lf_h = -2*(position - c)';                % grad(grad_h' * f(x))' * g(x)

    A_cbf = [-Lg_Lf_h, 0]; % 1x4
    b_cbf = Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar; % 1x1

    % ---- Step 4: CLF-CBF QP
    % Decision U = [u; delta]
    % Objective: minimize (u - u_nom)^2 + p_delta * delta^2
    % In standard quadprog form: 0.5*U'HU + f'U
    H   = blkdiag(2*eye(N_pos_control), 2*p_delta);    % 4x4
    fqp = [-2*u_nom; 0];                   % 4x1

    % Additional constraints:
    %  -1 <= u <= 1
    A_u = [ eye(N_pos_control),  zeros(N_pos_control,1);
           -eye(N_pos_control),  zeros(N_pos_control,1) ];     % 6x4
    b_u = [ u_max*ones(N_pos_control,1);
           -u_min*ones(N_pos_control,1) ];          % 6x1

    % delta >= 0 (optional but standard for "softening")
    A_del = [zeros(1,N_pos_control), -1];   % 1x4
    b_del = 0;

    % Stack constraints: A*U <= b
    Aqp = [A_clf;A_cbf; A_u; A_del];
    bqp = [b_clf; b_cbf; b_u; b_del];

    % Solve QP only if getting unsafe
    if h < 3
        [Uopt,~,exitflag] = quadprog(H, fqp, Aqp, bqp, [], [], [], [], [], opts);
        if exitflag <= 0 || isempty(Uopt)
            U_pos = min(max(u_nom, u_min), u_max);
            delta = 0;
        else
            U_pos = Uopt(1:3);
            delta = Uopt(4);
        end

    else
        delta = 0;
        U_pos = u_nom;
    end

end