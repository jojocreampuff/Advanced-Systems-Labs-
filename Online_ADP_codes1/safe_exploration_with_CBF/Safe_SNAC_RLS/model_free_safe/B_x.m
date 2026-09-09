function [B, gradB, h] = B_x(x, params)
    c     = params.barrier.c;
    rr    = params.barrier.r;
    gamma = params.barrier.gamma;
    clip = params.barrier.clip;

    %% barrier (safe Inside)
    % h = rr^2 -(x(1)-c(1)).^2 - (x(2)-c(2)).^2; % inside
    % B = gamma/(h);
    % 
    % grad_h = [-2*(x(1)-c(1)); 
    %         -2*(x(2)-c(2))];          % 2x1
    % gradB  = -gamma * grad_h / (h^2);
    %% barrier (safe outside)
    h = (x(1)-c(1)).^2 + (x(2)-c(2)).^2 - rr^2; % outside

    if clip == 1
        h = max(h,1e-10);
    end
    B = gamma/(h);
    
    grad_h = [2*(x(1)-c(1)); 
            2*(x(2)-c(2))];          % 2x1
    gradB  = -gamma * grad_h / (h^2);

end



%% reciprical barrier (with small unsafe region)
% r = 1;
% c = [2;2];
% gamma = 1e-3;
% 
% h_x = (x(1)-c(1)).^2 + (x(2)-c(2)).^2 - r.^2; % > 0
% h_x_non_clipped = h_x;
% 
% if h_x < 1e-10
%     h_x = 1e-10;
% end
% 
% B = gamma/(h_x);


%% log barrier
% B = -log( (h_x) /(1+h_x) );
% delB_delx1 = (2*x(1)-2*c(1)) / (h_x*(1+h_x)); 
% delB_delx2 = (2*x(2)-2*c(2)) / (h_x*(1+h_x));
% gradB = [delB_delx1; delB_delx2];

