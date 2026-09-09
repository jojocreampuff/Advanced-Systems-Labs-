function out = PSI_drift_approx(x)
% x_max = 2.0;         
%% change activations to test
% x = max(min(x, x_max), -x_max);
        % to get an idea of what activation are good to select
        % f_true = [x(2); 
        %          (1-x(1)^2)*x(2) - x(1) = x2 - x2*x1^2 - x1];
        % g      = [0; 1];
        % optimal weights H^* = [0    1   0
        %                       -1   1     -1]' = [-1, 0; 1  1; -1 , 0]
x1 = x(1); x2 = x(2);
out = [ 
    x1 
    x2
    x2*(x1^2)
    % x1*x2
    % x1^2 
    % x2^2
    % x1^3;  
    % x2^3;
    ];

end

% function psi = PSI_poly2D(xin, nn)
%     x = xin(:);
%     x = max(min(x, nn.psi_max), -nn.psi_max);
% 
%     x1 = x(1); x2 = x(2); deg = nn.deg;
% 
%     psi = zeros((deg+1)*(deg+2)/2, 1);
%     k = 1;
%     for d = 0:deg
%         for i = d:-1:0
%             j = d - i;
%             psi(k) = (x1^i) * (x2^j);
%             k = k + 1;
%         end
%     end
% end