function out = PSI_drift_approx(x)
% x_max = 2.0;         
%% change activations to test
% x = max(min(x, x_max), -x_max);
% out = [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
%                    (x(5)*cos(x(1)) - x(6)*sin(x(1)));
%                    (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
%                    ((Iy - Iz) / Ix * x(5) * x(6));
%                    ((Iz - Ix) / Iy * x(4) * x(6));
%                    ((Ix - Iy) / Iz * x(4) * x(5))];

x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
out = [ 
    x4; % exact activation
    x4*x5; % exact
    x4*x6; % exact
    x5*x6; % exact
    % x2^2;
    % x5^2;
    % x6^2;
    x5*sin(x1)*tan(x2) % exact
    x6*cos(x1)*tan(x2) % exact
    x5*cos(x1) % exact
    % x5*sin(x1);
    x6*sin(x1) % exact
    % x6*cos(x1)
    x5*sin(x1)/cos(x2) % exact
    x6*cos(x1)/cos(x2) % exact
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