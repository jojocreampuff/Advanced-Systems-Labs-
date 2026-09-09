function out = PHI(e, r)
e_max = 2.0;         
r_max = 2.0; 
%% change activations to test
% e = max(min(e, e_max), -e_max);
% r = max(min(r, r_max), -r_max);
e1 = e(1); e2 = e(2);
r1 = r(1); r2 = r(2);
out = [ 
    %% error
    e1; 
    e2;
    e1*e2;
    e1^2; 
    e2^2;
    e1^3;  
    e2^3;
    % sin(e1); sin(e2)  
    % sin(e1^2); sin(e2^2)
    % e1^2 * sin(e2); e2^2 * sin(e1)
    %% reference
    r1; 
    r2 
    % r1^2; r2^2;
    %% cross terms
    e1*r1; 
    e2*r2;
    e1*r2; 
    e2*r1;
    ];

end
