function out = PHI(e, r)
e_max = 2.0;         
r_max = 2.0; 
%% change activations to test
% e = max(min(e, e_max), -e_max);
% r = max(min(r, r_max), -r_max);
r1=r(1); r2=r(2); r3=r(3); r4=r(4); r5=r(5); r6=r(6);
e1=e(1); e2=e(2); e3=e(3); e4=e(4); e5=e(5); e6=e(6);
% careful with making the basis too large, P becomes very large very fast

out = [
    e1  
    e2
    e3
    e4
    e5
    e6
    e1.*e4
    e2.*e5
    e3.*e6
    % e1^2
    % e2^2
    % e3^2
    % e4^2
    % e5^2
    % e6^2
    % r1  
    % r2
    % r3
    % r4
    % r5
    % r6
    %% cross terms (relation)

    e1*r1; 
    e1*r4;
    e2*r2;
    e2*r5;
    % e3*r3; 
    e4*r4;
    e5*r5;

    % e6*r6;
    % e1*r4
    % e2*r5;
    % e3*r6;
    ];

end
