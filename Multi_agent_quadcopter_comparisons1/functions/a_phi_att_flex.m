function out = a_phi_att_flex(x)
x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
x4 = x(4,:);
x5 = x(5,:);
x6 = x(6,:);

out = [ 
    x1
    x2
    x3
    x4
    x5
    x6
    % polynomial terms
    x1.^2
    x2.^2
    x3.^2
    x4.^2
    x5.^2
    x6.^2
    % 2 - term combinations
    x1.*x2
    x1.*x3
    x1.*x4
    x2.*x3
    x1.*x5
    x2.*x4
    x1.*x6
    x2.*x5
    x3.*x4
    x2.*x6
    x3.*x5
    x3.*x6
    x4.*x5
    x4.*x6
    x5.*x6
 ];