function out = Basis_Func_84(x)
x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
x4 = x(4,:);
x5 = x(5,:);
x6 = x(6,:);

out = [ 
    [ones(1,length(x(1,:)))];
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
    x1.^3
    x2.^3
    x3.^3
    x4.^3
    x5.^3
    x6.^3
    x1.^4
    x2.^4
    x3.^4
    x4.^4
    x5.^4
    x6.^4
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
    % 3 term combinations
    x1.*x2.*x3
    x1.*x2.*x4
    x1.*x2.*x5
    x1.*x2.*x6
    x1.*x3.*x4
    x1.*x3.*x5
    x1.*x3.*x6
    x1.*x4.*x5
    x1.*x4.*x6
    x1.*x5.*x6
    x2.*x3.*x4
    x2.*x3.*x5
    x2.*x3.*x6
    x2.*x4.*x5
    x2.*x4.*x6
    x2.*x5.*x6
    x3.*x4.*x5
    x3.*x4.*x6
    x3.*x5.*x6
    x4.*x5.*x6
    % 4 term combinations
    x1.*x2.*x3.*x4
    x1.*x2.*x3.*x5
    x1.*x2.*x3.*x6
    x1.*x2.*x4.*x5
    x1.*x2.*x4.*x6
    x1.*x2.*x5.*x6
    x1.*x3.*x4.*x5
    x1.*x3.*x4.*x6
    x1.*x3.*x5.*x6
    x1.*x4.*x5.*x6
    x2.*x3.*x4.*x5
    x2.*x3.*x4.*x6
    x2.*x3.*x5.*x6
    x2.*x4.*x5.*x6
    x3.*x4.*x5.*x6
    % 5 and 6 term combinations
    x1.*x2.*x3.*x4.*x5
    x1.*x2.*x3.*x4.*x6
    x1.*x2.*x3.*x5.*x6
    x1.*x2.*x4.*x5.*x6
    x1.*x3.*x4.*x5.*x6
    x2.*x3.*x4.*x5.*x6
    x1.*x2.*x3.*x4.*x5.*x6
    % 2 terms with 1 squared
    x1.*x2.^2
    x1.^2.*x2
    x1.*x3.^2
    x1.^2.*x3
    x1.*x4.^2
    x1.^2.*x4
    x1.*x5.^2
    x1.^2.*x5
    x1.*x6.^2
    x1.^2.*x6
    x2.*x3.^2
    x2.^2.*x3
    x2.*x4.^2
    x2.^2.*x4
    x2.*x5.^2
    x2.^2.*x5
    x2.*x6.^2
    x2.^2.*x6
    x3.*x4.^2
    x3.^2.*x4
    x3.*x5.^2
    x3.^2.*x5
    x3.*x6.^2
    x3.^2.*x6
    x4.*x5.^2
    x4.^2.*x5
    x4.*x6.^2
    x4.^2.*x6
    x5.*x6.^2
    x5.^2.*x6
    % 3 terms with 1 squared
    x1.*x2.*x3.^2
    x1.*x2.^2.*x3
    x1.^2.*x2.*x3
    x1.*x2.*x4.^2
    x1.*x2.^2.*x4
    x1.^2.*x2.*x4
    x1.*x2.*x5.^2
    x1.*x2.^2.*x5
    x1.^2.*x2.*x5
    x1.*x2.*x6.^2
    x1.*x2.^2.*x6
    x1.^2.*x2.*x6
    x1.*x3.*x4.^2
    x1.*x3.^2.*x4
    x1.^2.*x3.*x4
    x1.*x3.*x5.^2
    x1.*x3.^2.*x5
    x1.^2.*x3.*x5
    x1.*x3.*x6.^2
    x1.*x3.^2.*x6
    x1.^2.*x3.*x6
    x1.*x4.*x5.^2
    x1.*x4.^2.*x5
    x1.^2.*x4.*x5
    x1.*x4.*x6.^2
    x1.*x4.^2.*x6
    x1.^2.*x4.*x6
    x1.*x5.*x6.^2
    x1.*x5.^2.*x6
    x1.^2.*x5.*x6
    x2.*x3.*x4.^2
    x2.*x3.^2.*x4
    x2.^2.*x3.*x4
    x2.*x3.*x5.^2
    x2.*x3.^2.*x5
    x2.^2.*x3.*x5
    x2.*x3.*x6.^2
    x2.*x3.^2.*x6
    x2.^2.*x3.*x6
    x2.*x4.*x5.^2
    x2.*x4.^2.*x5
    x2.^2.*x4.*x5
    x2.*x4.*x6.^2
    x2.*x4.^2.*x6
    x2.^2.*x4.*x6
    x2.*x5.*x6.^2
    x2.*x5.^2.*x6
    x2.^2.*x5.*x6
    x3.*x4.*x5.^2
    x3.*x4.^2.*x5
    x3.^2.*x4.*x5
    x3.*x4.*x6.^2
    x3.*x4.^2.*x6
    x3.^2.*x4.*x6
    x3.*x5.*x6.^2
    x3.*x5.^2.*x6
    x3.^2.*x5.*x6
    x4.*x5.*x6.^2
    x4.*x5.^2.*x6
    x4.^2.*x5.*x6
    % 2 terms with both squared
    x1.^2.*x2.^2
    x1.^2.*x3.^2
    x1.^2.*x4.^2
    x2.^2.*x3.^2
    x1.^2.*x5.^2
    x2.^2.*x4.^2
    x1.^2.*x6.^2
    x2.^2.*x5.^2
    x3.^2.*x4.^2
    % x2.^2.*x6.^2
    x3.^2.*x5.^2
    % x3.^2.*x6.^2
    x4.^2.*x5.^2
    % x4.^2.*x6.^2
    % x5.^2.*x6.^2
    % % 3 terms with all squared
    (x1.*x2.*x3).^2
    % (x1.*x2.*x4).^2
    % (x1.*x2.*x5).^2
    % (x1.*x2.*x6).^2
    % (x1.*x3.*x4).^2
    % (x1.*x3.*x5).^2
    % (x1.*x3.*x6).^2
    % (x1.*x4.*x5).^2
    % (x1.*x4.*x6).^2
    % (x1.*x5.*x6).^2
    % (x2.*x3.*x4).^2
    % (x2.*x3.*x5).^2
    % (x2.*x3.*x6).^2
    % (x2.*x4.*x5).^2
    % (x2.*x4.*x6).^2
    % (x2.*x5.*x6).^2
    % (x3.*x4.*x5).^2
    % (x3.*x4.*x6).^2
    % (x3.*x5.*x6).^2
    % (x4.*x5.*x6).^2
 ];