function out = basis_2(x)
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
    x1.*x2
    x1.*x3
    x1.*x4
    x1.*x5
    x1.*x6
    x2.*x3
    x2.*x4
    x2.*x5
    x2.*x6
    x3.*x4
    x3.*x5
    x3.*x6
    x4.*x5
    x4.*x6
    x5.*x6
    x1.*x2.*x3
    x1.*x2.*x4
    x1.*x2.*x5
    x1.*x3.*x4
    x1.*x2.*x6
    x1.*x3.*x5
    x2.*x3.*x4
    x1.*x3.*x6
    x1.*x4.*x5
    x2.*x3.*x5
    x1.*x4.*x6
    x2.*x3.*x6
    x2.*x4.*x5
    x1.*x5.*x6
    x2.*x4.*x6
    x3.*x4.*x5
    x2.*x5.*x6
    x3.*x4.*x6
    x3.*x5.*x6
    x4.*x5.*x6
    % x1.*x2.*x3.*x4.*x5.*x6
    % (x1.^2).*(x2.^2).*(x3.^2).*(x4.^2).*(x5.^2).*(x6.^2)
    x1.*x2.^2
    x1.^2.*x2
    x1.*x3.^2
    x1.^2.*x3
    x1.*x4.^2
    x2.*x3.^2
    x1.^2.*x4
    x2.^2.*x3
    x1.*x5.^2
    x2.*x4.^2
    x1.^2.*x5
    x2.^2.*x4
    x1.*x6.^2
    x2.*x5.^2
    x3.*x4.^2
    x1.^2.*x6
    x2.^2.*x5
    x3.^2.*x4
    x2.*x6.^2
    x3.*x5.^2
    x2.^2.*x6
    x3.^2.*x5
    x3.*x6.^2
    x4.*x5.^2
    x3.^2.*x6
    x4.^2.*x5
    x4.*x6.^2
    x4.^2.*x6
    x5.*x6.^2
    x5.^2.*x6
    x1.^2
    x1.^3
    x2.^2
    x2.^3
    x3.^2
    x3.^3
    x4.^2
    x4.^3
    x5.^2
    x5.^3
    x6.^2
    x6.^3
    x1.^4
    x2.^4
    x3.^4
    x4.^4
    x5.^4
    x6.^4

    % x1.^5
    % x2.^5
    % x3.^5
    % x4.^5
    % x5.^5
    % x6.^5

    % x1.^6
    % x2.^6
    % x3.^6
    % x4.^6
    % x5.^6
    % x6.^6

    % x1.^7
    % x2.^7
    % x3.^7
    % x4.^7
    % x5.^7
    % x6.^7
    % 
    % x1.^8
    % x2.^8
    % x3.^8
    % x4.^8
    % x5.^8
    % x6.^8
    % sin(x1)
    % sin(x2)
    % sin(x3)
    % sin(x4)
    % sin(x5)
    % sin(x6)
    % cos(x1)
    % cos(x2)
    % cos(x3)
    % cos(x4)
    % cos(x5)
    % cos(x6)
    % tan(x1)% tan is defined in the training domain. 
    % tan(x2)
    % tan(x3)
    % tan(x4)
    % tan(x5)
    % tan(x6)

 ];