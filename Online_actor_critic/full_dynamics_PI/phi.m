function out = phi(x)

x1 = x(1,1);
x2 = x(2,1);
x3 = x(3,1);
x4 = x(4,1);
x5 = x(5,1);
x6 = x(6,1);
x7 = x(7,1);
x8 = x(8,1);
x9 = x(9,1);
x10 = x(10,1);
x11 = x(11,1);
x12 = x(12,1);



out = [ 
    [ones(1,length(x(1,:)))];
    x1
    x2
    x3
    x4
    x5
    x6
    x7
    x8
    x9
    x10
    x11
    x12
    % polynomial terms
    x1.^2
    x2.^2
    x3.^2
    x4.^2
    x5.^2
    x6.^2
    x7.^2
    x8.^2
    x9.^2
    x10.^2
    x11.^2
    x12.^2
    % x1.^3
    % x2.^3
    % x3.^3
    % x4.^3
    % x5.^3
    % x6.^3
    x7.^3
    x8.^3
    x9.^3
    x10.^3
    x11.^3
    x12.^3
    % x1.^4
    % x2.^4
    % x3.^4
    % x4.^4
    % x5.^4
    % x6.^4
    x7.^4
    x8.^4
    x9.^4
    x10.^4
    x11.^4
    x12.^4
    % 2-term combinations
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

    x7.*x8
    x7.*x9
    x7.*x10
    x7.*x11
    x7.*x12
    x8.*x9
    x8.*x10
    x8.*x11
    x8.*x12
    x9.*x10
    x9.*x11
    x9.*x12
    x10.*x11
    x10.*x12
    x11.*x12
    % 3-term combinations
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
    x3.*x4.*x5
    x3.*x4.*x6
    x3.*x5.*x6
    x4.*x5.*x6

    x7.*x8.*x9
    x7.*x8.*x10
    x7.*x8.*x11
    x7.*x8.*x12
    x7.*x9.*x10
    x7.*x9.*x11
    x7.*x9.*x12
    x7.*x10.*x11
    x7.*x10.*x12
    x7.*x11.*x12
    x8.*x9.*x10
    x8.*x9.*x11
    x8.*x9.*x12
    x8.*x10.*x11
    x8.*x10.*x12
    x8.*x11.*x12
    x9.*x10.*x11
    x9.*x10.*x12
    x9.*x11.*x12
    x10.*x11.*x12
    % 4-term combinations
    x7.*x8.*x9.*x10
    x7.*x8.*x9.*x11
    x7.*x8.*x9.*x12
    x7.*x9.*x10.*x11
    x7.*x9.*x10.*x12
    x7.*x10.*x11.*x12
    x8.*x9.*x10.*x11
    x8.*x9.*x10.*x12
    x8.*x10.*x11.*x12
    x9.*x10.*x11.*x12
    % 5-term combinations
    x1.*x2.*x3.*x4.*x5
    x1.*x2.*x3.*x4.*x6
    x2.*x3.*x4.*x5.*x6
    x7.*x8.*x9.*x10.*x11
    x7.*x8.*x9.*x10.*x12
    x7.*x9.*x10.*x11.*x12
    x8.*x9.*x10.*x11.*x12
    % 6-term combinations
    x7.*x8.*x9.*x10.*x11.*x12

];