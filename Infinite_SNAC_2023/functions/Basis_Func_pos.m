function out = Basis_Func_pos(x)
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
    x1.^2
    x2.^2
    x3.^2
    x4.^2
    x5.^2
    x6.^2
 ];