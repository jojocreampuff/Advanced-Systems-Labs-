function out = Basis_Func_att(e)
% the input is euler angle errors
e1=e(1); e2=e(2); e3=e(3); e4=e(4); e5=e(5); e6=e(6);
% careful with making the basis too large, P becomes very large very fast

out = [
    e1  
    e2
    e3
    e4
    e5
    e6
    % 1
    e1.*e4
    e2.*e5
    e3.*e6
    e1^2
    e2^2
    e3^2
    e4^2
    e5^2
    e6^2

];
end
