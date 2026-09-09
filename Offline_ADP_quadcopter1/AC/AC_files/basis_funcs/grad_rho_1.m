function y = grad_rho_1(x)
  y = grad_rhomat(x(1, :),x(2, :),x(3, :))';
end

function out = grad_rhomat(a,b,c)

out = [
ones(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.*2.0
b
c
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.^2.*3.0
a.*b.*2.0
a.*c.*2.0
b.^2
b.*c
c.^2
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
    ];
end