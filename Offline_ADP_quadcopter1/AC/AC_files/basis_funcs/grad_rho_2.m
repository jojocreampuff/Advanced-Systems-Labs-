function y = grad_rho_2(x)
  y = grad_rhomat(x(1, :),x(2, :),x(3, :))';
end

function out = grad_rhomat(a,b,c)

out = [
zeros(1, length(a))
ones(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a
zeros(1, length(a))
b.*2.0
c
zeros(1, length(a))
zeros(1, length(a))
a.^2
zeros(1, length(a))
a.*b.*2.0
a.*c
zeros(1, length(a))
b.^2.*3.0
b.*c.*2.0
c.^2
zeros(1, length(a))
zeros(1, length(a))
    ];
end