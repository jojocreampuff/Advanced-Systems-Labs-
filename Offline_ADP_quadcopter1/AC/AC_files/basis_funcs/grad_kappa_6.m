function y = grad_kappa_6(x)
  y = grad_kappamat(x(1, :),x(2, :),x(3, :),x(4, :),x(5, :),x(6, :))';
end

function out = grad_kappamat(a,b,c,d,e,f)

out = [
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
ones(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
b
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
c
zeros(1, length(a))
zeros(1, length(a))
d
zeros(1, length(a))
e
f.*2.0
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.^2
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.*b
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.*c
zeros(1, length(a))
zeros(1, length(a))
a.*d
zeros(1, length(a))
a.*e
a.*f.*2.0
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
b.^2
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
b.*c
zeros(1, length(a))
zeros(1, length(a))
b.*d
zeros(1, length(a))
b.*e
b.*f.*2.0
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
c.^2
zeros(1, length(a))
zeros(1, length(a))
c.*d
zeros(1, length(a))
c.*e
c.*f.*2.0
zeros(1, length(a))
zeros(1, length(a))
d.^2
zeros(1, length(a))
d.*e
d.*f.*2.0
zeros(1, length(a))
e.^2
e.*f.*2.0
f.^2.*3.0
zeros(1, length(a))
    ];
end