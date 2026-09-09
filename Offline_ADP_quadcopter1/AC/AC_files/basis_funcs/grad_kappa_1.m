function y = grad_kappa_1(x)
  y = grad_kappamat(x(1, :),x(2, :),x(3, :),x(4, :),x(5, :),x(6, :))';
end

function out = grad_kappamat(a,b,c,d,e,f)

out = [
ones(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.*2.0
b
c
d
e
f
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.^2.*3.0
a.*b.*2.0
a.*c.*2.0
a.*d.*2.0
a.*e.*2.0
a.*f.*2.0
b.^2
b.*c
b.*d
b.*e
b.*f
c.^2
c.*d
c.*e
c.*f
d.^2
d.*e
d.*f
e.^2
e.*f
f.^2
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
    ];
end