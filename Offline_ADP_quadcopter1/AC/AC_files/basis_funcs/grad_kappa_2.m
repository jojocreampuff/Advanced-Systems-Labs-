function y = grad_kappa_2(x)
  y = grad_kappamat(x(1, :),x(2, :),x(3, :),x(4, :),x(5, :),x(6, :))';
end

function out = grad_kappamat(a,b,c,d,e,f)

out = [
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
b.*2.0
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
a.^2
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
zeros(1, length(a))
a.*b.*2.0
a.*c
a.*d
a.*e
a.*f
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
b.^2.*3.0
b.*c.*2.0
b.*d.*2.0
b.*e.*2.0
b.*f.*2.0
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
    ];
end