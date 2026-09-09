function y = rho(x)
  y = rhomat(x(1, :),x(2, :),x(3, :));
end

function out = rhomat(a,b,c)
  out = [
          a
          b
          c
          a.*a
          a.*b
          a.*c
          b.*b
          b.*c
          c.*c
          a.*a.*a
          a.*a.*b
          a.*a.*c
          a.*b.*b
          a.*b.*c
          a.*c.*c
          b.*b.*b
          b.*b.*c
          b.*c.*c
          c.*c.*c
          ones(1, size(a, 2))

  ];
end