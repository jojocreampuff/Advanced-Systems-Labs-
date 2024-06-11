function F = smooth(f, iniPos, iniVel, time)
   a = 4;
   
   normT = time/time(end);
   normV = iniVel/norm(iniVel);
   if isnan(normV), normV = 0; end  % Edge case for when iniVel is 0
   modT = (exp(-a*normT) - 1)/(exp(-a) - 1);
   l = phi(modT);

   F = (1 - l).*(normT.*normV' + iniPos') + l.*f;
   F(:, 1) = iniPos;
end

function y = phi(t)
  a1 = psi(t); a2 = psi(1 - t);
  y = (a1./(a1 + a2));
end

function y = psi(t)
  y = exp(-1./t);
end
