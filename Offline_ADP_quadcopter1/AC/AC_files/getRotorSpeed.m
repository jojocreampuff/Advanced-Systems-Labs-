function O = getRotorSpeed(ft, tx, ty, tz)
  constants

  O2 = ft/(4*b) - ty/(2*b*l) + tz/(4*d);
  O1 = O2 - tx/(2*b*l) + ty/(2*b*l) - tz/(2*d);
  O3 = tx/(b*l) + O1;
  O4 = ty/(b*l) + O2;

  O = [O1; O2; O3; O4];
end