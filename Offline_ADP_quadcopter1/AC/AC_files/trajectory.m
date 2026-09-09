function ref = trajectory(Fx, Fy, Fz)
    Fu = getDDerivative(Fx);
    Fv = getDDerivative(Fy);
    Fw = getDDerivative(Fz);

    ref = [Fx; Fy; Fz; Fu; Fv; Fw];
end

 