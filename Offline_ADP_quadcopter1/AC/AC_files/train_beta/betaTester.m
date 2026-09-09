function xk = betaTester(iniAtt, uk)

    constants;
    betaDynamics;
    
    NTime = length(uk);
    NStates = 6;

    xk = zeros(NStates, NTime);
    
    xk(:, 1) = iniAtt;
    for k = 1 : (NTime - 1)
        xk(:, k + 1) = model_sim(xk, uk, k);
    end

end