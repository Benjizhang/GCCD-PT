% function to fit the sin and cos for Psi based on pnx, pny, pnz, pnpsi
% Ref: eq. (14)
% @ Benji Z. Zhang

function [pnSinPsi, pnCosPsi, maxErr] = fitSinCosPsi(pnx,pny,pnz,pnpsi,t_via,g)
    % unit: cm
    %%% g = 9.80665*100;
    fitDeg = 4;

    t = t_via(1):0.1:t_via(2);
    psi = deg2rad(polyval(pnpsi,t));

    % generate \Psi
    sinPsi = sin(psi);
    pnSinPsi = polyfit(t,sinPsi,fitDeg);
    
    cosPsi = cos(psi);
    pnCosPsi= polyfit(t,cosPsi,fitDeg);
    
    % cal. max err (deg)
    maxErrPsiDeg = 0;
    sinPsiFit = polyval(pnSinPsi,t);
    maxErrPsiSin = rad2deg(max(abs(sinPsiFit - sinPsi)));
    cosPsiFit = polyval(pnCosPsi,t);
    maxErrPsiCos = rad2deg(max(abs(cosPsiFit - cosPsi)));
   
    
    maxErr = round([maxErrPsiDeg maxErrPsiSin maxErrPsiCos],6);

end