% function to fit the sin and cos for theta based on pnx, pny, pnz, pnpsi
% Ref: eq. (14)
% @ Benji Z. Zhang

function [pnSinTheta, pnCosTheta, maxErr] = fitSinCosTheta(pnx,pny,pnz,pnpsi,t_via,g)
    % unit: cm
    %%% g = 9.80665*100;
    fitDeg = 4;

    t = t_via(1):0.1:t_via(2);
    pnddx = polyder(polyder(pnx));
    pnddy = polyder(polyder(pny));
    pnddz = polyder(polyder(pnz));
    ddx = polyval(pnddx,t);
    ddy = polyval(pnddy,t);
    ddz = polyval(pnddz,t);
    psi = deg2rad(polyval(pnpsi,t));

    % generate \theta
    tanTheta_nume = cos(psi).*ddx+sin(psi).*ddy;
    tanTheta_deno = ddz+g;
    Theta = atan(tanTheta_nume./tanTheta_deno); % rad [-π/2, π/2]
    pnTheta = polyfit(t,Theta,fitDeg);
    sinTheta = sin(Theta);
    pnSinTheta = polyfit(t,sinTheta,fitDeg);
    cosTheta = cos(Theta);
    pnCosTheta = polyfit(t,cosTheta,fitDeg);

    % cal. max err (deg)
    ThetaFit = polyval(pnTheta,t);
    maxErrThetaDeg = rad2deg(max(abs(ThetaFit - Theta)));
    sinThetaFit = polyval(pnSinTheta,t);
    maxErrThetaSin = rad2deg(max(abs(sinThetaFit - sinTheta)));
    cosThetaFit = polyval(pnCosTheta,t);
    maxErrThetaCos = rad2deg(max(abs(cosThetaFit - cosTheta)));
   
    
    maxErr = round([maxErrThetaDeg maxErrThetaSin maxErrThetaCos],6);

end