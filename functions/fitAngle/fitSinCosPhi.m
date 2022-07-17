% function to fit the sin and cos for phi based on pnx, pny, pnz, pnpsi
% Ref: eq. (14)
% @ Benji Z. Zhang

function [pnSinPhi, pnCosPhi, maxErr] = fitSinCosPhi(pnx,pny,pnz,pnpsi,t_via,g)
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

    % generate \phi 
    sinPhi_nume = sin(psi).*ddx-cos(psi).*ddy;
    sinPhi_deno = sqrt(ddx.^2+ddy.^2+(ddz+g).^2);
    Phi = asin(sinPhi_nume./sinPhi_deno); % rad [-π/2, π/2]
    % fit \phi (in rad)
    pnPhi = polyfit(t,Phi,fitDeg);
    % fit sin\phi
    SinPhi = sinPhi_nume./sinPhi_deno;
    pnSinPhi = polyfit(t,SinPhi,fitDeg);
    % fit cos\phi
    CosPhi = cos(Phi);
    pnCosPhi = polyfit(t,CosPhi,fitDeg);

    % cal. max err (deg)
    PhiFit = polyval(pnPhi,t);
    [maxErrPhi, ~] = max(abs(PhiFit - Phi));
    maxErrPhiDeg = rad2deg(maxErrPhi);
    sinPhiFit = polyval(pnSinPhi,t);
    [maxErrSin, ~] = max(abs(sinPhiFit - SinPhi));
    maxErrSinPhi = rad2deg(maxErrSin);
    cosPhiFit = polyval(pnCosPhi,t);
    [maxErrCos, ~] = max(abs(cosPhiFit - CosPhi));
    maxErrCosPhi = rad2deg(maxErrCos);
    
    maxErr = round([maxErrPhiDeg maxErrSinPhi maxErrCosPhi],6);

end