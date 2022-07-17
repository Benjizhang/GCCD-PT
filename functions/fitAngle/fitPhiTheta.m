% function to fit the phi and theta based on pnx, pny, pnz, pnpsi
% @ Benji Z. Zhang

function [pnPhi, pnTheta, maxErrDeg] = fitPhiTheta(pnx,pny,pnz,pnpsi,t_via,g)
    % unit: cm
    %%% g = 9.80665*100;
    fitDeg = 5;

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
    % cal. max err (deg)
    ThetaFit = polyval(pnTheta,t);
    [maxErr,~] = max(abs(ThetaFit - Theta));
    maxErrThetaDeg = rad2deg(maxErr);

    % generate \phi 
    sinPhi_nume = sin(psi).*ddx-cos(psi).*ddy;
    sinPhi_deno = sqrt(ddx.^2+ddy.^2+(ddz+g).^2);
    Phi = asin(sinPhi_nume./sinPhi_deno); % rad [-π/2, π/2]
    % fit \phi (in rad)
    pnPhi = polyfit(t,Phi,fitDeg);
    % cal. max err (deg)
    PhiFit = polyval(pnPhi,t);
    [maxErr, ~] = max(abs(PhiFit - Phi));
    maxErrPhiDeg = rad2deg(maxErr);
    
    maxErrDeg = [maxErrPhiDeg maxErrThetaDeg];

end