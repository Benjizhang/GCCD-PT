% function to output current rotation matrix
% based on the pnSin/pnCos forms
% INPUTs： path - 1*9 cell
% OUTPUT:  R - 3*3 matrix
%
% @ Benji Z. Zhang

function R = getRotMat(path,cur_time)
    % get pnSin/pnCos
    pnSinPhi = path{4};
    pnCosPhi = path{5};
    pnSinTheta = path{6};
    pnCosTheta = path{7};
    pnSinPsi = path{8};
    pnCosPsi = path{9};

    % elements in the rotation matrix
    pnr11 = conv(pnCosTheta,pnCosPsi);
    pnr12 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnCosPsi),-conv(pnCosPhi,pnSinPsi));
    pnr13 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnCosPsi),conv(pnSinPhi,pnSinPsi));

    pnr21 = conv(pnCosTheta,pnSinPsi);
    pnr22 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnSinPsi),conv(pnCosPhi,pnCosPsi));
    pnr23 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnSinPsi),-conv(pnSinPhi,pnCosPsi));

    pnr31 = -pnSinTheta;
    pnr32 = conv(pnSinPhi,pnCosTheta);
    pnr33 = conv(pnCosPhi,pnCosTheta);

    R = zeros(3,3);
    R(1,1) = polyval(pnr11, cur_time);
    R(1,2) = polyval(pnr12, cur_time);
    R(1,3) = polyval(pnr13, cur_time);
    R(2,1) = polyval(pnr21, cur_time);
    R(2,2) = polyval(pnr22, cur_time);
    R(2,3) = polyval(pnr23, cur_time);
    R(3,1) = polyval(pnr31, cur_time);
    R(3,2) = polyval(pnr32, cur_time);
    R(3,3) = polyval(pnr33, cur_time);
end