% Affine Transformation (AT): (Ref: eq.(17))
%    function to transfer a POINT in frame {O} into the frame {\tilde{O}}
% Used for the collision between seg and ellipsoid
% [NOTE] pnx,pny,pnz ONLY refer to the coordinates of a POINT
%        they cannot represent a vector!
% INPUTs: cent - center of the ellipsoid (row vector)
%
% @ Benji Z. Zhang

function [pnxTilde, pnyTilde, pnzTilde]= ellipsoidTransfer(Lambda, Q, cent, pnx, pny, pnz)
    % 3 * 3 matrix
    transferMat = Lambda^(1/2)*Q';
    xc = cent(1);
    yc = cent(2);
    zc = cent(3);
    pnx_temp = [pnx(1:end-1) pnx(end)-xc];
    pny_temp = [pny(1:end-1) pny(end)-yc];
    pnz_temp = [pnz(1:end-1) pnz(end)-zc];
    
    % transfer the edge to frame {\tilde{O}} 
    % Method: ptXTilde(:,i) = Lambda^(1/2)*Q'*(ptX - [xc, yc, zc]'); (Ref: eq.(17))
    pnxTilde = AddPolyCoeff_2P(AddPolyCoeff_2P(transferMat(1,1)*pnx_temp,transferMat(1,2)*pny_temp),transferMat(1,3)*pnz_temp);
    pnyTilde = AddPolyCoeff_2P(AddPolyCoeff_2P(transferMat(2,1)*pnx_temp, transferMat(2,2)*pny_temp),transferMat(2,3)*pnz_temp);
    pnzTilde = AddPolyCoeff_2P(AddPolyCoeff_2P(transferMat(3,1)*pnx_temp, transferMat(3,2)*pny_temp),transferMat(3,3)*pnz_temp);  

end