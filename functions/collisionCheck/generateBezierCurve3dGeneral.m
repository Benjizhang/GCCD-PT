% function to generate 3D Bezier curves 
% w.r.t. normalized parameter s \in [0,1]
% Input:  
% controlPts: ANY control points 
%             in Matrix form
% Outpput: 
% x,y,z: the 3D Bezier curve
% 
% ---- v1.0 ----
% For any control points
% @ Benji Z. Zhang
% 11/2021

function [x, y, z] = generateBezierCurve3dGeneral(controlPts)
    if(size(controlPts,1) == -2)
        % linear Bezier curve
        % B(t) = t(P1-P0) + P0
        P0 = controlPts(1,:);
        P1 = controlPts(2,:);
        x = [(P1(1)-P0(1)) P0(1)];
        y = [(P1(2)-P0(2)) P0(2)];
        z = [(P1(3)-P0(3)) P0(3)];
    elseif(size(controlPts,1) == -3)
        % translation path in the form of the 2nd degree Bezier curve
        % B(t) = P1 + (1-t)^2*(P0-P1) + t^2*(P2-P1)
        %      = (P0+P2-2P1)t^2 + (2(P1-P0))t + P0
        P0 = controlPts(1,:);
        P1 = controlPts(2,:);
        P2 = controlPts(3,:);
        TT = P0+P2-2*P1;
        T  = 2*(P1-P0);
        c  = P0;
        x = [TT(1) T(1) c(1)];
        y = [TT(2) T(2) c(2)];
        z = [TT(3) T(3) c(3)];
    elseif(size(controlPts,1) == -4)
        P0 = controlPts(1,:);
        P1 = controlPts(2,:);
        P2 = controlPts(3,:);
        P3 = controlPts(4,:);
        p0 = conv([-1 1],conv([-1 1],[-1 1]));
        p1 = 3*conv([1 0],conv([-1 1],[-1 1]));
        p2 = 3*conv([1 0 0], [-1 1]);
        p3 = [1 0 0 0];
        temp1 = AddPolyCoeff_2P(P0(1)*p0,P1(1)*p1);
        temp2 = AddPolyCoeff_2P(P2(1)*p2,P3(1)*p3);
        x = AddPolyCoeff_2P(temp1, temp2);
        temp1 = AddPolyCoeff_2P(P0(2)*p0,P1(2)*p1);
        temp2 = AddPolyCoeff_2P(P2(2)*p2,P3(2)*p3);
        y = AddPolyCoeff_2P(temp1, temp2);
        temp1 = AddPolyCoeff_2P(P0(3)*p0,P1(3)*p1);
        temp2 = AddPolyCoeff_2P(P2(3)*p2,P3(3)*p3);
        z = AddPolyCoeff_2P(temp1, temp2);
    else
        numPts = size(controlPts,1);
        n = numPts-1;
        pnBx = 0; 
        pnBy = 0; 
        pnBz = 0;
        for i=0:1:n
            % for calculating (x!/(y!(x-y)!)) value
            sigma=factorial(n)/(factorial(i)*factorial(n-i));  
            % t^i
            pnTerm1 = PowerPoly([1 0], i);
            % (1-t)^(n-i)
            pnTerm2 = PowerPoly([-1 1], (n-i));
            % AllinOne (ref: https://en.wikipedia.org/wiki/B%C3%A9zier_curve)
            pnbin = sigma*conv(pnTerm1,pnTerm2);
            clear sigma pnTerm1 pnTerm2
            
            % current control pts \in R(1,3)
            curPt = controlPts(i+1,:);
            % Bix(t)
            pnBix = curPt(1)*pnbin;
            % Biy(t)
            pnBiy = curPt(2)*pnbin;
            % Biz(t)
            pnBiz = curPt(3)*pnbin;
            
            % Bx(t)
            pnBx = AddPolyCoeff_2P(pnBx,pnBix);
            % By(t)
            pnBy = AddPolyCoeff_2P(pnBy,pnBiy);
            % Bz(t)
            pnBz = AddPolyCoeff_2P(pnBz,pnBiz);
        end
        x = pnBx;
        y = pnBy;
        z = pnBz;
    end
end

