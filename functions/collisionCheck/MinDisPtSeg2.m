% func. to cal. collision intervals based on the collision conditions 
% between the Edge/Seg e and the Point Vp (Ref: eq.(4) < safe_dis)
% Inputs:
%      safe_dis: the prescribed distance threshold \underline{d}
%      end_time: time t \in [0, end_time]
%      pt: external point
%      segendpt: start & end points of the edge
%      seg: edge vector
%      *Note: pt,segendpt,seg are all in polynomial forms
% Output:
%      intvPtSeg: collision intervals for the edge/seg and the point, where
%                 each row refers to one collision interval. (Ref: eq.(20))
%
% --- v2.0 ---
% simplify the codes
%
% @ Benji Z. Zhang

function intvPtSeg =  MinDisPtSeg2(safe_dis, end_time, pt, segendpt, seg)    
    % vector from the start point of the edge to the external point 
    APt = AddPolyCoeff_2P(pt, -segendpt{1}); 
    % vector from the end point of the edge to the external point 
    BPt = AddPolyCoeff_2P(pt, -segendpt{2});
    % Note: tilde{p}_s in eq.(18) is the vector from the origion to the start
    % point of the edge, thus tilde{p}_s = -APt
    % Note: tilde{p}_e in eq.(18) is the vector from the origion to the end
    % point of the edge, thus tilde{p}_e = -BPt


    %% potential MinDis < diameter 
    % norm(APt) < dia
    temp1 = SquarePoly(APt);
    temp2 = -(safe_dis)^2;
    mindisPoly{1} = AddPolyCoeff_2P(temp1, temp2); % This is -w1 in eq.(18)
    % cross(APt, l)/norm(l) < dia
    temp1 = CrossProductPoly(APt,seg);
    temp12 = SquarePoly(temp1);
    temp22 = -(safe_dis)^2 * SquarePoly(seg); 
    mindisPoly{2} = AddPolyCoeff_2P(temp12, temp22); % This is -w3 in eq.(18)
    % norm(BPt) < dia
    temp1 = SquarePoly(BPt);
    temp2 = -(safe_dis)^2;
    mindisPoly{3} = AddPolyCoeff_2P(temp1, temp2); % This is -w6 in eq.(18)


    %% form the condition polynomial
    % dot(APt, l)
    tempdot = conv(APt(1,:), seg(1,:))+conv(APt(2,:), seg(2,:))+conv(APt(3,:), seg(3,:));
    % dot(APt, l) = 0
    CondPoly{1} = tempdot; % This is -w2 in eq.(18)
    % dot(APt, l) = norm(l)^2
    temp_norm2 = -SquarePoly(seg);
    % CondPoly{2} = tempdot - temp_norm2;
    CondPoly{2} = AddPolyCoeff_2P(tempdot, temp_norm2); % This is -w5 in eq.(18)


    %% interval calculation 
    % MinDisPtSeg is a piece-wise function with 3 cases (Ref: eq.(4))
    
    % Note: here we use '<', but '>=' is given in eq.(20)
    inequal = '<'; 

    % case1: C1 in eq.(20)
    CoUniPol_temp{1} = mindisPoly{1}; % i.e., -w1 < 0
    CoUniPol_temp{2} = CondPoly{1};   % i.e., -w2 < 0
    intv1 = RealRootsIntersectSet(end_time, CoUniPol_temp, inequal);
    clear CoUniPol_temp;

    % case2: C2 in eq.(20)
    CoUniPol_temp{1} = mindisPoly{2}; % i.e., -w3 < 0
    CoUniPol_temp{2} = -CondPoly{1};  % i.e., -(-w2) = -w4 < 0
    CoUniPol_temp{3} = CondPoly{2};   % i.e., -w5 < 0
    intv2 = RealRootsIntersectSet(end_time, CoUniPol_temp, inequal);
    clear CoUniPol_temp;

    % case3: C3 in eq.(20)
    CoUniPol_temp{1} = mindisPoly{3}; % i.e., -w6 < 0
    CoUniPol_temp{2} = -CondPoly{2};  % i.e., -(-w5) = -w7 < 0
    intv3 = RealRootsIntersectSet(end_time, CoUniPol_temp, inequal);
    clear CoUniPol_temp;

    % union set of (intv1, intv2, intv3)
    % i.e., C1 U C2 U C3 in eq.(20)
    intv = [intv1; intv2; intv3];
    if ~isempty(intv)
        L_temp = intv(:,1);
        R_temp = intv(:,2);
        [L_intv, R_intv] = Or_interval(L_temp, R_temp);
        intv_temp = [L_intv', R_intv'];
    else
        intv_temp = [];
    end

    % collision intervals of MinDis from Edge v.s. Pt
    % i.e., C_{EP}^l in eq.(20)
    intvPtSeg = intv_temp;
end