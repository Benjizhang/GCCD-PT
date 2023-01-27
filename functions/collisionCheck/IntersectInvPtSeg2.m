% function to cal. collision interval between a point and a segment
% i.e., cal. C2-C5 below eq.(24)
%
% Benji Z. Zhang
% 2023/1

function intvPtSeg = IntersectInvPtSeg2(safe_dis, t_via, CoUniPol, inequal, pt_j, edgeendpt, edge)
    intv_temp1 = RealRootsIntersectSet(t_via(2), CoUniPol, inequal);
    if ~isempty(intv_temp1)
        intv_temp2 = MinDisPtSeg2(safe_dis, t_via(2), pt_j, edgeendpt, edge);
        % intersetion
        if ~isempty(intv_temp2)
            [resultL, resultR] = or_and_or(intv_temp1(:,1), intv_temp1(:,2), intv_temp2(:,1), intv_temp2(:,2));
            intvPtSeg = [resultL', resultR'];
        else
            intvPtSeg =[];
        end
    else
        intvPtSeg = [];
    end
end