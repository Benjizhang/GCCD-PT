% Intersection Operation: interval_L and interval_R
% @ Benji Z. Zhang

function [L_and, R_and] = And_interval(L, R)
    if length(L) ~= length(R)
        error('Invaid Input');
    elseif max(L > R)
        error('Make sure L <= R');
    elseif isempty(L)
        warning('[WARNING] Empty Input') % inters(one null set); see Input-2)
        L_and = [];
        R_and = [];
    else % the input is not one null interval
        L_and = max(L);
        R_and = min(R);
        if L_and > R_and
            L_and = [];
            R_and = [];
        end
    end
end