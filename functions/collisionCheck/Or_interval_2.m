% @ Benji Z. Zhang

function [L_or, R_or] = Or_interval_2(L, R)
    if length(L) ~= length(R)
        error('Invaid Input');
    elseif max(L > R)
        error('Make sure L <= R');
    elseif isempty(L) 
        warning('[WARNING] Empty Input') % union(null space)
        L_or = [];
        R_or = [];
    else 
        l_fix = L(1);
        r_fix = R(1);
        l_var = L(2);
        r_var = R(2);
        if r_var < l_fix      % 2 non-intersecting intervals
            L_or(1) = l_var;
            R_or(1) = r_var;
            L_or(2) = l_fix;
            R_or(2) = r_fix;
        elseif l_var > r_fix  % 2 non-intersecting intervals
            L_or(1) = l_fix;
            R_or(1) = r_fix;
            L_or(2) = l_var;
            R_or(2) = r_var;
        else                  % combine 2 intervals into one interval
            l_temp = min(L);
            r_temp = max(R);        
            L_or(1) = l_temp;
            R_or(1) = r_temp;
        end
    end
end