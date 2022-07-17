% @ Benji Z. Zhang

function [L_comp, R_comp] = Comp_interval_12(L, R)
    if length(L) ~= length(R)
        error('Invaid Input');
    elseif max(L > R)
        error('Plz make sure L <= R');
    elseif isempty(L) 
        warning('[WARNING] Empty Input')  % comp(null set)
        L_comp = -inf;
        R_comp = +inf;
    elseif length(L) == 1
        % 4 cases for an interval
        if L == -inf && R ~= +inf
            L_comp(1) = R;
            R_comp(1) = +inf;
        elseif L~= -inf && R == +inf
            L_comp(1) = -inf;
            R_comp(1) = L;
        elseif L == -inf && R == +inf
            L_comp = [];
            R_comp = [];
        else
            L_comp(1) = -inf;
            R_comp(1) = L;
            L_comp(2) = R;   
            R_comp(2) = +inf;
        end
    elseif length(L) == 2 % two non-null intervals 
        temp = And_interval(L, R); % L and R are non-null
        if isempty(temp) % non-intersecting
            % suppose L(1) < L(2)
            if L(1) > L(2)
            temp = L(1);
            L(1) = L(2);
            L(2) = temp;
            temp = R(1);
            R(1) = R(2);
            R(2) = temp;
            end
            % 4 cases for 2 non-intersecting intervals
            if L(1) ~= -inf && R(2) ~= +inf
                L_comp(1) = -inf;
                R_comp(1) = L(1);
                L_comp(2) = R(1);
                R_comp(2) = L(2);
                L_comp(3) = R(2);
                R_comp(3) = +inf;
            elseif L(1) ~= -inf && R(2) == +inf
                L_comp(1) = -inf;
                R_comp(1) = L(1);
                L_comp(2) = R(1);
                R_comp(2) = L(2);
            elseif L(1) == -inf && R(2) ~= +inf
                L_comp(1) = R(1);
                R_comp(1) = L(2);
                L_comp(2) = R(2);
                R_comp(2) = +inf;
            elseif  L(1) == -inf && R(2) == +inf
                L_comp(1) = R(1);
                R_comp(1) = L(2);

            end    
        else
            error('Make sure 2 non-intersecting intervals');
        end
    else
        error('Make sure the number of inputs <= 2');
    end
end