% @ Benji Z. Zhang

function [L_or, R_or] = Or_interval(L, R)
    if length(L) ~= length(R)
        error('Invaid Input');
    elseif max(L > R)                
        error('Make sure L <= R');
    elseif isempty(L) % the input is one null interval
        warning('[WARNING] Empty Input') % union(a null space)
        L_or = [];
        R_or = [];
    else
        % sort input in ascending order 
        [L, I] = sort(L);
        R_temp = zeros(1, length(L));
        for i = 1: length(L)
            index = I(i);
            R_temp(i) = R(index);
        end
        R = R_temp;
        % cal. union interval ONE by ONE 
        t = 1;
        L_store(t) = L(1); % store the finial results
        R_store(t) = R(1);
        for i = 1: length(L)-1
            temp = And_interval([L_store(t) L(i+1)], [R_store(t) R(i+1)]); 
            if isempty(temp) % two intervals are non-intersecting
                L_store(t+1) = L(i+1); % L(i+1) is non-null, so L_store(t+1) is non-null
                R_store(t+1) = R(i+1);
                t = t + 1;
            else % two intervals have intersections
                [L_2inter, R_2inter] = Or_interval_2([L_store(t) L(i+1)], [R_store(t) R(i+1)]); % result is one interval
                L_store(t) = L_2inter; R_store(t) = R_2inter;
            end
        end
        L_or = L_store;
        R_or = R_store;
    end
end