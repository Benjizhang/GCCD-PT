% @ Benji Z. Zhang

function [L_comp, R_comp] = Comp_interval(L, R)
    if length(L) ~= length(R)
        error('Invaid Input');
    elseif max(L > R)
        error('Make sure L <= R');
    elseif isempty(L)
        warning('[WARNING] Empty Input')  % comp(null)
        L_comp = -inf;
        R_comp = +inf;
    else
        [L_ABC, R_ABC] = Or_interval(L, R);
        % initial value of L_accum and R_accum is comp. set of 1st interval
        [L_accum, R_accum] = Comp_interval_12(L_ABC(1), R_ABC(1)); % L_ABC(1) refers to one interval
        % cal. inters(A^c, B^c), where A^c refers to [L_accum, R_accum]
        for i= 2: length(L_ABC)
            temp1 = Comp_interval_12(L_ABC(i), R_ABC(i));
            if ~isempty(temp1) && ~isempty(L_accum)
                % cal. B^c, which is [L_comp_B, R_comp_B]
                [L_comp_B, R_comp_B] = Comp_interval_12(L_ABC(i), R_ABC(i));
                % cal. inters(A^c, B^c), the results go to L_accum etc.; 
                L_mat = {L_accum; L_comp_B};
                R_mat = {R_accum; R_comp_B};
                L_A = L_mat{1, :};
                R_A = R_mat{1, :};
                for n = 1 : size(L_mat, 1)-1
                    [resultL, resultR] = or_and_or(L_A, R_A, L_mat{n+1, :}, R_mat{n+1, :});
                    L_A = resultL;
                    R_A = resultR;
                end
                L_accum = L_A;
                R_accum = R_A;
            else
                L_accum = [];
                R_accum = [];
                break;
            end
        end
        L_comp = L_accum;
        R_comp = R_accum;
    end
end