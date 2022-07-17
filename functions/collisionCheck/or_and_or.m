% @ Benji Z. Zhang

function [resultL, resultR] = or_and_or(L_A, R_A, L_B, R_B)
    % make sure elements in A and B are non-intersecting
    [AL, AR] = Or_interval(L_A, R_A); 
    [BL, BR] = Or_interval(L_B, R_B);
    % -- V.2.0 --
    if ~isempty(AL) && ~isempty(BL) % if AL is not empty, then AR is not empty as well  
        t =1;
        storeL = cell(1, length(AL)*length(BL));
        storeR = cell(1, length(AL)*length(BL));
        for i = 1: length(AL)
            for j = 1: length(BL) 
                [storeL{t}, storeR{t}] = And_interval([AL(i) BL(j)], [AR(i) BR(j)]); % null sets CAN be inputted into cell{t}
                t = t +1;
            end
        end
        [resultL, resultR] = Or_interval([storeL{1, :}], [storeR{1, :}]);
    else
        resultL = [];
        resultR = [];
    end
end