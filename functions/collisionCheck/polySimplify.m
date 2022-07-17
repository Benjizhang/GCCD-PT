% function to clear 0 from left to right of the input polynomial
%
% @ Benji Z. Zhang

function pn = polySimplify(pn_input)
    for i = 1:length(pn_input)
        if round(pn_input(i),7) == 0
            pn_input(i) = NaN;
        else
            break
        end
    end
    pn_input(isnan(pn_input)) = [];
    if isempty(pn_input)
        pn = 0;
    else
        pn = pn_input;
    end
end

