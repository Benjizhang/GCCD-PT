% function to cal. the coefficients of power of the polynomial
%
% @ Benji Z. Zhang

function coeff = PowerPoly(poly, pwr)
    res = 1;
    pwr = round(pwr,6);
    if pwr < 0
        error('Not Support for Negative Power')
    end
    
    if pwr == 0
        % do nothing
    else
        for k1 = 1:pwr
            res = conv(res, poly);
        end
    end
    coeff = res;
end