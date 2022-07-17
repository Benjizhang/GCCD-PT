% function about substituting the polynomial A into the polynomial B
%
% @ Benji Z. Zhang

function pn = SubPoly(pnA, pnB)
    if length(pnB) <= 0
        error('Invalid Polynomial')
    end
    pn = 0;
    for i = 1:length(pnB)
        pwr = length(pnB)-i;
        coeff_temp = pnB(i)*PowerPoly(pnA, pwr);
        pn = AddPolyCoeff_2P(pn, coeff_temp);
    end
end