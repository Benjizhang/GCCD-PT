% fill to the same length of vector of input polynomials
% pnMat: 3 rows
%
% @ Benji Z. Zhang

function pnMat = fill2mat(pn1,pn2,pn3)
    max_len = max([length(pn1),length(pn2),length(pn3)]);
    pnNew1 = padarray(pn1, [0 max_len-length(pn1)], 'pre');
    pnNew2 = padarray(pn2, [0 max_len-length(pn2)], 'pre');
    pnNew3 = padarray(pn3, [0 max_len-length(pn3)], 'pre');
    pnMat = [pnNew1; pnNew2; pnNew3];
end