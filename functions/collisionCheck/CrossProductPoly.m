% cross product of two polynomial eq.
%
% @ Benji Z. Zhang

function y = CrossProductPoly(x1, x2)
    % input should be of column 'vector' form
    lix = x1(1, :);
    liy = x1(2, :);
    liz = x1(3, :);
    ljx = x2(1, :);
    ljy = x2(2, :);
    ljz = x2(3, :);
    y_x = conv(liy, ljz)-conv(liz, ljy);
    y_y = conv(liz, ljx)-conv(lix, ljz); 
    y_z = conv(lix, ljy)-conv(liy, ljx);
    y = [y_x; y_y; y_z];
end
