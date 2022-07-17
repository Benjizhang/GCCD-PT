% return the resulting polynomial after squaring the input polynomial eq.
% output = input^2
% input \in 3*n matrix
%
% @ Benji Z. Zhang

function output = SquarePoly(input)    
    x = input(1,:);
    y = input(2,:);
    z = input(3,:);
    output = conv(x,x)+conv(y,y)+conv(z,z);
end