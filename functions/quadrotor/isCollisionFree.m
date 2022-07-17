% function to check collision or not for the given time
%
% @ Benji Z. Zhang

function flagCF = isCollisionFree(curT,intvCF)
    flagCF = 0;
    numIntvCF = size(intvCF,1);
    for i = 1:numIntvCF
        curIntvCF = intvCF(i,:);
        if curT >= curIntvCF(1,1) && curT <= curIntvCF(1,2)
            flagCF = 1;
            break;
        end
    end
end