% function to find collision or not by investigate the # of Real Roots
% of a set of polynomial equations using Sturm’s Theorem [21]
% NOTE: &{CoUniPoli>=0} = nothing => no collision interval
%       once any {CoUniPoli>=0} automatically NOT holds during t_via, then
%       no collision anymore
%
% Benji Z. Zhang
% 2023/1

function [isCollision, intvCollision] = calCheckRealRootsSetPolyv2(t_via, CoUniPol, inequal)
    if round(t_via(1),5) ~= 0
        error('Duration Must Start From 0')
    end
    numIneq = length(CoUniPol); % num of inequalities 
    toCheckIndLst = [];
    for i = 1:numIneq
        pn = CoUniPol{i};
        signA = sign(round(polyval(pn,t_via(1)),5));
        signB = sign(round(polyval(pn,t_via(2)),5));
        if signA ~= signB
            % it has one root at least
            toCheckIndLst = [toCheckIndLst i];
            continue;
        end
        rootsNum = sturmTheorem(pn,t_via);
        if rootsNum == 0 % no roots
            signa = sign(round(polyval(pn,t_via(1)),5));
            % signb = sign(round(polyval(pn,t_via(2)),5));
            if strcmp(inequal, '<=') ||  strcmp(inequal, '<')
                if signa == -1 % -1
                    % poly. inequalty automatically holds during t_Via 
                    % no need to solve out the exact roots
                elseif signa == 0
                    signb = sign(round(polyval(pn,t_via(2)),5));
                    if signb == -1
                        % poly. inequalty automatically holds during t_Via
                        % no need to solve out the exact roots
                    elseif signb == 1
                        % poly. inequalty automatically NOT holds
                        % must be no collision
                        isCollision = 0;
                        intvCollision = [];
                        return;    
                    else
                        error('sign of b can not be 0')                    
                    end
                elseif signa == 1
                    % poly. inequalty automatically NOT holds
                    % must be no collision
                    isCollision = 0;
                    intvCollision = [];
                    return;
                end                
            elseif strcmp(inequal, '>=') ||  strcmp(inequal, '>')
                if signa == 1 % 1
                    % poly. inequalty automatically holds during t_Via
                    % no need to solve out the exact roots
                elseif signa == 0
                    signb = sign(round(polyval(pn,t_via(2)),5));
                    if signb == 1
                        % poly. inequalty automatically holds during t_Via
                        % no need to solve out the exact roots
                    elseif signb == -1
                        % poly. inequalty automatically NOT holds
                        % must be no collision
                        isCollision = 0;
                        intvCollision = [];
                        return;    
                    else
                        error('sign of b can not be 0')                    
                    end
                elseif signa == -1
                    % poly. inequalty automatically NOT holds
                    % must be no collision
                    isCollision = 0;
                    intvCollision = [];
                    return;
                end
            else
                error('Invalid Inequality Sign!')
            end
        else
            % have roots
            % so we need to solve this poly. to get the exact roots in t_via
            toCheckIndLst = [toCheckIndLst i];
        end
    end
    if isempty(toCheckIndLst)
        % all poly. inequalities automatically holds
        % collision occurs during the whole t_via
        isCollision = 1;
        intvCollision = t_via;
        return;
    else
        pnSet = cell(1,length(toCheckIndLst));
        for j = 1:length(toCheckIndLst)
            pnSet{j} =  CoUniPol{toCheckIndLst(j)};
        end
        Real_invt = RealRootsIntersectSet(t_via(2), pnSet, inequal);
        if isempty(Real_invt)
            % no collision
            isCollision = 0;
            intvCollision = [];
        else
            isCollision = 1;
            intvCollision = Real_invt;
        end
    end

end