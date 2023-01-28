% using Sturm's Theorem [21] to check the roots existance in [t_via(1), t_via(2)]
% for ONE polynomial equation
%
% Benji Z. Zhang
% 2023/1

function rootsNum = sturmTheorem(pn,t_via)   
    
    X{ind(0)} = pn;
    X{ind(1)} = polyder(pn);
    
    if ~any(round(X{ind(1)},5)) 
        % i.e., X{ind(1)} = 0, X{ind(0)} is a constant
        if round(X{ind(0)},5) ==  0
            % in fact, infinite roots
            rootsNum = 100;
            return;
        else
            rootsNum = 0;
            return;
        end
    end

    k = 2;
    % genereate Sturm Sequence
    while true
        [~,rem_temp] = deconv(X{ind(k-2)},X{ind(k-1)});
        rem = simplifyPn(rem_temp);
        if any(round(rem,5)~=0)
            X{ind(k)} = -rem;        
            k = k+1;
        else
            break;
        end
    end
    % obtain the sign of the Sturm Sequence
    a = t_via(1);
    b = t_via(2);
    if a >= b
        error('left value < right value!')
    end
    signVec = -2*ones(2,length(X));
    for i = 1:length(X)
        % pn
        signVec(1,i) = sign(round(polyval(X{i},a),5));
        signVec(2,i) = sign(round(polyval(X{i},b),5));
    end

    % number of sign variations
    % counting zeros as NOT a sign change
    V = [0 0]';
    for row = 1:2
        tempVec = signVec(row,:);
        % ignore zeros
        tempVec(tempVec==0)=[];
        for k = 2:length(tempVec)
            prior_value = tempVec(k-1);
            current_value = tempVec(k);
            if (prior_value)*(current_value) == -1
                V(row) = V(row)+1;
            end
        end
    end

    % number of roots
    rootsNum = V(1)-V(2);
    if rootsNum < 0
        rootsNum = 0;
        % error('Negative Num of Roots')
    end
end

function index = ind(num)
    index = num+1;
end