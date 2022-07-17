function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = zeros(n_order+1);
        for i=0:n_order
            if (i-4)>=0
                for j=0:n_order
                    if (j-4)>=0
                        curr_order = i+j-7;
                        Q_k(i+1, j+1)=(i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/curr_order)*ts(k)^curr_order/ts(k)^(2*4-3);
                    end
                end
            end
        end
        % generate Q matrix according to the previous jerk settings
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end