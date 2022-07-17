function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    max_deriv_num = 2;
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1);
    % the first control point in the first segment
    for deriv_order=0:max_deriv_num
        for k=0:deriv_order
            Aeq_start(deriv_order+1, deriv_order-k+1) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*ts(1)^(1-deriv_order);
        end
        beq_start(deriv_order+1) = start_cond(deriv_order+1);
    end
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    beq_end = zeros(3, 1);
    for deriv_order=0:max_deriv_num
        for k=0:deriv_order
            Aeq_end(deriv_order+1, end-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*ts(end)^(1-deriv_order);
        end
        beq_end(deriv_order+1) = end_cond(deriv_order+1);
    end
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    
    deriv_order = 0;
    for comm_point = 1:n_seg-1
        % predecesor
        pred_start_ind = (comm_point-1)*(n_order+1)+1;
        pred_end_ind = pred_start_ind+n_order;
        for k=0:deriv_order
            Aeq_con_p(comm_point, pred_end_ind-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*ts(comm_point)^(1-deriv_order);
        end
        % succesor
        succ_start_ind = pred_end_ind+1;
        for k=0:deriv_order
            Aeq_con_p(comm_point, succ_start_ind+deriv_order-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*-1*ts(comm_point+1)^(1-deriv_order);
        end
    end
    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    
    deriv_order = 1;
    for comm_point = 1:n_seg-1
        % predecesor
        pred_start_ind = (comm_point-1)*(n_order+1)+1;
        pred_end_ind = pred_start_ind+n_order;
        for k=0:deriv_order
            Aeq_con_v(comm_point, pred_end_ind-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*ts(comm_point)^(1-deriv_order);
        end
        % succesor
        succ_start_ind = pred_end_ind+1;
        for k=0:deriv_order
            Aeq_con_v(comm_point, succ_start_ind+deriv_order-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*-1*ts(comm_point+1)^(1-deriv_order);
        end
    end
    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    
    deriv_order = 2;
    for comm_point = 1:n_seg-1
        % predecesor
        pred_start_ind = (comm_point-1)*(n_order+1)+1;
        pred_end_ind = pred_start_ind+n_order;
        for k=0:deriv_order
            Aeq_con_a(comm_point, pred_end_ind-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*ts(comm_point)^(1-deriv_order);
        end
        % succesor
        succ_start_ind = pred_end_ind+1;
        for k=0:deriv_order
            Aeq_con_a(comm_point, succ_start_ind+deriv_order-k) = ...
                factorial(n_order)/factorial(n_order-deriv_order)* ...
                factorial(deriv_order)/(factorial(k)*factorial(deriv_order-k))*(-1)^(k)*-1*ts(comm_point+1)^(1-deriv_order);
        end
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end