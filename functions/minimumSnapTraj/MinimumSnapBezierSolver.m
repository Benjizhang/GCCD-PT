% func. to generate the coordinates of control points of the minimum snap
% Bezier trajctory
% Outputs:
%        ctlPt_: coordinates of control points of the resulting Bezier traj
%
% This func. is based on the following paper:
% F. Gao, et al. Online safe trajectory generation for quadrotors using 
% fast marching method and bernstein basis polynomial. 2018 IEEE ICRA.
% 
% Author: @Yinqiang Zhang (zyq507@connect.hku.hk)

function ctlPt_ = MinimumSnapBezierSolver(axis, waypoints, vBound, aBound, ts, n_seg, n_order, v_max, a_max)
    start_cond = [waypoints(1), vBound(1), aBound(1)];
    end_cond   = [waypoints(end), vBound(2), aBound(2)];
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order, ts);
    Q_0 = M'*Q*M; % the variables to be optimized are control points
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get Aieq and bieq
    [Aieq, bieq] = getAbieq2(n_seg, n_order, ts, v_max, a_max);
    
    f = zeros(size(Q_0,1),1);
    ctlPt_ = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end