% function to calculate the collision free intervals between 
% edge/seg(i.e., the edge of quadrotor)-ellipsoid
%
% Input: 
%      Eedge_i: 6 columns contain the start(1-3 col) and end(4-6 col)
%               points of quadrotor edges (in local frame {0})
%      Path: cell includes coefficients of 3(for x,y,z)+6(sin/cos Phi,Theta, Psi) polynomial equations
%      t_via: duration interval of traj.
%      ellip: info of the ellipsoid
% Output: 
%      intvCF: collision FREE intervals, where each row refers to one collision FREE interval.
%
% Please cite the following paper if you employ this function:
%      Zhang, Z., et al. (2022). "A Generalized Continuous Collision Detection Framework of 
%      Polynomial Trajectory for Mobile Robots in Cluttered Environments." 2022 IEEE Robotics
%      and Automation Letters. (DOI: 10.1109/LRA.2022.3191934)
%
% Benji Z. Zhang
% 12/2021

function intvCF = calCFIntvSegEllip(Eedge_i,Path,t_via,ellip)
    if size(Eedge_i,2) ~= 6
        error('Invalid Eedge_i')
    end

    % parameters of the ellipsoid
    Lambda = ellip{1}; % Ref: eq.(17)
    Q = ellip{2};      % Ref: eq.(17)
    cent = ellip{3};   % Ref: c in eq.(16),(17)/ Vp in Fig. 4(a)
    
    % Collect All Collision Intervals of all quadrotor edges
    caci = [];
    num_edge_i = size(Eedge_i, 1);
    
    % trajectory elements in polynomial forms
    pn_x     = Path{1};
    pn_y     = Path{2};
    pn_z     = Path{3};
    pnSinPhi = Path{4};
    pnCosPhi = Path{5};
    pnSinTheta = Path{6};
    pnCosTheta = Path{7};
    pnSinPsi = Path{8};
    pnCosPsi = Path{9};

    % elements in the rotation matrix (Ref: eq.(3))
    r11 = conv(pnCosTheta,pnCosPsi);
    r12 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnCosPsi),-conv(pnCosPhi,pnSinPsi));
    r13 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnCosPsi),conv(pnSinPhi,pnSinPsi));

    r21 = conv(pnCosTheta,pnSinPsi);
    r22 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnSinPsi),conv(pnCosPhi,pnCosPsi));
    r23 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnSinPsi),-conv(pnSinPhi,pnCosPsi));

    r31 = -pnSinTheta;
    r32 = conv(pnSinPhi,pnCosTheta);
    r33 = conv(pnCosPhi,pnCosTheta);
    
    %% cal. collision intervals between each edge and the ellipsoid
    for i = 1:num_edge_i
        % ---- local frame {0} ----
        % Ref: Fig. 3(a)
        % start point of quadrotor edge
        Evixs = Eedge_i(i,1); % constant
        Eviys = Eedge_i(i,2); % constant
        Evizs = Eedge_i(i,3); % constant
        % end point of quadrotor edge
        Evixe = Eedge_i(i,4); % constant
        Eviye = Eedge_i(i,5); % constant
        Evize = Eedge_i(i,6); % constant
        
        % ---- world frame {O} ----
        % Ref: Fig. 3(a)
        % p_s in eq.(19): start point of quadrotor edge (by eq.(1))
        pnAix_temp = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixs, r12*Eviys), r13*Evizs)));
        pnAiy_temp = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixs, r22*Eviys), r23*Evizs)));
        pnAiz_temp = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixs, r32*Eviys), r33*Evizs)));
        % p_e in eq.(19): end point of quadrotor edge (by eq.(1))
        pnBix_temp = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixe, r12*Eviye), r13*Evize)));
        pnBiy_temp = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixe, r22*Eviye), r23*Evize)));
        pnBiz_temp = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixe, r32*Eviye), r33*Evize)));
        
        % ---- frame {tilde{O}} ----
        % Ref: Fig. 4(b)
        % tilde{p}_s in eq.(19) (by affine transformation(AT) in eq.(17))
        [pnAix, pnAiy, pnAiz]= ellipsoidTransfer(Lambda, Q, cent, pnAix_temp, pnAiy_temp, pnAiz_temp);        
        SegEndpt{1,1} = fill2mat(pnAix,pnAiy,pnAiz);
        % tilde{p}_e in eq.(19) (by affine transformation(AT) in eq.(17))
        [pnBix, pnBiy, pnBiz]= ellipsoidTransfer(Lambda, Q, cent, pnBix_temp, pnBiy_temp, pnBiz_temp);
        SegEndpt{1,2} = fill2mat(pnBix,pnBiy,pnBiz);
        
        % ## Since in Euclidean space, the affine transformation (AT) eq.(17) is a geometric transformation that preserves lines,
        % ## then the edge tilde{e}, transformed from the edge e := VsVe in frame {O}
        % ## by eq.(17), is still an edge in frame {tilde{O}} (Ref: Sec. 4-B-1))
        % edge tilde{e} = AT(edge e) (Ref: edge e in eq.(2))
        pnsix = pnBix - pnAix;
        pnsiy = pnBiy - pnAiy;
        pnsiz = pnBiz - pnAiz;
        % tilde{e} in eq.(19)
        L{1} = fill2mat(pnsix, pnsiy, pnsiz); %\in 3*polyDeg(sin)
        
        % ---- collision conditions for Edge-Point(origin) in frame {tilde{O}} ----
        % ## dEP(tilde{e}, tilde{O})<=1 (Ref: Sec. 4-B-1))
        % ## dEP() is eq.(4) given in Sec. 3-B-1).
        safe_dis = 1;    % unit sphere
        pt_j = [0 0 0]'; % centered at the origin (Ref: Fig. 4(b))
        % tilde{p}_s & tilde{p}_e in eq.(19)：start & end points of edge tilde{e}
        edgeendpt = SegEndpt(1,:);
        % tilde{e} in eq.(19) 
        edge = L{1};
        
        % ---- collision intervals for ONE edge and the ellipsoid ----
        % i.e., C_{EP}^l in eq.(20)
        intv_PS =  MinDisPtSeg2(safe_dis, t_via(2), pt_j, edgeendpt, edge);

        % collect all collsion intervals
        caci = [caci; intv_PS];
    end
    
    %% Union set of COLLISION intervals for all edges
    if ~isempty(caci)
        Left = caci(:, 1);
        R = caci(:, 2);
        [L_intv, R_intv] = Or_interval(Left, R);
        tmp = round(L_intv - R_intv, 10);
        L_intv(tmp==0) =[];
        R_intv(tmp==0) =[];
        out_temp = [L_intv', R_intv']; % n*2 matrix
    else
        out_temp = [];
    end
    % ---- collision intervals for ALL edges of quadrotor and the ellisoid ----
    % i.e., C_{Elip} = U^{epsilon}_{l=1} C_{EP}^l below eq.(20)
    UnionCollisionIntv = out_temp;
    
    %% COLLISION FREE interval: Complement set of the 'UnionCollisionIntv'
    minVal = t_via(1);
    maxVal = t_via(2);
    if ~isempty(UnionCollisionIntv)
        [L_comp, R_comp] = Comp_interval(UnionCollisionIntv(:,1), UnionCollisionIntv(:,2));
        if ~isempty(L_comp)
            [resultL, resultR] = or_and_or(minVal, maxVal, L_comp, R_comp);
            tmp = round(resultL - resultR, 10);
            resultL(tmp==0) =[];
            resultR(tmp==0) =[];
            intvcf_temp = [resultL', resultR'];
        else
            intvcf_temp = [];
        end
    else
        intvcf_temp = [minVal, maxVal];
    end
    
    % ---- collision FREE intervals between quadrotor and ellipsoid ----
    intvCF = intvcf_temp;
end
