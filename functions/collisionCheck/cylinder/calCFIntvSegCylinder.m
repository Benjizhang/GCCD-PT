% function to [cal]culate the [C]ollision [F]ree [Int]er[v]als between 
% edge/[Seg](i.e., edge of quadrotor) - [Cylinder]s(def: segment with given radius)
%
% Input: 
%      Eedge_i: \in R^{numEdge,6}, 6 columns contain the start(1-3 col) and end(4-6 col)
%               points of quadrotor edges (in local frame {0})
%      Eedge_j: \in R^{numCly*6}, 6 columns contain the start(1-3 col) and end(4-6 col)
%               points of cylinder
%      Path: cell includes coefficients of 3(for x,y,z)+6(sin/cos Phi,Theta, Psi) polynomial equations
%      safe_dis_Vec: radius of each cylinder
%      t_via: duration interval of traj.
% Output: 
%      intvCF: collision FREE intervals, where each row refers to one collision FREE interval.
%
% Please cite the following paper if you employ this function:
%      Zhang, Z., et al. (2022). "A Generalized Continuous Collision Detection Framework of 
%      Polynomial Trajectory for Mobile Robots in Cluttered Environments." 2022 IEEE Robotics
%      and Automation Letters. (DOI: 10.1109/LRA.2022.3191934)
%
% Benji Z. Zhang
% 2023/1


function intvCF = calCFIntvSegCylinder(Eedge_i,Eedge_j,Path,safe_dis_Vec,t_via)
    % ASSUMPE: UAV edge --- si (moving); cylinder axis --- sj (constant)
    if size(Eedge_i,2) ~= 6
        error('Invalid Eedge_i')
    end
    if size(Eedge_j,2) ~= 6
       error('Invalid Cylinder Info') 
    end
    if size(Eedge_j,1) ~= length(safe_dis_Vec)
       error('Invalid Cylinder Radius') 
    end
    
    % the end of time
    theta = t_via(2);
    
    collision2Cables = [];
    num_edge     = size(Eedge_i, 1);
    num_cylinder = size(Eedge_j,1);
    
    % path
    pn_x     = Path{1};
    pn_y     = Path{2};
    pn_z     = Path{3};
    pnSinPhi = Path{4};
    pnCosPhi = Path{5};
    pnSinTheta = Path{6};
    pnCosTheta = Path{7};
    pnSinPsi = Path{8};
    pnCosPsi = Path{9};

    % elements in the rotation matrix
    r11 = conv(pnCosTheta,pnCosPsi);
    r12 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnCosPsi),-conv(pnCosPhi,pnSinPsi));
    r13 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnCosPsi),conv(pnSinPhi,pnSinPsi));

    r21 = conv(pnCosTheta,pnSinPsi);
    r22 = AddPolyCoeff_2P(conv(conv(pnSinPhi,pnSinTheta),pnSinPsi),conv(pnCosPhi,pnCosPsi));
    r23 = AddPolyCoeff_2P(conv(conv(pnCosPhi,pnSinTheta),pnSinPsi),-conv(pnSinPhi,pnCosPsi));

    r31 = -pnSinTheta;
    r32 = conv(pnSinPhi,pnCosTheta);
    r33 = conv(pnCosPhi,pnCosTheta);
    
    for i = 1:num_edge
        % si in polynomial form
        % ---- local frame ----        
        % start pt
        Evixs = Eedge_i(i,1); % constant
        Eviys = Eedge_i(i,2); % constant
        Evizs = Eedge_i(i,3); % constant
        % end pt
        Evixe = Eedge_i(i,4); % constant
        Eviye = Eedge_i(i,5); % constant
        Evize = Eedge_i(i,6); % constant
        
        % ---- world frame ----
        % ---------------- for 3d uav --------------
        % start pt
        pnAix = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixs, r12*Eviys), r13*Evizs)));
        pnAiy = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixs, r22*Eviys), r23*Evizs)));
        pnAiz = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixs, r32*Eviys), r33*Evizs)));
        SegEndpt{1,1} = fill2mat(pnAix,pnAiy,pnAiz);
        % end pt
        pnBix = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixe, r12*Eviye), r13*Evize)));
        pnBiy = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixe, r22*Eviye), r23*Evize)));
        pnBiz = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixe, r32*Eviye), r33*Evize)));
        SegEndpt{1,2} = fill2mat(pnBix,pnBiy,pnBiz);
        % edge vector： e
        pnsix = pnBix - pnAix;
        pnsiy = pnBiy - pnAiy;
        pnsiz = pnBiz - pnAiz;        
        L{1} = fill2mat(pnsix, pnsiy, pnsiz); %\in 3*polyDeg(sin)
        % =============== for 3d uav ===============

        for j = 1:num_cylinder
            % sj CONSTANT
            % axis vector
            Ev_j = Eedge_j(j,4:6) - Eedge_j(j,1:3);
            sjx = Ev_j(1); % constant
            sjy = Ev_j(2); % constant
            sjz = Ev_j(3); % constant
            % ---- world frame ----
            % Aj: start pt of sj
            SegEndpt{2,1} = Eedge_j(j,1:3)'; % R^3
            % Bj: end pt of sj
            SegEndpt{2,2} = Eedge_j(j,4:6)'; % R^3
            % edge vector
            L{2} = [sjx; sjy; sjz]; %\in 3*1
                
            % s_ij = Aj - Ai
            pnsijx = polySimplify(AddPolyCoeff_2P(Eedge_j(j,1),-pnAix));
            pnsijy = polySimplify(AddPolyCoeff_2P(Eedge_j(j,2),-pnAiy));
            pnsijz = polySimplify(AddPolyCoeff_2P(Eedge_j(j,3),-pnAiz));
            
            %%% PowerPoly(,2)
            % d in polynomial form
            % 
            % temp1 := pnsix^2*sjy^2+pnsix^2*sjz^2-2*pnsix*pnsiy*sjx*sjy
            temp1 = AddPolyCoeff_2P(PowerPoly(pnsix,2)*sjy^2,AddPolyCoeff_2P(+PowerPoly(pnsix,2)*sjz^2,-2*(conv(pnsix,pnsiy))*sjx*sjy));
            % temp2 := -2*pnsix*pnsiz*sjx*sjz+pnsiy^2*sjx^2
            temp2 = AddPolyCoeff_2P(-2*conv(pnsix,pnsiz)*sjx*sjz,+PowerPoly(pnsiy,2)*sjx^2);
            % temp3 := +pnsiy^2*sjz^2-2*pnsiy*pnsiz*sjy*sjz
            temp3 = +AddPolyCoeff_2P(PowerPoly(pnsiy,2)*sjz^2,-2*conv(pnsiy,pnsiz)*sjy*sjz);
            % temp4 := +pnsiz^2*sjx^2+pnsiz^2*sjy^2
            temp4 = +AddPolyCoeff_2P(PowerPoly(pnsiz,2)*sjx^2,+PowerPoly(pnsiz,2)*sjy^2);
            d = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),AddPolyCoeff_2P(temp3,temp4));
            
            % nti in polynomial form
            % 
            % temp1 := pnsijx*pnsix*sjy^2+pnsijx*pnsix*sjz^2-pnsijx*pnsiy*sjx*sjy
            temp1 = AddPolyCoeff_2P(conv(pnsijx,pnsix)*sjy^2,AddPolyCoeff_2P(+conv(pnsijx,pnsix)*sjz^2,-conv(pnsijx,pnsiy)*sjx*sjy));
            % temp2 := -pnsijx*pnsiz*sjx*sjz-pnsijy*pnsix*sjx*sjy+pnsijy*pnsiy*sjx^2
            temp2 = AddPolyCoeff_2P(-conv(pnsijx,pnsiz)*sjx*sjz,AddPolyCoeff_2P(-conv(pnsijy,pnsix)*sjx*sjy,+conv(pnsijy,pnsiy)*sjx^2));
            % temp3 := +pnsijy*pnsiy*sjz^2-pnsijy*pnsiz*sjy*sjz-pnsijz*pnsix*sjx*sjz
            temp3 = AddPolyCoeff_2P(+conv(pnsijy,pnsiy)*sjz^2,AddPolyCoeff_2P(-conv(pnsijy,pnsiz)*sjy*sjz,-conv(pnsijz,pnsix)*sjx*sjz));
            % temp4 := -pnsijz*pnsiy*sjy*sjz+pnsijz*pnsiz*sjx^2+pnsijz*pnsiz*sjy^2
            temp4 = AddPolyCoeff_2P(-conv(pnsijz,pnsiy)*sjy*sjz,AddPolyCoeff_2P(+conv(pnsijz,pnsiz)*sjx^2,+conv(pnsijz,pnsiz)*sjy^2));
            nti = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),AddPolyCoeff_2P(temp3,temp4));
            
            % ntj in polynomial form
            % pnsijx*pnsix*pnsiy*sjy+pnsijx*pnsix*pnsiz*sjz-pnsijx*pnsiy^2*sjx-pnsijx*pnsiz^2*sjx-pnsijy*pnsix^2*sjy+pnsijy*pnsix*pnsiy*sjx+pnsijy*pnsiy*pnsiz*sjz-pnsijy*pnsiz^2*sjy-pnsijz*pnsix^2*sjz+pnsijz*pnsix*pnsiz*sjx-pnsijz*pnsiy^2*sjz+pnsijz*pnsiy*pnsiz*sjy
            % 
            % temp1 := pnsijx*pnsix*pnsiy*sjy+pnsijx*pnsix*pnsiz*sjz-pnsijx*pnsiy^2*sjx
            temp1 = AddPolyCoeff_2P(conv(pnsijx,conv(pnsix,pnsiy))*sjy,AddPolyCoeff_2P(+conv(pnsijx,conv(pnsix,pnsiz))*sjz,-conv(pnsijx,PowerPoly(pnsiy,2))*sjx));
            % temp2 := -pnsijx*pnsiz^2*sjx-pnsijy*pnsix^2*sjy+pnsijy*pnsix*pnsiy*sjx
            temp2 = AddPolyCoeff_2P(-conv(pnsijx,PowerPoly(pnsiz,2))*sjx,AddPolyCoeff_2P(-conv(pnsijy,PowerPoly(pnsix,2))*sjy,+conv(pnsijy,conv(pnsix,pnsiy))*sjx));
            % temp3 := +pnsijy*pnsiy*pnsiz*sjz-pnsijy*pnsiz^2*sjy-pnsijz*pnsix^2*sjz
            temp3 = AddPolyCoeff_2P(+conv(pnsijy,conv(pnsiy,pnsiz))*sjz,AddPolyCoeff_2P(-conv(pnsijy,PowerPoly(pnsiz,2))*sjy,-conv(pnsijz,PowerPoly(pnsix,2))*sjz));
            % temp4 := +pnsijz*pnsix*pnsiz*sjx-pnsijz*pnsiy^2*sjz+pnsijz*pnsiy*pnsiz*sjy
            temp4 = AddPolyCoeff_2P(+conv(pnsijz,conv(pnsix,pnsiz))*sjx,AddPolyCoeff_2P(-conv(pnsijz,PowerPoly(pnsiy,2))*sjz,+conv(pnsijz,conv(pnsiy,pnsiz))*sjy));
            ntj = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),AddPolyCoeff_2P(temp3,temp4));
            
            % np in polynomial form
            % -pnsijx*pnsiy*sjz+pnsijx*pnsiz*sjy+pnsijy*pnsix*sjz-pnsijy*pnsiz*sjx-pnsijz*pnsix*sjy+pnsijz*pnsiy*sjx
            % 
            % temp1 := -pnsijx*pnsiy*sjz+pnsijx*pnsiz*sjy+pnsijy*pnsix*sjz
            temp1 = AddPolyCoeff_2P(-conv(pnsijx,pnsiy)*sjz,AddPolyCoeff_2P(+conv(pnsijx,pnsiz)*sjy,+conv(pnsijy,pnsix)*sjz));
            % temp2 := -pnsijy*pnsiz*sjx-pnsijz*pnsix*sjy+pnsijz*pnsiy*sjx
            temp2 = AddPolyCoeff_2P(-conv(pnsijy,pnsiz)*sjx,AddPolyCoeff_2P(-conv(pnsijz,pnsix)*sjy,+conv(pnsijz,pnsiy)*sjx));
            np = AddPolyCoeff_2P(temp1,temp2);
            
            coeff_nti = nti;
            coeff_ntj = ntj;
            coeff_den = d;
            coeff_np  = np;            
            safe_dis = safe_dis_Vec(j);
            % ONLY consider nonparallel case
            if any(round(d,5)) 
                % ---------(MinDis is the length of Common Perpendicular Segment (C.P.S))-start-----------------------
                % Ref: C1 in eq. (24)
                % i.e. 0<ti & ti<1 & 0<tj & tj<1 & norm(C.P.S.)^2<dia^2
                % obtain the Coefficient of 5 Univariable Polynomials
                CoUniPol{1}  = coeff_nti;
                CoUniPol{2}  = coeff_ntj;
                CoUniPol{3}  = AddPolyCoeff_2P(coeff_den, -coeff_nti);
                CoUniPol{4}  = AddPolyCoeff_2P(coeff_den, -coeff_ntj);
                CoUniPol{5}  = AddPolyCoeff_2P((safe_dis^2)*coeff_den, -conv(coeff_np,coeff_np));            
                
                % ----------------- solver v2.0 ----------------------
                inequal = '>=';
                intv_CP = RealRootsIntersectSet(t_via(2), CoUniPol, inequal);
                clear CoUniPol;
                
                % ---------(MinDis is the distance between one end-pt and the other segment)-start-----------------*
                % Ref: C2-C5 below eq. (24)
                % ti<0 & 0<tj & tj<1 & invt(pt,seg)
                % ti<0 & 0<tj & tj<1
                CoUniPol{1} = -coeff_nti;
                CoUniPol{2} = coeff_ntj;
                CoUniPol{3} = AddPolyCoeff_2P(coeff_den, -coeff_ntj);
                % --- v2.0 ---
                % invt(pt,seg): pt=Ai, seg = lj
                pt = SegEndpt{1,1};
                segendpt = SegEndpt(2,:);
                seg = L{2};
                intv_Ailj = IntersectInvPtSeg2(safe_dis, t_via, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;
                
                % 0<ti & ti<1 & tj<0 & invt(pt,seg)
                % 0<ti & ti<1 & tj<0
                % [[[NA for the case of Fig.5 (a)]]]
                % [[[b/c in Fig.5 (a), the cylinder is higher than any point of quadrotor's path]]]
                % [[[i.e., we do not consider collision cases that includs two end points of cylinders]]]
    
                % 0<ti & ti<1 & tj>1 & invt(pt,seg)
                % 0<ti & ti<1 & tj>1
                % [[[NA for the case of Fig.5 (a)]]]                
    
                % 1<ti & 0<tj & tj<1 & invt(pt,seg)
                % 1<ti & 0<tj & tj<1
                CoUniPol{1} = AddPolyCoeff_2P(coeff_nti, -coeff_den);
                CoUniPol{2} = coeff_ntj;
                CoUniPol{3} = AddPolyCoeff_2P(coeff_den, -coeff_ntj);
                % --- v2.0 ---
                % invt(pt,seg): pt=Bi, seg = lj
                pt = SegEndpt{1,2};
                segendpt = SegEndpt(2,:);
                seg = L{2};
                intv_Bilj = IntersectInvPtSeg2(safe_dis, t_via, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;
                % ---------(MinDis is the distance between one end-pt and the other segment)-end-----------------*
                
                % ---------(MinDis occurs when 2 foot pts of C.P.S. are both beyond (0,1))-start-----------------#
                % Ref: C6-C9 below equation C2
                % ti>1 & tj>1
                % [[[NA for the case of Fig.5 (a)]]]
    
                % ti<0 & tj>1
                % [[[NA for the case of Fig.5 (a)]]]
                
                % ti>1 & tj<0
                % [[[NA for the case of Fig.5 (a)]]]
                
                % ti<0 & tj<0
                % [[[NA for the case of Fig.5 (a)]]]
                % ---------(MinDis occurs when 2 foot pts of C.P.S. are both beyond (0,1))-end-----------------#
    
                intv = [intv_CP; intv_Ailj; intv_Bilj];
                if ~isempty(intv)
                    Left = intv(:, 1);
                    R = intv(:, 2);
                    [L_intv, R_intv] = Or_interval(Left, R); % L_intv \in row vector form
                    % modify to open set (delete the isolated point)
                    tmp = round(L_intv - R_intv, 10);
                    L_intv(tmp==0) =[];
                    R_intv(tmp==0) =[];
                    out_temp = [L_intv', R_intv']; % n*2 matrix
                else
                    out_temp = [];
                end
                intvInterf = out_temp; % (t:\in [0,1])
    
                % Union set of collsion intervals
                collision2Cables = [collision2Cables; intvInterf];
            end
        end
    end
    
    %% Union sets of collision intervals for all edges
    if ~isempty(collision2Cables)
        Left = collision2Cables(:, 1);
        R = collision2Cables(:, 2);
        [L_intv, R_intv] = Or_interval(Left, R); % L_intv \in row vector form
        % modify to open set (delete the isolated point)
        tmp = round(L_intv - R_intv, 10);
        L_intv(tmp==0) =[];
        R_intv(tmp==0) =[];
        out_temp = [L_intv', R_intv']; % n*2 matrix
    else
        out_temp = [];
    end
    UnionCollisionIntv = out_temp;
    
    %% NO COLLISION intervals: Complement sets of the 'UnionCollisionIntv'
    minVal = 0;
    maxVal = t_via(2);
    if ~isempty(UnionCollisionIntv)
        [L_comp, R_comp] = Comp_interval(UnionCollisionIntv(:,1), UnionCollisionIntv(:,2));
        if ~isempty(L_comp)
            % results from fun'or_and_or' is closed intervals
            [resultL, resultR] = or_and_or(minVal, maxVal, L_comp, R_comp);
            tmp = round(resultL - resultR, 10);
            resultL(tmp==0) =[];
            resultR(tmp==0) =[];
            intvifw_temp = [resultL', resultR'];
        else
            intvifw_temp = [];
        end
    else
        intvifw_temp = [minVal, maxVal];
    end
    
    intvCF = intvifw_temp;
end