% function to [cal]culate the [C]ollision [F]ree [Int]er[v]als between 
% edge/[Seg]ment(i.e., edge of quadrotor) - polyhedra(def: [Tri]angle [Mesh])
%
% Input: 
%      Eedge:   \in R^{numEdge,6}, 6 columns contain the start(1-3 col) and end(4-6 col)
%               points of quadrotor edges (in local frame {0})
%      TriMesh: \in R^{numTriMesh,9}, each row gives one triangle defined
%               by 3 points and each point has x,y,z coordinates
%      Path: cell includes coefficients of 3(for x,y,z)+6(sin/cos Phi,Theta, Psi) polynomial equations
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

function intvCF = calCFIntvSegTriMesh(Eedge,TriMesh,Path,t_via)
    if size(Eedge,2) ~= 6
        error('Invalid Eedge_i')
    end
    if size(TriMesh,2) ~= 9
       error('Invalid Triangle Mesh') 
    end
    num_edge = size(Eedge(1:4,:), 1);
    num_trimesh = size(TriMesh, 1);

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
    
    collisionSegTri = [];
    for i =  1:num_edge
        % edge ei in polynomial form
        % ---- local frame ----
        % start pt
        Evixs = Eedge(i,1); % constant 
        Eviys = Eedge(i,2); % constant
        Evizs = Eedge(i,3); % constant
        % end pt
        Evixe = Eedge(i,4); % constant
        Eviye = Eedge(i,5); % constant
        Evize = Eedge(i,6); % constant

        % ---- world frame ----
        % ---------------- for 3d uav --------------
        % start pt
        pnAix = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixs, r12*Eviys), r13*Evizs)));
        pnAiy = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixs, r22*Eviys), r23*Evizs)));
        pnAiz = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixs, r32*Eviys), r33*Evizs)));
        % SegEndpt{1,1} = fill2mat(pnAix,pnAiy,pnAiz);
        % end pt
        pnBix = polySimplify(AddPolyCoeff_2P(pn_x,AddPolyCoeff_2P(AddPolyCoeff_2P(r11*Evixe, r12*Eviye), r13*Evize)));
        pnBiy = polySimplify(AddPolyCoeff_2P(pn_y,AddPolyCoeff_2P(AddPolyCoeff_2P(r21*Evixe, r22*Eviye), r23*Evize)));
        pnBiz = polySimplify(AddPolyCoeff_2P(pn_z,AddPolyCoeff_2P(AddPolyCoeff_2P(r31*Evixe, r32*Eviye), r33*Evize)));
        % SegEndpt{1,2} = fill2mat(pnBix,pnBiy,pnBiz);
        % edge vector： e
        pnex = AddPolyCoeff_2P(pnBix, - pnAix);
        pney = AddPolyCoeff_2P(pnBiy, - pnAiy);
        pnez = AddPolyCoeff_2P(pnBiz, - pnAiz);
        % L{1} = fill2mat(pnex, pney, pnez); %\in 3*polyDeg(sin)
        % =============== for 3d uav ===============
        for j = 1:num_trimesh           
            % j-th triangle mesh --- wold frame ---
            V0j = TriMesh(j,1:3); % 1*3 
            V1j = TriMesh(j,4:6);
            V2j = TriMesh(j,7:9);

            % edge vector: e1 (contant)
            e1j = V1j - V0j;
            e1x = e1j(1);
            e1y = e1j(2);
            e1z = e1j(3);

            % edge vector: e2 (contant)
            e2j = V2j - V0j;
            e2x = e2j(1);
            e2y = e2j(2);
            e2z = e2j(3);

            % edge vector: e0 (pn form)
            pne0x = [pnAix(1:end-1) pnAix(end)-V0j(1)];
            pne0y = [pnAiy(1:end-1) pnAiy(end)-V0j(2)];
            pne0z = [pnAiz(1:end-1) pnAiz(end)-V0j(3)];
            
            % Note: following u0-u3 refers to v0-v3 in eq.(11) & eq.(27)
            % Q:=Matrix([-e,e1,e2])
            % u3 =det(Q)
            % u3 := -e1x*e2y*pnez+e1x*e2z*pney+e1y*e2x*pnez-e1y*e2z*pnex-e1z*e2x*pney+e1z*e2y*pnex
            temp1 = AddPolyCoeff_2P(-e1x*e2y*pnez,e1x*e2z*pney);
            temp2 = AddPolyCoeff_2P(+e1y*e2x*pnez,-e1y*e2z*pnex);
            temp3 = AddPolyCoeff_2P(-e1z*e2x*pney,+e1z*e2y*pnex);
            u3 = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),temp3);

            % u0 =det([e0 e1 e2])
            % u0 := e1x*e2y*pne0z-e1x*e2z*pne0y-e1y*e2x*pne0z+e1y*e2z*pne0x+e1z*e2x*pne0y-e1z*e2y*pne0x
            temp1 = AddPolyCoeff_2P(e1x*e2y*pne0z,-e1x*e2z*pne0y);
            temp2 = AddPolyCoeff_2P(-e1y*e2x*pne0z,+e1y*e2z*pne0x);
            temp3 = AddPolyCoeff_2P(+e1z*e2x*pne0y,-e1z*e2y*pne0x);
            u0 = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),temp3);

            % u1 =det([-e e0 e2])
            % u1 := e2x*pne0y*pnez-e2x*pne0z*pney-e2y*pne0x*pnez+e2y*pne0z*pnex+e2z*pne0x*pney-e2z*pne0y*pnex
            temp1 = AddPolyCoeff_2P(e2x*conv(pne0y,pnez),-e2x*conv(pne0z,pney));
            temp2 = AddPolyCoeff_2P(-e2y*conv(pne0x,pnez),+e2y*conv(pne0z,pnex));
            temp3 = AddPolyCoeff_2P(+e2z*conv(pne0x,pney),-e2z*conv(pne0y,pnex));
            u1 = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),temp3);

            % u2 =det([-e e1 e0])
            % u2 := -e1x*pne0y*pnez+e1x*pne0z*pney+e1y*pne0x*pnez-e1y*pne0z*pnex-e1z*pne0x*pney+e1z*pne0y*pnex
            temp1 = AddPolyCoeff_2P(-e1x*conv(pne0y,pnez),+e1x*conv(pne0z,pney));
            temp2 = AddPolyCoeff_2P(+e1y*conv(pne0x,pnez),-e1y*conv(pne0z,pnex));
            temp3 = AddPolyCoeff_2P(-e1z*conv(pne0x,pney),+e1z*conv(pne0y,pnex));
            u2 = AddPolyCoeff_2P(AddPolyCoeff_2P(temp1,temp2),temp3);
            
                
            % form collision conditions （5 poly. inequalities in eq.(12)）
            % Note: following u0-u3 refers to v0-v3 in eq.(11)
            CoUniPol = cell(1,5);
            tempp1 = conv(u3,u3); % Note: here we multiply v3 both at numerator and denominator of eq.(11)
            tempp2 = conv(u3,u0); % Note: thus, no need to judge the denominator of eq.(11) v3 to be >0 or <0
            tempp3 = conv(u3,u1); % Note: so, it is a little bit different process to calculate C_{ET}^{lm} below eq.(27)
            tempp4 = conv(u3,u2);
            % 1. k     >= 0
            CoUniPol{1} = tempp2;
            % 2. k     <= 1            
            CoUniPol{2} = AddPolyCoeff_2P(tempp1,-tempp2);
            % 3. k1    >= 0
            CoUniPol{3} = tempp3;
            % 4. k2    >= 0
            CoUniPol{4} = tempp4;
            % 5. k1+k2 <= 1
            tempp5 = AddPolyCoeff_2P(tempp3,tempp4);
            CoUniPol{5} = AddPolyCoeff_2P(tempp1,-tempp5);
            
            % ----------------- polynomial inequality solver ----------------------
            inequal = '>=';
            [~, intvColli] = calCheckRealRootsSetPolyv2(t_via, CoUniPol, inequal);
            clear CoUniPol;
            
            intv = intvColli;
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
            collisionSegTri = [collisionSegTri; intvInterf];

        end
    end
     %% Union sets of collision intervals for all edges
    if ~isempty(collisionSegTri)
        Left = collisionSegTri(:, 1);
        R = collisionSegTri(:, 2);
        [L_intv, R_intv] = Or_interval(Left, R); % L_intv \in row vector form
        % modify to open set (delete the isolated point)
        tmp = round(L_intv - R_intv, 10);
        L_intv(tmp==0) =[];
        R_intv(tmp==0) =[];
        out_temp = [L_intv', R_intv']; % n*2 matrix
    else
        out_temp = [];
    end
    UnionCollisionRays = out_temp;

    %% NO COLLISION intervals: Complement sets of the 'UnionCollisionRays'
    minVal = t_via(1);
    maxVal = t_via(2);
    if ~isempty(UnionCollisionRays)
        [L_comp, R_comp] = Comp_interval(UnionCollisionRays(:,1), UnionCollisionRays(:,2));
        if ~isempty(L_comp)
            % results from fun'or_and_or' is closed intervals
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
    tinyInd = round(intvcf_temp(:,2)-intvcf_temp(:,1),2)<0.01;
    intvcf_temp(tinyInd,:)=[];
    intvCF = intvcf_temp;
end
