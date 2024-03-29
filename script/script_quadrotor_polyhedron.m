% script to show the quadrotor moving toward a list of polyhedra (triangle mesh)
% the GCCD-PT can return the collision-free intervals (presented in Fig.5 (b)/(c))
%
% Reference paper:
%      Zhang, Z., et al. (2022). "A Generalized Continuous Collision Detection Framework of 
%      Polynomial Trajectory for Mobile Robots in Cluttered Environments." 2022 IEEE Robotics
%      and Automation Letters. (DOI: 10.1109/LRA.2022.3191934)
%
% Auther: @Benji Z. Zhang (zzqing@connect.hku.hk)
% Date: 2023/01


clc; clear; close all;

%% root path
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
root_path = [filepath '\..'];
cd(root_path)

%% generate & plot polyhedra (triangle mesh)
area = [3 6.5 2.5 7 2.8 7.2]*100;
% one row: one triangle mesh defined by 3 points
tri(1,:) = [5 3.5 3 4.5 5 3 5.5 4.5 3]*100;
tri(2,:) = [5 4.5 7 4.5 5 3 5.5 4.5 3]*100;
tri(3,:) = [5 3.5 3 5 4.5 7 5.5 4.5 3]*100;
tri(4,:) = [5 3.5 3 4.5 5 3 5 4.5 7]*100;
tri(5,:) = [3 5.5 3 4.5 5 7 3 5.5 7]*100;

TriMesh = tri(1:5,:);
triPlot = tri;
% plot triangle mesh
figure
set(gca,'position',[0.1,0.1,0.85,0.85])
set(gcf,'position',[10,10,800,800])
ax1=gca;
triMeshX = triPlot(:,[1 4 7])';
triMeshY = triPlot(:,[2 5 8])';
triMeshZ = triPlot(:,[3 6 9])';
patch(ax1, triMeshX,triMeshY,triMeshZ,[.8,.8,.8],'EdgeColor','k','FaceAlpha',0.6,'LineWidth',1)
view([-77 15])
axis equal
xlabel('x (cm)','FontWeight','bold')
ylabel('y (cm)','FontWeight','bold')
zlabel('z (cm)','FontWeight','bold')
axis(area)
hold on
grid on
box on

%% generate the minimum snap trajectories
waypts = [300, 300, 300;
         600, 650, 700];   
n_order = 7;  % 8 control points / 7-th degree of polynomial traj.
n_seg =1;     % one traj. includes n_seg-piece polynomial segments
% specify the duration (s) for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 3;
end

% generate a set of traj. accroding to different start velocities
num_traj = 7;
deltaAngle = deg2rad(90/(num_traj-1));
for trajId = 1: num_traj
    % start & end conditions of velocity
    iniVx = 320*cos(deltaAngle*(trajId-1));
    iniVy = 320*sin(deltaAngle*(trajId-1));
    vBound = [iniVx iniVy 200;
              0 0 0];
    % start & end conditions of acceleration
    aBound = [0 0 0;
              0 0 0];
    % maximum vel & acc
    v_max = 400;
    a_max = 400;
    
    % coordinates of control points of the resulting minimum sanp Bezier traj.
    % Note: x,y,z in polynormal forms
    ctlPt_x = MinimumSnapBezierSolver(1, waypts(:, 1), vBound(:, 1), aBound(:, 1), ts, n_seg, n_order, v_max, a_max);
    ctlPt_y = MinimumSnapBezierSolver(2, waypts(:, 2), vBound(:, 2), aBound(:, 2), ts, n_seg, n_order, v_max, a_max);
    ctlPt_z = MinimumSnapBezierSolver(3, waypts(:, 3), vBound(:, 3), aBound(:, 3), ts, n_seg, n_order, v_max, a_max);
    
    if isempty(ctlPt_x) || isempty(ctlPt_y) || isempty(ctlPt_z)
        error('No Optimal Control Pts.')
    end
    ctlPt_x_temp = ctlPt_x;
    ctlPt_y_temp = ctlPt_y;
    ctlPt_z_temp = ctlPt_z;
    controlPts = ts(1)*[ctlPt_x_temp ctlPt_y_temp ctlPt_z_temp];
    % generate bezier polynomial eq. from control pts. (w.r.t. normalized para. s)
    [pnx, pny, pnz] = generateBezierCurve3dGeneral(controlPts);
    % convert bezier polynomial eq. from s \in [0,1] to time t \in [0,ts(1)]
    trajXALL{trajId} = SubPoly([1/ts(1) 0], pnx);
    trajYALL{trajId} = SubPoly([1/ts(1) 0], pny);
    trajZALL{trajId} = SubPoly([1/ts(1) 0], pnz);
    
    % uncomment to plot whole paths 
    %%% pnxyz{1}=trajXALL{trajId};
    %%% pnxyz{2}=trajYALL{trajId};
    %%% pnxyz{3}=trajZALL{trajId};
    %%% plotB3dTime3(ax1,pnxyz,[],[0,ts(1)],'c')
    %%% hold on
end
trajALL = {trajXALL,trajYALL,trajZALL};

%% -------- following is the key of this paper ---------
g = 9.80665*100; % cm

% geometry of quadrotor (in the local frame {0} in Fig. 3(a))
% Note: uncomment to select quadrotors with different sizes
robpts_temp = [1 1; -1 1; -1 -1; 1 -1; 1 1; -1 1; -1 -1; 1 -1;]*0.3; % Fig. 5(b)
% robpts_temp = [1 1; -1 1; -1 -1; 1 -1; 1 1; -1 1; -1 -1; 1 -1;]*0.2;
% robpts_temp = [1 1; -1 1; -1 -1; 1 -1; 1 1; -1 1; -1 -1; 1 -1;]*0.1; % Fig. 5(c)

robpts = [robpts_temp [0.1*ones(4,1); -0.1*ones(4,1)]]*100;
% start points of edge
Er_s = robpts([end 1:(end-1)],:);
% end points of edge
Er_e = robpts;
% start & end pts of all edgess
Eedge_i = [Er_s Er_e];

for trajId = 1:num_traj
    % time interval of the traj.
    tm_via = [0 ts(1)];
    % polynomial equations for x,y,z of the trajId-th trajctory (Ref: eq.(13))
    path = getTrajXYZ3d(trajALL, trajId);
    % since Psi = 0, so its polynomial equation (a.k.a. pn) is given as 0
    pnpsi = 0;
    % estimate the sin() & cos() by polynomial equations (Ref: eq.(14))
    [pnSinPhi, pnCosPhi, maxErrPhi] = fitSinCosPhi(path{1},path{2},path{3},pnpsi,tm_via,g);
    [pnSinTheta, pnCosTheta, maxErrTheta] = fitSinCosTheta(path{1},path{2},path{3},pnpsi,tm_via,g);
    [pnSinPsi, pnCosPsi, maxErrPsi] = fitSinCosPsi(path{1},path{2},path{3},pnpsi,tm_via,g);
    
    % path info (w.r.t. time t)
    path{4} = pnSinPhi;
    path{5} = pnCosPhi;
    path{6} = pnSinTheta;
    path{7} = pnCosTheta;
    path{8} = 0; % Psi = 0, so sin(Psi) = 0
    path{9} = 1; % Psi = 0, so cos(Psi) = 1    

    % -------- GCCD-PT --------
    % cal. collision FREE intervals for the quadrotor and polyhedra (triangle mesh)
    intvCF = calCFIntvSegTriMesh(Eedge_i,TriMesh,path,tm_via)
    % -------- GCCD-PT --------

    pnxyz{1}=trajXALL{trajId};
    pnxyz{2}=trajYALL{trajId};
    pnxyz{3}=trajZALL{trajId};
    if size(intvCF,1) >= 2
        num_IF = size(intvCF,1);
        for k = 1:num_IF-1
            % uncomment to plot quadrotor pos at start & end pts of collision
            %%% plotCur6DofUAVFillBox3d(ax1,robpts,path,intvCF(k,2))
            %%% plotCur6DofUAVFillBox3d(ax1,robpts,path,intvCF(k+1,1))
            % plot collision interval & no collision interval   
            plotB3dTime3(ax1,pnxyz,[],[intvCF(k,1),intvCF(k,2)],'c')
            plotB3dTime3(ax1,pnxyz,[],[intvCF(k,2),intvCF(k+1,1)],'r')
        end
        plotB3dTime3(ax1,pnxyz,[],[intvCF(num_IF,1),intvCF(num_IF,2)],'c')
    else
        plotB3dTime3(ax1,pnxyz,[],[tm_via(1),tm_via(2)],'c')
    end

    % plot goal
    plot3(ax1,waypts(2,1),waypts(2,2),waypts(2,3),'*k')
    view([-64 15])
end

%% add trajectory id
for kk = 1:num_traj
    txtTraj = ['T' num2str(kk)];
    text(600-55*(kk-1),550,650,txtTraj,'FontSize',20)
end
% add 'Goal'
text(waypts(2,1),waypts(2,2),waypts(2,3)+20,'Goal','FontSize',20)
% add legend
w = [1 1 1; 1 2 1];
p1 = plot3(ax1,w(:,1),w(:,2),w(:,3),'c','LineWidth',1.2);
w = [2 1 1; 2 2 1];
p2 = plot3(ax1,w(:,1),w(:,2),w(:,3),'r','LineWidth',1.2);

% set legend to the custom position 
h = legend([p1 p2],{'collision-free','collision'},'FontSize',20);
rect = [0.09, 0.89, .2, .01];
set(h, 'Position', rect)

%% animation for a path (should set 'isAnimate' = 1)
isAnimate = 0;
if isAnimate == 1
    figure
    ax2 = gca;
    set(gca,'position',[0.05,0.03,0.95,0.96] )
    set(gcf,'position',[0,0,1000,1000])
    g = 9.80665*100; % cm
    % select which path to be animated
    trajId = 5;
    % specify the video name
    videoname = ['T' num2str(trajId) '_10fps'];    
    animationUAVSegTri2(ax2,robpts,trajALL,trajId,TriMesh,tm_via,g,[260 area(2:end)],videoname)
end



