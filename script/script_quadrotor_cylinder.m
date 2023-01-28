% script to show the quadrotor moving toward a list of cylindrical obstacles
% the GCCD-PT can return the collision-free intervals (presented in Fig.5 (a))
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

%% generate & plot the cylindrical obstacles
area = [3 6.5 2.5 7 2.8 7.2]*100;
% define cylinder as two end pt & its radius
endPtsCy(1,:) = [3.5 3.5 2 5.5 3.5 8]*100;
cyRadius(1) = 0.25*100;
endPtsCy(2,:) = [4 5 2 4.5 5.5 8]*100;
cyRadius(2) = 0.25*100;
endPtsCy(3,:) = [6 4 2 5 5.5 8]*100;
cyRadius(3) = 0.3*100;

Cylinder = endPtsCy;
safe_dis = cyRadius;
% plot cylinder 
figure
set(gca,'position',[0.08,0.03,0.9,0.97])
set(gcf,'position',[100,100,810,810])
ax1=gca;
plotMultiCylinder(ax1,endPtsCy,cyRadius)
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
% start & end positions
waypts = [300, 300, 300;
         600, 650, 700];   
n_order = 7;  % 8 control points / 7-th degree of polynomial traj.
n_seg =1;     % one traj. includes n_seg-piece polynomial segments

% specify the duration (s) for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 3; % unit: sec
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
end
trajALL = {trajXALL,trajYALL,trajZALL};


%% -------- following is the key of this paper ---------
g = 9.80665*100; % cm
% geometry of quadrotor (in the local frame {0} in Fig. 3(a))
robpts_temp = [1 1; -1 1; -1 -1; 1 -1; 1 1; -1 1; -1 -1; 1 -1;]*0.3;
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
    % since Psi = 0, so its polynomial equation (a.k.a. pn) is given as 0.
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
    % cal. collision FREE intervals for the quadrotor and cylinders
    intvCF = calCFIntvSegCylinder(Eedge_i,Cylinder,path,safe_dis,tm_via)
    % -------- GCCD-PT --------

    pnxyx{1}=trajXALL{trajId};
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
legend([p1 p2],{'collision-free','collision'},'Location','best','FontSize',20)

%% animation for a path (should set 'isAnimate' = 1)
isAnimate = 0;
if isAnimate == 1
    figure
    ax2 = gca;
    set(gca,'position',[0.05,0.03,0.95,0.96] )
    set(gcf,'position',[0,0,1000,1000])
    g = 9.80665*100; % cm
    % specify which path to be animated
    trajId = 7;    
    % specify the video name
    videoname = ['T' num2str(trajId) '_10fps'];
    obsInfo{1} = endPtsCy;
    obsInfo{2} = cyRadius;
    animationUAVSegCyl(ax2,robpts,trajALL,trajId,obsInfo,tm_via,g,[260 area(2:end)],videoname)
end


























