% script to show the quadrotor moving toward a tilted ellipsoid
% the GCCD-PT can return the collision-free intervals (presented in Fig.1(b)) 
%
% Reference paper:
%      Zhang, Z., et al. (2022). "A Generalized Continuous Collision Detection Framework of 
%      Polynomial Trajectory for Mobile Robots in Cluttered Environments." 2022 IEEE Robotics
%      and Automation Letters. (DOI: 10.1109/LRA.2022.3191934)
%
% Auther: @Benji Z. Zhang (zzqing@connect.hku.hk)
% Date: 2022/07

clc; clear; close all;

%% root path
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
root_path = [filepath '\..'];
cd(root_path)

%% generate & plot the tilted ellipsoid
area = [3 6.5 2.5 7 2.8 7.2]*100;
degX = 40; %(degree)
degY = 30; %(degree)

% rotation matrix
Rx = rotx(degX);
Ry = roty(degY);
R = Ry*Rx;

% normal ellipsoid
xc = 5*100;
yc = 4.5*100;
zc = 5*100;
xr = 5*0.2*100;
yr = 5*0.2*100;
zr = 5*0.5*100;
A = [1/xr^2 0 0;
     0 1/yr^2 0;
     0 0 1/zr^2];
% generate normal ellipsoid
[X,Y,Z] = ellipsoid(xc,yc,zc,xr,yr,zr);

% tilted ellipsoid
sEllip = surf(X,Y,Z,'FaceAlpha',0.4,'EdgeColor','k');
directionX = [1 0 0]; % [NOTE] rotate in the local frame!!
rotate(sEllip,directionX,degX)
directionY = [0 1 0]; % [NOTE] rotate in the local frame!!
rotate(sEllip,directionY,degY)
% plot the shadow
hold on
surf(sEllip.XData,sEllip.YData,area(5)*ones(size(sEllip.YData)),'FaceColor',[0.85 0.85 0.85],'EdgeColor','none');
% fig config
view([-77 15])
axis equal
xlabel('x (cm)','FontWeight','bold')
ylabel('y (cm)','FontWeight','bold')
zlabel('z (cm)','FontWeight','bold')
axis(area)
hold on
grid on
box on
ax1=gca;
set(gca,'position',[0.08,0.03,0.9,0.97])
set(gcf,'position',[100,100,810,810])

%% transfer from {O} to {\tilde{O}}
% The inverse of a rotation matrix is its transpose, which is also a rotation matrix
A_bar = R*A*R'; % This is A in eq.(16)

% diagonal matrix Q of eigenvalues
% matrix Lambda whose columns are the corresponding right eigenvectors
% A_bar*Q = Q*Lambda.
[Q,Lambda] = eig(A_bar); % Ref: eq.(17)

%% generate the minimum snap trajectories
% start & end positions
waypts = [300, 300, 300;
         600, 650, 700];   
n_order = 7; % 8 control points / 7-th degree of polynomial traj.
n_seg =1;    % one traj. includes n_seg-piece polynomial segments

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
    
    % cancel comments to plot the path 
% %     pnxyz{1}=trajXALL{trajId};
% %     pnxyz{2}=trajYALL{trajId};
% %     pnxyz{3}=trajZALL{trajId};
% %     plotB3dTime3(ax1,pnxyz,[],[0,ts(1)],'c')
% %     hold on
end
trajALL = {trajXALL,trajYALL,trajZALL};


%% -------- following is the key of this paper ---------
g = 9.80665*100; % cm/s^2
% geometry of the quadrotor (in the local frame {0} in Fig. 3(a))
robpts_temp = [1 1; -1 1; -1 -1; 1 -1; 1 1; -1 1; -1 -1; 1 -1;]*0.3;
robpts = [robpts_temp [0.1*ones(4,1); -0.1*ones(4,1)]]*100;
% start points of edge
Er_s = robpts([end 1:(end-1)],:);
% end points of edge
Er_e = robpts;
% start & end pts of all edges
Eedge_i = [Er_s Er_e];

for trajId = 1
    % time interval of the traj.
    tm_via = [0 ts(1)];
    % polynomial equations for x,y,z of the trajId-th trajctory (Ref: eq.(13))
    path = getTrajXYZ3d(trajALL, trajId);
    % Psi = 0, so its polynomial equation (a.k.a. pn) is given as 0.
    pnpsi = 0;
    % estimate the sin() & cos() by polynomial equations (Ref: eq.(14))
    [pnSinPhi, pnCosPhi, maxErrPhi] = fitSinCosPhi(path{1},path{2},path{3},pnpsi,tm_via,g);
    [pnSinTheta, pnCosTheta, maxErrTheta] = fitSinCosTheta(path{1},path{2},path{3},pnpsi,tm_via,g);
    
    % traj. info (w.r.t. time t)
    path{4} = pnSinPhi;
    path{5} = pnCosPhi;
    path{6} = pnSinTheta;
    path{7} = pnCosTheta;
    path{8} = 0; % Psi = 0, so sin(Psi) = 0
    path{9} = 1; % Psi = 0, so cos(Psi) = 1
    
    ellip{1}=Lambda; % Lambda in eq.(17)
    ellip{2}=Q;      % Q in eq.(17)
    ellip{3}=[xc,yc,zc]; % c in eq.(16),(17)

    % -------- GCCD-PT --------
    % cal. collision FREE intervals for the quadrotor and the ellipsoid
    intvCF = calCFIntvSegEllip(Eedge_i,path,tm_via,ellip)
    % -------- GCCD-PT --------

    pnxyz{1}=trajXALL{trajId};
    pnxyz{2}=trajYALL{trajId};
    pnxyz{3}=trajZALL{trajId};
    if size(intvCF,1) == 2
        % plot start & end pos of collision
        plotCur6DofUAVFillBox3d(ax1,robpts,path,intvCF(1,2))
        plotCur6DofUAVFillBox3d(ax1,robpts,path,intvCF(2,1))
        % plot collision intervals & no collision intervals       
        plotB3dTime3(ax1,pnxyz,[],[intvCF(1,2),intvCF(2,1)],'r')
        plotB3dTime3(ax1,pnxyz,[],[0,intvCF(1,2)],'c')
        plotB3dTime3(ax1,pnxyz,[],[intvCF(2,1),tm_via(2)],'c')
    else
        plotB3dTime3(ax1,pnxyz,[],[tm_via(1),tm_via(2)],'c')
    end

    % plot goal
    plot3(ax1,waypts(2,1),waypts(2,2),waypts(2,3),'*k')
    % plot collision info
    view([-64 15])    
    txt1 = ['t_1 = ' num2str(round(intvCF(1,2),2)) ' s'];
    text(500, 350, 400,txt1,'FontSize',25)
    txt2 = ['t_2 = ' num2str(round(intvCF(2,1),2)) ' s'];
    text(400, 570, 620,txt2,'FontSize',25)
end

%% add trajectory id
text(600,550,650,'T1','FontSize',20)
% add 'Goal'
text(waypts(2,1),waypts(2,2),waypts(2,3)+20,'Goal','FontSize',20)
% add legend
w = [1 1 1; 1 2 1];
p1 = plot3(ax1,w(:,1),w(:,2),w(:,3),'c','LineWidth',1.2);
w = [2 1 1; 2 2 1];
p2 = plot3(ax1,w(:,1),w(:,2),w(:,3),'r','LineWidth',1.2);
legend([p1 p2],{'collision-free','collision'},'Location','best','FontSize',20)

%% animation for a path
%%% Note: Do NOT close/maximize/minimize the window, where the animation is playing
isAnimate = 1;
if isAnimate == 1
    figure
    ax2 = gca;
    cf = gcf;
    set(ax2,'position',[0.08,0.03,0.82,0.95])
    set(cf,'position',[920,100,910,810])
    trajId = 1;
    g = 9.80665*100; % cm
    % specify the video name
    videoname = ['T' num2str(trajId) '_10fps'];

    obsInfo.XData = sEllip.XData;
    obsInfo.YData = sEllip.YData;
    obsInfo.ZData = sEllip.ZData;
    animationUAVEllip(ax2,cf,robpts,trajALL,trajId,obsInfo,tm_via,g,[260 area(2:end)],videoname,intvCF)
end

