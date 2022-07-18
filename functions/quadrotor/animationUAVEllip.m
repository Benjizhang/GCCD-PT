% function to animate uav along a path
% for ellipsoid ONLY
% with the given name of the video
% 
% @ Benji Z. Zhang

function animationUAVEllip(ax,robpts,trajALL,trajId,obsInfo,tm_via,g,area,videoName,intvIF)

    % get path (x,y,z)
    path = getTrajXYZ3d(trajALL, trajId);
    pnpsi = 0;
    % get path (orientation)
    [pnSinPhi, pnCosPhi, ~] = fitSinCosPhi(path{1},path{2},path{3},pnpsi,tm_via,g);
    [pnSinTheta, pnCosTheta, ~] = fitSinCosTheta(path{1},path{2},path{3},pnpsi,tm_via,g);    
    [pnPhi, pnTheta, ~] = fitPhiTheta(path{1},path{2},path{3},pnpsi,tm_via,g);

    % goal
    goalx = polyval(path{1},tm_via(2));
    goaly = polyval(path{2},tm_via(2));
    goalz = polyval(path{3},tm_via(2));
    
    % path info (w.r.t. time)
    path{4} = pnSinPhi;
    path{5} = pnCosPhi;
    path{6} = pnSinTheta;
    path{7} = pnCosTheta;
    path{8} = 0;
    path{9} = 1;

    cnt = 0;
    t_f = 10;
    % 50 fps
    for curT = tm_via(1):0.02:tm_via(2)
        cnt = cnt + 1;
        hold off
        plotCur6DofUAVFillBox3d(ax,robpts,path,curT)
        % update x,y,z,phi,theta,psi 
        curx = polyval(path{1},curT);
        cury = polyval(path{2},curT);
        curz = polyval(path{3},curT);
        curPhi = polyval(pnPhi,curT); % rad
        curTheta = polyval(pnTheta,curT); % rad
        txtPx = ['x: ' num2str(round(curx,2))] ;
        txtPy = ['y: ' num2str(round(cury,2))];
        txtPz = ['z: ' num2str(round(curz,2))];
        txtPphi = ['\phi: ' num2str(round(curPhi,2))];
        txtPthe = ['\theta: ' num2str(round(curTheta,2))];
        txtPpsi = '\psi: 0';
        text(640,260,670,txtPx,'FontSize',17)
        text(640,260,640,txtPy,'FontSize',17)
        text(640,260,610,txtPz,'FontSize',17)
        text(640,260,580,txtPphi,'FontSize',17)
        text(640,260,550,txtPthe,'FontSize',17)
        text(640,260,520,txtPpsi,'FontSize',17)
        txtT = ['Time: ' num2str(round(curT,2)) ' s'];
        text(600,260,700,txtT,'FontSize',17)        
        
        % show collision or collision free
        flagCollisionFree = isCollisionFree(curT,intvIF);
        if flagCollisionFree
            txtCol = 'Collision Free';
            text(310,670,670,txtCol,'FontSize',17,'Color','c')
        else
            txtCol = 'Collision!';
            text(310,670,670,txtCol,'FontSize',17,'Color','r')
        end
        if curT <= intvIF(1,2)
            % history traj.
            historyx = polyval(path{1},linspace(tm_via(1),curT,10));
            historyy = polyval(path{2},linspace(tm_via(1),curT,10));
            historyz = polyval(path{3},linspace(tm_via(1),curT,10));
            plot3(ax,historyx,historyy,historyz,'c','LineWidth',1.2);
        elseif curT >= intvIF(1,2) && curT <= intvIF(2,1)
            pathfreex = polyval(path{1},linspace(tm_via(1),intvIF(1,2),10));
            pathfreey = polyval(path{2},linspace(tm_via(1),intvIF(1,2),10));
            pathfreez = polyval(path{3},linspace(tm_via(1),intvIF(1,2),10));
            plot3(ax,pathfreex,pathfreey,pathfreez,'c','LineWidth',1.2);
            % history traj.
            historyx = polyval(path{1},linspace(intvIF(1,2),curT,10));
            historyy = polyval(path{2},linspace(intvIF(1,2),curT,10));
            historyz = polyval(path{3},linspace(intvIF(1,2),curT,10));
            plot3(ax,historyx,historyy,historyz,'r','LineWidth',1.2);
        else
            pathfreex = polyval(path{1},linspace(tm_via(1),intvIF(1,2),10));
            pathfreey = polyval(path{2},linspace(tm_via(1),intvIF(1,2),10));
            pathfreez = polyval(path{3},linspace(tm_via(1),intvIF(1,2),10));
            plot3(ax,pathfreex,pathfreey,pathfreez,'c','LineWidth',1.2);
            % history traj.
            historyyx = polyval(path{1},linspace(intvIF(1,2),intvIF(2,1),10));
            historyyy = polyval(path{2},linspace(intvIF(1,2),intvIF(2,1),10));
            historyyz = polyval(path{3},linspace(intvIF(1,2),intvIF(2,1),10));
            plot3(ax,historyyx,historyyy,historyyz,'r','LineWidth',1.2);
            historyx = polyval(path{1},linspace(intvIF(2,1),curT,10));
            historyy = polyval(path{2},linspace(intvIF(2,1),curT,10));
            historyz = polyval(path{3},linspace(intvIF(2,1),curT,10));
            plot3(ax,historyx,historyy,historyz,'c','LineWidth',1.2);
        end
        % goal
        plot3(ax,goalx,goaly,goalz,'k*')
        
        % plot the ellipsoid
        sEllip = obsInfo;        
        surf(sEllip.XData,sEllip.YData,sEllip.ZData,'FaceAlpha',0.4,'EdgeColor','k');
        surf(sEllip.XData,sEllip.YData,area(5)*ones(size(sEllip.YData)),'FaceColor',[0.85 0.85 0.85],'EdgeColor','none');

        view([-64 15])
        axis equal
        xlabel('x (cm)','FontWeight','bold')
        ylabel('y (cm)','FontWeight','bold')
        zlabel('z (cm)','FontWeight','bold')
        axis(area)
        grid on        
        box on        
        drawnow;
        mov(cnt) = getframe(gcf);%  记录动画帧

        pause(0.1)
    end
    v = VideoWriter(['.\video\' videoName]);
    v.FrameRate = t_f;
    open(v)
    writeVideo(v,mov);
    close(v)
end