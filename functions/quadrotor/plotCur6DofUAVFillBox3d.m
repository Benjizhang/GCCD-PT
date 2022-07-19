% function to DIRECTLY plot current UAV at given time
% tiem \in [t_via(1),t_via(2)] RATHER THAN t \in [0,1]
% 6Dof: x,y,z,phi(pitch),theta(roll),psi(yaw)
% filled polygonal regions in a color (e.g., blue)
% UAV is modelled as a box, rather than a 2d square
%
% @ Benji Z. Zhang

function plotCur6DofUAVFillBox3d(ax,robpts,path,cur_time)

    curx = polyval(path{1},cur_time);
    cury = polyval(path{2},cur_time);
    curz = polyval(path{3},cur_time);
    
    R_cur = getRotMat(path,cur_time);
    temp1 = [curx; cury; curz]*ones(1,size(robpts,1)); % R^{3*num_edge}
    temp2 = R_cur*robpts';
    robpts_wd = (temp1 + temp2)'; % R^{num_edge*3}
    % --- v2.0 ---
    ptOrder = [1:4 1 5:8 5];    
    X_rbt = robpts_wd(ptOrder,1); % R^{num_edge*3}
    Y_rbt = robpts_wd(ptOrder,2);
    Z_rbt = robpts_wd(ptOrder,3);
    plot3(ax, X_rbt,Y_rbt,Z_rbt,'Color','m','LineWidth',1,'LineStyle','--');
    hold(ax,"on")
    ptOrder2 = [2 6; 7 3; 4 8];
    for i = 1:size(ptOrder2,1)
        X_rbt = robpts_wd(ptOrder2(i,:),1); % R^{num_edge*3}
        Y_rbt = robpts_wd(ptOrder2(i,:),2);
        Z_rbt = robpts_wd(ptOrder2(i,:),3);
        plot3(ax, X_rbt,Y_rbt,Z_rbt,'Color','m','LineWidth',1,'LineStyle','--');
        hold(ax,"on")
    end

    % plot 4 quadmotors
    motorCent = robpts(1:4,:)*0.5;  % R^{4*3}
    motorRadius = 0.8*min(abs(motorCent(1,1:2)));
    for i = 1:4
        curCent = motorCent(i,:); % R^{1*3}
        angle = -pi:0.1:pi;
        motor_x = curCent(1)+motorRadius*cos(angle);
        motor_y = curCent(2)+motorRadius*sin(angle);
        motor_z = curCent(3)*ones(size(motor_y));
        temp3 = [curx; cury; curz]*ones(size(motor_y));
        temp4 = R_cur*[motor_x; motor_y; motor_z];
        motor_wd = temp3+temp4;
        patch(ax, motor_wd(1,:),motor_wd(2,:),motor_wd(3,:),'m','EdgeColor',[0.4940 0.1840 0.5560],'FaceAlpha',1,'LineWidth',2)
    end
    % plot the cross of the uav body
    temp5= R_cur*[motorCent(:,1:2) -motorCent(:,3)]'; % R^{3*4}
    motorCent_wd = (temp1(:,1:4) + temp5)'; % R^{4*3}
    plot3(ax, motorCent_wd([1 3],1),motorCent_wd([1 3],2),motorCent_wd([1 3],3),'k','LineWidth',3);
    plot3(ax, motorCent_wd([2 4],1),motorCent_wd([2 4],2),motorCent_wd([2 4],3),'k','LineWidth',3);
    % connect cross to the center of 4 propellers
    for i = 1:4
        curCent = [curx; cury; curz]+ R_cur*motorCent(i,:)'; % R^{3*1}
        plot3(ax, [curCent(1) motorCent_wd(i,1)],[curCent(2) motorCent_wd(i,2)],[curCent(3) motorCent_wd(i,3)],'k','LineWidth',3)
        hold(ax,"on")
    end

end

