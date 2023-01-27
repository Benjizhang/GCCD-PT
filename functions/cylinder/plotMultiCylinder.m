% function to plot the mulitple (tilded) cylinder obstacles
% 
% Benji Z. Zhang
% 1/2022

function plotMultiCylinder(ax,endPtsCy,cyRadius)
    num_cylinder = length(cyRadius);
    for i=1:num_cylinder
        % plot the central axis
        ptsj = endPtsCy(i,1:3);
        ptej = endPtsCy(i,4:6);
        plot3(ax,[ptsj(1) ptej(1)],[ptsj(2) ptej(2)],[ptsj(3) ptej(3)], 'k--')
        hold on
        
        % plot the cylinder surface
        D = ptej-ptsj;
        hg = norm(D,2);
        curR = cyRadius(i);
        [X,Y,Z] = cylinder2(curR,D,20,hg); 
        %%% surf(ax,X+endPtsCy(i,1),Y+endPtsCy(i,2),Z+endPtsCy(i,3),'FaceAlpha',0.4,'FaceColor',[.8,.8,.8],'EdgeColor','none')
        surf(ax,X+endPtsCy(i,1),Y+endPtsCy(i,2),Z+endPtsCy(i,3),'FaceColor',[.8,.8,.8],'FaceAlpha',0.6,'EdgeColor','none')        
        hold on
    end
end