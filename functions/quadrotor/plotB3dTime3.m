% function to plot 3d Bazier curve + control pts (if any)
% w.r.t. time \in [t_via(1),t_via(2)]
% ---- v2.0 ----
% pn is a cell \in R(1,3)
% ---- v3.0 ----
% input color of curve
% using linespace to discretize time interval
%
% @ Benji Z. Zhang

function plotB3dTime3(ax,pn,controlPts,t_via,color)
    w = linspace(t_via(1),t_via(2)); % a row vector of 100 evenly spaced points between x1 and x2.
    bcurx = polyval(pn{1},w);
    bcury = polyval(pn{2},w);
    bcurz = polyval(pn{3},w);
    if ~isempty(controlPts)
        plot3(ax,controlPts(:,1),controlPts(:,2),controlPts(:,3),'rx');
        hold on
    end
    plot3(ax,bcurx,bcury,bcurz,color,'LineWidth',1.2)
    hold on
end