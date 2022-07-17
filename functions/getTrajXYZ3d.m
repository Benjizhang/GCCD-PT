% function to get the trajectory according to the trajId
% 3D ONLY!
%
% @ Benji Z. Zhang

function traj = getTrajXYZ3d(trajALL, trajId)

    trajXALL = trajALL{1};
    trajYALL = trajALL{2};
    trajZALL = trajALL{3};
    traj{1} = trajXALL{trajId};
    traj{2} = trajYALL{trajId};
    traj{3} = trajZALL{trajId};
end