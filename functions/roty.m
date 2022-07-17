% function about roty

function Ry = roty(deg)
    
    Ry = [cos(deg2rad(deg)) 0 sin(deg2rad(deg));
          0 1 0;
          -sin(deg2rad(deg)) 0 cos(deg2rad(deg))];

end