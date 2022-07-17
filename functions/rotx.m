% function about rotx

function Rx = rotx(deg)
    
    Rx = [1 0 0;
          0 cos(deg2rad(deg)) -sin(deg2rad(deg));
          0 sin(deg2rad(deg)) cos(deg2rad(deg))];

end
