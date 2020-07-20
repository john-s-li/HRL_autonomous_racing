%% carPolygon.m
% Function to plot the polygon for the car

function car = carPolygon(x,y,yaw,w,l)
    X = [-w/2, w/2, w/2, -w/2];
    Y = [-l/2, -l/2, l/2, l/2];
    
    cy = cos(-yaw); sy = sin(-yaw);
    
    X_rot =  X*cy + Y*sy;
    Y_rot = -X*sy + Y*cy;
    
    car = fill(x + X_rot, y + Y_rot, 'r');
end