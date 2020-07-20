%% plotLog.m
% File to plot out simulation results

function plotLog(x_log, vehParams, track)

    % Plot should look like this (do at end)
    % |  |  |  |  ||   s   |   % 5 rows x 4 columns
    % |   RACE    || e_lat |  
    % |   TRACK   || e_psi |  
    % |  |  |  |  ||   v   |  

    % Plot the track
    plotTrack(track);
    hold on
    
    % Plot the car
    pause;
    disp('Enter Space Bar to start animation')
    
    % Initialize the polygon
    
    for i = 1:size(x_log,2)        
        s = x_log(1,i);
        e_lat = x_log(2,i);
        e_psi = x_log(3,i);
        v = x_log(4,i);
        
        % Get global position of the car
        [vehX, vehY] = track.getGlobalPosition(s,e_lat);
        
        % Get the orientation of the car
        vehYaw = track.getGlobalOrientation(s, e_psi);
        
        % Plot the car polygon
        car = carPolygon(vehX, vehY, vehYaw, vehParams.l, vehParams.w);
        pause(0.0001)
        delete(car)
        
    end
    
    % Plot the optimal trajectory (to do after figure out curvature function) 
    
    disp('Animation Done')
        
end