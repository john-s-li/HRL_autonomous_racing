%% plotLog.m
% File to plot out simulation results

function plotLog(x_log, x_traj_log, vehParams, track, pause_time)

    % Plot should look like this (do at end)
    % |  |  |  |  ||   s   |   % 5 rows x 4 columns
    % |   RACE    || e_lat |  
    % |   TRACK   || e_psi |  
    % |  |  |  |  ||   v   |  

    % Plot the track
    plotTrack(track);
    hold on
    
    % Plot the car
    disp('Enter Space Bar to start animation')
    pause;
    
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
        
        % Plot the trajectory
        traj = carTrajectoryPlot(x_traj_log(num2str(i)), track);
        
        pause(pause_time)
        
        % disp('Press a key to continue')
        % pause;
        
        delete(car);
        delete(traj);
    end
    
    % Plot the optimal trajectory (to do after figure out curvature function) 
    disp('Animation Done')
        
end