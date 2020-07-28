%% plotLog.m
% File to plot out simulation results

function plotLog(x_log, x_traj_log, vehParams, track, pause_time)

    % Plot the track
    plotTrack(track);
    hold on
    
    % Plot the car
    disp('Press any key to start animation')
    pause;
    
    % Initialize the polygon
    
    for i = 1:size(x_log,2)        
        s = x_log(4,i);
        e_lat = x_log(5,i);
        e_psi = x_log(6,i);
 
        % Get global position of the car
        [vehX, vehY] = track.getGlobalPosition(s, e_lat);
        
        % Get the orientation of the car
        vehYaw = track.getGlobalOrientation(s, e_psi);
        
        % Plot the car polygon
        car = carPolygon(vehX, vehY, vehYaw, vehParams.l, vehParams.w);
        
        % Plot the trajectory
        if ~isempty(x_traj_log)
            traj = carTrajectoryPlot(x_traj_log(num2str(i)), track);
        end
        
        pause(pause_time)
        
        % disp('Press a key to continue')
        % pause;
        
        delete(car);
        
        if ~isempty(x_traj_log)
            delete(traj);
        end
        
    end
    
    % Plot the optimal trajectory (to do after figure out curvature function) 
    disp('Animation Done')
        
end