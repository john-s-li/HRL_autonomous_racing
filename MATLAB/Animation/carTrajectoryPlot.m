%% carTrajectoryPlot.m
% Function to plot out the predicted trajectory from MPC

function traj = carTrajectoryPlot(x_traj, track)
    
    xs = zeros(1,size(x_traj,2));
    ys = zeros(1,size(x_traj,2));
    
    % transform the curvilinear coordinates to the global frame
    for i = 1:size(x_traj,2)
        s = x_traj(4,i);
        e_lat = x_traj(5,i);
        [xs(i),ys(i)] = track.getGlobalPosition(s, e_lat);
    end
    
    traj = plot(xs, ys,'--r*','LineWidth',1.2);
end