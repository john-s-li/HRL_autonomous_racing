%% plotTrack.m
% File to plot the racetrack

function plotTrack(track)
    % Plot the racetrack
    num_pts = floor(10*track.trackLength);
    
    pts0 = zeros(num_pts,2);
    pts1 = zeros(num_pts,2);
    pts2 = zeros(num_pts,2);
    
    for i = 1:num_pts        
       [pts0(i,1), pts0(i,2)] = track.getGlobalPosition(i * 0.1,  0);
       [pts1(i,1), pts1(i,2)] = track.getGlobalPosition(i * 0.1,  track.width);
       [pts2(i,1), pts2(i,2)] = track.getGlobalPosition(i * 0.1, -track.width);
    end
    
    PaT = track.pointAndTangent;
        
    plot(PaT(:,1),PaT(:,2),'o') % plot the x-y points of the spec
    hold on
    % Plot the track center, outer and inner tubes
    plot(pts0(:,1),pts0(:,2),'k--')
    plot(pts1(:,1),pts1(:,2),'k-')
    plot(pts2(:,1),pts2(:,2),'k-')
    
    % Fix the axis limits 
    xlim([min(pts1(:,1)) - 0.5 , max(pts1(:,1)) + 0.5])
    ylim([min(pts1(:,2)) - 0.5 , max(pts1(:,2)) + 0.5])
    
end