%% get_curvature.m

function k = get_curvature(s, Track)
    % Curvature Computation
    % s: curvilinear abscissa in which curvature needs to be evaluated
    % Track: class Track object
    
    while s > Track.trackLength
       s = s - track.trackLength; 
    end
    
    % Given s in [0, trackLength], compute the curvature
    [index,~] = find(s >= Track.pointAndTangent(:,4) & ... 
                     s < Track.pointAndTangent(:,4) + Track.pointAndTangent(:,5));
                                  
    k = Track.pointAndTangent(index,end); 
end