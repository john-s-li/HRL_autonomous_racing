%% get_curvature.m

function k = get_curvature(s, Track)
    % Curvature Computation
    % s: curvilinear abscissa in which curvature needs to be evaluated
    % Track: class Track object
    
    if s > Track.trackLength
        s = mod(s, Track.trackLength); % in case if s is after first lap
    end
    
    % Given s in [0, trackLength], compute the curvature
    [index,~] = find(s >= Track.pointAndTangent(:,4) & ... 
                     s < Track.pointAndTangent(:,4) + Track.pointAndTangent(:,5));
                 
    k = Track.pointAndTangent(index,end);
end