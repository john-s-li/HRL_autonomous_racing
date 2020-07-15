%% get_curvature.m

function k = get_curvature(s, Track)
    % Curvature Computation
    % s: curvilinear abscissa in which curvature needs to be evaluated
    % Track: class Track object
    
%     if_else(s > Track.trackLength, mod(s, Track.trackLength), s);
%             
%     % Given s in [0, trackLength], compute the curvature
%     [index,~] = find(s_r >= Track.pointAndTangent(:,4) & ... 
%                      s_r < Track.pointAndTangent(:,4) + Track.pointAndTangent(:,5));
%                                   
%     k = Track.pointAndTangent(index,end); 
    
    k = 1/-4;
end