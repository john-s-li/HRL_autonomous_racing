%% get_curvature.m

function k = get_curvature(s, track)
    s = mod(s, track.trackLength);
    
    % Heaviside Function (MUST be updated based on track's specification...see Track.m)
    % Will parametrize later when source is to be published
%     c = -1;
%     k = c*(heaviside(s-3) - heaviside(s-4.57)) + ...
%         c*(heaviside(s-6.57) - heaviside(s-8.1416)) + ...
%         c*(heaviside(s-14.1416) - heaviside(s-15.7124)) + ...
%         c*(heaviside(s-17.7124) - heaviside(s-19.2832));

    % Given s in [0, trackLength], compute the curvature
    [index,~] = find(s >= track.pointAndTangent(:,4) & ... 
                     s < track.pointAndTangent(:,4) + track.pointAndTangent(:,5));
                                  
    k = track.pointAndTangent(index,end); 
end