%% Track.m
% File to create the race track

classdef Track
    properties 
        width = 0;
        trackLength = 0;
        pointAndTangent = [];
    end
    
    methods
        function obj = Track(width) % Class constructor
            obj.width = width; 
            
            % Define the racetrack --------------------------------
            % Element = [segment length = r * theta, 
            %            radius = segment length / theta if theta != 0 else 0]
            % radius < 0 = clockwise angle span
            % radius > 0 = counter-clockwise angle span
            % radius = 0 = straight line
            
            spec = [3, 0;
                    pi/2 * 1, -pi/2 * 1 / (pi/2);
                    2, 0;
                    pi/2 * 1, -pi/2 * 1 / (pi/2);
                    6,0;
                    pi/2 * 1, -pi/2 * 1 / (pi/2);
                    2, 0;
                    pi/2 * 1, -pi/2 * 1 / (pi/2)];
                         
            % Compute the (x,y) points of the track ----------------
            % pointAndTangent = [x, y, psi, cumulative s, segment length, signed curvature]
            
            pointAndTangent = zeros(size(spec,1)+1,6);
            
            for i = 1:size(spec,1) 
               if spec(i,2) == 0.0 % Segment is a straight line
                   l = spec(i,1); % Length of the line segment
                   if i == 1
                       ang = 0; % Angle of the tangent vector psi @ start
                       x = 0 + l*cos(ang); % x coord @ end of segment
                       y = 0 + l*sin(ang); % y coord @ end of segment

                       pointAndTangent(i,:) = [x, y, ang, pointAndTangent(i,4), l, 0];
                   else
                       ang = pointAndTangent(i-1,3);
                       x = pointAndTangent(i-1,1) + l*cos(ang);
                       y = pointAndTangent(i-1,2) + l*sin(ang);
                       pointAndTangent(i,:) = [x, y, ang, pointAndTangent(i-1,4) + ...
                                               pointAndTangent(i-1,5), l, 0];
                   end
               else % Segment is curved
                   l = spec(i,1); % Length of the segment
                   r = spec(i,2); % Radius of the segment

                   if r >= 0
                       direction = 1;
                   else
                       direction = -1;
                   end
                   
                   if i == 1
                       ang = 0;
                       centerX = 0 + abs(r)*cos(ang + direction*pi/2); % circle center x coord
                       centerY = 0 + abs(r)*sin(ang + direction*pi/2); % circle center y coord
                   else
                       ang = pointAndTangent(i-1,3);
                       centerX = pointAndTangent(i-1,1) + abs(r)*cos(ang + direction*pi/2);
                       centerY = pointAndTangent(i-1,2) + abs(r)*sin(ang + direction*pi/2);
                   end
                   
                   spanAng = l/abs(r); % l = r*theta --> theta = l/r
                   psi = wrap(ang + spanAng*sign(r)); % tangent vector angle of segment's last pt
                   
                   angleNormal = wrap(direction*pi/2 + ang);
                   angle = -(pi - abs(angleNormal)) * my_sign(angleNormal);
                   
                   x = centerX + abs(r)*cos(angle + direction*spanAng);
                   y = centerY + abs(r)*sin(angle + direction*spanAng);
                                      
                   if i == 1
                       pointAndTangent(i,:) = [x, y, psi, pointAndTangent(i,4), l, 1/r];
                   else
                       pointAndTangent(i,:) = [x, y, psi, pointAndTangent(i-1,4) + ... 
                                                          pointAndTangent(i-1,5), l, 1/r];
                   end
                   
               end 
               
            end % for loop ---
            
            % Fill in the last vector from last point to start
            xs = pointAndTangent(end-1,1);
            ys = pointAndTangent(end-1,2);
            xf = 0; yf = 0; psi_f = 0;

            l = sqrt((xf - xs)^2 + (yf - ys)^2);
            pointAndTangent(end,:) = [xf,yf,psi_f, ...
                                     pointAndTangent(end-1,4) + pointAndTangent(end-1,5), ...
                                     l, 0];

            obj.pointAndTangent = pointAndTangent;
            obj.trackLength = pointAndTangent(end,4) + pointAndTangent(end,5);
            
        end % -------
        
        function [x,y] = getGlobalPosition(obj,s,ey)
            error('notImplementedError')
        end % -------
        
        function [s,ey,epsi,flag] = getLocalPosition(obj,x,y,psi)
            error('notImplementedError')
        end % -------
        
    end
end

%% Internal Functions ----------------------------------------------

function signed = my_sign(angle)
    if angle >= 0
        signed = 1;
    else
        signed = -1;
    end
end

function wrap_angle = wrap(angle)
    if angle < -pi
       wrap_angle = 2*pi + angle; 
    elseif angle > pi
        wrap_angle = angle - 2*pi;
    else
        wrap_angle = angle;
    end
end


