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
            
            s = 2;
            
            if s == 1
                spec = [3, 0;
                        pi/2 * 1, -pi/2 * 1 / (pi/2);
                        2, 0;
                        pi/2 * 1, -pi/2 * 1 / (pi/2);
                        6,0;
                        pi/2 * 1, -pi/2 * 1 / (pi/2);
                        2, 0;
                        pi/2 * 1, -pi/2 * 1 / (pi/2)];
                
            elseif s == 2         
                spec = [pi/2*4, -4;
                        pi/2*4, -4;
                        pi/2*4, -4;
                        pi/2*4, -4];
                    
            elseif s == 3
                spec = [9,0];
            end
                         
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
            % Coordinate transformation from curvilinear frame (e, ey) to
            % intertial reference frame (X, Y)
            
            s = mod(s, obj.trackLength);
            
            % Compute the segment in which the car is on
            PaT = obj.pointAndTangent;
            
            [i, ~] = find(s >= PaT(:,4) & s < PaT(:,4) + PaT(:,5));
            
            if PaT(i,end) == 0.0 % Segment is a straight line
                
                % extract the first and initial point of the segment
                xf = PaT(i,1); yf = PaT(i,2); psi = PaT(i,3);
                
                if i ~= 1
                    xs = PaT(i-1,1); ys = PaT(i-1,2);
                else
                    xs = PaT(end,1); ys = PaT(end,2);
                end
                
                % Compute the length of the segment
                sL = PaT(i,5); % segment length
                pS = s - PaT(i,4); % "progress" on the segment
                
                % Linear Combination
                c = pS/sL;
                x = (1 - c) * xs + c * xf + ey * cos(psi + pi/2);
                y = (1 - c) * ys + c * yf + ey * sin(psi + pi/2);
                
            else
                r = 1/PaT(i,end); % extract the curvature
                
                if r >= 0
                    direction = 1;
                else
                    direction = -1;
                end
                
                % Extract angle of tangent at the initial point
                if i ~= 1
                    ang = PaT(i-1,3);
                    centerX = PaT(i-1,1) + abs(r) * cos(ang + direction * pi/2); 
                    centerY = PaT(i-1,2) + abs(r) * sin(ang + direction * pi/2); 
                    
                else
                    ang = PaT(end,3);
                    centerX = PaT(end,1) + abs(r) * cos(ang + direction * pi/2); 
                    centerY = PaT(end,2) + abs(r) * sin(ang + direction * pi/2); 
                    
                end
                
                spanAng = (s - PaT(i,4)) / (pi * abs(r)) * pi;
                angleNormal = wrap(direction * pi/2 + ang);
                
                angle = -(pi - abs(angleNormal)) * my_sign(angleNormal);
                
                % X and Y position of last point in last segment
                x = centerX + (abs(r) - direction * ey) * cos(angle + direction*spanAng);
                y = centerY + (abs(r) - direction * ey) * sin(angle + direction*spanAng);
                   
            end
            
        end % -------
        
        function yaw = getGlobalOrientation(obj,s,e_psi)
           % Function to compute the yaw angle of the car in the (X,Y) frame
           
            s = mod(s, obj.trackLength);

            % Compute the segment in which the car is on
            PaT = obj.pointAndTangent;

            [i, ~] = find(s >= PaT(:,4) & s < PaT(:,4) + PaT(:,5));

            if PaT(i,end) == 0.0 % Segment is a straight line

                % extract the tangent vector of the segment
                psi = PaT(i,3);
                
            else
                
                r = 1/PaT(i,end); % extract the curvature

                if r >= 0
                    direction = 1;
                else
                    direction = -1;
                end

                % Extract angle of tangent at the initial point
                if i ~= 1
                    ang = PaT(i-1,3);
                    centerX = PaT(i-1,1) + abs(r) * cos(ang + direction * pi/2); 
                    centerY = PaT(i-1,2) + abs(r) * sin(ang + direction * pi/2); 

                else
                    ang = PaT(end,3);
                    centerX = PaT(end,1) + abs(r) * cos(ang + direction * pi/2); 
                    centerY = PaT(end,2) + abs(r) * sin(ang + direction * pi/2); 
                end

                spanAng = (s - PaT(i,4)) / (pi * abs(r)) * pi;
                angleNormal = wrap(direction * pi/2 + ang);

                angle = -(pi - abs(angleNormal)) * my_sign(angleNormal);
                
                psi = angle + direction*spanAng + pi/2;

            end % -- IF/ELSE
            
            yaw = e_psi + psi; % Check to see if any wrapping is needed...
                      
        end
        
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


