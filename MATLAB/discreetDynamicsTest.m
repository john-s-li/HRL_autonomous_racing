%% discreetDynamcicsTest.m
% test to see if discreet dynamics are stable

clear
clc
close all

addpath('Classes')
addpath('Utilities')

% Load vehicle and racetrack parameters
vehParams = vehicleParams();
classTrack = Track(0.8);

dt = 0.05;
x0 = [1.0; 0; 0.0; 0.0; 0.0; 0.0]; % [vx; vy; wz; s; e_lat; e_psi]

% Logging Variables
x = x0;
x_next = zeros(size(x0,1),1);
t = [0];
u = [];

while (x0(4) <= classTrack.trackLength)
    % Do some PID control
    u_PID = get_PID(x0, 2.0);
    u = [u, u_PID];
    
    dx = vehicleDynamics(0, x0, u_PID, vehParams, classTrack);
    
    x_next = x0 + dt*dx;
    
    x = [x, x_next];
    
    t = [t, t(end)+dt];
    
    % Update the IC
    x0 = x_next;
end

% Animations
plotLog(x, [], vehParams, classTrack, dt);
statePlot(x,[],u,t)


%% Internal Functions
function u = get_PID(x, vx_ref)
    vx = x(1); % velocity in x-axis wrt/vehicle frame
    e_psi = x(6); % heading error wrt/track frame
    e_lat = x(5); % lateral error wrt/track centerline

     % PID Control -----------------------------------------------
    K_v = 1.5;
    K_psi = 0.6;
    K_lat = 0.9;
  
    delta = -K_psi*(e_psi) - K_lat*(e_lat); % Drive to zero
    accel = -K_v*(vx - vx_ref);
    
    u = [accel;delta];
end
