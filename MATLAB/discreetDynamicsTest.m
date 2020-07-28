%% discreetDynamicsTest.m
% Test to see if discreet dynamics are stable or not

clear
clc
close all

addpath('Classes')
addpath('Utilities')

% Load vehicle and racetrack parameters
vehParams = vehicleParams();
classTrack = Track(0.8);

dt = 0.1;
x0 = [0; 0.3; 0.3; 1.0]; % start aligned and on centerline with v0 = 1.0 m/s

% Apply no acceleration and constant steering angle

x = x0;
x_next = zeros(size(x0,1),1);
t = [0];
u = [];

while (x0(1) <= classTrack.trackLength)
    s = x0(1);
    e_lat = x0(2);
    e_psi = x0(3);
    v = x0(4);
    
    % Do some PID control
    u_PID = PID(x0, 1.5);
    u = [u, u_PID];
    
    dx = vehicleDynamics(0, x0, u_PID, vehParams, classTrack);
    
    x_next(1) = s + dt*dx(1);
    x_next(2) = e_lat + dt*dx(2);
    x_next(3) = e_psi + dt*dx(3);
    x_next(4) = v + dt*dx(4);
    
    x = [x, x_next];
    
    t = [t, t(end)+dt];
    
    % Update the IC
    x0 = x_next;
end

% animation and plotting
plotLog(x, [], vehParams, classTrack, dt)
statePlot(x,[], u, dt)

%% Control Functions
function u = PID(x, v_ref)
    K_psi = 0.6;
    K_lat = 3.5;
    K_v = 1.5;
    
    accel = K_v*(v_ref - x(4));
    delta = -K_psi*x(3) - K_lat*(x(2));

    u = [accel; delta];
end

function u = constant_steer(R,vehParams)
    u = [0.0; 
         atan((vehParams.lf + vehParams.lr)/R)];     
end
