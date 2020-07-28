%% PID_Test.m
% Script to ensure that curvilinear dynamics are okay
clear
clc
close all

addpath('Classes')
addpath('Utilities')
addpath('Animation')

vehParams = vehicleParams(); % Load vehicle params
track = Track(0.8); % Make a track object

vx_ref = 3.0;

tspan = [0 25];
x0 = [1.0; 0; 0.3; 0.0; 0.3; 0.2]; % [vx; vy; wz; s; e_lat; e_psi]

opts = odeset('RelTol', 1e-3, 'Events', @event);
[t,x] = ode45(@(t,x)curv_bicycle_ode(t,x,vx_ref,vehParams,track),tspan, x0, opts);

x = x';
% plotLog(x, [], vehParams, track, 0.2);

%% get the PID control vector back
u = [];

for i = 1:size(x,2)
    u = [u get_PID(x(:,i),vx_ref)];
end
statePlot(x,[],u,t)

%% Internal functions
function dx = curv_bicycle_ode(t,x,vx_ref,vehParams,track)
    % PID Control -----------------------------------------------
    u = get_PID(x, vx_ref);    
    dx = vehicleDynamics(t,x,u,vehParams,track);
end

function [value,isterminal,direction] = event(t,x)
    s = x(4);
    value = s - 24;
    isterminal = 1;
    direction = 1;
end

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