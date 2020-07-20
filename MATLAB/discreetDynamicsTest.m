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

dt = 0.2;
tspan = [0 10];
x0 = [0; 0; 0; 1.0]; % start aligned and on centerline with v0 = 1.0 m/s

% Apply no acceleration and constant steering angle

x = x0;
x_next = zeros(size(x0,1),1);
t = [0];

while (x0(1) <= classTrack.trackLength)
    s = x0(1);
    e_lat = x0(2);
    e_psi = x0(3);
    v = x0(4);
    
    R = -4;
    u = [0.0; atan((vehParams.lf + vehParams.lr)/R)];
    
    dx = vehDynamics(0, x0, u, vehParams, classTrack, 1);
    
    x_next(1) = s + dt*dx(1);
    x_next(2) = e_lat + dt*dx(2);
    x_next(3) = e_psi + dt*dx(3);
    x_next(4) = v + dt*dx(4);
    
    x = [x, x_next];
    
    t = [t, t(end)+0.2];
    
    % Update the IC
    x0 = x_next;
end

s = x(1,:);
e_lat = x(2,:);
e_psi = x(3,:);
v = x(4,:);

plot(t,s,'DisplayName','$s$','LineWidth',2)
hold on
plot(t,e_lat,'DisplayName','$e_{y}$','LineWidth',2)
plot(t,e_psi,'DisplayName','$e_{\psi}$','LineWidth',2)
plot(t,v,'DisplayName','$v$','LineWidth',2)
hold off
xlabel('time $t$','Interpreter','latex')
title_str = ['Simplified Bicycle Dynamics'];
title(title_str,'Interpreter','latex')
legend('show','Interpreter','latex','FontSize',14)
grid on
