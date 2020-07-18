%% dynamicsTest.m
% Script to ensure that curvilinear dynamics are okay
clear
clc
close all

addpath('Classes')
addpath('Utilities')

% Load vehicle and racetrack parameters
vehParams = vehicleParams();
classTrack = Track(0.8);

v_ref = 0.8;

tspan = [0 10];
x0 = [0; 0; 0; 1.0]; % start aligned and on centerline with v0 = 1.0 m/s

[t,x] = ode45(@(t,x)vehDynamics(t,x,vehParams,classTrack), tspan, x0);

s = x(:,1);
e_lat = x(:,2);
e_psi = x(:,3);
v = x(:,4);

plot(t,s,'DisplayName','$s$')
hold on
plot(t,e_lat,'DisplayName','$e_{y}$')
plot(t,e_psi,'DisplayName','$e_{\psi}$')
plot(t,v,'DisplayName','$v$')
hold off
xlabel('time $t$','Interpreter','latex')
title('Simplified Bicycle Dynamics with PID Control','Interpreter','latex')
legend('show','Interpreter','latex','FontSize',14)
grid on
