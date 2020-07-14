%% dynamicsTest.m
% Script to ensure that curvilinear dynamics are okay
clear
clc
close all

addpath('Classes')
addpath('Utilities')

vehicleParams; % Load vehicle params
track = Track(0.8); % Make a track object

v_ref = 0.8;

tspan = [0 10];
x0 = [0; 0; 0; 1.0]; % start aligned and on centerline with v0 = 1.0 m/s

[t,x] = ode45(@(t,x)curv_bicycle_dyn(t,x,v_ref,lr,lf,track), tspan, x0);

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

function [dx] = curv_bicycle_dyn(t,x,v_ref,lr,lf,track)
    % Define the ODEs    
    s = x(1);
    e_lat = x(2);
    e_psi = x(3);
    v = x(4);
    
    % Do some PID control
    K_lat = 0.6;
    K_psi = 0.9;
    K_v = 1.5;
    accel = K_v*(v_ref - v);
    delta = -K_lat*(e_lat) -K_psi*(e_psi);
   
    beta = atan((lr/(lf+lr))*tan(delta));

    ds = v*(cos(e_psi + beta))/(1 - e_lat*get_curvature(s,track));
    de_lat = v*sin(e_psi + beta);
    de_psi = (v/lf)*sin(beta) - get_curvature(s,track)*ds;
    dv = accel;
    
    dx = [ds;
          de_lat;
          de_psi;
          dv];
end